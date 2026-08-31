#include "tflm_face_detect.h"

#include <algorithm>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <components/bk_frame_buffer.h>
#include <driver/aon_rtc.h>

#include "box.h"
#include "os/mem.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_log.h"

#if defined(__ARM_FEATURE_MVE)
#include <arm_mve.h>
#endif

static constexpr int kFaceInputW = 320;
static constexpr int kFaceInputH = 320;
static constexpr int kFaceInputC = 3;
static constexpr int kFaceInputBytes = kFaceInputW * kFaceInputH * kFaceInputC;
static constexpr int kFaceScratchSize = 80 * 1024;
static constexpr int kFaceArenaSize = 750 * 1024;
static constexpr int kFaceNumOutputs = 9;
static constexpr int kFaceNumStrides = 3;
static constexpr int kFaceNumAnchors = 2;
static constexpr int kFaceScoreChannels = 1;
static constexpr int kFaceBboxChannels = 4;
static constexpr int kFaceKeypointChannels = 10;
static constexpr int kFaceTopkCandidates = 32;
static constexpr int kFaceMaxDetections = 32;
static constexpr float kFaceScoreThreshold = 0.5f;
static constexpr float kFaceNmsThreshold = 0.4f;

static FaceDetectDetection *s_face_candidates = nullptr;
static FaceDetectDetection *s_face_detections = nullptr;
static Box *s_face_boxes = nullptr;

static bool face_detection_workbufs_ready(void)
{
    return s_face_candidates != nullptr &&
        s_face_detections != nullptr &&
        s_face_boxes != nullptr;
}

static void face_detection_free_workbufs(void)
{
    if (s_face_candidates != nullptr) {
        bk_frame_buffer_free(s_face_candidates);
    }
    if (s_face_detections != nullptr) {
        bk_frame_buffer_free(s_face_detections);
    }
    if (s_face_boxes != nullptr) {
        bk_frame_buffer_free(s_face_boxes);
    }

    s_face_candidates = nullptr;
    s_face_detections = nullptr;
    s_face_boxes = nullptr;
}

static bool face_detection_alloc_workbufs(void)
{
    if (face_detection_workbufs_ready()) {
        return true;
    }

    face_detection_free_workbufs();

    s_face_candidates = (FaceDetectDetection *)bk_frame_buffer_malloc(
        MEM_SLAB_HEAP_UNCODED, sizeof(FaceDetectDetection) * kFaceTopkCandidates);
    s_face_detections = (FaceDetectDetection *)bk_frame_buffer_malloc(
        MEM_SLAB_HEAP_UNCODED, sizeof(FaceDetectDetection) * kFaceMaxDetections);
    s_face_boxes = (Box *)bk_frame_buffer_malloc(
        MEM_SLAB_HEAP_UNCODED, sizeof(Box) * kFaceMaxDetections);

    if (!face_detection_workbufs_ready()) {
        MicroPrintf("FaceDetection work buffer alloc failed\r\n");
        face_detection_free_workbufs();
        return false;
    }

    return true;
}

static void face_detection_log_interval(const char *prefix, unsigned long long start_us)
{
    const unsigned long long elapsed_us = bk_aon_rtc_get_us() - start_us;
    MicroPrintf("%s: %llu us\r\n", prefix, elapsed_us);
}

static bool face_detection_rgb_to_rgb_int8_mve(const uint8_t *src,
                                               int8_t *dst,
                                               uint32_t width,
                                               uint32_t height)
{
    const uint32_t bytes = width * height * 3U;
    uint32_t i = 0;

#if defined(__ARM_FEATURE_MVE)
    for (; i + 16U <= bytes; i += 16U) {
        uint8x16_t rgb = vld1q_u8(src + i);
        rgb = vsubq_n_u8(rgb, 128);
        vst1q_u8((uint8_t *)dst + i, rgb);
    }
#endif

    for (; i < bytes; i++) {
        dst[i] = (int8_t)((int)src[i] - 128);
    }

    return true;
}

static bool face_detection_bgrx_to_rgb_int8_mve(const uint8_t *src,
                                                int8_t *dst,
                                                uint32_t width,
                                                uint32_t height)
{
    const uint32_t pixels = width * height;
    uint32_t i = 0;

#if defined(__ARM_FEATURE_MVE)
    for (; i + 16U <= pixels; i += 16U) {
        static const uint8_t k_mve_rgb_offsets[16] = {
            0, 3, 6, 9, 12, 15, 18, 21,
            24, 27, 30, 33, 36, 39, 42, 45,
        };
        const uint8x16x4_t bgrx = vld4q_u8(src + i * 4U);
        const uint8x16_t offset = vld1q_u8(k_mve_rgb_offsets);
        uint8_t *d = (uint8_t *)dst + i * 3U;

        vstrbq_scatter_offset_u8(d, offset, vsubq_n_u8(bgrx.val[2], 128));
        vstrbq_scatter_offset_u8(d + 1, offset, vsubq_n_u8(bgrx.val[1], 128));
        vstrbq_scatter_offset_u8(d + 2, offset, vsubq_n_u8(bgrx.val[0], 128));
    }
#endif

    for (; i < pixels; i++) {
        const uint8_t *s = src + i * 4U;
        int8_t *d = dst + i * 3U;

        d[0] = (int8_t)((int)s[2] - 128);
        d[1] = (int8_t)((int)s[1] - 128);
        d[2] = (int8_t)((int)s[0] - 128);
    }

    return true;
}

static bool face_detection_prepare_input(const uint8_t *data,
                                         uint32_t size,
                                         bk_pixel_format_t format,
                                         TfLiteTensor *input)
{
    const uint32_t rgb888_size = (uint32_t)kFaceInputBytes;
    const uint32_t bgra8888_max_size = (uint32_t)kFaceInputW * (uint32_t)kFaceInputH * 4U;
    const uint32_t bgra8888_line_size = (uint32_t)kFaceInputW * 4U;

    if (data == nullptr || input == nullptr) {
        return false;
    }

    if (format != BK_PIXEL_FORMAT_RGB888 && format != BK_PIXEL_FORMAT_BGRA8888) {
        MicroPrintf("FaceDetection unsupported source format=%u\r\n", (unsigned)format);
        return false;
    }

    if (format == BK_PIXEL_FORMAT_RGB888 && size != rgb888_size) {
        MicroPrintf("FaceDetection frame size=%u mismatch for format=%u\r\n",
                    (unsigned)size,
                    (unsigned)format);
        return false;
    }
    if (format == BK_PIXEL_FORMAT_BGRA8888 &&
        (size == 0 || size > bgra8888_max_size || (size % bgra8888_line_size) != 0)) {
        MicroPrintf("FaceDetection BGRA frame size=%u invalid\r\n", (unsigned)size);
        return false;
    }

    if (input->type != kTfLiteInt8) {
        MicroPrintf("FaceDetection unsupported input type=%d\r\n", input->type);
        return false;
    }

    if (format == BK_PIXEL_FORMAT_BGRA8888) {
        uint32_t src_height = size / bgra8888_line_size;
        os_memset(input->data.int8, 0, kFaceInputBytes);
        return face_detection_bgrx_to_rgb_int8_mve(data, input->data.int8,
                                                   kFaceInputW, src_height);
    }

    return face_detection_rgb_to_rgb_int8_mve(data, input->data.int8,
                                              kFaceInputW, kFaceInputH);
}

static float face_detection_iou(const FaceDetectDetection &a,
                                const FaceDetectDetection &b)
{
    const float xx1 = fmaxf(a.x1, b.x1);
    const float yy1 = fmaxf(a.y1, b.y1);
    const float xx2 = fminf(a.x2, b.x2);
    const float yy2 = fminf(a.y2, b.y2);
    const float w = fmaxf(0.0f, xx2 - xx1 + 1.0f);
    const float h = fmaxf(0.0f, yy2 - yy1 + 1.0f);
    const float inter = w * h;
    const float area_a = fmaxf(0.0f, a.x2 - a.x1 + 1.0f) *
                         fmaxf(0.0f, a.y2 - a.y1 + 1.0f);
    const float area_b = fmaxf(0.0f, b.x2 - b.x1 + 1.0f) *
                         fmaxf(0.0f, b.y2 - b.y1 + 1.0f);
    const float denom = area_a + area_b - inter;
    return denom > 1e-6f ? inter / denom : 0.0f;
}

FaceDetectModel::FaceDetectModel()
    : output_specs{{8, 3200, nullptr, nullptr, nullptr},
                   {16, 800, nullptr, nullptr, nullptr},
                   {32, 200, nullptr, nullptr, nullptr}},
      output_tensors_ready(false)
{
}

void FaceDetectModel::resolverLoad(void)
{
    if (micro_op_resolver.AddEthosU() != kTfLiteOk) {
        MicroPrintf("FaceDetection AddEthosU failed\r\n");
    }
    MicroPrintf("FaceDetection resolverLoad\r\n");
}

void FaceDetectModel::resourceLoad(void)
{
    name = "FaceDetectionModel";
    width = kFaceInputW;
    height = kFaceInputH;
    format = BK_PIXEL_FORMAT_RGB888;

    model_type = AVDK_NN_MODEL_TYPE_NPU;
    model_ram_type = AVDK_NN_MEM_TYPE_PSRAM_SLAB_UNCODED;
    model_flash_data = (uint8_t *)face_detection_vela_tflite;
    model_flash_data_size = face_detection_vela_tflite_len;
    model_data = (uint8_t *)face_detection_vela_tflite;
    model_data_size = face_detection_vela_tflite_len;

    fast_ram_type = AVDK_NN_MEM_TYPE_HSRAM;
    fast_ram_data_size = kFaceScratchSize;
    fast_ram_data = NULL;

    arena_data_size = kFaceArenaSize;
    arena_ram_type = AVDK_NN_MEM_TYPE_PSRAM_SLAB_UNCODED;
    arena_ram_data = NULL;

    MicroPrintf("FaceDetection resourceLoad model=%u arena=%u scratch=%u\r\n",
                (unsigned)model_flash_data_size,
                (unsigned)arena_data_size,
                (unsigned)fast_ram_data_size);
}

void FaceDetectModel::resourceUnload(void)
{
    resetOutputSpecs();
    face_detection_free_workbufs();
}

void FaceDetectModel::resetOutputSpecs(void)
{
    for (int i = 0; i < kFaceNumStrides; i++) {
        output_specs[i].score = nullptr;
        output_specs[i].bbox = nullptr;
        output_specs[i].keypoints = nullptr;
    }
    output_tensors_ready = false;
}

bool FaceDetectModel::buildOutputSpecs(void)
{
    if (pinterpreter == nullptr) {
        return false;
    }
    if (output_tensors_ready) {
        return true;
    }

    resetOutputSpecs();
    if (pinterpreter->outputs_size() != kFaceNumOutputs) {
        MicroPrintf("FaceDetection output count mismatch expect=%d got=%u\r\n",
                    kFaceNumOutputs, (unsigned)pinterpreter->outputs_size());
        return false;
    }

    for (size_t i = 0; i < pinterpreter->outputs_size(); i++) {
        TfLiteTensor *tensor = pinterpreter->output(i);
        if (tensor == nullptr || tensor->dims == nullptr || tensor->dims->size != 3) {
            MicroPrintf("FaceDetection output[%u] invalid dims\r\n", (unsigned)i);
            return false;
        }

        const int feature_count = tensor->dims->data[1];
        const int channels = tensor->dims->data[2];
        OutputSpec *spec = nullptr;
        for (int s = 0; s < kFaceNumStrides; s++) {
            if (output_specs[s].feature_count == feature_count) {
                spec = &output_specs[s];
                break;
            }
        }
        if (spec == nullptr) {
            MicroPrintf("FaceDetection output[%u] unexpected feature_count=%d\r\n",
                        (unsigned)i, feature_count);
            return false;
        }

        if (channels == kFaceScoreChannels) {
            spec->score = tensor;
        } else if (channels == kFaceBboxChannels) {
            spec->bbox = tensor;
        } else if (channels == kFaceKeypointChannels) {
            spec->keypoints = tensor;
        } else {
            MicroPrintf("FaceDetection output[%u] unexpected channels=%d\r\n",
                        (unsigned)i, channels);
            return false;
        }
    }

    for (int i = 0; i < kFaceNumStrides; i++) {
        if (output_specs[i].score == nullptr ||
            output_specs[i].bbox == nullptr ||
            output_specs[i].keypoints == nullptr) {
            MicroPrintf("FaceDetection missing tensors for stride=%d\r\n",
                        output_specs[i].stride);
            return false;
        }
    }

    output_tensors_ready = true;
    return true;
}

float FaceDetectModel::tensorValue(const TfLiteTensor *tensor, int flat_index) const
{
    if (tensor == nullptr) {
        return 0.0f;
    }
    if (tensor->type == kTfLiteInt8) {
        return ((float)tensor->data.int8[flat_index] -
                (float)tensor->params.zero_point) * tensor->params.scale;
    }
    if (tensor->type == kTfLiteFloat32) {
        return tensor->data.f[flat_index];
    }
    return 0.0f;
}

float FaceDetectModel::keypointTensorValue(const TfLiteTensor *tensor,
                                           int logical_idx,
                                           int channel,
                                           int stride) const
{
    int flat_index = logical_idx * kFaceKeypointChannels + channel;
    const int height = kFaceInputH / stride;
    const int width = kFaceInputW / stride;
    const int pretranspose_channels = kFaceNumAnchors * kFaceKeypointChannels;

    if (height == width && width == pretranspose_channels) {
        const int out_flat = flat_index;
        const int h = out_flat / (width * pretranspose_channels);
        const int rem = out_flat % (width * pretranspose_channels);
        const int w = rem / pretranspose_channels;
        const int c = rem % pretranspose_channels;
        flat_index = (c * height + h) * width + w;
    }

    return tensorValue(tensor, flat_index);
}

int FaceDetectModel::decode(FaceDetectDetection *candidates, int max_candidates)
{
    int count = 0;
    int order = 0;

    for (int spec_i = 0; spec_i < kFaceNumStrides; spec_i++) {
        const OutputSpec *spec = &output_specs[spec_i];
        const int stride = spec->stride;
        const int cols = kFaceInputW / stride;

        for (int idx = 0; idx < spec->feature_count; idx++) {
            const float score = tensorValue(spec->score, idx);
            if (score < kFaceScoreThreshold) {
                continue;
            }

            int out_index = count;
            if (count < max_candidates) {
                count++;
            } else {
                int min_index = 0;
                for (int i = 1; i < max_candidates; i++) {
                    if (candidates[i].score < candidates[min_index].score) {
                        min_index = i;
                    }
                }
                if (score <= candidates[min_index].score) {
                    order++;
                    continue;
                }
                out_index = min_index;
            }

            const int grid_index = idx / kFaceNumAnchors;
            const int r = grid_index / cols;
            const int c = grid_index % cols;
            const float cx = (float)(c * stride);
            const float cy = (float)(r * stride);

            FaceDetectDetection *det = &candidates[out_index];
            det->order = order++;
            det->score = score;
            det->x1 = cx - tensorValue(spec->bbox, idx * 4 + 0) * stride;
            det->y1 = cy - tensorValue(spec->bbox, idx * 4 + 1) * stride;
            det->x2 = cx + tensorValue(spec->bbox, idx * 4 + 2) * stride;
            det->y2 = cy + tensorValue(spec->bbox, idx * 4 + 3) * stride;

            for (int k = 0; k < FACE_DETECT_KEYPOINT_COUNT; k++) {
                det->keypoints[k].x =
                    cx + keypointTensorValue(spec->keypoints, idx, k * 2 + 0, stride) * stride;
                det->keypoints[k].y =
                    cy + keypointTensorValue(spec->keypoints, idx, k * 2 + 1, stride) * stride;
            }
        }
    }

    return count;
}

int FaceDetectModel::nms(FaceDetectDetection *candidates,
                         int count,
                         FaceDetectDetection *out,
                         int max_out)
{
    if (count <= 0) {
        return 0;
    }

    std::sort(candidates, candidates + count,
              [](const FaceDetectDetection &a, const FaceDetectDetection &b) {
                  if (a.score > b.score) {
                      return true;
                  }
                  if (a.score < b.score) {
                      return false;
                  }
                  return a.order > b.order;
              });

    int kept = 0;
    for (int i = 0; i < count && kept < max_out; i++) {
        bool keep = true;
        for (int j = 0; j < kept; j++) {
            if (face_detection_iou(candidates[i], out[j]) > kFaceNmsThreshold) {
                keep = false;
                break;
            }
        }
        if (!keep) {
            continue;
        }
        out[kept++] = candidates[i];
    }

    MicroPrintf("FaceDetection candidates=%d kept=%d\r\n", count, kept);
    return kept;
}

int FaceDetectModel::run(uint8_t *data, uint32_t size, bk_pixel_format_t format)
{
    if (pinterpreter == nullptr) {
        return 0;
    }

    TfLiteTensor *input = pinterpreter->input(0);
    if (input == nullptr ||
        input->dims == nullptr ||
        input->dims->size != 4 ||
        input->dims->data[1] != kFaceInputH ||
        input->dims->data[2] != kFaceInputW ||
        input->dims->data[3] != kFaceInputC ||
        input->bytes != kFaceInputBytes) {
        MicroPrintf("FaceDetection input shape mismatch\r\n");
        return 0;
    }
    if (!face_detection_alloc_workbufs()) {
        onBoxDetectionCallback(NULL, 0);
        return 0;
    }

    unsigned long long start_us = bk_aon_rtc_get_us();
    if (!face_detection_prepare_input(data, size, format, input)) {
        onBoxDetectionCallback(NULL, 0);
        return 0;
    }
    face_detection_log_interval("FaceDetection input prepare", start_us);

    start_us = bk_aon_rtc_get_us();
    if (pinterpreter->Invoke() != kTfLiteOk) {
        MicroPrintf("FaceDetection Invoke failed\r\n");
        onBoxDetectionCallback(NULL, 0);
        return 0;
    }
    face_detection_log_interval("FaceDetection Invoke", start_us);

    if (!output_tensors_ready && !buildOutputSpecs()) {
        onBoxDetectionCallback(NULL, 0);
        return 0;
    }

    start_us = bk_aon_rtc_get_us();
    const int candidate_count = decode(s_face_candidates, kFaceTopkCandidates);
    const int detection_count = nms(s_face_candidates, candidate_count,
                                    s_face_detections, kFaceMaxDetections);
    face_detection_log_interval("FaceDetection decode + NMS", start_us);

    int box_count = 0;
    for (int i = 0; i < detection_count && box_count < kFaceMaxDetections; i++) {
        float x1 = s_face_detections[i].x1;
        float y1 = s_face_detections[i].y1;
        float x2 = s_face_detections[i].x2;
        float y2 = s_face_detections[i].y2;

        if (x1 < 0.0f) x1 = 0.0f;
        if (y1 < 0.0f) y1 = 0.0f;
        if (x1 > (float)kFaceInputW) x1 = (float)kFaceInputW;
        if (y1 > (float)kFaceInputH) y1 = (float)kFaceInputH;
        if (x2 < 0.0f) x2 = 0.0f;
        if (y2 < 0.0f) y2 = 0.0f;
        if (x2 > (float)kFaceInputW) x2 = (float)kFaceInputW;
        if (y2 > (float)kFaceInputH) y2 = (float)kFaceInputH;
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

        s_face_boxes[box_count].x = x1;
        s_face_boxes[box_count].y = y1;
        s_face_boxes[box_count].w = x2 - x1;
        s_face_boxes[box_count].h = y2 - y1;
        s_face_boxes[box_count].score = s_face_detections[i].score;
        box_count++;
        MicroPrintf("FaceDetection box[0] x=%d y=%d w=%d h=%d score=%d/1000\r\n",
                    (int)x1,
                    (int)y1,
                    (int)(x2 - x1),
                    (int)(y2 - y1),
                    (int)(s_face_detections[i].score * 1000.0f));
        break;
    }

    onBoxDetectionCallback(box_count > 0 ? s_face_boxes : NULL,
                           box_count);
    return box_count;
}
