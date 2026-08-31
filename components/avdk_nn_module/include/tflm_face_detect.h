// Copyright 2020-2021 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>

#include "AvdkDetectionModel.h"
#include "tensorflow/lite/c/common.h"

extern "C" {
extern const unsigned char face_detection_vela_tflite[];
extern const unsigned int face_detection_vela_tflite_len;
}

#define FACE_DETECT_KEYPOINT_COUNT 5

typedef struct {
    float x;
    float y;
} FaceDetectPoint;

typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int order;
    FaceDetectPoint keypoints[FACE_DETECT_KEYPOINT_COUNT];
} FaceDetectDetection;

class FaceDetectModel : public AvdkDetectionModel {
public:
    FaceDetectModel();

    void resolverLoad(void) override;
    void resourceLoad(void) override;
    void resourceUnload(void) override;
    int run(uint8_t *data, uint32_t size, bk_pixel_format_t format) override;

private:
    struct OutputSpec {
        int stride;
        int feature_count;
        TfLiteTensor *score;
        TfLiteTensor *bbox;
        TfLiteTensor *keypoints;
    };

    void resetOutputSpecs(void);
    bool buildOutputSpecs(void);
    float tensorValue(const TfLiteTensor *tensor, int flat_index) const;
    float keypointTensorValue(const TfLiteTensor *tensor,
                              int logical_idx,
                              int channel,
                              int stride) const;
    int decode(FaceDetectDetection *candidates, int max_candidates);
    int nms(FaceDetectDetection *candidates,
            int count,
            FaceDetectDetection *out,
            int max_out);

    OutputSpec output_specs[3];
    bool output_tensors_ready;
};
