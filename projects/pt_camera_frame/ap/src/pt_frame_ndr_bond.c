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

/*
 * Project-local frame-mode ISP -> NDR -> H264 bond for pt_camera_frame.
 *
 * Threading model (see 3dnr/Frame模式零拷贝图传_NDR_新工程设计方案.md §5.4):
 *   The bond ACQUIRES the MP channel and uses a two-stage pipeline. The
 *   producer pops ISP frames and performs DNR, while a second thread drives
 *   synchronous H264 encoding. Thus the CPU DNR of frame N+1 overlaps the
 *   hardware encode of frame N.
 *
 * DNR output buffers (MP NV12 1280x720 ~1.32MB/frame):
 *   Three independent output slots are allocated from uncoded PSRAM. One slot
 *   is the temporal history, one can be read by H264, and the third can receive
 *   the next DNR result. ISP inputs are returned immediately after DNR.
 */

#include <os/os.h>
#include <os/mem.h>
#include <driver/isp_base.h>
#include <driver/isp_types.h>
#include <components/bk_isp_camera.h>
#include <components/bk_camera_isp_ctlr.h>
#include <components/bk_frame_buffer.h>
#include <components/bk_encode/bk_h264_encode_ctlr.h>
#include <components/avdk_utils/avdk_types.h>
#include <modules/veri_isp/mpi_isp_ae.h>
#include <modules/private/veri_isp/mpi_isp.h>
#include <modules/dnr.h>
#include <cache.h>
#include "pt_frame_ndr_bond.h"

#define TAG "pt_ndr_bond"

#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/*
 * Raised to the same level as the HW encoder driver task
 * (CONFIG_BK_ENCODER_HW_TASK_PRIORITY=1, internal prio 8 / highest) so the
 * two pipeline stages' scheduling latency is minimised. Beken priority is
 * inverted (smaller = higher). */
#define PT_NDR_BOND_TASK_PRIO   (1)
#define PT_NDR_BOND_TASK_SIZE   4096
#define PT_NDR_BOND_POP_TIMEOUT 1000U
#define PT_NDR_BOND_STOP_WAIT_MS 2000U
#define PT_NDR_OUTPUT_SLOT_COUNT 3U
#define PT_NDR_ENCODE_QUEUE_DEPTH 6U

typedef enum {
	PT_NDR_ENCODE_JOB_ISP = 0,
	PT_NDR_ENCODE_JOB_OUTPUT,
	PT_NDR_ENCODE_JOB_STOP,
} pt_ndr_encode_job_type_t;

typedef struct {
	pt_ndr_encode_job_type_t type;
	uint8_t index;
	uint8_t slot;
	uint32_t addr;
	uint32_t size;
} pt_ndr_encode_job_t;

typedef struct {
	bk_isp_camera_ctlr_handle_t camera;
	bk_h264_encode_ctlr_handle_t h264;
	uint8_t channel;
	volatile uint8_t enable;   /* thread run flag */
	uint8_t acquired;          /* channel taken over */
	uint32_t frame_width;
	uint32_t frame_height;

	/* NDR runtime state */
	uint8_t  ndr_on;
	int8_t   history_slot;
	uint32_t eval_cnt;         /* throttles the NDR-gate ISO query while OFF */
	uint32_t output_addr[PT_NDR_OUTPUT_SLOT_COUNT];
	uint8_t output_busy[PT_NDR_OUTPUT_SLOT_COUNT];

	beken_thread_t thread;
	beken_thread_t encoder_thread;
	beken_queue_t encoder_queue;
	beken_semaphore_t stop_sem;
	beken_semaphore_t encoder_stop_sem;
} pt_frame_ndr_bond_t;

/* Fixed-point IIR weight of the current frame, [1,256]. 100/256 ~= 0.39. */
static uint32_t s_dnr_alpha = 100;
/* Above this ISO, image is noisy enough to warrant temporal NDR. */
static uint32_t s_iso_thres = 400;
/* Above this many intra 8x8 CUs the scene is moving too much: keep NDR off. */
static uint32_t s_intra_cu8_thres = 2000;
static pt_frame_ndr_mode_t s_ndr_mode = PT_FRAME_NDR_MODE_AUTO;

void pt_frame_ndr_alpha_set(uint32_t alpha)
{
	alpha = (alpha == 0U) ? 1U : (alpha > 256U ? 256U : alpha);
	__atomic_store_n(&s_dnr_alpha, alpha, __ATOMIC_RELEASE);
}

uint32_t pt_frame_ndr_alpha_get(void)
{
	return __atomic_load_n(&s_dnr_alpha, __ATOMIC_ACQUIRE);
}

void pt_frame_ndr_iso_thres_set(uint32_t thres)
{
	thres = thres > 5000U ? 5000U : thres;
	__atomic_store_n(&s_iso_thres, thres, __ATOMIC_RELEASE);
}

uint32_t pt_frame_ndr_iso_thres_get(void)
{
	return __atomic_load_n(&s_iso_thres, __ATOMIC_ACQUIRE);
}

void pt_frame_ndr_intra_thres_set(uint32_t thres)
{
	__atomic_store_n(&s_intra_cu8_thres, thres, __ATOMIC_RELEASE);
}

uint32_t pt_frame_ndr_intra_thres_get(void)
{
	return __atomic_load_n(&s_intra_cu8_thres, __ATOMIC_ACQUIRE);
}

void pt_frame_ndr_mode_set(pt_frame_ndr_mode_t mode)
{
	if (mode > PT_FRAME_NDR_MODE_MANUAL_ON) {
		return;
	}
	__atomic_store_n(&s_ndr_mode, mode, __ATOMIC_RELEASE);
}

pt_frame_ndr_mode_t pt_frame_ndr_mode_get(void)
{
	return __atomic_load_n(&s_ndr_mode, __ATOMIC_ACQUIRE);
}

static isp_control_t *bond_get_isp_control(bk_isp_camera_ctlr_handle_t cam)
{
	bk_camera_isp_ctlr_t *ctlr;

	if (cam == NULL) {
		return NULL;
	}
	ctlr = __containerof(cam, bk_camera_isp_ctlr_t, ops);
	return (isp_control_t *)ctlr->isp_handle;
}

static avdk_err_t bond_channel_control(pt_frame_ndr_bond_t *bond,
				       bk_cam_interface_ioctl_t command)
{
	uint8_t channel = bond->channel;

	return bk_isp_camera_ctlr_ioctl(bond->camera, command, &channel);
}

static avdk_err_t bond_frame_pop(pt_frame_ndr_bond_t *bond,
				 bk_isp_camera_frame_info_t *info)
{
	os_memset(info, 0, sizeof(*info));
	info->channel = bond->channel;
	info->timeout = PT_NDR_BOND_POP_TIMEOUT;
	return bk_isp_camera_ctlr_ioctl(bond->camera, BK_CAM_IOCTL_FRAME_POP, info);
}

static avdk_err_t bond_frame_qbuf(pt_frame_ndr_bond_t *bond, uint8_t index)
{
	bk_isp_camera_frame_info_t info = {
		.channel = bond->channel,
		.index = index,
	};

	return bk_isp_camera_ctlr_ioctl(bond->camera, BK_CAM_IOCTL_FRAME_QBUF, &info);
}

static uint32_t bond_query_iso(pt_frame_ndr_bond_t *bond)
{
	isp_control_t *isp_control = bond_get_isp_control(bond->camera);
	ISP_EXPOSURE_INFO_S expInfo = {0};

	if (isp_control == NULL) {
		return 0;
	}

	ISP_PORT IspPort = {
		.devId  = isp_control->port.devId,
		.portId = isp_control->chn[ISP_MP_CHN_ID].channel.portId,
	};
	if (VSI_MPI_ISP_QueryExposureInfo(IspPort, &expInfo) != VSI_SUCCESS) {
		return 0;
	}
	return expInfo.iso;
}

/*
 * Decide whether temporal NDR should be active for the frame we are about to
 * encode. Uses the CURRENT exposure ISO and the PREVIOUS encoded frame's
 * intra_cu8_num (a motion proxy): denoise only when the sensor gain is high
 * (noisy) and the scene is not moving fast (low intra), matching the reference.
 */
static uint8_t bond_ndr_should_be_on(pt_frame_ndr_bond_t *bond)
{
	bk_h264_encode_stream_info_t stream_info = {0};
	uint32_t iso;
	pt_frame_ndr_mode_t mode = pt_frame_ndr_mode_get();

	if (mode == PT_FRAME_NDR_MODE_MANUAL_ON) {
		return 1;
	}
	if (mode == PT_FRAME_NDR_MODE_MANUAL_OFF) {
		return 0;
	}

	/*
	 * ISO/motion only need re-checking every few frames to decide the NDR
	 * on/off gate; VSI_MPI_ISP_QueryExposureInfo is a heavy per-frame call
	 * that hurts throughput (frame mode is already CPU-overhead bound, unlike
	 * flexa). While NDR is already ON we must keep re-evaluating every frame
	 * (motion may spike and force it off); while OFF we can throttle.
	 */
	if (!bond->ndr_on) {
		if ((bond->eval_cnt++ & 0x7u) != 0u) {
			return 0;
		}
	}

	(void)bk_h264_encode_ioctl(bond->h264,
				  BK_H264_ENCODE_IOCTL_GET_STREAM_INFO,
				  &stream_info);
	iso = bond_query_iso(bond);

	if (iso <= pt_frame_ndr_iso_thres_get()) {
		return 0;
	}
	if (stream_info.intra_cu8_num > pt_frame_ndr_intra_thres_get()) {
		return 0;
	}

	return 1;
}

static avdk_err_t bond_encode_one(pt_frame_ndr_bond_t *bond, uint32_t input_buf, uint32_t input_size)
{
	bk_h264_encode_input_t input = {
		.input_buf = input_buf,
		.input_size = input_size,
	};
	avdk_err_t ret = bk_h264_encode_ioctl(bond->h264,
					     BK_H264_ENCODE_IOCTL_SET_INPUT_BUF,
					     &input);
	if (ret != AVDK_ERR_OK) {
		LOGE("set_input failed %d\n", ret);
		return ret;
	}
	ret = bk_h264_encode_start(bond->h264);
	if (ret != AVDK_ERR_OK) {
		LOGE("encode_start failed %d\n", ret);
		return ret;
	}
	return ret;
}

static int32_t bond_find_free_output(pt_frame_ndr_bond_t *bond)
{
	for (uint32_t i = 0; i < PT_NDR_OUTPUT_SLOT_COUNT; i++) {
		if ((int32_t)i == bond->history_slot) {
			continue;
		}
		if (__atomic_load_n(&bond->output_busy[i], __ATOMIC_ACQUIRE) == 0U) {
			return (int32_t)i;
		}
	}
	return -1;
}

static bk_err_t bond_queue_encode(pt_frame_ndr_bond_t *bond,
				  pt_ndr_encode_job_t *job)
{
	return rtos_push_to_queue(&bond->encoder_queue, job, BEKEN_NO_WAIT);
}

static void bond_process_dnr_frame(pt_frame_ndr_bond_t *bond,
				   const bk_isp_camera_frame_info_t *info)
{
	int32_t slot = bond_find_free_output(bond);
	if (slot < 0) {
		bond_frame_qbuf(bond, info->index);
		return;
	}

	uint32_t dst = bond->output_addr[slot];
	uint32_t y_size = bond->frame_width * bond->frame_height;
	arch_dcache_invd_range((void *)(uintptr_t)info->frame_addr, info->frame_size);

	if (!bond->ndr_on || bond->history_slot < 0) {
		os_memcpy((void *)(uintptr_t)dst,
			  (const void *)(uintptr_t)info->frame_addr, info->frame_size);
		bond->ndr_on = 1;
	} else {
		uint32_t history = bond->output_addr[bond->history_slot];
		bk_dnr_blend_planes_to((uint8_t *)(uintptr_t)dst,
				       (uint8_t *)(uintptr_t)(dst + y_size),
				       (const uint8_t *)(uintptr_t)history,
				       (const uint8_t *)(uintptr_t)info->frame_addr,
				       bond->frame_width, bond->frame_height,
				       pt_frame_ndr_alpha_get());
	}

	bond->history_slot = (int8_t)slot;
	__atomic_store_n(&bond->output_busy[slot], 1U, __ATOMIC_RELEASE);

	pt_ndr_encode_job_t job = {
		.type = PT_NDR_ENCODE_JOB_OUTPUT,
		.slot = (uint8_t)slot,
		.addr = dst,
		.size = info->frame_size,
	};
	if (bond_queue_encode(bond, &job) != BK_OK) {
		__atomic_store_n(&bond->output_busy[slot], 0U, __ATOMIC_RELEASE);
	}
	bond_frame_qbuf(bond, info->index);
}

static void bond_process_frame(pt_frame_ndr_bond_t *bond,
			       const bk_isp_camera_frame_info_t *info)
{
	if (bond_ndr_should_be_on(bond)) {
		bond_process_dnr_frame(bond, info);
		return;
	}

	bond->ndr_on = 0;
	bond->history_slot = -1;
	pt_ndr_encode_job_t job = {
		.type = PT_NDR_ENCODE_JOB_ISP,
		.index = info->index,
		.addr = info->frame_addr,
		.size = info->frame_size,
	};
	if (bond_queue_encode(bond, &job) != BK_OK) {
		bond_frame_qbuf(bond, info->index);
	}
}

static void pt_frame_ndr_encoder_task(beken_thread_arg_t arg)
{
	pt_frame_ndr_bond_t *bond = (pt_frame_ndr_bond_t *)arg;
	pt_ndr_encode_job_t job;

	while (true) {
		if (rtos_pop_from_queue(&bond->encoder_queue, &job,
					BEKEN_WAIT_FOREVER) != BK_OK) {
			continue;
		}
		if (job.type == PT_NDR_ENCODE_JOB_STOP) {
			break;
		}

		(void)bond_encode_one(bond, job.addr, job.size);
		if (job.type == PT_NDR_ENCODE_JOB_ISP) {
			bond_frame_qbuf(bond, job.index);
		} else {
			__atomic_store_n(&bond->output_busy[job.slot], 0U,
					 __ATOMIC_RELEASE);
		}
	}

	if (bond->encoder_stop_sem != NULL) {
		(void)rtos_set_semaphore(&bond->encoder_stop_sem);
	}
	bond->encoder_thread = NULL;
	rtos_delete_thread(NULL);
}

static void pt_frame_ndr_bond_task(beken_thread_arg_t arg)
{
	pt_frame_ndr_bond_t *bond = (pt_frame_ndr_bond_t *)arg;
	bk_isp_camera_frame_info_t info;

	while (bond->enable) {
		avdk_err_t ret = bond_frame_pop(bond, &info);
		if (ret != AVDK_ERR_OK) {
			continue;
		}
		if (info.frame_addr == 0U || info.frame_size == 0U) {
			bond_frame_qbuf(bond, info.index);
			continue;
		}
		if (!bond->enable) {
			bond_frame_qbuf(bond, info.index);
			break;
		}
		bond_process_frame(bond, &info);
	}

	if (bond->stop_sem != NULL) {
		(void)rtos_set_semaphore(&bond->stop_sem);
	}
	bond->thread = NULL;
	rtos_delete_thread(NULL);
}

static void bond_free_output_buffers(pt_frame_ndr_bond_t *bond)
{
	for (uint32_t i = 0; i < PT_NDR_OUTPUT_SLOT_COUNT; i++) {
		if (bond->output_addr[i] != 0U) {
			bk_frame_buffer_free((void *)(uintptr_t)bond->output_addr[i]);
			bond->output_addr[i] = 0U;
		}
	}
}

static bk_err_t bond_alloc_output_buffers(pt_frame_ndr_bond_t *bond)
{
	uint32_t frame_size = bond->frame_width * bond->frame_height * 3U / 2U;

	for (uint32_t i = 0; i < PT_NDR_OUTPUT_SLOT_COUNT; i++) {
		void *buffer = bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED,
						     frame_size);
		if (buffer == NULL) {
			bond_free_output_buffers(bond);
			return BK_ERR_NO_MEM;
		}
		bond->output_addr[i] = (uint32_t)(uintptr_t)buffer;
	}
	return BK_OK;
}

avdk_err_t pt_frame_ndr_bond_start(void **bond, void *camera, bk_h264_encode_ctlr_handle_t h264)
{
	bk_isp_camera_ctlr_handle_t cam = (bk_isp_camera_ctlr_handle_t)camera;
	pt_frame_ndr_bond_t *bond_new;
	isp_control_t *isp_control;
	bk_err_t ret;

	if (bond == NULL || cam == NULL || h264 == NULL) {
		LOGE("invalid args bond=%p camera=%p h264=%p\n", bond, camera, h264);
		return AVDK_ERR_INVAL;
	}
	if (*bond != NULL) {
		LOGE("already started\n");
		return AVDK_ERR_INVAL;
	}

	bond_new = (pt_frame_ndr_bond_t *)os_malloc(sizeof(pt_frame_ndr_bond_t));
	if (bond_new == NULL) {
		LOGE("malloc bond failed\n");
		return AVDK_ERR_NOMEM;
	}
	os_memset(bond_new, 0, sizeof(pt_frame_ndr_bond_t));
	bond_new->camera = cam;
	bond_new->h264 = h264;
	bond_new->channel = ISP_MP_CHN_ID;
	bond_new->frame_width = 1280;
	bond_new->frame_height = 720;
	bond_new->history_slot = -1;

	isp_control = bond_get_isp_control(cam);
	if (isp_control != NULL) {
		uint32_t w = isp_control->chn[ISP_MP_CHN_ID].chn_attr.chnFormat.width;
		uint32_t h = isp_control->chn[ISP_MP_CHN_ID].chn_attr.chnFormat.height;
		if (w != 0U && h != 0U) {
			bond_new->frame_width = w;
			bond_new->frame_height = h;
		}
	}

	ret = bond_alloc_output_buffers(bond_new);
	if (ret != BK_OK) {
		LOGE("allocate NDR output buffers failed\n");
		os_free(bond_new);
		return AVDK_ERR_NOMEM;
	}

	ret = rtos_init_semaphore(&bond_new->stop_sem, 1);
	if (ret != BK_OK) {
		LOGE("init stop_sem failed %d\n", ret);
		bond_free_output_buffers(bond_new);
		os_free(bond_new);
		return AVDK_ERR_GENERIC;
	}
	ret = rtos_init_semaphore(&bond_new->encoder_stop_sem, 1);
	if (ret != BK_OK) {
		LOGE("init encoder_stop_sem failed %d\n", ret);
		rtos_deinit_semaphore(&bond_new->stop_sem);
		bond_free_output_buffers(bond_new);
		os_free(bond_new);
		return AVDK_ERR_GENERIC;
	}
	ret = rtos_init_queue(&bond_new->encoder_queue, "pt_ndr_enc_q",
			      sizeof(pt_ndr_encode_job_t),
			      PT_NDR_ENCODE_QUEUE_DEPTH);
	if (ret != BK_OK) {
		LOGE("init encoder queue failed %d\n", ret);
		rtos_deinit_semaphore(&bond_new->encoder_stop_sem);
		rtos_deinit_semaphore(&bond_new->stop_sem);
		bond_free_output_buffers(bond_new);
		os_free(bond_new);
		return AVDK_ERR_GENERIC;
	}

	/* Take the MP channel away from cam_thread BEFORE we start popping. */
	if (bond_channel_control(bond_new, BK_CAM_IOCTL_CHANNEL_ACQUIRE) != AVDK_ERR_OK) {
		LOGE("channel_acquire failed\n");
		rtos_deinit_queue(&bond_new->encoder_queue);
		rtos_deinit_semaphore(&bond_new->encoder_stop_sem);
		rtos_deinit_semaphore(&bond_new->stop_sem);
		bond_free_output_buffers(bond_new);
		os_free(bond_new);
		return AVDK_ERR_GENERIC;
	}
	bond_new->acquired = 1;
	bond_new->enable = 1;

	ret = rtos_create_hsram_thread(&bond_new->encoder_thread,
				       PT_NDR_BOND_TASK_PRIO, "pt_ndr_enc",
				       (beken_thread_function_t)pt_frame_ndr_encoder_task,
				       PT_NDR_BOND_TASK_SIZE, bond_new);
	if (ret != BK_OK) {
		LOGE("create encoder thread failed %d\n", ret);
		bond_new->enable = 0;
		bond_channel_control(bond_new, BK_CAM_IOCTL_CHANNEL_RELEASE);
		rtos_deinit_queue(&bond_new->encoder_queue);
		rtos_deinit_semaphore(&bond_new->encoder_stop_sem);
		rtos_deinit_semaphore(&bond_new->stop_sem);
		bond_free_output_buffers(bond_new);
		os_free(bond_new);
		return AVDK_ERR_GENERIC;
	}

	ret = rtos_create_hsram_thread(&bond_new->thread, PT_NDR_BOND_TASK_PRIO,
				       "pt_ndr_bond",
				       (beken_thread_function_t)pt_frame_ndr_bond_task,
				       PT_NDR_BOND_TASK_SIZE, bond_new);
	if (ret != BK_OK) {
		LOGE("create thread failed %d\n", ret);
		bond_new->enable = 0;
		pt_ndr_encode_job_t stop_job = {
			.type = PT_NDR_ENCODE_JOB_STOP,
		};
		rtos_push_to_queue(&bond_new->encoder_queue, &stop_job,
				   BEKEN_WAIT_FOREVER);
		rtos_get_semaphore(&bond_new->encoder_stop_sem,
				   PT_NDR_BOND_STOP_WAIT_MS);
		bond_channel_control(bond_new, BK_CAM_IOCTL_CHANNEL_RELEASE);
		rtos_deinit_queue(&bond_new->encoder_queue);
		rtos_deinit_semaphore(&bond_new->encoder_stop_sem);
		rtos_deinit_semaphore(&bond_new->stop_sem);
		bond_free_output_buffers(bond_new);
		os_free(bond_new);
		return AVDK_ERR_GENERIC;
	}

	*bond = bond_new;
	LOGI("bond started, channel=%u %ux%u\n", bond_new->channel,
	     bond_new->frame_width, bond_new->frame_height);
	return AVDK_ERR_OK;
}

void pt_frame_ndr_bond_stop(void *bond)
{
	pt_frame_ndr_bond_t *bond_p = (pt_frame_ndr_bond_t *)bond;

	if (bond_p == NULL) {
		return;
	}

	/* Stop the producer first so no new encode jobs can be queued. */
	bond_p->enable = 0;
	if (bond_p->thread != NULL && bond_p->stop_sem != NULL) {
		if (rtos_get_semaphore(&bond_p->stop_sem, PT_NDR_BOND_POP_TIMEOUT + PT_NDR_BOND_STOP_WAIT_MS) != BK_OK) {
			LOGW("stop wait timeout\n");
		}
	}

	/* A stop marker drains all queued frames before the encoder exits. */
	if (bond_p->encoder_thread != NULL && bond_p->encoder_queue != NULL) {
		pt_ndr_encode_job_t stop_job = {
			.type = PT_NDR_ENCODE_JOB_STOP,
		};
		if (rtos_push_to_queue(&bond_p->encoder_queue, &stop_job,
				       BEKEN_WAIT_FOREVER) == BK_OK) {
			if (rtos_get_semaphore(&bond_p->encoder_stop_sem,
					       PT_NDR_BOND_STOP_WAIT_MS) != BK_OK) {
				LOGW("encoder stop wait timeout\n");
			}
		}
	}

	if (bond_p->acquired) {
		bond_channel_control(bond_p, BK_CAM_IOCTL_CHANNEL_RELEASE);
		bond_p->acquired = 0;
	}

	if (bond_p->encoder_queue != NULL) {
		rtos_deinit_queue(&bond_p->encoder_queue);
		bond_p->encoder_queue = NULL;
	}
	if (bond_p->encoder_stop_sem != NULL) {
		rtos_deinit_semaphore(&bond_p->encoder_stop_sem);
		bond_p->encoder_stop_sem = NULL;
	}
	if (bond_p->stop_sem != NULL) {
		rtos_deinit_semaphore(&bond_p->stop_sem);
		bond_p->stop_sem = NULL;
	}

	bond_free_output_buffers(bond_p);
	os_free(bond_p);
	LOGI("bond stopped\n");
}
