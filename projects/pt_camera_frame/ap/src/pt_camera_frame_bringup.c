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

#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>
#include <common/bk_err.h>
#include "cli.h"

#include "driver/isp.h"
#include "driver/isp_base.h"
#include "components/bk_frame_buffer.h"
#include "components/bk_encode/bk_h264_encode_ctlr.h"
#include "app_camera.h"
#include "mds_img_manager.h"
#include "pt_day_night.h"
#include "pt_frame_ndr_bond.h"
#include "pt_camera_frame_bringup.h"
#if CONFIG_H264E_STREAM_SESSION
#include "h264e_stream_session.h"
#endif
#if CONFIG_NTWK_H264_DROP_POLICY
#include "h264_backpressure_drop.h"
#endif

#define TAG "pt-frame-bringup"

#define LOGI(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/*
 * Reuse the stock output-buffer request callback from multimedia_device_service's
 * app_codec.c (non-static global). It wraps the encoded-data manager exactly as
 * the flexa path does, so the network/session plumbing is unchanged.
 *
 * The completion callback, however, is project-local: the stock
 * encoder_buffer_complete() re-targets the drop-policy force-IDR at
 * app_codec_enc_handler, which stays NULL in the frame path (this project owns a
 * private encoder s_frame_enc instead). Routing the force-IDR to app_codec's NULL
 * handle silently swallows it, so after a network-congestion frame drop the
 * decoder never gets an IDR to resync and moving areas stay mosaicked until the
 * next natural GOP boundary. We mirror the stock plumbing but drive force-IDR on
 * s_frame_enc so recovery actually happens.
 */
extern void *encoder_buffer_request(uint32_t buffer_len, void *args);

static bk_h264_encode_ctlr_handle_t s_frame_enc = NULL;
static void *s_ndr_bond = NULL;

static uint32_t pt_frame_encoder_buffer_complete(bk_h264_encode_outbuf_info_t *info)
{
	if (info == NULL || info->outbuf == NULL) {
		return BK_FAIL;
	}

	uint32_t frame_size = ((sizeof(frame_buffer_t) + 63) >> 6) << 6;
	frame_buffer_t *buffer = (frame_buffer_t *)((uint8_t *)info->outbuf - frame_size);
	if (info->status == BK_OK) {
		buffer->length = info->length;
		buffer->h264_type = info->type;
		buffer->fmt = PIXEL_FMT_H264;
		buffer->sequence = info->sequence;
		bk_encoded_data_complete_request((uint8_t *)buffer);
#if CONFIG_NTWK_H264_DROP_POLICY
		if (ntwk_h264_backpressure_drop_consume_force_idr() && s_frame_enc != NULL) {
			bk_h264_encode_force_idr(s_frame_enc);
		}
#endif
	} else {
		bk_encoded_data_free_request((uint8_t *)buffer);
	}

	return BK_OK;
}

static const char *pt_frame_ndr_mode_name(pt_frame_ndr_mode_t mode)
{
	switch (mode) {
	case PT_FRAME_NDR_MODE_AUTO:
		return "auto";
	case PT_FRAME_NDR_MODE_MANUAL_OFF:
		return "manual-off";
	case PT_FRAME_NDR_MODE_MANUAL_ON:
		return "manual-on";
	default:
		return "unknown";
	}
}

static void pt_frame_ndr_print_status(void)
{
	LOGI("NDR mode=%s alpha=%u iso_thres=%u intra_thres=%u\r\n",
	     pt_frame_ndr_mode_name(pt_frame_ndr_mode_get()),
	     pt_frame_ndr_alpha_get(),
	     pt_frame_ndr_iso_thres_get(),
	     pt_frame_ndr_intra_thres_get());
}

static uint8_t pt_frame_parse_u32(const char *text, uint32_t *value)
{
	char *end = NULL;
	unsigned long parsed;

	if (text == NULL || value == NULL || text[0] == '\0') {
		return 0;
	}
	parsed = os_strtoul(text, &end, 0);
	if (end == text || end == NULL || *end != '\0') {
		return 0;
	}
	*value = (uint32_t)parsed;
	return 1;
}

static void cli_pt_dnr_cmd(char *pcWriteBuffer, int xWriteBufferLen,
			   int argc, char **argv)
{
	(void)pcWriteBuffer;
	(void)xWriteBufferLen;
	uint32_t value;

	if (argc == 2 && os_strcmp(argv[1], "auto") == 0) {
		pt_frame_ndr_mode_set(PT_FRAME_NDR_MODE_AUTO);
	} else if (argc == 2 && os_strcmp(argv[1], "on") == 0) {
		pt_frame_ndr_mode_set(PT_FRAME_NDR_MODE_MANUAL_ON);
	} else if (argc == 2 && os_strcmp(argv[1], "off") == 0) {
		pt_frame_ndr_mode_set(PT_FRAME_NDR_MODE_MANUAL_OFF);
	} else if (argc == 3 && os_strcmp(argv[1], "alpha") == 0 &&
		   pt_frame_parse_u32(argv[2], &value) &&
		   value >= 1U && value <= 256U) {
		pt_frame_ndr_alpha_set(value);
	} else if (argc == 3 && os_strcmp(argv[1], "iso") == 0 &&
		   pt_frame_parse_u32(argv[2], &value) && value <= 5000U) {
		pt_frame_ndr_iso_thres_set(value);
	} else if (argc == 3 && os_strcmp(argv[1], "intra") == 0 &&
		   pt_frame_parse_u32(argv[2], &value)) {
		pt_frame_ndr_intra_thres_set(value);
	} else if (!(argc == 2 && os_strcmp(argv[1], "status") == 0)) {
		LOGI("usage: pt_dnr <auto|on|off|status|alpha 1..256|"
		     "iso 0..5000|intra value>\r\n");
		return;
	}

	pt_frame_ndr_print_status();
}

static const char *pt_day_night_mode_name(pt_day_night_mode_t mode)
{
	switch (mode) {
	case PT_DAY_NIGHT_MODE_AUTO:
		return "auto";
	case PT_DAY_NIGHT_MODE_MANUAL_DAY:
		return "manual-color";
	case PT_DAY_NIGHT_MODE_MANUAL_NIGHT:
		return "manual-ir";
	default:
		return "unknown";
	}
}

static void pt_ir_print_status(void)
{
	LOGI("IR mode=%s optics=%s\r\n",
	     pt_day_night_mode_name(pt_day_night_mode_get()),
	     pt_day_night_is_night() ? "night-infrared" : "day-color");
}

/*
 * pt_ir: control the infrared (night) vs full-color (day) optics.
 *   auto  - luminance-driven strategy: day full-color, night infrared.
 *   color - manually force full-color (day) mode.
 *   ir    - manually force infrared (night) mode.
 *   status- print the current mode and applied optical state.
 */
static void cli_pt_ir_cmd(char *pcWriteBuffer, int xWriteBufferLen,
			  int argc, char **argv)
{
	(void)pcWriteBuffer;
	(void)xWriteBufferLen;

	if (argc == 2 && os_strcmp(argv[1], "auto") == 0) {
		pt_day_night_mode_set(PT_DAY_NIGHT_MODE_AUTO);
	} else if (argc == 2 && (os_strcmp(argv[1], "color") == 0 ||
				 os_strcmp(argv[1], "day") == 0)) {
		pt_day_night_mode_set(PT_DAY_NIGHT_MODE_MANUAL_DAY);
	} else if (argc == 2 && (os_strcmp(argv[1], "ir") == 0 ||
				 os_strcmp(argv[1], "night") == 0)) {
		pt_day_night_mode_set(PT_DAY_NIGHT_MODE_MANUAL_NIGHT);
	} else if (!(argc == 2 && os_strcmp(argv[1], "status") == 0)) {
		LOGI("usage: pt_ir <auto|color|ir|status>\r\n");
		return;
	}

	pt_ir_print_status();
}

static const struct cli_command s_pt_camera_frame_commands[] = {
	{"pt_dnr", "pt_dnr <auto|on|off|status|alpha|iso|intra>",
	 cli_pt_dnr_cmd},
	{"pt_ir", "pt_ir <auto|color|ir|status>", cli_pt_ir_cmd},
};

int pt_camera_frame_cli_init(void)
{
	return cli_register_commands(s_pt_camera_frame_commands,
				     sizeof(s_pt_camera_frame_commands) /
				     sizeof(s_pt_camera_frame_commands[0]));
}

/*
 * The frame-mode bond drives a PRIVATE encoder (s_frame_enc) that never passes
 * through the doorbell path, so it would otherwise run with the encoder's raw
 * defaults (no bitrate cap, very low QP) -> huge frames and bad motion mosaic.
 * Apply the SAME CONFIG_H264_QP_PRESET_* rate control the doorbell/pt_camera
 * path uses (doorbell_apply_h264_qp_preset), keeping both projects aligned. */
static bk_err_t pt_frame_apply_h264_qp_preset(bk_h264_encode_ctlr_handle_t enc)
{
	bk_h264_encode_rate_ctrl_t rate_ctrl = {0};
	const char *preset_name = NULL;

	if (enc == NULL) {
		return BK_ERR_PARAM;
	}

#if CONFIG_H264_QP_PRESET_QUALITY
	preset_name = "quality";
	rate_ctrl.bitrate = 2000000;
	rate_ctrl.qp_min_i = 20;
	rate_ctrl.qp_max_i = 40;
	rate_ctrl.qp_min_p = 24;
	rate_ctrl.qp_max_p = 40;
#elif CONFIG_H264_QP_PRESET_FIXED_QP
	preset_name = "fixed-qp";
	rate_ctrl.bitrate = 0;
	rate_ctrl.qp_min_i = 21;
	rate_ctrl.qp_max_i = 21;
	rate_ctrl.qp_min_p = 26;
	rate_ctrl.qp_max_p = 26;
#elif CONFIG_H264_QP_PRESET_ANTI_STUTTER
	preset_name = "anti-stutter";
	rate_ctrl.bitrate = 1200000;
	rate_ctrl.qp_min_i = 24;
	rate_ctrl.qp_max_i = 45;
	rate_ctrl.qp_min_p = 28;
	rate_ctrl.qp_max_p = 48;
#elif CONFIG_H264_QP_PRESET_LAN_HD
	preset_name = "lan-hd";
	rate_ctrl.bitrate = 3000000;
	rate_ctrl.qp_min_i = 18;
	rate_ctrl.qp_max_i = 36;
	rate_ctrl.qp_min_p = 22;
	rate_ctrl.qp_max_p = 38;
#else /* default / CONFIG_H264_QP_PRESET_BALANCED */
	preset_name = "balanced";
	rate_ctrl.bitrate = 1500000;
	rate_ctrl.qp_min_i = 26;
	rate_ctrl.qp_max_i = 43;
	rate_ctrl.qp_min_p = 34;
	rate_ctrl.qp_max_p = 43;
#endif

	avdk_err_t ret = bk_h264_encode_set_rate_ctrl(enc, &rate_ctrl);
	if (ret != AVDK_ERR_OK) {
		LOGE("apply h264 qp preset failed, ret=%d\r\n", ret);
		return BK_FAIL;
	}
	LOGI("h264 qp preset %s: bitrate=%u i=[%u,%u] p=[%u,%u]\r\n",
	     preset_name, rate_ctrl.bitrate, rate_ctrl.qp_min_i, rate_ctrl.qp_max_i,
	     rate_ctrl.qp_min_p, rate_ctrl.qp_max_p);
	return BK_OK;
}

#if CONFIG_H264E_STREAM_SESSION
static beken_thread_t s_transfer_thread = NULL;
static beken_semaphore_t s_transfer_sem = NULL;
static volatile uint8_t s_transfer_enable = 0;

/* Local copy of app_codec.c's transfer loop (that one is static): drain encoded
 * frames from the manager's complete queue and push them into the H264 stream
 * session, keeping this bring-up fully project-local. */
static void pt_frame_transfer_task(beken_thread_arg_t data)
{
	(void)data;
	frame_buffer_t *frame = NULL;

	rtos_set_semaphore(&s_transfer_sem);

	while (s_transfer_enable) {
		frame = (frame_buffer_t *)bk_encoded_complete_data_request(50);
		if (frame == NULL) {
			continue;
		}
		(void)h264e_stream_session_send_h264((uint8_t *)frame, frame->length);
		bk_encoded_data_free_request((uint8_t *)frame);
	}

	s_transfer_thread = NULL;
	rtos_set_semaphore(&s_transfer_sem);
	rtos_delete_thread(NULL);
}

static bk_err_t pt_frame_transfer_start(void)
{
	bk_err_t ret;

	if (s_transfer_thread != NULL) {
		return BK_OK;
	}
	if (s_transfer_sem == NULL) {
		ret = rtos_init_semaphore(&s_transfer_sem, 1);
		if (ret != BK_OK) {
			LOGE("transfer sem init failed: %d\r\n", ret);
			return ret;
		}
	}
	s_transfer_enable = 1;
	ret = rtos_create_hsram_thread(&s_transfer_thread, BEKEN_DEFAULT_WORKER_PRIORITY,
				       "pt_h264_trs", (beken_thread_function_t)pt_frame_transfer_task,
				       4096, NULL);
	if (ret != BK_OK) {
		LOGE("transfer thread create failed: %d\r\n", ret);
		s_transfer_enable = 0;
		if (s_transfer_sem != NULL) {
			rtos_deinit_semaphore(&s_transfer_sem);
			s_transfer_sem = NULL;
		}
		return ret;
	}
	rtos_get_semaphore(&s_transfer_sem, BEKEN_NEVER_TIMEOUT);
	return BK_OK;
}

static void pt_frame_transfer_stop(void)
{
	s_transfer_enable = 0;
	if (s_transfer_thread != NULL) {
		rtos_get_semaphore(&s_transfer_sem, BEKEN_NEVER_TIMEOUT);
		s_transfer_thread = NULL;
	}
	if (s_transfer_sem != NULL) {
		rtos_deinit_semaphore(&s_transfer_sem);
		s_transfer_sem = NULL;
	}
}
#endif /* CONFIG_H264E_STREAM_SESSION */

int pt_camera_frame_codec_bond_start(void)
{
	bk_err_t ret;

	if (s_frame_enc != NULL || s_ndr_bond != NULL) {
		LOGW("already started\r\n");
		return BK_OK;
	}

	isp_control_t *isp_control = (isp_control_t *)app_isp_handle_get();
	if (isp_control == NULL) {
		LOGE("isp handle NULL\r\n");
		return BK_FAIL;
	}
	uint8_t chnl = ISP_MP_CHN_ID;
	uint32_t width = isp_control->chn[chnl].chn_attr.chnFormat.width;
	uint32_t height = isp_control->chn[chnl].chn_attr.chnFormat.height;

	bk_encoded_data_manager_init();

	bk_h264_encode_frame_config_t config = {
		.width = width,
		.height = height,
		.input_format = BK_PIXEL_FORMAT_NV12,
		.gop_frame_count = 40,
		.input_flexa_cnt = 0, /* full-frame: single slice */
		.input_buf = isp_control->chn[chnl].y_addr, /* overridden per-frame via set_input */
		.input_size = width * height * 3U / 2U,
		.outbuf_malloc = encoder_buffer_request,
		.outbuf_malloc_args = NULL,
		.outbuf_complete = pt_frame_encoder_buffer_complete,
		.outbuf_complete_args = NULL,
	};

	ret = bk_h264_encode_frame_new(&s_frame_enc, &config);
	if (ret != BK_OK) {
		LOGE("frame encoder new failed: %d\r\n", ret);
		goto err_mgr;
	}
	ret = bk_h264_encode_init(s_frame_enc);
	if (ret != BK_OK) {
		LOGE("frame encoder init failed: %d\r\n", ret);
		goto err_enc_del;
	}
	ret = bk_h264_encode_open(s_frame_enc);
	if (ret != BK_OK) {
		LOGE("frame encoder open failed: %d\r\n", ret);
		goto err_enc_deinit;
	}

	/* Align rate control with the doorbell/pt_camera QP preset (see helper). */
	(void)pt_frame_apply_h264_qp_preset(s_frame_enc);

	uint32_t debug_interval = 2000;
	bk_h264_encode_ioctl(s_frame_enc, BK_H264_ENCODE_IOCTL_DEBUG_START, &debug_interval);

#if CONFIG_H264E_STREAM_SESSION
	ret = pt_frame_transfer_start();
	if (ret != BK_OK) {
		LOGE("transfer start failed: %d\r\n", ret);
		goto err_enc_close;
	}
#endif

	ret = pt_frame_ndr_bond_start(&s_ndr_bond, app_isp_camera_ctlr_handle_get(), s_frame_enc);
	if (ret != BK_OK) {
		LOGE("ndr bond start failed: %d\r\n", ret);
		goto err_transfer;
	}

	ret = pt_day_night_start();
	if (ret != BK_OK) {
		LOGE("day/night start failed: %d\r\n", ret);
		goto err_bond;
	}

	LOGI("pt frame codec+bond started %ux%u\r\n", width, height);
	return BK_OK;

err_bond:
	pt_frame_ndr_bond_stop(s_ndr_bond);
	s_ndr_bond = NULL;
err_transfer:
#if CONFIG_H264E_STREAM_SESSION
	pt_frame_transfer_stop();
err_enc_close:
#endif
	bk_h264_encode_ioctl(s_frame_enc, BK_H264_ENCODE_IOCTL_DEBUG_STOP, NULL);
	bk_h264_encode_close(s_frame_enc);
err_enc_deinit:
	bk_h264_encode_deinit(s_frame_enc);
err_enc_del:
	bk_h264_encode_delete(s_frame_enc);
	s_frame_enc = NULL;
err_mgr:
	bk_encoded_data_manager_deinit(1);
	return BK_FAIL;
}

int pt_camera_frame_codec_bond_stop(void)
{
	pt_day_night_stop();

	if (s_ndr_bond != NULL) {
		pt_frame_ndr_bond_stop(s_ndr_bond);
		s_ndr_bond = NULL;
	}

#if CONFIG_H264E_STREAM_SESSION
	pt_frame_transfer_stop();
#endif

	if (s_frame_enc != NULL) {
		bk_h264_encode_ioctl(s_frame_enc, BK_H264_ENCODE_IOCTL_DEBUG_STOP, NULL);
		bk_h264_encode_close(s_frame_enc);
		bk_h264_encode_deinit(s_frame_enc);
		bk_h264_encode_delete(s_frame_enc);
		s_frame_enc = NULL;
	}

	bk_encoded_data_manager_deinit(1);
	return BK_OK;
}
