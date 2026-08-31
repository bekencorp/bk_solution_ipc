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

#include <components/avdk_utils/avdk_types.h>
#include <components/bk_encode/bk_h264_encode_ctlr.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	PT_FRAME_NDR_MODE_AUTO = 0,
	PT_FRAME_NDR_MODE_MANUAL_OFF,
	PT_FRAME_NDR_MODE_MANUAL_ON,
} pt_frame_ndr_mode_t;

/**
 * @brief Start the project-local frame-mode ISP -> (NDR) -> H264 bond.
 *
 * Zero-copy alternative to the stock flexa bond for the MP path:
 *   1. Takes over the MP ISP channel via BK_CAM_IOCTL_CHANNEL_ACQUIRE so the
 *      built-in cam_thread stops consuming it (no 4MB memcpy).
 *   2. Runs its OWN thread that pops full frames (BK_CAM_IOCTL_FRAME_POP),
 *      optionally blends temporal NDR into a retained history buffer
 *      (bk_dnr_blend_planes), then feeds the frame encoder via
 *      BK_H264_ENCODE_IOCTL_SET_INPUT_BUF + bk_h264_encode_start().
 *   3. Gates NDR on/off at runtime from ISO (exposure) and the previous frame's
 *      intra_cu8_num (BK_H264_ENCODE_IOCTL_GET_STREAM_INFO).
 *
 * @param bond   Out: opaque bond handle (set to NULL on failure).
 * @param camera ISP camera controller handle (bk_isp_camera_ctlr_handle_t).
 * @param h264   Frame-mode H264 encoder handle.
 * @return AVDK error code.
 */
avdk_err_t pt_frame_ndr_bond_start(void **bond, void *camera, bk_h264_encode_ctlr_handle_t h264);

/**
 * @brief Stop the bond: stop the thread, release the channel, return buffers.
 * @param bond Bond handle from pt_frame_ndr_bond_start().
 */
void pt_frame_ndr_bond_stop(void *bond);

/** @brief Set the temporal blend weight of the current frame, alpha in [1,256]. */
void pt_frame_ndr_alpha_set(uint32_t alpha);

/** @brief Return the temporal blend weight. */
uint32_t pt_frame_ndr_alpha_get(void);

/** @brief Set the ISO threshold above which NDR is allowed to turn on. */
void pt_frame_ndr_iso_thres_set(uint32_t thres);

/** @brief Return the ISO enable threshold. */
uint32_t pt_frame_ndr_iso_thres_get(void);

/** @brief Set the intra_cu8_num threshold above which NDR is forced off (motion). */
void pt_frame_ndr_intra_thres_set(uint32_t thres);

/** @brief Return the intra_cu8_num motion threshold. */
uint32_t pt_frame_ndr_intra_thres_get(void);

/** @brief Select automatic policy, forced-off, or forced-on NDR operation. */
void pt_frame_ndr_mode_set(pt_frame_ndr_mode_t mode);

/** @brief Return the currently selected NDR operation mode. */
pt_frame_ndr_mode_t pt_frame_ndr_mode_get(void);

#ifdef __cplusplus
}
#endif
