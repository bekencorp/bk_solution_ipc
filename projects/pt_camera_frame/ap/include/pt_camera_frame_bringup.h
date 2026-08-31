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

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Project-local frame-mode codec + NDR bond bring-up (replaces the stock
 *        flexa app_h264e_turn_on + bk_flexa_isp_h264e_bond_start for pt_camera_frame).
 *
 * Creates a frame-mode H264 encoder, starts the H264 stream transfer, then
 * starts the zero-copy NDR bond which takes over the MP channel. Call after the
 * MP camera has been turned on (frame mode) by app_isp_mipi_camera_turn_on().
 *
 * @return BK_OK on success.
 */
int pt_camera_frame_codec_bond_start(void);

/**
 * @brief Symmetric teardown: stop the bond, then the transfer and the encoder.
 * @return BK_OK on success.
 */
int pt_camera_frame_codec_bond_stop(void);

/** @brief Register project-local NDR test commands. */
int pt_camera_frame_cli_init(void);

#ifdef __cplusplus
}
#endif
