#ifndef VIDEO_OSD_H
#define VIDEO_OSD_H

#include <components/avdk_utils/avdk_error.h>
#include <components/bk_encode/bk_h264_encode_ctlr.h>

#ifdef __cplusplus
extern "C" {
#endif

avdk_err_t video_osd_time_on(bk_h264_encode_ctlr_handle_t enc, const char *datetime);
void video_osd_time_off(void);

#ifdef __cplusplus
}
#endif

#endif
