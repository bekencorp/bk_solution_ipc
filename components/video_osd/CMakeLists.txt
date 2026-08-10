set(incs include)
set(srcs)

if (CONFIG_VIDEO_OSD)
list(APPEND srcs
    src/video_osd.c
    assets/video_osd_font.c
)
endif()

armino_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "${incs}"
    PRIV_REQUIRES bk_draw_osd bk_encoder bk_frame_buffer avdk_utils
)
