set(incs include)
set(srcs .)
set(priv_req bk_common)
set(req)

if (CONFIG_VIDEO_OSD)
list(APPEND incs ${CMAKE_CURRENT_SOURCE_DIR}/../video_osd/include)
list(APPEND req video_osd)
list(APPEND priv_req video_osd)
endif()

list(APPEND srcs 
	src/mds_img_manager.c
	src/app_camera.c
	src/app_codec.c
	src/devices_mgmt.c
	src/encode_frame_que.c
	src/app_jpeg_decode.c
	src/app_h264_encode.c
)

if (CONFIG_BK_DISPLAY)
list(APPEND srcs src/app_display.c)
endif()

if (CONFIG_VG_LITE_GPU)
list(APPEND srcs src/app_gpu.c)
endif()

if (CONFIG_MDS_CLI)
list(APPEND srcs src/devices_cli.c)
endif()

if (CONFIG_USB_CAMERA)
list(APPEND srcs src/app_uvc.c)
endif()

if (CONFIG_MDS_SNAPSHOT)
list(APPEND incs
	bk_codec_util/bk_snapshot/include
)
list(APPEND srcs
	bk_codec_util/bk_snapshot/src/bk_snapshot.c
	bk_codec_util/bk_snapshot/src/sw_isp_snapshot.c
)
list(APPEND priv_req bk_encoder bk_camera)
endif()

list(APPEND priv_req unity include avdk_utils avdk_driver bk_peripheral bk_encoder media_service bk_network_transfer bk_frame_buffer)

armino_component_register(SRCS "${srcs}" INCLUDE_DIRS "${incs}" REQUIRES "${req}" PRIV_REQUIRES "${priv_req}")

add_definitions(-Wno-unused-variable
				-Wno-unused-but-set-variable
				-Wno-int-to-pointer-cast
				-Wno-pointer-to-int-cast)
