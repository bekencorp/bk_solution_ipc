set(incs . include src)

set(srcs
    ap_main.c
)

if (CONFIG_PT_MP_H264_FRAME_MODE)
    list(APPEND srcs
        src/pt_frame_ndr_bond.c
        src/pt_camera_frame_bringup.c
        src/pt_ir_led.c
        src/pt_ircut.c
        src/pt_day_night.c
    )
endif()

if (CONFIG_PT_TRACKING)
    list(APPEND srcs
        src/pt_face_detect.cc
        src/pt_motor.c
    )
endif()

if (CONFIG_AUD_PARAM_CTRL)
    list(APPEND incs ./audio_param)
    list(APPEND srcs audio_param/audio_para.c)
endif()

set(priv_req
    multimedia_device_service
    avdk_utils
    avdk_driver
    media_service
    smart_lock
    avdk_nn_module
    bk_tflite_micro
    bk_network_transfer
    lwip_intf_v2_1
    bk_wifi_driver
    bk_peripheral
    bk_camera
    bk_encoder
)
armino_component_register(SRCS "${srcs}" INCLUDE_DIRS "${incs}" PRIV_REQUIRES "${priv_req}")
