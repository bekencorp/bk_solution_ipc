set(incs .
    state_machine
    ../common
)

set(srcs
    ap_main.c
    aov_ap_cli.c
    state_machine/aov_ap_state_machine.c
    state_machine/aov_ap_motion.c
    state_machine/aov_ap_snapshot.c
    state_machine/aov_ap_qr_provision.c
    state_machine/aov_ap_wifi.c
)

if (CONFIG_AUD_PARAM_CTRL)
    list(APPEND incs ./audio_param)
    list(APPEND srcs audio_param/audio_para.c)
endif()

set(priv_req
    bk_vfs
    multimedia_device_service
    avdk_utils
    media_service
    smart_lock
    bk_network_transfer
    lwip_intf_v2_1
    bk_wifi_driver
    bk_frame_buffer
    bk_peripheral
    bk_encoder
)
armino_component_register(SRCS "${srcs}" INCLUDE_DIRS "${incs}" PRIV_REQUIRES "${priv_req}")
