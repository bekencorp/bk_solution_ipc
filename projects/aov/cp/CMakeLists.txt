set(incs .
         db_ipc_msg
         keepalive
         powerctrl
         db_pack
         state_machine
         ../common
    )

set(srcs
    cp_main.c
    )

list(APPEND srcs
    db_ipc_msg/db_ipc_msg.c
    db_pack/db_pack.c
    keepalive/db_keepalive.c
    powerctrl/powerctrl.c
    state_machine/aov_cp_state_machine.c
    state_machine/aov_cp_device_store.c
    state_machine/aov_cp_device.c
)

if (CONFIG_VND_CAL)
if (CONFIG_OVERRIDE_VND_CAL)
    list(APPEND srcs
        vnd_cal.c
    )
endif()
endif()

set(priv_req controller_if lwip_intf_v2_1 bk_bluetooth)

armino_component_register(SRCS "${srcs}" INCLUDE_DIRS "${incs}" PRIV_REQUIRES "${priv_req}")
