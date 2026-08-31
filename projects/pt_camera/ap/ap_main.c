#include "bk_private/bk_init.h"
#include <components/system.h>
#include <components/log.h>
#include <os/os.h>
#include <components/shell_task.h>
#include <components/bk_frame_buffer.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <driver/gpio_types.h>
#include "gpio_driver.h"
#include "media_service.h"
#include "app_camera.h"
#include "devices_mgmt.h"
#include "avdk_monitor.h"
#ifdef CONFIG_INTEGRATION_DOORBELL
#include "doorbell_comm.h"
#include "bk_smart_config.h"
#endif
#include "doorbell_ipc_msg.h"
#include "doorbell_keepalive.h"
#if CONFIG_PT_TRACKING
#include "pt_face_detect.h"
#endif

int main(void)
{
    bk_init();
    media_service_init();

    BK_LOGI(NULL, "AP main running...\r\n");

    camera_board_config_t camera_board = {0};

    camera_board.mipi.enable = true;
    camera_board.mipi.pin_scl = GPIO_64;
    camera_board.mipi.pin_sda = GPIO_65;
    camera_board.mipi.i2c_id = 1;
    camera_board.mipi.pin_reset = GPIO_60;
    camera_board.mipi.pin_pwdn = -1;
    camera_board.mipi.pin_xclk = GPIO_59;
    camera_board.mipi.sensor_max_width = 2304;
    camera_board.mipi.sensor_max_height = 1296;
    camera_board.mipi.sensor_fps = 20;
    camera_board.mipi.hmirror = 0;
    camera_board.mipi.vflip = 0;
    camera_board.isp.mp_enable = true;
    camera_board.isp.mp_flexa = true;
    camera_board.isp.mp_width = 2304;
    camera_board.isp.mp_height = 1296;
    camera_board.isp.mp_format = BK_PIXEL_FORMAT_NV12;
    camera_board.isp.sp_enable = false;
    camera_board.isp.sp_flexa = false;
    camera_board.mipi.hmirror = 0;
    camera_board.mipi.vflip = 1;

    bk_frame_buffer_init();

    /* Board config for Multimedia config */
    app_camera_board_config_set(&camera_board);

#if CONFIG_PT_TRACKING
    if (pt_face_detect_init() != BK_OK) {
        BK_LOGE(NULL, "pt face detect init failed\r\n");
    } else if (pt_face_detect_start() != BK_OK) {
        BK_LOGE(NULL, "pt face detect start failed\r\n");
    }
#endif

    /* Debug config for Multimedia */
    avdk_monitor_init();
    avdk_monitor_start();

    devices_mgmt_init();

#if (defined(CONFIG_INTEGRATION_DOORBELL))
    bk_smart_config_init();
    doorbell_core_init();

#if (CONFIG_ASR_SERVICE_WITH_MIC) && (!CONFIG_PT_TRACKING)
    extern int doorbell_asr_turn_on(void);
    doorbell_asr_turn_on();
#endif

#endif

#if CONFIG_VOICE_SERVICE_TEST
    int cli_voice_init(void);
    cli_voice_init();
#endif

    doorbell_ipc_wakeup_env_init();
    doorbell_keepalive_handle_wakeup_reason();
    //doorbell_keepalive_cli_init();

    return 0;
}
