#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AOV_PROTOCOL_MAGIC                  (0x414F5653u) /* "AOVS" */
#define AOV_PROTOCOL_VERSION                (2u)
#define AOV_GRAY_WIDTH                      (240u)
#define AOV_GRAY_HEIGHT                     (240u)
#define AOV_GRAY_STRIDE                     (AOV_GRAY_WIDTH)
#define AOV_GRAY_BUFFER_SIZE                (AOV_GRAY_STRIDE * AOV_GRAY_HEIGHT)
#define AOV_FIRST_MOTION_SAMPLE_INTERVAL_MS (1000u)
#define AOV_IPC_PAYLOAD_MAX                 (256u)
#define AOV_IPC_EVENT_MAGIC                 (0xA5A6u)
#define AOV_WIFI_SSID_MAX_LEN               (32u)
#define AOV_WIFI_PASSWORD_MAX_LEN           (64u)

typedef enum {
    AOV_IPC_CMD_GET_SHARED_ENV_ADDR = 0x0100,
    AOV_IPC_CMD_AP_REPORT = 0x0101,
    AOV_IPC_EVENT_WIFI_CONNECT_REQUEST = 0x1100,
} aov_ipc_command_id_t;

typedef enum {
    AOV_CP_STATE_BOOT_INIT = 0,
    AOV_CP_STATE_UNPROVISIONED,
    AOV_CP_STATE_PROVISIONING,
    AOV_CP_STATE_WIFI_CONNECTING,
    AOV_CP_STATE_CLOUD_REGISTERING,
    AOV_CP_STATE_RETRY_WAIT,
    AOV_CP_STATE_KEEPALIVE,
    AOV_CP_STATE_AP_BOOTING,
    AOV_CP_STATE_AP_WORKING,
    AOV_CP_STATE_AP_POWERDOWN,
    AOV_CP_STATE_OTA_UPGRADING,
    AOV_CP_STATE_FACTORY_RESETTING,
    AOV_CP_STATE_MAX,
} aov_cp_state_t;

typedef enum {
    AOV_AP_STATE_OFF = 0,
    AOV_AP_STATE_BOOTING,
    AOV_AP_STATE_READY,
    AOV_AP_STATE_QR_PROVISION_CAPTURE,
    AOV_AP_STATE_SNAPSHOT_CAPTURE,
    AOV_AP_STATE_MOTION_DETECTING,
    AOV_AP_STATE_EVENT_ACTIVE,
    AOV_AP_STATE_LIVE_STREAMING,
    AOV_AP_STATE_STOPPING,
    AOV_AP_STATE_POWERDOWN_READY,
    AOV_AP_STATE_ERROR,
    AOV_AP_STATE_MAX,
} aov_ap_state_t;

typedef enum {
    AOV_AP_JOB_NONE = 0,
    AOV_AP_JOB_NORMAL_BOOT,
    AOV_AP_JOB_QR_PROVISION,
    AOV_AP_JOB_WIFI_CONNECT,
    AOV_AP_JOB_MOTION_CHECK,
    AOV_AP_JOB_LIVE_STREAM,
    AOV_AP_JOB_MAX,
} aov_ap_job_t;

typedef enum {
    AOV_AP_REPORT_BOOT_READY = 0,
    AOV_AP_REPORT_QR_CREDENTIAL,
    AOV_AP_REPORT_CAPTURE_READY,
    AOV_AP_REPORT_NO_MOTION,
    AOV_AP_REPORT_MOTION_DETECTED,
    AOV_AP_REPORT_EVENT_STARTED,
    AOV_AP_REPORT_EVENT_DONE,
    AOV_AP_REPORT_LAST_GRAY_READY,
    AOV_AP_REPORT_LIVE_STARTED,
    AOV_AP_REPORT_LIVE_STOPPED,
    AOV_AP_REPORT_POWERDOWN_READY,
    AOV_AP_REPORT_WIFI_CONNECTED,
    AOV_AP_REPORT_WIFI_CONNECT_FAILED,
    AOV_AP_REPORT_ERROR,
    AOV_AP_REPORT_MAX,
} aov_ap_report_id_t;

typedef enum {
    AOV_CP_EVENT_BOOT = 0,
    AOV_CP_EVENT_PROVISION_START,
    AOV_CP_EVENT_PROVISION_SUCCESS,
    AOV_CP_EVENT_PROVISION_FAIL,
    AOV_CP_EVENT_WIFI_CONNECTED,
    AOV_CP_EVENT_WIFI_CONNECT_FAIL,
    AOV_CP_EVENT_CLOUD_REGISTERED,
    AOV_CP_EVENT_CLOUD_REGISTER_FAIL,
    AOV_CP_EVENT_RETRY_TIMEOUT,
    AOV_CP_EVENT_DETECT_TIMER,
    AOV_CP_EVENT_STATE_TIMEOUT,
    AOV_CP_EVENT_LIVE_START,
    AOV_CP_EVENT_LIVE_STOP,
    AOV_CP_EVENT_AP_REPORT,
    AOV_CP_EVENT_FACTORY_RESET,
    AOV_CP_EVENT_OTA_START,
    AOV_CP_EVENT_OTA_FINISH,
    AOV_CP_EVENT_STOP,
    AOV_CP_EVENT_MAX,
} aov_cp_event_id_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t ssid_len;
    uint8_t password_len;
    uint8_t reserved[2];
    char ssid[AOV_WIFI_SSID_MAX_LEN + 1];
    char password[AOV_WIFI_PASSWORD_MAX_LEN + 1];
} aov_wifi_credentials_t;

typedef struct {
    uint16_t magic;
    uint16_t event_id;
    uint16_t control;
    uint16_t sequence;
    uint16_t checksum;
    uint16_t payload_len;
} aov_ipc_event_header_t;

typedef struct {
    aov_ipc_event_header_t header;
    uint8_t payload[AOV_IPC_PAYLOAD_MAX];
} aov_ipc_event_t;

typedef struct {
    uint32_t frame_id;
    uint32_t timestamp_ms;
    uint32_t buffer_addr;
    uint32_t data_length;
    uint32_t crc32;
    uint16_t width;
    uint16_t height;
    uint16_t stride;
    uint8_t format;
    uint8_t valid;
} aov_gray_frame_desc_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    uint32_t pending_job;
    uint32_t cp_state;
    uint32_t ap_state;
    uint32_t flags;
    aov_gray_frame_desc_t previous_gray;
    uint8_t previous_gray_data[AOV_GRAY_BUFFER_SIZE];
} aov_shared_env_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    uint32_t report_id;
    uint32_t ap_state;
    int32_t result;
    aov_gray_frame_desc_t gray;
} aov_ap_report_t;

typedef struct {
    uint32_t id;
    int32_t result;
    uint32_t arg0;
    uint32_t arg1;
    aov_ap_report_t report;
} aov_cp_event_t;

typedef struct {
    bool (*credentials_valid)(void *user_data);
    int (*start_provisioning)(void *user_data);
    int (*start_wifi_connect)(void *user_data);
    int (*start_cloud_register)(void *user_data);
    int (*factory_reset)(void *user_data);
    int (*state_load)(void *user_data, aov_cp_state_t *restored_state);
    int (*state_commit)(void *user_data, aov_cp_state_t state);
    void *user_data;
} aov_cp_device_ops_t;

typedef struct {
    int (*qr_provision_start)(void *user_data);
    int (*qr_provision_stop)(void *user_data);
    int (*capture_gray)(void *user_data, uint8_t *dst, uint32_t size);
    int (*motion_detect)(void *user_data, const uint8_t *previous, const uint8_t *current,
                         uint16_t width, uint16_t height, bool *motion);
    int (*event_start)(void *user_data);
    int (*event_stop)(void *user_data);
    int (*live_start)(void *user_data);
    int (*live_stop)(void *user_data);
    int (*stop_all)(void *user_data);
    void *user_data;
} aov_ap_backend_ops_t;

_Static_assert(sizeof(aov_ap_report_t) <= AOV_IPC_PAYLOAD_MAX,
               "aov_ap_report_t exceeds customer IPC payload");
_Static_assert(sizeof(aov_wifi_credentials_t) <= AOV_IPC_PAYLOAD_MAX,
               "aov_wifi_credentials_t exceeds customer IPC payload");

#ifdef __cplusplus
}
#endif
