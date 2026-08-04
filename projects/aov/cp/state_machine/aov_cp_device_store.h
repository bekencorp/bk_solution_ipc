#pragma once

#include <common/bk_err.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AOV_DEVICE_STORE_KEY                "aov_device_info"
#define AOV_DEVICE_STORE_MAGIC              (0x49504344u) /* "IPCD" */
#define AOV_DEVICE_STORE_VERSION            (2u)
#define AOV_DEVICE_WIFI_SSID_MAX_LEN        (32u)
#define AOV_DEVICE_WIFI_PASSWORD_MAX_LEN    (64u)

typedef enum {
    AOV_PERSIST_BOOT_UNPROVISIONED = 0,
    AOV_PERSIST_BOOT_PROVISIONED,
    AOV_PERSIST_BOOT_OTA_PENDING,
    AOV_PERSIST_BOOT_FACTORY_RESET_PENDING,
} aov_persist_boot_state_t;

typedef enum {
    AOV_RECORD_MODE_EVENT = 0,
    AOV_RECORD_MODE_CONTINUOUS,
    AOV_RECORD_MODE_AOV,
} aov_record_mode_t;

typedef enum {
    AOV_OTA_STAGE_IDLE = 0,
    AOV_OTA_STAGE_DOWNLOADING,
    AOV_OTA_STAGE_DOWNLOADED,
    AOV_OTA_STAGE_VERIFIED,
    AOV_OTA_STAGE_SWITCH_PENDING,
    AOV_OTA_STAGE_CONFIRM_PENDING,
    AOV_OTA_STAGE_ROLLBACK_PENDING,
} aov_ota_stage_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    uint32_t crc32;

    uint32_t boot_state;       /* aov_persist_boot_state_t */
    uint32_t record_mode;      /* aov_record_mode_t */
    uint32_t ota_stage;        /* aov_ota_stage_t */

    uint8_t credential_present;
    uint8_t credential_valid;
    uint8_t binding_valid;
    uint8_t reset_pending;
    uint8_t ota_pending;
    uint8_t reserved_flags[3];

    uint32_t credential_generation;
    uint32_t binding_generation;
    uint32_t keepalive_interval_ms;
    uint32_t event_record_segment_ms;
    uint32_t cloud_event_segment_ms;
    uint32_t aov_idle_capture_interval_ms;
    uint16_t aov_active_fps;
    uint16_t feature_flags;

    uint32_t active_ota_slot;
    uint32_t target_ota_slot;
    uint32_t ota_retry_count;
    uint8_t wifi_ssid_len;
    uint8_t wifi_password_len;
    uint8_t wifi_reserved[2];
    char wifi_ssid[AOV_DEVICE_WIFI_SSID_MAX_LEN + 1];
    char wifi_password[AOV_DEVICE_WIFI_PASSWORD_MAX_LEN + 1];
    uint8_t reserved[32];
} aov_cp_device_store_t;

void aov_cp_device_store_set_defaults(aov_cp_device_store_t *device);
bool aov_cp_device_store_is_valid(const aov_cp_device_store_t *device);
uint32_t aov_cp_device_store_crc32(const aov_cp_device_store_t *device);

bk_err_t aov_cp_device_store_load(aov_cp_device_store_t *device);
bk_err_t aov_cp_device_store_commit(aov_cp_device_store_t *device);
bk_err_t aov_cp_device_store_reset(aov_cp_device_store_t *device);

#ifdef __cplusplus
}
#endif
