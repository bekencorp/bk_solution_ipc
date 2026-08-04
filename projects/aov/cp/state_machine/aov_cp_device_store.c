#include "aov_cp_device_store.h"

#include <components/log.h>
#include <os/mem.h>

#include "bk_ef.h"

#define TAG "aov_device"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

static uint32_t aov_crc32_bytes(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8; bit++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

void aov_cp_device_store_set_defaults(aov_cp_device_store_t *device)
{
    if (device == NULL) {
        return;
    }

    os_memset(device, 0, sizeof(*device));
    device->magic = AOV_DEVICE_STORE_MAGIC;
    device->version = AOV_DEVICE_STORE_VERSION;
    device->size = sizeof(*device);
    device->boot_state = AOV_PERSIST_BOOT_UNPROVISIONED;
    device->record_mode = AOV_RECORD_MODE_AOV;
    device->ota_stage = AOV_OTA_STAGE_IDLE;
    device->keepalive_interval_ms = 30000;
    device->event_record_segment_ms = 30000;
    device->cloud_event_segment_ms = 10000;
    device->aov_idle_capture_interval_ms = 1000;
    device->aov_active_fps = 15;
}

uint32_t aov_cp_device_store_crc32(const aov_cp_device_store_t *device)
{
    aov_cp_device_store_t copy;

    if (device == NULL) {
        return 0;
    }

    os_memcpy(&copy, device, sizeof(copy));
    copy.crc32 = 0;
    return aov_crc32_bytes((const uint8_t *)&copy, sizeof(copy));
}

bool aov_cp_device_store_is_valid(const aov_cp_device_store_t *device)
{
    if (device == NULL ||
        device->magic != AOV_DEVICE_STORE_MAGIC ||
        device->version != AOV_DEVICE_STORE_VERSION ||
        device->size != sizeof(*device)) {
        return false;
    }

    if (device->boot_state > AOV_PERSIST_BOOT_FACTORY_RESET_PENDING ||
        device->record_mode > AOV_RECORD_MODE_AOV ||
        device->ota_stage > AOV_OTA_STAGE_ROLLBACK_PENDING) {
        return false;
    }

    if (device->wifi_ssid_len > AOV_DEVICE_WIFI_SSID_MAX_LEN ||
        device->wifi_password_len > AOV_DEVICE_WIFI_PASSWORD_MAX_LEN ||
        device->wifi_ssid[device->wifi_ssid_len] != '\0' ||
        device->wifi_password[device->wifi_password_len] != '\0') {
        return false;
    }

    if (device->credential_valid && device->wifi_ssid_len == 0) {
        return false;
    }

    return device->crc32 == aov_cp_device_store_crc32(device);
}

bk_err_t aov_cp_device_store_load(aov_cp_device_store_t *device)
{
    int saved_len;

    if (device == NULL) {
        return BK_ERR_PARAM;
    }

    os_memset(device, 0, sizeof(*device));
    saved_len = bk_get_env_enhance(AOV_DEVICE_STORE_KEY,
                                   device,
                                   sizeof(*device));
    if (saved_len != sizeof(*device) ||
        !aov_cp_device_store_is_valid(device)) {
        aov_cp_device_store_set_defaults(device);
        LOGW("no valid EasyFlash device info, use defaults (len=%d)\n",
             saved_len);
        return BK_ERR_NOT_FOUND;
    }

    LOGI("load EasyFlash key=%s sequence=%u boot=%u\n",
         AOV_DEVICE_STORE_KEY,
         (unsigned)device->sequence,
         (unsigned)device->boot_state);
    return BK_OK;
}

bk_err_t aov_cp_device_store_commit(aov_cp_device_store_t *device)
{
    aov_cp_device_store_t verify;
    int saved_len;
    int ret;

    if (device == NULL) {
        return BK_ERR_PARAM;
    }

    device->magic = AOV_DEVICE_STORE_MAGIC;
    device->version = AOV_DEVICE_STORE_VERSION;
    device->size = sizeof(*device);
    device->sequence++;
    device->crc32 = 0;
    device->crc32 = aov_cp_device_store_crc32(device);

    ret = bk_set_env_enhance(AOV_DEVICE_STORE_KEY,
                             device,
                             sizeof(*device));
    if (ret != EF_NO_ERR) {
        LOGE("write EasyFlash key=%s failed: %d\n",
             AOV_DEVICE_STORE_KEY, ret);
        return BK_FAIL;
    }

    os_memset(&verify, 0, sizeof(verify));
    saved_len = bk_get_env_enhance(AOV_DEVICE_STORE_KEY,
                                   &verify,
                                   sizeof(verify));
    if (saved_len != sizeof(verify) ||
        !aov_cp_device_store_is_valid(&verify) ||
        verify.sequence != device->sequence) {
        LOGE("verify EasyFlash key=%s failed len=%d seq=%u/%u\n",
             AOV_DEVICE_STORE_KEY, saved_len,
             (unsigned)verify.sequence,
             (unsigned)device->sequence);
        return BK_FAIL;
    }

    os_memcpy(device, &verify, sizeof(*device));
    LOGI("commit EasyFlash key=%s sequence=%u\n",
         AOV_DEVICE_STORE_KEY, (unsigned)device->sequence);
    return BK_OK;
}

bk_err_t aov_cp_device_store_reset(aov_cp_device_store_t *device)
{
    if (device == NULL) {
        return BK_ERR_PARAM;
    }

    aov_cp_device_store_set_defaults(device);
    return aov_cp_device_store_commit(device);
}
