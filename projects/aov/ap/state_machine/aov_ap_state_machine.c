#include "aov_ap_state_machine.h"

#include <common/bk_include.h>
#include <components/log.h>
#include <components/system.h>
#include <modules/wdrv_common.h>
#include <os/mem.h>
#include <os/os.h>
#include <stdint.h>

#define TAG "aov_ap_sm"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

typedef struct {
    bool initialized;
    bool started;
    uint32_t sequence;
    aov_ap_job_t job;
    aov_ap_state_t state;
    aov_shared_env_t *shared;
    aov_ap_backend_ops_t ops;
} aov_ap_sm_env_t;

static aov_ap_sm_env_t s_ap_sm;
static _Alignas(64) uint8_t s_current_gray[AOV_GRAY_BUFFER_SIZE];

static const char *const s_ap_state_names[AOV_AP_STATE_MAX] = {
    [AOV_AP_STATE_OFF] = "OFF",
    [AOV_AP_STATE_BOOTING] = "BOOTING",
    [AOV_AP_STATE_READY] = "READY",
    [AOV_AP_STATE_QR_PROVISION_CAPTURE] = "QR_PROVISION_CAPTURE",
    [AOV_AP_STATE_SNAPSHOT_CAPTURE] = "SNAPSHOT_CAPTURE",
    [AOV_AP_STATE_MOTION_DETECTING] = "MOTION_DETECTING",
    [AOV_AP_STATE_EVENT_ACTIVE] = "EVENT_ACTIVE",
    [AOV_AP_STATE_LIVE_STREAMING] = "LIVE_STREAMING",
    [AOV_AP_STATE_STOPPING] = "STOPPING",
    [AOV_AP_STATE_POWERDOWN_READY] = "POWERDOWN_READY",
    [AOV_AP_STATE_ERROR] = "ERROR",
};

const char *aov_ap_state_name(aov_ap_state_t state)
{
    if (state >= AOV_AP_STATE_MAX || s_ap_state_names[state] == NULL) {
        return "UNKNOWN";
    }
    return s_ap_state_names[state];
}

static uint32_t aov_gray_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8; bit++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

static void aov_ap_set_state(aov_ap_state_t state)
{
    aov_ap_state_t previous = s_ap_sm.state;

    if (state >= AOV_AP_STATE_MAX) {
        return;
    }

    s_ap_sm.state = state;
    if (s_ap_sm.shared) {
        s_ap_sm.shared->ap_state = state;
    }
    LOGI("AOV state: %s -> %s, job=%u seq=%u\n",
         aov_ap_state_name(previous), aov_ap_state_name(state),
         (unsigned)s_ap_sm.job, (unsigned)s_ap_sm.sequence);
}

static bk_err_t aov_ap_send_report(aov_ap_report_id_t report_id, int result,
                                   const aov_gray_frame_desc_t *gray)
{
    aov_ap_report_t report = {
        .magic = AOV_PROTOCOL_MAGIC,
        .version = AOV_PROTOCOL_VERSION,
        .size = sizeof(aov_ap_report_t),
        .sequence = s_ap_sm.sequence,
        .report_id = report_id,
        .ap_state = s_ap_sm.state,
        .result = result,
    };

    if (gray) {
        os_memcpy(&report.gray, gray, sizeof(report.gray));
    }

    int ret = bk_wdrv_customer_transfer(AOV_IPC_CMD_AP_REPORT,
                                        (uint8_t *)&report,
                                        sizeof(report));
    if (ret != BK_OK) {
        LOGE("send report %u failed: %d\n", (unsigned)report_id, ret);
    }
    return ret;
}

static bk_err_t aov_ap_query_shared_env(void)
{
    uint8_t response[sizeof(uint32_t)] = {0};
    uint16_t response_len = 0;

    int ret = bk_wdrv_customer_transfer_rsp(AOV_IPC_CMD_GET_SHARED_ENV_ADDR,
                                            NULL, 0,
                                            response, sizeof(response),
                                            &response_len);
    if (ret != BK_OK || response_len < sizeof(uint32_t)) {
        LOGW("query shared env failed: ret=%d len=%u\n", ret, response_len);
        return BK_FAIL;
    }

    uint32_t address;
    os_memcpy(&address, response, sizeof(address));
    s_ap_sm.shared = (aov_shared_env_t *)(uintptr_t)address;
    if (s_ap_sm.shared == NULL ||
        s_ap_sm.shared->magic != AOV_PROTOCOL_MAGIC ||
        s_ap_sm.shared->version != AOV_PROTOCOL_VERSION ||
        s_ap_sm.shared->size != sizeof(aov_shared_env_t)) {
        LOGE("invalid shared env: %p\n", s_ap_sm.shared);
        s_ap_sm.shared = NULL;
        return BK_FAIL;
    }

    s_ap_sm.sequence = s_ap_sm.shared->sequence;
    s_ap_sm.job = (aov_ap_job_t)s_ap_sm.shared->pending_job;
    if (s_ap_sm.job >= AOV_AP_JOB_MAX) {
        LOGE("invalid pending job=%u\n", (unsigned)s_ap_sm.job);
        s_ap_sm.job = AOV_AP_JOB_NONE;
        return BK_FAIL;
    }
    return BK_OK;
}

static void aov_ap_copy_current_gray_to_cp(aov_gray_frame_desc_t *desc)
{
    os_memset(desc, 0, sizeof(*desc));
    if (s_ap_sm.shared == NULL) {
        return;
    }

    uint8_t *dst = (uint8_t *)(uintptr_t)
        s_ap_sm.shared->previous_gray.buffer_addr;
    if (dst == NULL ||
        s_ap_sm.shared->previous_gray.data_length < AOV_GRAY_BUFFER_SIZE) {
        LOGE("invalid CP gray buffer\n");
        return;
    }

    os_memcpy(dst, s_current_gray, AOV_GRAY_BUFFER_SIZE);
    desc->frame_id = s_ap_sm.shared->previous_gray.frame_id + 1;
    desc->timestamp_ms = rtos_get_time();
    desc->buffer_addr = (uint32_t)(uintptr_t)dst;
    desc->data_length = AOV_GRAY_BUFFER_SIZE;
    desc->crc32 = aov_gray_crc32(s_current_gray, AOV_GRAY_BUFFER_SIZE);
    desc->width = AOV_GRAY_WIDTH;
    desc->height = AOV_GRAY_HEIGHT;
    desc->stride = AOV_GRAY_STRIDE;
    desc->format = 0;
    desc->valid = 1;
}

static void aov_ap_report_existing_gray_ready(void)
{
    if (s_ap_sm.shared && s_ap_sm.shared->previous_gray.valid) {
        aov_ap_send_report(AOV_AP_REPORT_LAST_GRAY_READY, BK_OK,
                           &s_ap_sm.shared->previous_gray);
    }
}

static void aov_ap_stop_and_report_ready(bool save_gray)
{
    aov_gray_frame_desc_t gray = {0};

    aov_ap_set_state(AOV_AP_STATE_STOPPING);
    if (s_ap_sm.ops.stop_all) {
        s_ap_sm.ops.stop_all(s_ap_sm.ops.user_data);
    }

    if (save_gray) {
        aov_ap_copy_current_gray_to_cp(&gray);
        if (gray.valid) {
            aov_ap_send_report(AOV_AP_REPORT_LAST_GRAY_READY, BK_OK, &gray);
        }
    }

    aov_ap_set_state(AOV_AP_STATE_POWERDOWN_READY);
    aov_ap_send_report(AOV_AP_REPORT_POWERDOWN_READY, BK_OK, NULL);
}

static void aov_ap_fail_and_stop(int result)
{
    aov_ap_set_state(AOV_AP_STATE_ERROR);
    aov_ap_send_report(AOV_AP_REPORT_ERROR, result, NULL);
    aov_ap_stop_and_report_ready(false);
}

static bk_err_t aov_ap_prepare_previous_gray(const uint8_t **previous)
{
    if (previous == NULL || s_ap_sm.shared == NULL) {
        return BK_ERR_STATE;
    }

    if (s_ap_sm.shared->previous_gray.valid) {
        *previous = (const uint8_t *)(uintptr_t)
            s_ap_sm.shared->previous_gray.buffer_addr;
        return (*previous != NULL) ? BK_OK : BK_ERR_STATE;
    }

    uint8_t *first = (uint8_t *)(uintptr_t)
        s_ap_sm.shared->previous_gray.buffer_addr;
    if (first == NULL ||
        s_ap_sm.shared->previous_gray.data_length < AOV_GRAY_BUFFER_SIZE) {
        return BK_ERR_STATE;
    }

    os_memcpy(first, s_current_gray, AOV_GRAY_BUFFER_SIZE);
    LOGI("no previous gray, wait %u ms for second sample\n",
         AOV_FIRST_MOTION_SAMPLE_INTERVAL_MS);
    rtos_delay_milliseconds(AOV_FIRST_MOTION_SAMPLE_INTERVAL_MS);

    int ret = s_ap_sm.ops.capture_gray(
        s_ap_sm.ops.user_data, s_current_gray, sizeof(s_current_gray));
    if (ret != BK_OK) {
        return ret;
    }

    *previous = first;
    return BK_OK;
}

static void aov_ap_run_motion_check(void)
{
    bool motion = false;
    bool had_previous_gray = false;
    int ret;

    aov_ap_set_state(AOV_AP_STATE_MOTION_DETECTING);
    if (!s_ap_sm.ops.capture_gray) {
        aov_ap_fail_and_stop(BK_ERR_NOT_SUPPORT);
        return;
    }
    ret = s_ap_sm.ops.capture_gray(s_ap_sm.ops.user_data,
                                   s_current_gray,
                                   sizeof(s_current_gray));
    if (ret != BK_OK) {
        aov_ap_fail_and_stop(ret);
        return;
    }

    const uint8_t *previous = NULL;
    had_previous_gray = s_ap_sm.shared && s_ap_sm.shared->previous_gray.valid;
    ret = aov_ap_prepare_previous_gray(&previous);
    if (ret != BK_OK) {
        aov_ap_fail_and_stop(ret);
        return;
    }

    if (!s_ap_sm.ops.motion_detect) {
        aov_ap_fail_and_stop(BK_ERR_NOT_SUPPORT);
        return;
    }
    ret = s_ap_sm.ops.motion_detect(s_ap_sm.ops.user_data,
                                    previous, s_current_gray,
                                    AOV_GRAY_WIDTH, AOV_GRAY_HEIGHT,
                                    &motion);
    if (ret != BK_OK) {
        aov_ap_fail_and_stop(ret);
        return;
    }

    aov_ap_set_state(AOV_AP_STATE_SNAPSHOT_CAPTURE);
    #if 0
    if (s_ap_sm.ops.capture_snapshot) {
        ret = s_ap_sm.ops.capture_snapshot(s_ap_sm.ops.user_data);
        if (ret != BK_OK) {
            aov_ap_fail_and_stop(ret);
            return;
        }
    }
    #endif

    if (!motion) {
        aov_ap_send_report(AOV_AP_REPORT_NO_MOTION, BK_OK, NULL);
        if (had_previous_gray) {
            aov_ap_report_existing_gray_ready();
        }
        aov_ap_stop_and_report_ready(!had_previous_gray);
        return;
    }

    aov_ap_send_report(AOV_AP_REPORT_MOTION_DETECTED, BK_OK, NULL);
    LOGI("motion confirmed; snapshot capture done\n");
    aov_ap_send_report(AOV_AP_REPORT_EVENT_DONE, BK_OK, NULL);
    aov_ap_stop_and_report_ready(true);
}

static void aov_ap_run_job(void)
{
    aov_ap_set_state(AOV_AP_STATE_BOOTING);
    aov_ap_send_report(AOV_AP_REPORT_BOOT_READY, BK_OK, NULL);
    aov_ap_set_state(AOV_AP_STATE_READY);

    switch (s_ap_sm.job) {
        case AOV_AP_JOB_QR_PROVISION:
            aov_ap_set_state(AOV_AP_STATE_QR_PROVISION_CAPTURE);
            if (!s_ap_sm.ops.qr_provision_start ||
                s_ap_sm.ops.qr_provision_start(s_ap_sm.ops.user_data) != BK_OK) {
                aov_ap_fail_and_stop(BK_ERR_NOT_SUPPORT);
            }
            break;
        case AOV_AP_JOB_WIFI_CONNECT:
            LOGI("wait for CP WiFi credentials command\n");
            break;
        case AOV_AP_JOB_MOTION_CHECK:
            aov_ap_run_motion_check();
            break;
        case AOV_AP_JOB_LIVE_STREAM:
            aov_ap_set_state(AOV_AP_STATE_LIVE_STREAMING);
            if (!s_ap_sm.ops.live_start ||
                s_ap_sm.ops.live_start(s_ap_sm.ops.user_data) != BK_OK) {
                aov_ap_fail_and_stop(BK_ERR_NOT_SUPPORT);
            } else {
                aov_ap_send_report(AOV_AP_REPORT_LIVE_STARTED, BK_OK, NULL);
            }
            break;
        case AOV_AP_JOB_NORMAL_BOOT:
        case AOV_AP_JOB_NONE:
            LOGI("normal AP boot, state machine remains ready\n");
            break;
        default:
            aov_ap_fail_and_stop(BK_ERR_PARAM);
            break;
    }
}

bk_err_t aov_ap_state_machine_init(const aov_ap_backend_ops_t *ops)
{
    if (s_ap_sm.initialized) {
        return BK_OK;
    }

    os_memset(&s_ap_sm, 0, sizeof(s_ap_sm));
    os_memset(s_current_gray, 0, sizeof(s_current_gray));
    s_ap_sm.state = AOV_AP_STATE_OFF;
    s_ap_sm.job = AOV_AP_JOB_NORMAL_BOOT;
    if (ops) {
        os_memcpy(&s_ap_sm.ops, ops, sizeof(s_ap_sm.ops));
    }

    if (aov_ap_query_shared_env() != BK_OK) {
        LOGW("continue as normal AP boot without AOV shared env\n");
        s_ap_sm.job = AOV_AP_JOB_NORMAL_BOOT;
    }
    s_ap_sm.initialized = true;
    return BK_OK;
}

bk_err_t aov_ap_state_machine_start(void)
{
    if (!s_ap_sm.initialized) {
        return BK_FAIL;
    }
    if (s_ap_sm.started) {
        return BK_OK;
    }
    s_ap_sm.started = true;
    if (s_ap_sm.shared) {
        aov_ap_run_job();
    } else {
        aov_ap_set_state(AOV_AP_STATE_READY);
    }
    return BK_OK;
}

bk_err_t aov_ap_state_machine_deinit(void)
{
    if (!s_ap_sm.initialized) {
        return BK_OK;
    }
    if (s_ap_sm.ops.stop_all) {
        s_ap_sm.ops.stop_all(s_ap_sm.ops.user_data);
    }
    os_memset(&s_ap_sm, 0, sizeof(s_ap_sm));
    s_ap_sm.state = AOV_AP_STATE_OFF;
    return BK_OK;
}

aov_ap_job_t aov_ap_state_machine_get_pending_job(void)
{
    return s_ap_sm.job;
}

aov_ap_state_t aov_ap_state_machine_get_state(void)
{
    return s_ap_sm.state;
}

bk_err_t aov_ap_state_machine_report_qr_credential(const void *data, uint16_t len)
{
    (void)data;
    (void)len;
    if (s_ap_sm.state != AOV_AP_STATE_QR_PROVISION_CAPTURE) {
        return BK_ERR_STATE;
    }
    return aov_ap_send_report(AOV_AP_REPORT_QR_CREDENTIAL, BK_OK, NULL);
}

bk_err_t aov_ap_state_machine_report_wifi_result(bool connected, int result)
{
    if (s_ap_sm.job != AOV_AP_JOB_WIFI_CONNECT) {
        return BK_ERR_STATE;
    }

    return aov_ap_send_report(
        connected ? AOV_AP_REPORT_WIFI_CONNECTED :
                    AOV_AP_REPORT_WIFI_CONNECT_FAILED,
        result,
        NULL);
}

bk_err_t aov_ap_state_machine_test_complete_qr_provision(void)
{
    bk_err_t ret;

    if (!s_ap_sm.initialized || !s_ap_sm.started ||
        (s_ap_sm.job != AOV_AP_JOB_QR_PROVISION &&
         s_ap_sm.job != AOV_AP_JOB_WIFI_CONNECT)) {
        return BK_ERR_STATE;
    }

    if (s_ap_sm.state == AOV_AP_STATE_QR_PROVISION_CAPTURE &&
        s_ap_sm.ops.qr_provision_stop) {
        s_ap_sm.ops.qr_provision_stop(s_ap_sm.ops.user_data);
    }

    s_ap_sm.job = AOV_AP_JOB_WIFI_CONNECT;
    aov_ap_set_state(AOV_AP_STATE_READY);

    ret = aov_ap_state_machine_report_wifi_result(true, BK_OK);
    if (ret != BK_OK) {
        return ret;
    }

    return aov_ap_state_machine_start_motion_check();
}

bk_err_t aov_ap_state_machine_start_motion_check(void)
{
    if (!s_ap_sm.initialized || !s_ap_sm.started ||
        s_ap_sm.job != AOV_AP_JOB_WIFI_CONNECT ||
        s_ap_sm.state != AOV_AP_STATE_READY) {
        return BK_ERR_STATE;
    }

    s_ap_sm.job = AOV_AP_JOB_MOTION_CHECK;
    aov_ap_run_motion_check();
    return BK_OK;
}

bk_err_t aov_ap_state_machine_notify_event_done(int result)
{
    if (s_ap_sm.state != AOV_AP_STATE_EVENT_ACTIVE) {
        return BK_ERR_STATE;
    }
    if (s_ap_sm.ops.event_stop) {
        s_ap_sm.ops.event_stop(s_ap_sm.ops.user_data);
    }
    aov_ap_send_report(AOV_AP_REPORT_EVENT_DONE, result, NULL);
    aov_ap_stop_and_report_ready(true);
    return BK_OK;
}

bk_err_t aov_ap_state_machine_notify_live_stopped(int result)
{
    if (s_ap_sm.state != AOV_AP_STATE_LIVE_STREAMING) {
        return BK_ERR_STATE;
    }
    if (s_ap_sm.ops.live_stop) {
        s_ap_sm.ops.live_stop(s_ap_sm.ops.user_data);
    }
    aov_ap_send_report(AOV_AP_REPORT_LIVE_STOPPED, result, NULL);
    aov_ap_stop_and_report_ready(false);
    return BK_OK;
}
