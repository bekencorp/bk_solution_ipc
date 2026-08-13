#include "aov_cp_state_machine.h"

#include <common/bk_include.h>
#include <components/log.h>
#include <driver/aon_rtc.h>
#include <driver/aon_rtc_types.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <stdint.h>
#include <stdlib.h>

#include "bk_pm_internal_api.h"
#include "cli.h"
#include "powerctrl.h"

#define TAG "aov_cp_sm"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)
#define LOGD(...) BK_LOGD(TAG, ##__VA_ARGS__)

#define AOV_CP_QUEUE_DEPTH              (16)
#define AOV_CP_TASK_PRIORITY            (4)
#define AOV_CP_TASK_STACK_SIZE          (4096)
#define AOV_CP_DETECT_INTERVAL_MS       (3000u)
#define AOV_CP_RETRY_INTERVAL_MS        (3000)
#define AOV_CP_AP_STATE_TIMEOUT_MS      (5000)
#define AOV_CP_AP_STATE_TIMEOUT_ENABLE  (0)

typedef struct {
    bool initialized;
    bool running;
    bool ap_vote_on;
    bool work_complete;
    bool last_gray_ready;
    bool powerdown_ready;
    bool aborting_job;
    volatile bool rtc_sleep_voted;
    uint32_t retry_count;
    uint32_t active_sequence;
    aov_ap_job_t active_job;
    aov_cp_state_t state;
    aov_cp_state_t return_state;
    beken_queue_t queue;
    beken_thread_t thread;
    alarm_info_t alarm;
    aov_cp_event_id_t timer_event;
    aov_cp_device_ops_t ops;
} aov_cp_sm_env_t;

static aov_cp_sm_env_t s_cp_sm;
static _Alignas(64) aov_shared_env_t s_shared_env;

static bk_err_t aov_cp_arm_timer(uint32_t interval_ms,
                                 aov_cp_event_id_t event_id);

static const char *const s_cp_state_names[AOV_CP_STATE_MAX] = {
    [AOV_CP_STATE_BOOT_INIT] = "BOOT_INIT",
    [AOV_CP_STATE_UNPROVISIONED] = "UNPROVISIONED",
    [AOV_CP_STATE_PROVISIONING] = "PROVISIONING",
    [AOV_CP_STATE_WIFI_CONNECTING] = "WIFI_CONNECTING",
    [AOV_CP_STATE_CLOUD_REGISTERING] = "CLOUD_REGISTERING",
    [AOV_CP_STATE_RETRY_WAIT] = "RETRY_WAIT",
    [AOV_CP_STATE_KEEPALIVE] = "KEEPALIVE",
    [AOV_CP_STATE_AP_BOOTING] = "AP_BOOTING",
    [AOV_CP_STATE_AP_WORKING] = "AP_WORKING",
    [AOV_CP_STATE_AP_POWERDOWN] = "AP_POWERDOWN",
    [AOV_CP_STATE_OTA_UPGRADING] = "OTA_UPGRADING",
    [AOV_CP_STATE_FACTORY_RESETTING] = "FACTORY_RESETTING",
};

const char *aov_cp_state_name(aov_cp_state_t state)
{
    if (state >= AOV_CP_STATE_MAX || s_cp_state_names[state] == NULL) {
        return "UNKNOWN";
    }
    return s_cp_state_names[state];
}

static void aov_cp_set_state(aov_cp_state_t next, aov_cp_event_id_t event)
{
    aov_cp_state_t previous = s_cp_sm.state;

    if (next >= AOV_CP_STATE_MAX) {
        LOGE("invalid next state %u\n", (unsigned)next);
        return;
    }

    s_cp_sm.state = next;
    s_shared_env.cp_state = next;
    LOGI("state %s -> %s, event=%u, seq=%u\n",
         aov_cp_state_name(previous), aov_cp_state_name(next),
         (unsigned)event, (unsigned)s_cp_sm.active_sequence);

    if (s_cp_sm.ops.state_commit) {
        int ret = s_cp_sm.ops.state_commit(s_cp_sm.ops.user_data, next);
        if (ret != BK_OK) {
            LOGW("state commit failed: %d\n", ret);
        }
    }
}

static void aov_cp_timer_callback(aon_rtc_id_t id, uint8_t *name_p, void *param)
{
    aov_cp_event_t event = {0};

    (void)id;
    (void)name_p;
    (void)param;

    if (!s_cp_sm.running) {
        return;
    }

    if (s_cp_sm.timer_event == AOV_CP_EVENT_DETECT_TIMER &&
        s_cp_sm.rtc_sleep_voted) {
        bk_pm_module_vote_sleep_ctrl(PM_SLEEP_MODULE_NAME_APP, 0x0, 0x0);
        s_cp_sm.rtc_sleep_voted = false;
    }

    event.id = s_cp_sm.timer_event;
    if (rtos_push_to_queue(&s_cp_sm.queue, &event, BEKEN_NO_WAIT) != BK_OK) {
        LOGE("timer event queue full, event=%u\n", (unsigned)event.id);
    }
}

static bk_err_t aov_cp_cancel_timer(void)
{
    if (s_cp_sm.rtc_sleep_voted) {
        bk_pm_module_vote_sleep_ctrl(PM_SLEEP_MODULE_NAME_APP, 0x0, 0x0);
        s_cp_sm.rtc_sleep_voted = false;
    }

    if (s_cp_sm.alarm.name[0] == '\0') {
        return BK_OK;
    }

    bk_alarm_unregister(AON_RTC_ID_1, s_cp_sm.alarm.name);
    os_memset(&s_cp_sm.alarm, 0, sizeof(s_cp_sm.alarm));
    return BK_OK;
}

static bk_err_t aov_cp_arm_detect_rtc(void)
{
    bk_err_t ret = aov_cp_arm_timer(AOV_CP_DETECT_INTERVAL_MS,
                                    AOV_CP_EVENT_DETECT_TIMER);
    if (ret != BK_OK) {
        return ret;
    }

    bk_pm_sleep_mode_set(PM_MODE_LOW_VOLTAGE);
    ret = bk_pm_module_vote_sleep_ctrl(PM_SLEEP_MODULE_NAME_APP, 0x1, 0x0);
    if (ret != BK_OK) {
        LOGE("RTC sleep vote failed: %d\n", ret);
        aov_cp_cancel_timer();
        return ret;
    }

    s_cp_sm.rtc_sleep_voted = true;
    LOGI("RTC motion wake armed: %u ms\n", AOV_CP_DETECT_INTERVAL_MS);
    return BK_OK;
}

static bk_err_t aov_cp_arm_timer(uint32_t interval_ms, aov_cp_event_id_t event_id)
{
    alarm_info_t alarm = {0};
    os_strncpy((char *)alarm.name, "aov_sm", sizeof(alarm.name) - 1);
    alarm.period_tick = interval_ms * AON_RTC_MS_TICK_CNT;
    alarm.period_cnt = 1;
    alarm.callback = aov_cp_timer_callback;

    aov_cp_cancel_timer();
    s_cp_sm.timer_event = event_id;
    os_memcpy(&s_cp_sm.alarm, &alarm, sizeof(alarm));

    bk_err_t ret = bk_alarm_register(AON_RTC_ID_1, &s_cp_sm.alarm);
    if (ret != BK_OK) {
        LOGE("arm timer failed: %d, interval=%u, event=%u\n",
             ret, (unsigned)interval_ms, (unsigned)event_id);
        os_memset(&s_cp_sm.alarm, 0, sizeof(s_cp_sm.alarm));
        return ret;
    }

    bk_pm_wakeup_source_set(PM_WAKEUP_SOURCE_INT_RTC, NULL);
    LOGD("timer armed: %u ms, event=%u\n", (unsigned)interval_ms, (unsigned)event_id);
    return BK_OK;
}

static void aov_cp_reset_job_flags(void)
{
    s_cp_sm.work_complete = false;
    s_cp_sm.last_gray_ready = false;
    s_cp_sm.powerdown_ready = false;
    s_cp_sm.aborting_job = false;
}

static bk_err_t aov_cp_start_ap_job(aov_ap_job_t job, aov_cp_state_t return_state)
{
    if (job <= AOV_AP_JOB_NONE || job >= AOV_AP_JOB_MAX) {
        return BK_ERR_PARAM;
    }

    if (s_cp_sm.ap_vote_on) {
        LOGW("AP already voted on, state=%s job=%u\n",
             aov_cp_state_name(s_cp_sm.state), (unsigned)s_cp_sm.active_job);
        return BK_ERR_BUSY;
    }

    aov_cp_cancel_timer();
    aov_cp_reset_job_flags();
    s_cp_sm.active_sequence++;
    s_cp_sm.active_job = job;
    s_cp_sm.return_state = return_state;

    s_shared_env.magic = AOV_PROTOCOL_MAGIC;
    s_shared_env.version = AOV_PROTOCOL_VERSION;
    s_shared_env.size = sizeof(s_shared_env);
    s_shared_env.sequence = s_cp_sm.active_sequence;
    s_shared_env.pending_job = job;
    s_shared_env.ap_state = AOV_AP_STATE_OFF;
    s_shared_env.previous_gray.buffer_addr =
        (uint32_t)(uintptr_t)&s_shared_env.previous_gray_data[0];
    s_shared_env.previous_gray.data_length = AOV_GRAY_BUFFER_SIZE;
    s_shared_env.previous_gray.width = AOV_GRAY_WIDTH;
    s_shared_env.previous_gray.height = AOV_GRAY_HEIGHT;
    s_shared_env.previous_gray.stride = AOV_GRAY_STRIDE;

    aov_cp_set_state(AOV_CP_STATE_AP_BOOTING, AOV_CP_EVENT_DETECT_TIMER);
    s_cp_sm.ap_vote_on = true;
    pl_wakeup_host(POWERUP_MULTIMEDIA_WAKEUP_HOST_FLAG);
    if (AOV_CP_AP_STATE_TIMEOUT_ENABLE) {
        aov_cp_arm_timer(AOV_CP_AP_STATE_TIMEOUT_MS, AOV_CP_EVENT_STATE_TIMEOUT);
    }
    return BK_OK;
}

static bool aov_cp_job_requires_gray(void)
{
    return s_cp_sm.active_job == AOV_AP_JOB_MOTION_CHECK && !s_cp_sm.aborting_job;
}

static void aov_cp_finish_ap_powerdown(aov_cp_event_id_t event_id)
{
    bool rearm_motion_rtc =
        s_cp_sm.active_job == AOV_AP_JOB_MOTION_CHECK;

    aov_cp_cancel_timer();
    if (s_cp_sm.ap_vote_on) {
        pl_power_down_host();
        s_cp_sm.ap_vote_on = false;
    }

    s_shared_env.pending_job = AOV_AP_JOB_NONE;
    s_shared_env.ap_state = AOV_AP_STATE_OFF;
    s_cp_sm.active_job = AOV_AP_JOB_NONE;

    aov_cp_set_state(s_cp_sm.return_state, event_id);
    if (rearm_motion_rtc &&
        (s_cp_sm.return_state == AOV_CP_STATE_KEEPALIVE ||
         s_cp_sm.return_state == AOV_CP_STATE_CLOUD_REGISTERING)) {
        aov_cp_arm_detect_rtc();
    }
}

static void aov_cp_try_powerdown(aov_cp_event_id_t event_id)
{
    if (!s_cp_sm.work_complete || !s_cp_sm.powerdown_ready) {
        return;
    }

    if (aov_cp_job_requires_gray() && !s_cp_sm.last_gray_ready) {
        LOGD("waiting for LAST_GRAY_READY\n");
        return;
    }

    aov_cp_finish_ap_powerdown(event_id);
}

static int aov_cp_call_start_wifi(void)
{
    if (!s_cp_sm.ops.start_wifi_connect) {
        return BK_FAIL;
    }
    return s_cp_sm.ops.start_wifi_connect(s_cp_sm.ops.user_data);
}

static int aov_cp_call_start_cloud(void)
{
    if (!s_cp_sm.ops.start_cloud_register) {
        return BK_FAIL;
    }
    return s_cp_sm.ops.start_cloud_register(s_cp_sm.ops.user_data);
}

static void aov_cp_enter_retry(aov_cp_event_id_t event_id, int result)
{
    s_cp_sm.retry_count++;
    LOGW("retry wait: stage=%s result=%d retry=%u\n",
         aov_cp_state_name(s_cp_sm.state), result, (unsigned)s_cp_sm.retry_count);
    aov_cp_set_state(AOV_CP_STATE_RETRY_WAIT, event_id);
    aov_cp_arm_timer(AOV_CP_RETRY_INTERVAL_MS, AOV_CP_EVENT_RETRY_TIMEOUT);
}

static void aov_cp_handle_ap_report(const aov_ap_report_t *report)
{
    if (report->magic != AOV_PROTOCOL_MAGIC ||
        report->version != AOV_PROTOCOL_VERSION ||
        report->size != sizeof(*report)) {
        LOGE("invalid AP report header\n");
        return;
    }

    if (report->sequence != s_cp_sm.active_sequence) {
        LOGW("stale AP report: report_seq=%u active_seq=%u\n",
             (unsigned)report->sequence, (unsigned)s_cp_sm.active_sequence);
        return;
    }

    s_shared_env.ap_state = report->ap_state;

    switch ((aov_ap_report_id_t)report->report_id) {
        case AOV_AP_REPORT_BOOT_READY:
            if (s_cp_sm.active_job == AOV_AP_JOB_WIFI_CONNECT) {
                aov_cp_set_state(AOV_CP_STATE_WIFI_CONNECTING,
                                 AOV_CP_EVENT_AP_REPORT);
                aov_cp_cancel_timer();
                int ret = aov_cp_call_start_wifi();
                if (ret != BK_OK) {
                    aov_cp_enter_retry(AOV_CP_EVENT_WIFI_CONNECT_FAIL, ret);
                }
            } else {
                aov_cp_set_state(AOV_CP_STATE_AP_WORKING,
                                 AOV_CP_EVENT_AP_REPORT);
            }
            if (s_cp_sm.active_job == AOV_AP_JOB_MOTION_CHECK) {
                if (AOV_CP_AP_STATE_TIMEOUT_ENABLE) {
                    aov_cp_arm_timer(AOV_CP_AP_STATE_TIMEOUT_MS, AOV_CP_EVENT_STATE_TIMEOUT);
                }
            } else if (s_cp_sm.active_job != AOV_AP_JOB_WIFI_CONNECT) {
                aov_cp_cancel_timer();
            }
            break;
        case AOV_AP_REPORT_WIFI_CONNECTED: {
            s_cp_sm.active_job = AOV_AP_JOB_MOTION_CHECK;
            s_cp_sm.return_state = AOV_CP_STATE_CLOUD_REGISTERING;
            s_shared_env.pending_job = AOV_AP_JOB_MOTION_CHECK;
            aov_cp_event_t event = {
                .id = AOV_CP_EVENT_WIFI_CONNECTED,
                .result = report->result,
            };
            aov_cp_state_machine_post_event(&event);
            break;
        }
        case AOV_AP_REPORT_WIFI_CONNECT_FAILED: {
            aov_cp_event_t event = {
                .id = AOV_CP_EVENT_WIFI_CONNECT_FAIL,
                .result = report->result,
            };
            aov_cp_state_machine_post_event(&event);
            break;
        }
        case AOV_AP_REPORT_NO_MOTION:
        case AOV_AP_REPORT_EVENT_DONE:
        case AOV_AP_REPORT_LIVE_STOPPED:
            s_cp_sm.work_complete = true;
            aov_cp_set_state(AOV_CP_STATE_AP_POWERDOWN, AOV_CP_EVENT_AP_REPORT);
            aov_cp_try_powerdown(AOV_CP_EVENT_AP_REPORT);
            break;
        case AOV_AP_REPORT_LAST_GRAY_READY:
            if (report->gray.buffer_addr !=
                    (uint32_t)(uintptr_t)&s_shared_env.previous_gray_data[0] ||
                report->gray.data_length > AOV_GRAY_BUFFER_SIZE ||
                report->gray.width != AOV_GRAY_WIDTH ||
                report->gray.height != AOV_GRAY_HEIGHT) {
                LOGE("invalid gray descriptor\n");
                s_cp_sm.aborting_job = true;
                break;
            }
            s_shared_env.previous_gray = report->gray;
            s_shared_env.previous_gray.valid = 1;
            s_cp_sm.last_gray_ready = true;
            aov_cp_try_powerdown(AOV_CP_EVENT_AP_REPORT);
            break;
        case AOV_AP_REPORT_POWERDOWN_READY:
            s_cp_sm.powerdown_ready = true;
            aov_cp_try_powerdown(AOV_CP_EVENT_AP_REPORT);
            break;
        case AOV_AP_REPORT_MOTION_DETECTED:
            s_cp_sm.aborting_job = true;
            LOGI("motion detected; wait for AP powerdown\n");
            break;
        case AOV_AP_REPORT_EVENT_STARTED:
        case AOV_AP_REPORT_LIVE_STARTED:
        case AOV_AP_REPORT_CAPTURE_READY:
            LOGI("AP progress report=%u result=%d\n",
                 (unsigned)report->report_id, report->result);
            break;
        case AOV_AP_REPORT_ERROR:
            LOGE("AP error: %d\n", report->result);
            s_cp_sm.aborting_job = true;
            s_cp_sm.work_complete = true;
            aov_cp_set_state(AOV_CP_STATE_AP_POWERDOWN, AOV_CP_EVENT_AP_REPORT);
            aov_cp_try_powerdown(AOV_CP_EVENT_AP_REPORT);
            break;
        default:
            LOGW("unknown AP report=%u\n", (unsigned)report->report_id);
            break;
    }
}

static void aov_cp_handle_event(const aov_cp_event_t *event)
{
    switch ((aov_cp_event_id_t)event->id) {
        case AOV_CP_EVENT_BOOT: {
            aov_cp_state_t restored_state = AOV_CP_STATE_UNPROVISIONED;
            int load_ret = BK_ERR_NOT_FOUND;

            if (s_cp_sm.ops.state_load) {
                load_ret = s_cp_sm.ops.state_load(s_cp_sm.ops.user_data,
                                                  &restored_state);
            }
            aov_cp_set_state(AOV_CP_STATE_BOOT_INIT, AOV_CP_EVENT_BOOT);

            if (load_ret != BK_OK && load_ret != BK_ERR_NOT_FOUND) {
                LOGW("state load failed: %d, use unprovisioned\n", load_ret);
                restored_state = AOV_CP_STATE_UNPROVISIONED;
            }

            if (restored_state == AOV_CP_STATE_FACTORY_RESETTING) {
                aov_cp_set_state(AOV_CP_STATE_FACTORY_RESETTING,
                                 AOV_CP_EVENT_BOOT);
                if (s_cp_sm.ops.factory_reset) {
                    s_cp_sm.ops.factory_reset(s_cp_sm.ops.user_data);
                }
                break;
            }

            if (restored_state == AOV_CP_STATE_OTA_UPGRADING) {
                aov_cp_set_state(AOV_CP_STATE_OTA_UPGRADING,
                                 AOV_CP_EVENT_BOOT);
                break;
            }

            bool credentials_valid = s_cp_sm.ops.credentials_valid &&
                s_cp_sm.ops.credentials_valid(s_cp_sm.ops.user_data);
            if (restored_state == AOV_CP_STATE_WIFI_CONNECTING &&
                credentials_valid) {
                aov_cp_set_state(AOV_CP_STATE_WIFI_CONNECTING, AOV_CP_EVENT_BOOT);
                int ret = aov_cp_start_ap_job(AOV_AP_JOB_WIFI_CONNECT,
                                              AOV_CP_STATE_WIFI_CONNECTING);
                if (ret != BK_OK) {
                    aov_cp_enter_retry(AOV_CP_EVENT_WIFI_CONNECT_FAIL, ret);
                }
            } else {
                aov_cp_set_state(AOV_CP_STATE_UNPROVISIONED, AOV_CP_EVENT_BOOT);
                if (s_cp_sm.ops.start_provisioning) {
                    s_cp_sm.ops.start_provisioning(s_cp_sm.ops.user_data);
                }
                aov_cp_start_ap_job(AOV_AP_JOB_QR_PROVISION,
                                    AOV_CP_STATE_UNPROVISIONED);
            }
            break;
        }
        case AOV_CP_EVENT_PROVISION_START:
            aov_cp_set_state(AOV_CP_STATE_PROVISIONING,
                             AOV_CP_EVENT_PROVISION_START);
            break;
        case AOV_CP_EVENT_PROVISION_SUCCESS: {
            aov_cp_set_state(AOV_CP_STATE_WIFI_CONNECTING,
                             AOV_CP_EVENT_PROVISION_SUCCESS);
            int ret;
            if (s_cp_sm.ap_vote_on) {
                ret = aov_cp_call_start_wifi();
            } else {
                ret = aov_cp_start_ap_job(AOV_AP_JOB_WIFI_CONNECT,
                                          AOV_CP_STATE_WIFI_CONNECTING);
            }
            if (ret != BK_OK) {
                aov_cp_enter_retry(AOV_CP_EVENT_WIFI_CONNECT_FAIL, ret);
            }
            break;
        }
        case AOV_CP_EVENT_PROVISION_FAIL:
            aov_cp_set_state(AOV_CP_STATE_UNPROVISIONED,
                             AOV_CP_EVENT_PROVISION_FAIL);
            break;
        case AOV_CP_EVENT_WIFI_CONNECTED: {
            aov_cp_set_state(AOV_CP_STATE_CLOUD_REGISTERING,
                             AOV_CP_EVENT_WIFI_CONNECTED);
            int ret = aov_cp_call_start_cloud();
            if (ret != BK_OK) {
                aov_cp_enter_retry(AOV_CP_EVENT_CLOUD_REGISTER_FAIL, ret);
            }
            break;
        }
        case AOV_CP_EVENT_WIFI_CONNECT_FAIL:
        case AOV_CP_EVENT_CLOUD_REGISTER_FAIL:
            aov_cp_enter_retry((aov_cp_event_id_t)event->id, event->result);
            break;
        case AOV_CP_EVENT_CLOUD_REGISTERED:
            s_cp_sm.retry_count = 0;
            if (s_cp_sm.ap_vote_on) {
                s_cp_sm.return_state = AOV_CP_STATE_KEEPALIVE;
            }
            aov_cp_set_state(AOV_CP_STATE_KEEPALIVE,
                             AOV_CP_EVENT_CLOUD_REGISTERED);
            if (!s_cp_sm.ap_vote_on) {
                aov_cp_arm_detect_rtc();
            }
            break;
        case AOV_CP_EVENT_RETRY_TIMEOUT: {
            aov_cp_set_state(AOV_CP_STATE_WIFI_CONNECTING,
                             AOV_CP_EVENT_RETRY_TIMEOUT);
            int ret;
            if (s_cp_sm.ap_vote_on) {
                ret = aov_cp_call_start_wifi();
            } else {
                ret = aov_cp_start_ap_job(AOV_AP_JOB_WIFI_CONNECT,
                                          AOV_CP_STATE_WIFI_CONNECTING);
            }
            if (ret != BK_OK) {
                aov_cp_enter_retry(AOV_CP_EVENT_WIFI_CONNECT_FAIL, ret);
            }
            break;
        }
        case AOV_CP_EVENT_DETECT_TIMER:
            if (s_cp_sm.state == AOV_CP_STATE_KEEPALIVE ||
                s_cp_sm.state == AOV_CP_STATE_CLOUD_REGISTERING) {
                aov_cp_start_ap_job(AOV_AP_JOB_MOTION_CHECK,
                                    s_cp_sm.state);
            }
            break;
        case AOV_CP_EVENT_LIVE_START:
            if (s_cp_sm.state == AOV_CP_STATE_KEEPALIVE) {
                aov_cp_start_ap_job(AOV_AP_JOB_LIVE_STREAM,
                                    AOV_CP_STATE_KEEPALIVE);
            }
            break;
        case AOV_CP_EVENT_STATE_TIMEOUT:
            LOGE("state timeout in %s\n", aov_cp_state_name(s_cp_sm.state));
            if (s_cp_sm.ap_vote_on) {
                s_cp_sm.aborting_job = true;
                s_cp_sm.work_complete = true;
                s_cp_sm.powerdown_ready = true;
                aov_cp_finish_ap_powerdown(AOV_CP_EVENT_STATE_TIMEOUT);
            }
            break;
        case AOV_CP_EVENT_AP_REPORT:
            aov_cp_handle_ap_report(&event->report);
            break;
        case AOV_CP_EVENT_FACTORY_RESET:
            aov_cp_set_state(AOV_CP_STATE_FACTORY_RESETTING,
                             AOV_CP_EVENT_FACTORY_RESET);
            if (s_cp_sm.ops.factory_reset) {
                s_cp_sm.ops.factory_reset(s_cp_sm.ops.user_data);
            }
            break;
        case AOV_CP_EVENT_OTA_START:
            aov_cp_set_state(AOV_CP_STATE_OTA_UPGRADING,
                             AOV_CP_EVENT_OTA_START);
            break;
        case AOV_CP_EVENT_OTA_FINISH:
            aov_cp_set_state(AOV_CP_STATE_BOOT_INIT,
                             AOV_CP_EVENT_OTA_FINISH);
            break;
        case AOV_CP_EVENT_LIVE_STOP:
            LOGD("live stop is handled by AP report in framework slice\n");
            break;
        case AOV_CP_EVENT_STOP:
            s_cp_sm.running = false;
            break;
        default:
            LOGW("event %u ignored in state %s\n",
                 (unsigned)event->id, aov_cp_state_name(s_cp_sm.state));
            break;
    }
}

static void aov_cp_worker(void *arg)
{
    aov_cp_event_t event;

    (void)arg;
    while (s_cp_sm.running) {
        if (rtos_pop_from_queue(&s_cp_sm.queue, &event,
                                BEKEN_WAIT_FOREVER) != BK_OK) {
            continue;
        }
        aov_cp_handle_event(&event);
    }

    s_cp_sm.thread = NULL;
    rtos_delete_thread(NULL);
}

bk_err_t aov_cp_state_machine_post_event(const aov_cp_event_t *event)
{
    aov_cp_event_t queued_event;

    if (!s_cp_sm.initialized || event == NULL ||
        event->id >= AOV_CP_EVENT_MAX) {
        return BK_ERR_PARAM;
    }

    os_memcpy(&queued_event, event, sizeof(queued_event));
    return rtos_push_to_queue(&s_cp_sm.queue, &queued_event, BEKEN_NO_WAIT);
}

bk_err_t aov_cp_state_machine_on_ap_report(const aov_ap_report_t *report)
{
    aov_cp_event_t event = {0};

    if (report == NULL) {
        return BK_ERR_PARAM;
    }

    event.id = AOV_CP_EVENT_AP_REPORT;
    os_memcpy(&event.report, report, sizeof(event.report));
    return aov_cp_state_machine_post_event(&event);
}

aov_cp_state_t aov_cp_state_machine_get_state(void)
{
    return s_cp_sm.state;
}

aov_shared_env_t *aov_cp_state_machine_get_shared_env(void)
{
    return &s_shared_env;
}

bk_err_t aov_cp_state_machine_init(const aov_cp_device_ops_t *ops)
{
    bk_err_t ret;
    aov_cp_event_t event = {.id = AOV_CP_EVENT_BOOT};

    if (s_cp_sm.initialized) {
        return BK_OK;
    }

    os_memset(&s_cp_sm, 0, sizeof(s_cp_sm));
    os_memset(&s_shared_env, 0, sizeof(s_shared_env));
    s_shared_env.magic = AOV_PROTOCOL_MAGIC;
    s_shared_env.version = AOV_PROTOCOL_VERSION;
    s_shared_env.size = sizeof(s_shared_env);
    s_shared_env.previous_gray.buffer_addr =
        (uint32_t)(uintptr_t)&s_shared_env.previous_gray_data[0];
    s_shared_env.previous_gray.data_length = AOV_GRAY_BUFFER_SIZE;
    s_shared_env.previous_gray.width = AOV_GRAY_WIDTH;
    s_shared_env.previous_gray.height = AOV_GRAY_HEIGHT;
    s_shared_env.previous_gray.stride = AOV_GRAY_STRIDE;
    s_cp_sm.state = AOV_CP_STATE_BOOT_INIT;
    s_cp_sm.return_state = AOV_CP_STATE_KEEPALIVE;
    if (ops) {
        os_memcpy(&s_cp_sm.ops, ops, sizeof(s_cp_sm.ops));
    }

    ret = rtos_init_queue(&s_cp_sm.queue, "aov_cp_sm",
                          sizeof(aov_cp_event_t), AOV_CP_QUEUE_DEPTH);
    if (ret != BK_OK) {
        return ret;
    }

    s_cp_sm.running = true;
    ret = rtos_create_thread(&s_cp_sm.thread, AOV_CP_TASK_PRIORITY,
                             "aov_cp_sm", aov_cp_worker,
                             AOV_CP_TASK_STACK_SIZE, NULL);
    if (ret != BK_OK) {
        s_cp_sm.running = false;
        rtos_deinit_queue(&s_cp_sm.queue);
        return ret;
    }

    s_cp_sm.initialized = true;
    return aov_cp_state_machine_post_event(&event);
}

bk_err_t aov_cp_state_machine_deinit(void)
{
    if (!s_cp_sm.initialized) {
        return BK_OK;
    }

    aov_cp_event_t event = {.id = AOV_CP_EVENT_STOP};
    aov_cp_state_machine_post_event(&event);
    aov_cp_cancel_timer();
    s_cp_sm.initialized = false;
    return BK_OK;
}

static void aov_cp_cli_cmd(char *pcWriteBuffer, int xWriteBufferLen,
                           int argc, char **argv)
{
    aov_cp_event_t event = {0};
    const char *result = "ERROR\r\n";

    (void)xWriteBufferLen;
    if (argc < 2) {
        goto out;
    }

    if (os_strcmp(argv[1], "status") == 0) {
        os_snprintf(pcWriteBuffer, xWriteBufferLen,
                    "state=%s(%u) ap_vote=%u job=%u seq=%u\r\n",
                    aov_cp_state_name(s_cp_sm.state),
                    (unsigned)s_cp_sm.state,
                    (unsigned)s_cp_sm.ap_vote_on,
                    (unsigned)s_cp_sm.active_job,
                    (unsigned)s_cp_sm.active_sequence);
        return;
    }

    if (os_strcmp(argv[1], "gray_info") == 0) {
        const aov_gray_frame_desc_t *gray = &s_shared_env.previous_gray;
        os_snprintf(pcWriteBuffer, xWriteBufferLen,
                    "gray valid=%u id=%u %ux%u stride=%u len=%u crc=0x%x\r\n",
                    gray->valid, (unsigned)gray->frame_id,
                    gray->width, gray->height, gray->stride,
                    (unsigned)gray->data_length, (unsigned)gray->crc32);
        return;
    }

    if (os_strcmp(argv[1], "inject") == 0 && argc >= 3) {
        event.id = (uint32_t)strtoul(argv[2], NULL, 0);
        event.result = (argc >= 4) ? atoi(argv[3]) : BK_OK;
        if (aov_cp_state_machine_post_event(&event) == BK_OK) {
            result = "OK\r\n";
        }
    }

out:
    if (pcWriteBuffer) {
        os_strncpy(pcWriteBuffer, result, xWriteBufferLen);
    }
}

static const struct cli_command s_aov_cp_cli[] = {
    {"aov_sm", "aov_sm status | gray_info | inject <event> [result]",
     aov_cp_cli_cmd},
};

int aov_cp_state_machine_cli_init(void)
{
    return cli_register_commands(s_aov_cp_cli,
                                 sizeof(s_aov_cp_cli) /
                                 sizeof(s_aov_cp_cli[0]));
}
