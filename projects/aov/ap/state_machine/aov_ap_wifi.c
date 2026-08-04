#include "aov_ap_wifi.h"

#include <components/event.h>
#include <components/log.h>
#include <components/netif.h>
#include <modules/wifi.h>
#include <modules/wifi_types.h>
#include <os/mem.h>
#include <os/os.h>

#include "aov_ap_state_machine.h"
#include "aov_state_protocol.h"
#include "wdrv_cntrl.h"

#define TAG "aov_ap_wifi"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define AOV_WIFI_TASK_PRIORITY             (5)
#define AOV_WIFI_TASK_STACK_SIZE           (4096)

typedef enum {
    AOV_AP_REQUEST_WIFI_CONNECT = 0,
    AOV_AP_REQUEST_MOTION_CHECK,
} aov_ap_request_id_t;

typedef struct {
    aov_ap_request_id_t id;
    aov_wifi_credentials_t credentials;
} aov_ap_request_t;

static beken_queue_t s_wifi_queue;
static beken_thread_t s_wifi_thread;
static volatile bool s_wifi_connecting;
static bool s_wifi_initialized;

static bk_err_t aov_ap_wifi_report(bool connected, int result)
{
    if (!s_wifi_connecting) {
        return BK_ERR_STATE;
    }

    s_wifi_connecting = false;
    bk_err_t ret =
        aov_ap_state_machine_report_wifi_result(connected, result);
    if (ret != BK_OK) {
        LOGE("report WiFi result failed: %d\n", ret);
    }
    return ret;
}

static int aov_ap_wifi_event_cb(void *arg, event_module_t event_module,
                                int event_id, void *event_data)
{
    (void)arg;
    (void)event_module;

    if (event_id == EVENT_WIFI_STA_DISCONNECTED && s_wifi_connecting) {
        wifi_event_sta_disconnected_t *event = event_data;
        int reason = event ? event->disconnect_reason : BK_FAIL;
        LOGW("WiFi connect failed, reason=%d\n", reason);
        aov_ap_wifi_report(false, reason);
    }

    return BK_OK;
}

static int aov_ap_netif_event_cb(void *arg, event_module_t event_module,
                                 int event_id, void *event_data)
{
    (void)arg;
    (void)event_module;

    if (event_id == EVENT_NETIF_GOT_IP4 && s_wifi_connecting) {
        netif_event_got_ip4_t *event = event_data;
        if (event && event->netif_if == NETIF_IF_STA) {
            LOGI("STA got IPv4 address\n");
            if (aov_ap_wifi_report(true, BK_OK) == BK_OK) {
                aov_ap_request_t request = {
                    .id = AOV_AP_REQUEST_MOTION_CHECK,
                };
                if (rtos_push_to_queue(&s_wifi_queue, &request,
                                       BEKEN_NO_WAIT) != BK_OK) {
                    LOGE("queue motion check failed\n");
                }
            }
        }
    }

    return BK_OK;
}

static void aov_ap_wifi_connect(const aov_wifi_credentials_t *credentials)
{
    wifi_sta_config_t config = {0};

    os_memcpy(config.ssid, credentials->ssid,
              credentials->ssid_len + 1);
    os_memcpy(config.password, credentials->password,
              credentials->password_len + 1);
    s_wifi_connecting = true;

    bk_err_t ret = bk_wifi_sta_set_config(&config);
    if (ret == BK_OK) {
        ret = bk_wifi_sta_start();
    }

    os_memset(&config, 0, sizeof(config));
    if (ret != BK_OK) {
        LOGE("start WiFi connect failed: %d\n", ret);
        aov_ap_wifi_report(false, ret);
    }
}

static void aov_ap_wifi_worker(beken_thread_arg_t arg)
{
    aov_ap_request_t request;

    (void)arg;
    while (true) {
        if (rtos_pop_from_queue(&s_wifi_queue, &request,
                                BEKEN_WAIT_FOREVER) != BK_OK) {
            continue;
        }

        if (request.id == AOV_AP_REQUEST_WIFI_CONNECT) {
            aov_ap_wifi_connect(&request.credentials);
        } else if (request.id == AOV_AP_REQUEST_MOTION_CHECK) {
            bk_err_t ret = aov_ap_state_machine_start_motion_check();
            if (ret != BK_OK) {
                LOGE("start motion check failed: %d\n", ret);
            }
        }
        os_memset(&request, 0, sizeof(request));
    }
}

static bool aov_ap_wifi_credentials_valid(
    const aov_wifi_credentials_t *credentials)
{
    return credentials &&
           credentials->magic == AOV_PROTOCOL_MAGIC &&
           credentials->version == AOV_PROTOCOL_VERSION &&
           credentials->size == sizeof(*credentials) &&
           credentials->ssid_len > 0 &&
           credentials->ssid_len <= AOV_WIFI_SSID_MAX_LEN &&
           credentials->password_len <= AOV_WIFI_PASSWORD_MAX_LEN &&
           credentials->ssid[credentials->ssid_len] == '\0' &&
           credentials->password[credentials->password_len] == '\0';
}

static void aov_ap_customer_event_handler(void *data, uint16_t len)
{
    aov_ipc_event_t *event = data;
    aov_ap_request_t request = {
        .id = AOV_AP_REQUEST_WIFI_CONNECT,
    };

    if (event == NULL || len < sizeof(aov_ipc_event_header_t) ||
        event->header.magic != AOV_IPC_EVENT_MAGIC ||
        event->header.event_id != AOV_IPC_EVENT_WIFI_CONNECT_REQUEST ||
        event->header.payload_len != sizeof(request.credentials) ||
        len < sizeof(aov_ipc_event_header_t) +
              sizeof(request.credentials)) {
        return;
    }

    os_memcpy(&request.credentials, event->payload,
              sizeof(request.credentials));
    if (!aov_ap_wifi_credentials_valid(&request.credentials)) {
        LOGE("invalid WiFi credentials command\n");
        return;
    }

    LOGI("receive WiFi connect request: ssid=%s password_len=%u\n",
         request.credentials.ssid, request.credentials.password_len);
    if (rtos_push_to_queue(&s_wifi_queue, &request,
                           BEKEN_NO_WAIT) != BK_OK) {
        LOGE("WiFi request queue is full\n");
        aov_ap_state_machine_report_wifi_result(false, BK_ERR_BUSY);
    }
    os_memset(&request, 0, sizeof(request));
}

bk_err_t aov_ap_wifi_init(void)
{
    bk_err_t ret;

    if (s_wifi_initialized) {
        return BK_OK;
    }

    ret = rtos_init_queue(&s_wifi_queue, "aov_wifi",
                          sizeof(aov_ap_request_t), 2);
    if (ret != BK_OK) {
        return ret;
    }

    ret = rtos_create_thread(&s_wifi_thread, AOV_WIFI_TASK_PRIORITY,
                             "aov_wifi", aov_ap_wifi_worker,
                             AOV_WIFI_TASK_STACK_SIZE, NULL);
    if (ret != BK_OK) {
        rtos_deinit_queue(&s_wifi_queue);
        return ret;
    }

    ret = bk_event_register_cb(EVENT_MOD_WIFI, EVENT_ID_ALL,
                               aov_ap_wifi_event_cb, NULL);
    if (ret != BK_OK) {
        return ret;
    }

    ret = bk_event_register_cb(EVENT_MOD_NETIF, EVENT_ID_ALL,
                               aov_ap_netif_event_cb, NULL);
    if (ret != BK_OK) {
        return ret;
    }

    bk_customer_event_register_callback(aov_ap_customer_event_handler);
    s_wifi_initialized = true;
    return BK_OK;
}
