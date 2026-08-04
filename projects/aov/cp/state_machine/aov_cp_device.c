#include "aov_cp_device.h"

#include <components/log.h>
#include <os/mem.h>
#include <os/str.h>

#include "aov_cp_device_store.h"
#include "cli.h"
#include "db_ipc_msg.h"

#define TAG "aov_device"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

_Static_assert(AOV_DEVICE_WIFI_SSID_MAX_LEN == AOV_WIFI_SSID_MAX_LEN,
               "CP store and IPC SSID lengths differ");
_Static_assert(AOV_DEVICE_WIFI_PASSWORD_MAX_LEN ==
                   AOV_WIFI_PASSWORD_MAX_LEN,
               "CP store and IPC password lengths differ");

typedef struct {
    bool loaded;
    aov_cp_device_store_t device;
} aov_cp_device_env_t;

static aov_cp_device_env_t s_device;

static bool aov_device_credentials_valid(void *user_data);

static bool aov_device_store_equal(const aov_cp_device_store_t *lhs,
                                   const aov_cp_device_store_t *rhs)
{
    aov_cp_device_store_t lhs_copy;
    aov_cp_device_store_t rhs_copy;

    os_memcpy(&lhs_copy, lhs, sizeof(lhs_copy));
    os_memcpy(&rhs_copy, rhs, sizeof(rhs_copy));
    lhs_copy.sequence = 0;
    lhs_copy.crc32 = 0;
    rhs_copy.sequence = 0;
    rhs_copy.crc32 = 0;
    return os_memcmp(&lhs_copy, &rhs_copy, sizeof(lhs_copy)) == 0;
}

static int aov_device_state_load(void *user_data,
                                 aov_cp_state_t *restored_state)
{
    aov_cp_device_env_t *env = user_data;
    bk_err_t ret;

    if (env == NULL || restored_state == NULL) {
        return BK_ERR_PARAM;
    }

    ret = aov_cp_device_store_load(&env->device);
    if (ret != BK_OK && ret != BK_ERR_NOT_FOUND) {
        LOGE("load flash state failed: %d\n", ret);
        return ret;
    }
    env->loaded = true;

    if (env->device.reset_pending ||
        env->device.boot_state ==
            AOV_PERSIST_BOOT_FACTORY_RESET_PENDING) {
        *restored_state = AOV_CP_STATE_FACTORY_RESETTING;
    } else if (env->device.ota_pending ||
               env->device.boot_state == AOV_PERSIST_BOOT_OTA_PENDING) {
        *restored_state = AOV_CP_STATE_OTA_UPGRADING;
    } else if (aov_device_credentials_valid(env)) {
        *restored_state = AOV_CP_STATE_WIFI_CONNECTING;
    } else {
        *restored_state = AOV_CP_STATE_UNPROVISIONED;
    }

    LOGI("restore state=%u boot=%u seq=%u credential=%u binding=%u "
         "reset=%u ota=%u\n",
         (unsigned)*restored_state,
         (unsigned)env->device.boot_state,
         (unsigned)env->device.sequence,
         env->device.credential_valid,
         env->device.binding_valid,
         env->device.reset_pending,
         env->device.ota_pending);
    return BK_OK;
}

static bool aov_device_credentials_valid(void *user_data)
{
    aov_cp_device_env_t *env = user_data;
    return env && env->loaded &&
           env->device.credential_present &&
           env->device.credential_valid &&
           env->device.wifi_ssid_len > 0 &&
           env->device.wifi_ssid_len <= AOV_DEVICE_WIFI_SSID_MAX_LEN &&
           env->device.wifi_password_len <= AOV_DEVICE_WIFI_PASSWORD_MAX_LEN;
}

static int aov_device_start_provisioning(void *user_data)
{
    (void)user_data;
    LOGI("provisioning is hosted by AP smart_lock/QR backend\n");
    return BK_OK;
}

static int aov_device_start_wifi_connect(void *user_data)
{
    aov_cp_device_env_t *env = user_data;
    aov_wifi_credentials_t credentials = {
        .magic = AOV_PROTOCOL_MAGIC,
        .version = AOV_PROTOCOL_VERSION,
        .size = sizeof(credentials),
    };

    if (!aov_device_credentials_valid(env)) {
        return BK_ERR_STATE;
    }

    credentials.ssid_len = env->device.wifi_ssid_len;
    credentials.password_len = env->device.wifi_password_len;
    os_memcpy(credentials.ssid, env->device.wifi_ssid,
              credentials.ssid_len + 1);
    os_memcpy(credentials.password, env->device.wifi_password,
              credentials.password_len + 1);

    LOGI("request AP WiFi connect: ssid=%s password_len=%u\n",
         credentials.ssid, credentials.password_len);
    int ret = db_ipc_send_event(AOV_IPC_EVENT_WIFI_CONNECT_REQUEST,
                                (uint8_t *)&credentials,
                                sizeof(credentials));
    os_memset(&credentials, 0, sizeof(credentials));
    return ret;
}

static int aov_device_start_cloud_register(void *user_data)
{
    (void)user_data;
    LOGI("cloud register requested; waiting for device event callback\n");
    return BK_OK;
}

static int aov_device_factory_reset(void *user_data)
{
    aov_cp_device_env_t *env = user_data;
    if (env == NULL) {
        return BK_ERR_PARAM;
    }

    env->device.boot_state = AOV_PERSIST_BOOT_FACTORY_RESET_PENDING;
    env->device.reset_pending = 1;
    bk_err_t ret = aov_cp_device_store_commit(&env->device);
    if (ret != BK_OK) {
        LOGE("persist factory reset pending failed: %d\n", ret);
        return ret;
    }

    LOGW("factory reset pending persisted; AP credential erase not wired yet\n");
    return BK_ERR_NOT_SUPPORT;
}

static int aov_device_state_commit(void *user_data,
                                   aov_cp_state_t state)
{
    aov_cp_device_env_t *env = user_data;
    aov_cp_device_store_t next;
    bool should_commit = false;

    if (env == NULL || !env->loaded) {
        return BK_ERR_STATE;
    }

    os_memcpy(&next, &env->device, sizeof(next));
    switch (state) {
        case AOV_CP_STATE_UNPROVISIONED:
            if (!next.credential_valid || !next.binding_valid) {
                next.boot_state = AOV_PERSIST_BOOT_UNPROVISIONED;
                should_commit = true;
            }
            break;
        case AOV_CP_STATE_KEEPALIVE:
            if (next.credential_valid && next.binding_valid) {
                next.boot_state = AOV_PERSIST_BOOT_PROVISIONED;
                should_commit = true;
            }
            break;
        case AOV_CP_STATE_OTA_UPGRADING:
            next.boot_state = AOV_PERSIST_BOOT_OTA_PENDING;
            next.ota_pending = 1;
            should_commit = true;
            break;
        case AOV_CP_STATE_FACTORY_RESETTING:
            next.boot_state = AOV_PERSIST_BOOT_FACTORY_RESET_PENDING;
            next.reset_pending = 1;
            should_commit = true;
            break;
        default:
            break;
    }

    if (!should_commit ||
        aov_device_store_equal(&next, &env->device)) {
        return BK_OK;
    }

    bk_err_t ret = aov_cp_device_store_commit(&next);
    if (ret == BK_OK) {
        os_memcpy(&env->device, &next, sizeof(env->device));
    }
    return ret;
}

static const aov_cp_device_ops_t s_device_ops = {
    .credentials_valid = aov_device_credentials_valid,
    .start_provisioning = aov_device_start_provisioning,
    .start_wifi_connect = aov_device_start_wifi_connect,
    .start_cloud_register = aov_device_start_cloud_register,
    .factory_reset = aov_device_factory_reset,
    .state_load = aov_device_state_load,
    .state_commit = aov_device_state_commit,
    .user_data = &s_device,
};

const aov_cp_device_ops_t *aov_cp_device_ops_get(void)
{
    return &s_device_ops;
}

bk_err_t aov_cp_device_set_credentials_state(bool present,
                                              bool credential_valid,
                                              bool binding_valid)
{
    if (!s_device.loaded) {
        aov_cp_device_store_set_defaults(&s_device.device);
        s_device.loaded = true;
    }

    s_device.device.credential_present = present;
    s_device.device.credential_valid = credential_valid;
    s_device.device.binding_valid = binding_valid;
    s_device.device.boot_state =
        credential_valid ?
        AOV_PERSIST_BOOT_PROVISIONED :
        AOV_PERSIST_BOOT_UNPROVISIONED;
    s_device.device.credential_generation++;
    return aov_cp_device_store_commit(&s_device.device);
}

bk_err_t aov_cp_device_set_wifi_credentials(const char *ssid,
                                             const char *password)
{
    size_t ssid_len;
    size_t password_len;

    if (ssid == NULL || password == NULL) {
        return BK_ERR_PARAM;
    }

    ssid_len = os_strlen(ssid);
    password_len = os_strlen(password);
    if (ssid_len == 0 || ssid_len > AOV_DEVICE_WIFI_SSID_MAX_LEN ||
        password_len > AOV_DEVICE_WIFI_PASSWORD_MAX_LEN) {
        return BK_ERR_PARAM;
    }

    if (!s_device.loaded) {
        aov_cp_device_store_set_defaults(&s_device.device);
        s_device.loaded = true;
    }

    os_memset(s_device.device.wifi_ssid, 0,
              sizeof(s_device.device.wifi_ssid));
    os_memset(s_device.device.wifi_password, 0,
              sizeof(s_device.device.wifi_password));
    os_memcpy(s_device.device.wifi_ssid, ssid, ssid_len);
    os_memcpy(s_device.device.wifi_password, password, password_len);
    s_device.device.wifi_ssid_len = ssid_len;
    s_device.device.wifi_password_len = password_len;
    s_device.device.credential_present = 1;
    s_device.device.credential_valid = 1;
    s_device.device.boot_state = AOV_PERSIST_BOOT_PROVISIONED;
    s_device.device.credential_generation++;

    return aov_cp_device_store_commit(&s_device.device);
}

bk_err_t aov_cp_device_clear_wifi_credentials(void)
{
    if (!s_device.loaded) {
        aov_cp_device_store_set_defaults(&s_device.device);
        s_device.loaded = true;
    }

    os_memset(s_device.device.wifi_ssid, 0,
              sizeof(s_device.device.wifi_ssid));
    os_memset(s_device.device.wifi_password, 0,
              sizeof(s_device.device.wifi_password));
    s_device.device.wifi_ssid_len = 0;
    s_device.device.wifi_password_len = 0;
    s_device.device.credential_present = 0;
    s_device.device.credential_valid = 0;
    s_device.device.binding_valid = 0;
    s_device.device.boot_state = AOV_PERSIST_BOOT_UNPROVISIONED;
    s_device.device.credential_generation++;

    return aov_cp_device_store_commit(&s_device.device);
}

bk_err_t aov_cp_device_factory_reset_complete(void)
{
    if (!s_device.loaded) {
        aov_cp_device_store_set_defaults(&s_device.device);
        s_device.loaded = true;
    }
    return aov_cp_device_store_reset(&s_device.device);
}

static void aov_device_wifi_cli(char *pc_write_buffer,
                                int write_buffer_len,
                                int argc,
                                char **argv)
{
    bk_err_t ret = BK_ERR_PARAM;

    if (argc >= 2 && os_strcmp(argv[1], "show") == 0) {
        os_snprintf(pc_write_buffer, write_buffer_len,
                    "ssid=%s password_len=%u valid=%u; "
                    "password is hidden\r\n",
                    s_device.device.wifi_ssid,
                    s_device.device.wifi_password_len,
                    s_device.device.credential_valid);
        return;
    }

    if (argc >= 2 && os_strcmp(argv[1], "clear") == 0) {
        ret = aov_cp_device_clear_wifi_credentials();
    } else if (argc >= 4 && os_strcmp(argv[1], "set") == 0) {
        const char *password =
            (os_strcmp(argv[3], "-") == 0) ? "" : argv[3];
        ret = aov_cp_device_set_wifi_credentials(argv[2], password);
    }

    if (ret == BK_OK) {
        os_snprintf(pc_write_buffer, write_buffer_len,
                    "OK; reboot to apply WiFi configuration\r\n");
    } else {
        os_snprintf(pc_write_buffer, write_buffer_len,
                    "ERROR=%d; usage: aov_wifi set <ssid> <password|-> | "
                    "show | clear\r\n", ret);
    }
}

static const struct cli_command s_aov_device_cli[] = {
    {
        "aov_wifi",
        "aov_wifi set <ssid> <password|-> | show | clear",
        aov_device_wifi_cli,
    },
};

int aov_cp_device_cli_init(void)
{
    return cli_register_commands(s_aov_device_cli,
                                 sizeof(s_aov_device_cli) /
                                 sizeof(s_aov_device_cli[0]));
}
