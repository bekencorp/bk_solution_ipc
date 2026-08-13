#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/stat.h>
#include <common/bk_err.h>
#include <components/log.h>
#include <os/str.h>
#include "aov_ap_state_machine.h"
#include "bk_partition.h"
#include "bk_posix.h"
#include "bk_private/bk_cli.h"
#define TAG "aov_ap_cli"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)
#define AOV_SD_DEMO_FILE PATH_SD_FILE("aov_sd_demo.txt")
#define AOV_SD_READ_CHUNK 64u
#define AOV_SD_MAX_PATH 320u
static bool s_aov_sd_mounted = false;
static int aov_sd_mount(void)
{
    int ret;
    struct bk_fatfs_partition partition = {0};
    if (s_aov_sd_mounted) {
        LOGI("SD card already mounted at %s\r\n", VFS_SD_0_PATITION_0);
        return BK_OK;
    }
    partition.part_type = FATFS_DEVICE;
    partition.part_dev.device_name = FATFS_DEV_SDCARD;
    partition.mount_path = VFS_SD_0_PATITION_0;
    ret = mount("SOURCE_NONE", partition.mount_path, FS_TYPE_FATFS, 0, &partition);
    if (ret == BK_OK) {
        s_aov_sd_mounted = true;
        LOGI("SD card mounted at %s\r\n", partition.mount_path);
    } else {
        LOGE("mount SD card failed, ret=%d\r\n", ret);
    }
    return ret;
}
static int aov_sd_umount(void)
{
    int ret;
    if (!s_aov_sd_mounted) {
        LOGI("SD card is not mounted\r\n");
        return BK_OK;
    }
    ret = umount(VFS_SD_0_PATITION_0);
    if (ret == BK_OK) {
        s_aov_sd_mounted = false;
        LOGI("SD card unmounted from %s\r\n", VFS_SD_0_PATITION_0);
    } else {
        LOGE("umount SD card failed, ret=%d\r\n", ret);
    }
    return ret;
}
static int aov_sd_ensure_mounted(void)
{
    if (s_aov_sd_mounted) {
        return BK_OK;
    }
    return aov_sd_mount();
}
static int aov_sd_open_close_file(const char *file_name)
{
    int fd;
    if (aov_sd_ensure_mounted() != BK_OK) {
        return BK_FAIL;
    }
    fd = open(file_name, O_RDONLY);
    if (fd < 0) {
        LOGE("open %s failed, fd=%d\r\n", file_name, fd);
        return BK_FAIL;
    }
    close(fd);
    LOGI("open/close %s success\r\n", file_name);
    return BK_OK;
}
static int aov_sd_read_file(const char *file_name)
{
    int fd;
    int total = 0;
    char buffer[AOV_SD_READ_CHUNK + 1];
    if (aov_sd_ensure_mounted() != BK_OK) {
        return BK_FAIL;
    }
    fd = open(file_name, O_RDONLY);
    if (fd < 0) {
        LOGE("open %s failed, fd=%d\r\n", file_name, fd);
        return BK_FAIL;
    }
    while (1) {
        int len = read(fd, buffer, AOV_SD_READ_CHUNK);
        if (len < 0) {
            LOGE("read %s failed, ret=%d\r\n", file_name, len);
            close(fd);
            return BK_FAIL;
        }
        if (len == 0) {
            break;
        }
        buffer[len] = '\0';
        LOGI("%s", buffer);
        total += len;
    }
    close(fd);
    LOGI("\r\nread %s done, total=%d bytes\r\n", file_name, total);
    return total;
}
static int aov_sd_write_file(const char *file_name, const char *content)
{
    int fd;
    int ret;
    uint32_t len = os_strlen(content);
    if (aov_sd_ensure_mounted() != BK_OK) {
        return BK_FAIL;
    }
    fd = open(file_name, O_RDWR | O_CREAT | O_APPEND);
    if (fd < 0) {
        LOGE("open %s failed, fd=%d\r\n", file_name, fd);
        return BK_FAIL;
    }
    ret = write(fd, content, len);
    close(fd);
    if (ret < 0) {
        LOGE("write %s failed, ret=%d\r\n", file_name, ret);
        return BK_FAIL;
    }
    LOGI("write %s done, %d/%u bytes\r\n", file_name, ret, len);
    return ret;
}
static int aov_sd_make_dir_if_needed(const char *dir_name)
{
    struct stat dir_stat = {0};
    if (mkdir(dir_name, 0) == BK_OK) {
        return BK_OK;
    }
    if (stat(dir_name, &dir_stat) == BK_OK && S_ISDIR(dir_stat.st_mode)) {
        return BK_OK;
    }
    LOGE("mkdir %s failed\r\n", dir_name);
    return BK_FAIL;
}
static int aov_sd_make_dir(const char *dir_name)
{
    if (aov_sd_ensure_mounted() != BK_OK) {
        return BK_FAIL;
    }
    if (aov_sd_make_dir_if_needed(dir_name) != BK_OK) {
        return BK_FAIL;
    }
    LOGI("mkdir %s success\r\n", dir_name);
    return BK_OK;
}
static int aov_sd_join_path(char *buffer, uint32_t buffer_size,
                            const char *dir_name, const char *entry_name)
{
    uint32_t dir_len = os_strlen(dir_name);
    uint32_t entry_len = os_strlen(entry_name);
    bool has_separator = (dir_len > 0 && dir_name[dir_len - 1] == '/');
    uint32_t need_len = dir_len + (has_separator ? 0 : 1) + entry_len + 1;
    if (need_len > buffer_size) {
        LOGE("path too long: %s/%s\r\n", dir_name, entry_name);
        return BK_FAIL;
    }
    os_snprintf(buffer, buffer_size, "%s%s%s",
                dir_name, has_separator ? "" : "/", entry_name);
    return BK_OK;
}
static int aov_sd_list_dir(const char *dir_name)
{
    int count = 0;
    DIR *dir;
    struct dirent *entry;
    if (aov_sd_ensure_mounted() != BK_OK) {
        return BK_FAIL;
    }
    dir = opendir(dir_name);
    if (dir == NULL) {
        LOGE("opendir %s failed\r\n", dir_name);
        return BK_FAIL;
    }
    LOGI("list %s:\r\n", dir_name);
    while ((entry = readdir(dir)) != NULL) {
        LOGI("  %s\r\n", entry->d_name);
        count++;
    }
    closedir(dir);
    LOGI("list %s done, count=%d\r\n", dir_name, count);
    return count;
}
static int aov_sd_dir_size_recursive(const char *dir_name,
                                     uint64_t *total_size,
                                     uint32_t *file_count,
                                     uint32_t *dir_count)
{
    DIR *dir;
    struct dirent *entry;
    dir = opendir(dir_name);
    if (dir == NULL) {
        LOGE("opendir %s failed\r\n", dir_name);
        return BK_FAIL;
    }
    while ((entry = readdir(dir)) != NULL) {
        char child_path[AOV_SD_MAX_PATH];
        struct stat child_stat = {0};
        if (os_strcmp(entry->d_name, ".") == 0 ||
            os_strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (aov_sd_join_path(child_path, sizeof(child_path),
                             dir_name, entry->d_name) != BK_OK) {
            closedir(dir);
            return BK_FAIL;
        }
        if (stat(child_path, &child_stat) != BK_OK) {
            LOGE("stat %s failed\r\n", child_path);
            closedir(dir);
            return BK_FAIL;
        }
        if (S_ISDIR(child_stat.st_mode)) {
            (*dir_count)++;
            if (aov_sd_dir_size_recursive(child_path, total_size,
                                          file_count, dir_count) != BK_OK) {
                closedir(dir);
                return BK_FAIL;
            }
        } else {
            *total_size += (uint64_t)child_stat.st_size;
            (*file_count)++;
        }
    }
    closedir(dir);
    return BK_OK;
}
static int aov_sd_path_size(const char *path)
{
    struct stat path_stat = {0};
    uint64_t total_size = 0;
    uint32_t file_count = 0;
    uint32_t dir_count = 0;
    if (aov_sd_ensure_mounted() != BK_OK) {
        return BK_FAIL;
    }
    if (stat(path, &path_stat) != BK_OK) {
        LOGE("stat %s failed\r\n", path);
        return BK_FAIL;
    }
    if (!S_ISDIR(path_stat.st_mode)) {
        LOGI("%s file_size=%lu bytes\r\n",
             path, (unsigned long)path_stat.st_size);
        return (int)path_stat.st_size;
    }
    if (aov_sd_dir_size_recursive(path, &total_size,
                                  &file_count, &dir_count) != BK_OK) {
        return BK_FAIL;
    }
    LOGI("%s dir_size=%llu bytes, files=%u, dirs=%u\r\n",
         path,
         (unsigned long long)total_size,
         (unsigned)file_count,
         (unsigned)dir_count);
    return (int)total_size;
}
static int aov_sd_show_info(void)
{
    int ret;
    struct statfs fs_stat = {0};
    uint64_t total_bytes;
    uint64_t free_bytes;
    uint64_t avail_bytes;
    uint64_t used_bytes;
    uint32_t used_percent = 0;
    if (aov_sd_ensure_mounted() != BK_OK) {
        return BK_FAIL;
    }
    ret = statfs(VFS_SD_0_PATITION_0, &fs_stat);
    if (ret != BK_OK) {
        LOGE("statfs %s failed, ret=%d\r\n", VFS_SD_0_PATITION_0, ret);
        return BK_FAIL;
    }
    total_bytes = (uint64_t)fs_stat.f_blocks * fs_stat.f_bsize;
    free_bytes = (uint64_t)fs_stat.f_bfree * fs_stat.f_bsize;
    avail_bytes = (uint64_t)fs_stat.f_bavail * fs_stat.f_bsize;
    used_bytes = (total_bytes >= free_bytes) ? (total_bytes - free_bytes) : 0;
    if (total_bytes > 0) {
        used_percent = (uint32_t)((used_bytes * 100) / total_bytes);
    }
    LOGI("SD card info:\r\n");
    LOGI("  mount_path: %s\r\n", VFS_SD_0_PATITION_0);
    LOGI("  block_size: %lu bytes\r\n", fs_stat.f_bsize);
    LOGI("  blocks: total=%lu free=%lu avail=%lu\r\n",
         fs_stat.f_blocks,
         fs_stat.f_bfree,
         fs_stat.f_bavail);
    LOGI("  total: %llu bytes (%llu MB)\r\n",
         (unsigned long long)total_bytes,
         (unsigned long long)(total_bytes / 1024 / 1024));
    LOGI("  used : %llu bytes (%llu MB, %u%%)\r\n",
         (unsigned long long)used_bytes,
         (unsigned long long)(used_bytes / 1024 / 1024),
         used_percent);
    LOGI("  free : %llu bytes (%llu MB)\r\n",
         (unsigned long long)free_bytes,
         (unsigned long long)(free_bytes / 1024 / 1024));
    LOGI("  avail: %llu bytes (%llu MB)\r\n",
         (unsigned long long)avail_bytes,
         (unsigned long long)(avail_bytes / 1024 / 1024));
    return BK_OK;
}
static bool aov_sd_is_safe_delete_path(const char *path)
{
    const char *component;
    const uint32_t mount_len = os_strlen(VFS_SD_0_PATITION_0);
    bool has_name = false;
    if (path == NULL) {
        return false;
    }
    if (os_strncmp(path, VFS_SD_0_PATITION_0, mount_len) != 0 ||
        path[mount_len] != '/') {
        return false;
    }
    component = path + mount_len + 1;
    while (*component != '\0') {
        const char *next;
        uint32_t component_len;
        while (*component == '/') {
            component++;
        }
        next = component;
        while (*next != '\0' && *next != '/') {
            next++;
        }
        component_len = (uint32_t)(next - component);
        if ((component_len == 1 && component[0] == '.') ||
            (component_len == 2 && component[0] == '.' && component[1] == '.')) {
            return false;
        }
        if (component_len > 0) {
            has_name = true;
        }
        component = next;
    }
    return has_name;
}
static int aov_sd_delete_recursive(const char *path)
{
    struct stat path_stat = {0};
    if (stat(path, &path_stat) != BK_OK) {
        LOGE("stat %s failed\r\n", path);
        return BK_FAIL;
    }
    if (S_ISDIR(path_stat.st_mode)) {
        DIR *dir;
        struct dirent *entry;
        dir = opendir(path);
        if (dir == NULL) {
            LOGE("opendir %s failed\r\n", path);
            return BK_FAIL;
        }
        while ((entry = readdir(dir)) != NULL) {
            char child_path[AOV_SD_MAX_PATH];
            if (os_strcmp(entry->d_name, ".") == 0 ||
                os_strcmp(entry->d_name, "..") == 0) {
                continue;
            }
            if (aov_sd_join_path(child_path, sizeof(child_path),
                                 path, entry->d_name) != BK_OK) {
                closedir(dir);
                return BK_FAIL;
            }
            if (aov_sd_delete_recursive(child_path) != BK_OK) {
                LOGE("remove %s failed\r\n", child_path);
                closedir(dir);
                return BK_FAIL;
            }
        }
        closedir(dir);
        if (rmdir(path) != BK_OK) {
            LOGE("remove dir %s failed\r\n", path);
            return BK_FAIL;
        }
        LOGI("remove dir %s success\r\n", path);
        return BK_OK;
    }
    if (unlink(path) != BK_OK) {
        LOGE("remove file %s failed\r\n", path);
        return BK_FAIL;
    }
    LOGI("remove file %s success\r\n", path);
    return BK_OK;
}
static int aov_sd_remove_path(const char *path)
{
    if (!aov_sd_is_safe_delete_path(path)) {
        LOGE("refuse to delete unsafe path %s, use %s/<name>\r\n",
             path ? path : "(null)",
             VFS_SD_0_PATITION_0);
        return BK_FAIL;
    }
    if (aov_sd_ensure_mounted() != BK_OK) {
        return BK_FAIL;
    }
    if (aov_sd_delete_recursive(path) != BK_OK) {
        return BK_FAIL;
    }
    LOGI("delete %s success\r\n", path);
    return BK_OK;
}
static void aov_sd_show_usage(void)
{
    LOGI("usage:\r\n");
    LOGI("  aov qr provision\r\n");
    LOGI("  aov sd help\r\n");
    LOGI("  aov sd status\r\n");
    LOGI("  aov sd mount\r\n");
    LOGI("  aov sd umount\r\n");
    LOGI("  aov sd df|info                    (show total/used/free SD size)\r\n");
    LOGI("  aov sd ls [path]                  (default: %s)\r\n", VFS_SD_0_PATITION_0);
    LOGI("  aov sd mkdir <path>\r\n");
    LOGI("  aov sd rm|rm_rf <path>            (delete file or dir under %s)\r\n",
         VFS_SD_0_PATITION_0);
    LOGI("  aov sd size <path>                (file size or recursive dir size)\r\n");
    LOGI("  aov sd open_close <path>\r\n");
    LOGI("  aov sd write <path> <content>\r\n");
    LOGI("  aov sd read <path>\r\n");
    LOGI("  aov sd demo                       (write/read %s)\r\n", AOV_SD_DEMO_FILE);
}
static void aov_ap_sd_cli_cmd(int argc, char **argv)
{
    if (argc < 3 || os_strcmp(argv[2], "help") == 0) {
        aov_sd_show_usage();
        return;
    }
    if (os_strcmp(argv[2], "status") == 0) {
        LOGI("mount_path=%s, mounted=%s\r\n",
             VFS_SD_0_PATITION_0,
             s_aov_sd_mounted ? "true" : "false");
    } else if (os_strcmp(argv[2], "mount") == 0) {
        LOGI("mount ret=%d\r\n", aov_sd_mount());
    } else if (os_strcmp(argv[2], "umount") == 0) {
        LOGI("umount ret=%d\r\n", aov_sd_umount());
    } else if (os_strcmp(argv[2], "ls") == 0) {
        const char *dir_name = (argc >= 4) ? argv[3] : VFS_SD_0_PATITION_0;
        LOGI("ls ret=%d\r\n", aov_sd_list_dir(dir_name));
    } else if (os_strcmp(argv[2], "df") == 0 || os_strcmp(argv[2], "info") == 0) {
        LOGI("%s ret=%d\r\n", argv[2], aov_sd_show_info());
    } else if (os_strcmp(argv[2], "mkdir") == 0) {
        if (argc < 4) {
            LOGE("usage: aov sd mkdir <path>\r\n");
            return;
        }
        LOGI("mkdir ret=%d\r\n", aov_sd_make_dir(argv[3]));
    } else if (os_strcmp(argv[2], "size") == 0) {
        if (argc < 4) {
            LOGE("usage: aov sd size <path>\r\n");
            return;
        }
        LOGI("size ret=%d\r\n", aov_sd_path_size(argv[3]));
    } else if (os_strcmp(argv[2], "open_close") == 0) {
        if (argc < 4) {
            LOGE("usage: aov sd open_close <path>\r\n");
            return;
        }
        LOGI("open_close ret=%d\r\n", aov_sd_open_close_file(argv[3]));
    } else if (os_strcmp(argv[2], "write") == 0) {
        if (argc < 5) {
            LOGE("usage: aov sd write <path> <content>\r\n");
            return;
        }
        LOGI("write ret=%d\r\n", aov_sd_write_file(argv[3], argv[4]));
    } else if (os_strcmp(argv[2], "read") == 0) {
        if (argc < 4) {
            LOGE("usage: aov sd read <path>\r\n");
            return;
        }
        LOGI("read ret=%d\r\n", aov_sd_read_file(argv[3]));
    } else if (os_strcmp(argv[2], "rm") == 0 ||
               os_strcmp(argv[2], "rm_rf") == 0) {
        if (argc < 4) {
            LOGE("usage: aov sd %s <path>\r\n", argv[2]);
            return;
        }
        LOGI("%s ret=%d\r\n", argv[2], aov_sd_remove_path(argv[3]));
    } else if (os_strcmp(argv[2], "demo") == 0) {
        const char *content = "aov sd demo\r\n";
        int ret = aov_sd_write_file(AOV_SD_DEMO_FILE, content);
        if (ret >= 0) {
            ret = aov_sd_read_file(AOV_SD_DEMO_FILE);
        }
        LOGI("demo ret=%d\r\n", ret);
    } else {
        aov_sd_show_usage();
    }
}
static void aov_ap_cli_cmd(char *pcWriteBuffer, int xWriteBufferLen,
                           int argc, char **argv)
{
    bk_err_t ret = BK_ERR_PARAM;
    if (argc >= 3 &&
        os_strcmp(argv[1], "qr") == 0 &&
        os_strcmp(argv[2], "provision") == 0) {
        ret = aov_ap_state_machine_test_complete_qr_provision();
        if (ret == BK_OK) {
            LOGI("QR provision mocked; WiFi connected report sent\n");
            if (pcWriteBuffer) {
                os_snprintf(pcWriteBuffer, xWriteBufferLen, "OK\r\n");
            }
            return;
        }
        LOGE("mock QR provision failed: %d\n", ret);
    } else if (argc >= 2 && os_strcmp(argv[1], "sd") == 0) {
        aov_ap_sd_cli_cmd(argc, argv);
        if (pcWriteBuffer) {
            os_snprintf(pcWriteBuffer, xWriteBufferLen, "OK\r\n");
        }
        return;
    }
    if (pcWriteBuffer) {
        os_snprintf(pcWriteBuffer, xWriteBufferLen,
                    "ERROR=%d; usage: aov qr provision | aov sd help\r\n", ret);
    }
}
static const struct cli_command s_aov_ap_cli_commands[] = {
    {"aov",
     "aov qr provision | aov sd help|status|mount|umount|df|info|ls|mkdir|rm|rm_rf|size|open_close|write|read|demo",
     aov_ap_cli_cmd},
};
void aov_ap_cli_init(void)
{
    cli_register_commands(s_aov_ap_cli_commands,
                          sizeof(s_aov_ap_cli_commands) /
                          sizeof(s_aov_ap_cli_commands[0]));
}
