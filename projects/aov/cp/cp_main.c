#include "bk_private/bk_init.h"
#include <components/system.h>
#include <os/os.h>
#include <components/shell_task.h>
#include <modules/pm.h>
#include <driver/pwr_clk.h>
#include "bk_api_ipc_test.h"
#include <components/ate.h>
#include "powerctrl.h"
#include "db_ipc_msg.h"
#include "aov_cp_state_machine.h"
#include "aov_cp_device.h"

extern void rtos_set_user_app_entry(beken_thread_function_t entry);


void user_app_main(void) {
    if (!ate_is_enabled())
    {
        db_ipc_msg_init();
        aov_cp_state_machine_init(aov_cp_device_ops_get());
        aov_cp_state_machine_cli_init();
        aov_cp_device_cli_init();
    }
}

int main(void)
{
    rtos_set_user_app_entry((beken_thread_function_t)user_app_main);
    bk_init();

#if (BK_IPC_UT_TEST)
    bk_ipc_test_init();
#endif

    return 0;
}