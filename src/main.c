// Interrupts service routines are in ISR.c
#include <device.h>
#include <driverlib.h>
#include "defines.h"
#include "actions.h"
#include "hardware.h"
#include "error_management.h"

void main()
{

    initialize();
    g_close_interlock_after_next_valid_error_check = true; // if there are no errors

    clearLed0();

    clear_all_errors();

    while (1)
    {
        SysCtl_serviceWatchdog();


        if (g_errorTimerMs > g_Param.error_check_ms){
            error_management();
            if (g_close_interlock_after_next_valid_error_check==1){
                g_close_interlock_after_next_valid_error_check = 0;

                // This is the only place close_interlock() should be called
                closeInterlock();
            }
        }

        manageStatus();
        perform_actions();// most sensor readings and management are done in here

        if(g_Time > 8 && get_error_bit(ERROR_INTERLOCK_IMD_OPEN)) {
            openInterlock();
            saveErrorStatus();
            while(1) {if(g_errorTimerMs > g_Param.error_check_ms) error_management(); manageStatus(); perform_actions(); msg_manage();}
        }


        /* FIXME HACK */
        if(g_Time > 8 && get_error_bit(ERROR_INTERLOCK_AMS_OPEN)) {
            openInterlock();
            saveErrorStatus();
            //while(1) msg_manage();
        }

        // Manage messages
        msg_manage();
    }
}
