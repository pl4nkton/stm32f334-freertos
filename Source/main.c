#include "stm32f30x.h"
#include "term_rs232.h"
#include "fault_handler.h"
#include "watchdog.h"
#include "version.h"
#include "board.h"
#include "shell_task.h"
#include "led_task.h"
#include "syscalls.h"
#include "FreeRTOS.h"
#include "task.h"
#include "parameter.h"
//#include "i2c_driver.h"
//#include "sensors.h"
#include <stdio.h>
#include <unistd.h>

#include "matrix3f.h"

static TaskHandle_t wdog_handle;
//static TaskHandle_t sensor_handle;
static TaskHandle_t led_handle;
static TaskHandle_t shell_handle;

/* bekannte probleme
* tim7 timer fuer os timing koennte falsch gehen
* flash_erase in parameter.c ist noch nicht angepasst
* crash bei weiteren tasks
*/

static void init(void)
{
    board_init();
    board_set_leds(LED_GREEN);

    rs232_init();

    printf("\n");
//    print_version_info(0);
    print_fault_info();

//    param_set_defaults();
//    param_load();

//    i2c_init();

    // The watchdog task is created with idle priority to check
    // for CPU starvation as a side effect.
    //
/*
    printf("Starting watchdog task..\n");
    xTaskCreate(watchdog_task, "watchdog", 256, NULL, 0, &wdog_handle);
*/


    printf("Starting LED task..\n");
    xTaskCreate(led_task, "LEDs", 256, NULL, 0, &led_handle);


/*
    printf("Starting sensor task..\n");
    xTaskCreate(sensor_task, "sensors", 1024, NULL, 3, &sensor_handle);
    vTaskDelay(100);
*/

    printf("Starting serial shell task..\n");
    xTaskCreate(shell_task, "uart_shell", 1024, NULL, 0, &shell_handle);

}


int main(void)
{
    // Relocate interrupt vectors
    //
    extern void *g_pfnVectors;
    SCB->VTOR = (uint32_t) &g_pfnVectors;

    // Set up NVIC priorities for FreeRTOS
    //
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // Create init task and start the scheduler
    //
    init();
    vTaskStartScheduler();
}

