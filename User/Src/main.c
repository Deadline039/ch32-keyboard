/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include <ch32v20x.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "debug.h"

#include <usb_hid_kbd.h>

/* Global typedef */

/* Global define */

/* Global Variable */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    // vTaskStartScheduler();

    printf("Running!\r\n");
    __enable_irq();
    usb_hid_kbd_init(0, 0x40005C00L);

    // Wait until configured
    while (!usb_device_is_configured(0)) {
    }

    static uint32_t wait_ct = 100000;
    // Everything is interrupt driven so just loop here
    while (1) {
        usb_hid_kbd_test(0);
        wait_ct = 400000;
        while (wait_ct--) {
        }
    }
}
