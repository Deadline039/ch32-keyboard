/**
 * Copyright (c) 2026, Deadline039
 *
 * SPDX-License-Identifier: MIT
 */

#include "usb_fsdev_reg.h"
#include <ch32v20x.h>
#include <usbd_core.h>

#define USB ((USB_TypeDef *)g_usbdev_bus[0].reg_base)

void USBWakeUp_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*******************************************************************************
 * @fn        USBWakeUp_IRQHandler
 *
 * @brief     This function handles USB wake up exception.
 *
 * @return    None
 */
void USBWakeUp_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line18);
}

/*******************************************************************************
 * @fn           USB_LP_CAN1_RX0_IRQHandler
 *
 * @brief        This function handles USB exception.
 *
 * @return None
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    void USBD_IRQHandler(uint8_t busid);
    USBD_IRQHandler(g_usbdev_bus[0].busid);
}
/**
 * @brief     Set_USBConfig
 *
 * @return    None
 */
void Set_USBConfig(void)
{
    RCC_ClocksTypeDef RCC_ClocksStatus = { 0 };
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    if (RCC_ClocksStatus.SYSCLK_Frequency == 144000000) {
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div3);
    } else if (RCC_ClocksStatus.SYSCLK_Frequency == 96000000) {
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);
    } else if (RCC_ClocksStatus.SYSCLK_Frequency == 48000000) {
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
    }
#if defined(CH32V20x_D8W) || defined(CH32V20x_D8)
    else if (RCC_ClocksStatus.SYSCLK_Frequency == 240000000 && RCC_USB5PRE_JUDGE() == SET) {
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div5);
    }
#endif
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
 * @fn         USB_Port_Set
 *
 * @brief      Set USB IO port.
 *
 * @param      NewState: DISABLE or ENABLE.
 *             Pin_In_IPU: Enables or Disables intenal pull-up resistance.
 *
 * @return     None
 */
void USB_Port_Set(FunctionalState NewState, FunctionalState Pin_In_IPU)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    if (NewState) {
        CLEAR_BIT(USB->CNTR, USB_CNTR_PDWN);
        GPIOA->CFGHR &= 0XFFF00FFF;
        GPIOA->OUTDR &= ~(3 << 11);
        GPIOA->CFGHR |= 0X00044000;
    } else {
        SET_BIT(USB->CNTR, USB_CNTR_PDWN);
        GPIOA->CFGHR &= 0XFFF00FFF;
        GPIOA->OUTDR &= ~(3 << 11);
        GPIOA->CFGHR |= 0X00033000;
    }

    if (Pin_In_IPU) {
        SET_BIT(EXTEN->EXTEN_CTR, EXTEN_USBD_PU_EN);
    } else {
        CLEAR_BIT(EXTEN->EXTEN_CTR, EXTEN_USBD_PU_EN);
    }
}

/*******************************************************************************
 * @fn         USB_Interrupts_Config
 *
 * @brief      Configrate USB interrupt.
 *
 * @return     None
 */
void USB_Interrupts_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_ClearITPendingBit(EXTI_Line18);
    EXTI_InitStructure.EXTI_Line = EXTI_Line18;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

void usb_dc_low_level_init(void)
{
    Set_USBConfig();

    USB_Port_Set(DISABLE, DISABLE);
    Delay_Ms(20);
    USB_Port_Set(ENABLE, ENABLE);
    USB_Interrupts_Config();
}
