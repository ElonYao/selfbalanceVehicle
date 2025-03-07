/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOARD_H
#define BOARD_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// CANB -> mainCAN Pinmux
//
//
// CANRXB - GPIO Settings
//
#define GPIO_PIN_CANRXB 17
#define mainCAN_CANRX_GPIO 17
#define mainCAN_CANRX_PIN_CONFIG GPIO_17_CANRXB
//
// CANTXB - GPIO Settings
//
#define GPIO_PIN_CANTXB 12
#define mainCAN_CANTX_GPIO 12
#define mainCAN_CANTX_PIN_CONFIG GPIO_12_CANTXB

//
// EPWM6 -> MotorControl Pinmux
//
//
// EPWM6A - GPIO Settings
//
#define GPIO_PIN_EPWM6A 10
#define MotorControl_EPWMA_GPIO 10
#define MotorControl_EPWMA_PIN_CONFIG GPIO_10_EPWM6A
//
// EPWM6B - GPIO Settings
//
#define GPIO_PIN_EPWM6B 11
#define MotorControl_EPWMB_GPIO 11
#define MotorControl_EPWMB_PIN_CONFIG GPIO_11_EPWM6B

//
// EQEP1 -> EQEP_motorA Pinmux
//
//
// EQEP1A - GPIO Settings
//
#define GPIO_PIN_EQEP1A 20
#define EQEP_motorA_EQEPA_GPIO 20
#define EQEP_motorA_EQEPA_PIN_CONFIG GPIO_20_EQEP1A
//
// EQEP1B - GPIO Settings
//
#define GPIO_PIN_EQEP1B 21
#define EQEP_motorA_EQEPB_GPIO 21
#define EQEP_motorA_EQEPB_PIN_CONFIG GPIO_21_EQEP1B
//
// EQEP1I - GPIO Settings
//
#define GPIO_PIN_EQEP1I 99
#define EQEP_motorA_EQEPINDEX_GPIO 99
#define EQEP_motorA_EQEPINDEX_PIN_CONFIG GPIO_99_EQEP1I

//
// EQEP2 -> EQEP_motorB Pinmux
//
//
// EQEP2A - GPIO Settings
//
#define GPIO_PIN_EQEP2A 54
#define EQEP_motorB_EQEPA_GPIO 54
#define EQEP_motorB_EQEPA_PIN_CONFIG GPIO_54_EQEP2A
//
// EQEP2B - GPIO Settings
//
#define GPIO_PIN_EQEP2B 55
#define EQEP_motorB_EQEPB_GPIO 55
#define EQEP_motorB_EQEPB_PIN_CONFIG GPIO_55_EQEP2B
//
// EQEP2I - GPIO Settings
//
#define GPIO_PIN_EQEP2I 57
#define EQEP_motorB_EQEPINDEX_GPIO 57
#define EQEP_motorB_EQEPINDEX_PIN_CONFIG GPIO_57_EQEP2I
//
// GPIO14 - GPIO Settings
//
#define A1_GPIO_PIN_CONFIG GPIO_14_GPIO14
//
// GPIO15 - GPIO Settings
//
#define A2_GPIO_PIN_CONFIG GPIO_15_GPIO15
//
// GPIO27 - GPIO Settings
//
#define B1_GPIO_PIN_CONFIG GPIO_27_GPIO27
//
// GPIO25 - GPIO Settings
//
#define B2_GPIO_PIN_CONFIG GPIO_25_GPIO25

//
// I2CA -> IMU_6050 Pinmux
//
//
// SDAA - GPIO Settings
//
#define GPIO_PIN_SDAA 104
#define IMU_6050_I2CSDA_GPIO 104
#define IMU_6050_I2CSDA_PIN_CONFIG GPIO_104_SDAA
//
// SCLA - GPIO Settings
//
#define GPIO_PIN_SCLA 105
#define IMU_6050_I2CSCL_GPIO 105
#define IMU_6050_I2CSCL_PIN_CONFIG GPIO_105_SCLA

//
// SCIA -> Data_output Pinmux
//
//
// SCIRXDA - GPIO Settings
//
#define GPIO_PIN_SCIRXDA 43
#define Data_output_SCIRX_GPIO 43
#define Data_output_SCIRX_PIN_CONFIG GPIO_43_SCIRXDA
//
// SCITXDA - GPIO Settings
//
#define GPIO_PIN_SCITXDA 42
#define Data_output_SCITX_GPIO 42
#define Data_output_SCITX_PIN_CONFIG GPIO_42_SCITXDA

//
// SCIB -> BL_PIC Pinmux
//
//
// SCIRXDB - GPIO Settings
//
#define GPIO_PIN_SCIRXDB 19
#define BL_PIC_SCIRX_GPIO 19
#define BL_PIC_SCIRX_PIN_CONFIG GPIO_19_SCIRXDB
//
// SCITXDB - GPIO Settings
//
#define GPIO_PIN_SCITXDB 18
#define BL_PIC_SCITX_GPIO 18
#define BL_PIC_SCITX_PIN_CONFIG GPIO_18_SCITXDB

//*****************************************************************************
//
// CAN Configurations
//
//*****************************************************************************
#define mainCAN_BASE CANB_BASE

#define mainCAN_MessageObj1_ID 201391869
#define mainCAN_MessageObj2_ID 217060350
#define mainCAN_MessageObj3_ID 217056510
void mainCAN_init();


//*****************************************************************************
//
// CPUTIMER Configurations
//
//*****************************************************************************
#define mainController_BASE CPUTIMER0_BASE
void mainController_init();

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
#define MotorControl_BASE EPWM6_BASE
#define MotorControl_TBPRD 5000
#define MotorControl_COUNTER_MODE EPWM_COUNTER_MODE_UP
#define MotorControl_TBPHS 0
#define MotorControl_CMPA 10
#define MotorControl_CMPB 10
#define MotorControl_CMPC 0
#define MotorControl_CMPD 0
#define MotorControl_DBRED 0
#define MotorControl_DBFED 0
#define MotorControl_TZA_ACTION EPWM_TZ_ACTION_LOW
#define MotorControl_TZB_ACTION EPWM_TZ_ACTION_LOW
#define MotorControl_OSHT_SOURCES EPWM_TZ_SIGNAL_OSHT5
#define MotorControl_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED

//*****************************************************************************
//
// EQEP Configurations
//
//*****************************************************************************
#define EQEP_motorA_BASE EQEP1_BASE
void EQEP_motorA_init();
#define EQEP_motorB_BASE EQEP2_BASE
void EQEP_motorB_init();

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
#define A1 14
void A1_init();
#define A2 15
void A2_init();
#define B1 27
void B1_init();
#define B2 25
void B2_init();

//*****************************************************************************
//
// I2C Configurations
//
//*****************************************************************************
#define IMU_6050_BASE I2CA_BASE
#define IMU_6050_BITRATE 400000
#define IMU_6050_TARGET_ADDRESS 104
#define IMU_6050_OWN_ADDRESS 0
#define IMU_6050_MODULE_CLOCK_FREQUENCY 10000000
void IMU_6050_init();

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_mainController
// ISR need to be defined for the registered interrupts
#define INT_mainController INT_TIMER0
#define INT_mainController_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1
extern __interrupt void INT_mainController_ISR(void);

// Interrupt Settings for INT_BL_PIC_RX
// ISR need to be defined for the registered interrupts
#define INT_BL_PIC_RX INT_SCIB_RX
#define INT_BL_PIC_RX_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP9
extern __interrupt void INT_BL_PIC_RX_ISR(void);

//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
#define Data_output_BASE SCIA_BASE
#define Data_output_BAUDRATE 115200
#define Data_output_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define Data_output_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define Data_output_CONFIG_PAR SCI_CONFIG_PAR_NONE
void Data_output_init();
#define BL_PIC_BASE SCIB_BASE
#define BL_PIC_BAUDRATE 9600
#define BL_PIC_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define BL_PIC_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define BL_PIC_CONFIG_PAR SCI_CONFIG_PAR_NONE
#define BL_PIC_FIFO_TX_LVL SCI_FIFO_TX0
#define BL_PIC_FIFO_RX_LVL SCI_FIFO_RX8
void BL_PIC_init();

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	Board_init();
void	CAN_init();
void	CPUTIMER_init();
void	EPWM_init();
void	EQEP_init();
void	GPIO_init();
void	I2C_init();
void	INTERRUPT_init();
void	SCI_init();
void	SYNC_init();
void	PinMux_init();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // end of BOARD_H definition
