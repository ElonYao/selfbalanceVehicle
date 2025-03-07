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

#include "board.h"

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules. 
// Call this function in your application if you wish to do all module 
// initialization.
// If you wish to not use some of the initializations, instead of the 
// Board_init use the individual Module_inits
//
//*****************************************************************************
void Board_init()
{
	EALLOW;

	PinMux_init();
	SYNC_init();
	CAN_init();
	CPUTIMER_init();
	EPWM_init();
	EQEP_init();
	GPIO_init();
	I2C_init();
	SCI_init();
	INTERRUPT_init();

	EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
	//
	// PinMux for modules assigned to CPU1
	//
	
	//
	// CANB -> mainCAN Pinmux
	//
	GPIO_setPinConfig(mainCAN_CANRX_PIN_CONFIG);
	GPIO_setPadConfig(mainCAN_CANRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(mainCAN_CANRX_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(mainCAN_CANTX_PIN_CONFIG);
	GPIO_setPadConfig(mainCAN_CANTX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(mainCAN_CANTX_GPIO, GPIO_QUAL_ASYNC);

	//
	// EPWM6 -> MotorControl Pinmux
	//
	GPIO_setPinConfig(MotorControl_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(MotorControl_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MotorControl_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(MotorControl_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(MotorControl_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MotorControl_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EQEP1 -> EQEP_motorA Pinmux
	//
	GPIO_setPinConfig(EQEP_motorA_EQEPA_PIN_CONFIG);
	GPIO_setPadConfig(EQEP_motorA_EQEPA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(EQEP_motorA_EQEPA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(EQEP_motorA_EQEPB_PIN_CONFIG);
	GPIO_setPadConfig(EQEP_motorA_EQEPB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(EQEP_motorA_EQEPB_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(EQEP_motorA_EQEPINDEX_PIN_CONFIG);
	GPIO_setPadConfig(EQEP_motorA_EQEPINDEX_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(EQEP_motorA_EQEPINDEX_GPIO, GPIO_QUAL_SYNC);

	//
	// EQEP2 -> EQEP_motorB Pinmux
	//
	GPIO_setPinConfig(EQEP_motorB_EQEPA_PIN_CONFIG);
	GPIO_setPadConfig(EQEP_motorB_EQEPA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(EQEP_motorB_EQEPA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(EQEP_motorB_EQEPB_PIN_CONFIG);
	GPIO_setPadConfig(EQEP_motorB_EQEPB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(EQEP_motorB_EQEPB_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(EQEP_motorB_EQEPINDEX_PIN_CONFIG);
	GPIO_setPadConfig(EQEP_motorB_EQEPINDEX_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(EQEP_motorB_EQEPINDEX_GPIO, GPIO_QUAL_SYNC);

	// GPIO14 -> A1 Pinmux
	GPIO_setPinConfig(GPIO_14_GPIO14);
	// GPIO15 -> A2 Pinmux
	GPIO_setPinConfig(GPIO_15_GPIO15);
	// GPIO27 -> B1 Pinmux
	GPIO_setPinConfig(GPIO_27_GPIO27);
	// GPIO25 -> B2 Pinmux
	GPIO_setPinConfig(GPIO_25_GPIO25);
	//
	// I2CA -> IMU_6050 Pinmux
	//
	GPIO_setPinConfig(IMU_6050_I2CSDA_PIN_CONFIG);
	GPIO_setPadConfig(IMU_6050_I2CSDA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(IMU_6050_I2CSDA_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(IMU_6050_I2CSCL_PIN_CONFIG);
	GPIO_setPadConfig(IMU_6050_I2CSCL_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(IMU_6050_I2CSCL_GPIO, GPIO_QUAL_ASYNC);

	//
	// SCIA -> Data_output Pinmux
	//
	GPIO_setPinConfig(Data_output_SCIRX_PIN_CONFIG);
	GPIO_setPadConfig(Data_output_SCIRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(Data_output_SCIRX_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(Data_output_SCITX_PIN_CONFIG);
	GPIO_setPadConfig(Data_output_SCITX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(Data_output_SCITX_GPIO, GPIO_QUAL_ASYNC);

	//
	// SCIB -> BL_PIC Pinmux
	//
	GPIO_setPinConfig(BL_PIC_SCIRX_PIN_CONFIG);
	GPIO_setPadConfig(BL_PIC_SCIRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(BL_PIC_SCIRX_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(BL_PIC_SCITX_PIN_CONFIG);
	GPIO_setPadConfig(BL_PIC_SCITX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(BL_PIC_SCITX_GPIO, GPIO_QUAL_ASYNC);


}

//*****************************************************************************
//
// CAN Configurations
//
//*****************************************************************************
void CAN_init(){
	mainCAN_init();
}

void mainCAN_init(){
	CAN_initModule(mainCAN_BASE);
	//
	// Refer to the Driver Library User Guide for information on how to set
	// tighter timing control. Additionally, consult the device data sheet
	// for more information about the CAN module clocking.
	//
	CAN_setBitTiming(mainCAN_BASE, 15, 0, 15, 7, 3);
	//
	// Enable CAN Interrupts
	//
	CAN_enableInterrupt(mainCAN_BASE, CAN_INT_ERROR|CAN_INT_IE0|CAN_INT_STATUS);
	CAN_enableGlobalInterrupt(mainCAN_BASE, CAN_GLOBAL_INT_CANINT0);
	//
	// Initialize the transmit message object used for sending CAN messages.
	// Message Object Parameters:
	//      Message Object ID Number: 1
	//      Message Identifier: 201391869
	//      Message Frame: CAN_MSG_FRAME_EXT
	//      Message Type: CAN_MSG_OBJ_TYPE_RX
	//      Message ID Mask: 0
	//      Message Object Flags: CAN_MSG_OBJ_RX_INT_ENABLE,CAN_MSG_OBJ_USE_ID_FILTER
	//      Message Data Length: 0 Bytes
	//
	CAN_setupMessageObject(mainCAN_BASE, 1, mainCAN_MessageObj1_ID, CAN_MSG_FRAME_EXT,CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE|CAN_MSG_OBJ_USE_ID_FILTER,0);
	//
	// Initialize the transmit message object used for sending CAN messages.
	// Message Object Parameters:
	//      Message Object ID Number: 2
	//      Message Identifier: 217060350
	//      Message Frame: CAN_MSG_FRAME_EXT
	//      Message Type: CAN_MSG_OBJ_TYPE_TX
	//      Message ID Mask: 0
	//      Message Object Flags: 
	//      Message Data Length: 8 Bytes
	//
	CAN_setupMessageObject(mainCAN_BASE, 2, mainCAN_MessageObj2_ID, CAN_MSG_FRAME_EXT,CAN_MSG_OBJ_TYPE_TX, 0, 0,8);
	//
	// Initialize the transmit message object used for sending CAN messages.
	// Message Object Parameters:
	//      Message Object ID Number: 3
	//      Message Identifier: 217056510
	//      Message Frame: CAN_MSG_FRAME_EXT
	//      Message Type: CAN_MSG_OBJ_TYPE_TX
	//      Message ID Mask: 0
	//      Message Object Flags: 
	//      Message Data Length: 8 Bytes
	//
	CAN_setupMessageObject(mainCAN_BASE, 3, mainCAN_MessageObj3_ID, CAN_MSG_FRAME_EXT,CAN_MSG_OBJ_TYPE_TX, 0, 0,8);
	CAN_setInterruptMux(mainCAN_BASE, 0);
	//
	// Start CAN module operations
	//
	CAN_startModule(mainCAN_BASE);
}

//*****************************************************************************
//
// CPUTIMER Configurations
//
//*****************************************************************************
void CPUTIMER_init(){
	mainController_init();
}

void mainController_init(){
	CPUTimer_setEmulationMode(mainController_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
	CPUTimer_setPreScaler(mainController_BASE, 0U);
	CPUTimer_setPeriod(mainController_BASE, 1000000U);
	CPUTimer_enableInterrupt(mainController_BASE);
	CPUTimer_stopTimer(mainController_BASE);

	CPUTimer_reloadTimerCounter(mainController_BASE);
	CPUTimer_startTimer(mainController_BASE);
}

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
void EPWM_init(){
    EPWM_setClockPrescaler(MotorControl_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setPeriodLoadMode(MotorControl_BASE, EPWM_PERIOD_DIRECT_LOAD);	
    EPWM_setTimeBasePeriod(MotorControl_BASE, 5000);	
    EPWM_setTimeBaseCounter(MotorControl_BASE, 0);	
    EPWM_setTimeBaseCounterMode(MotorControl_BASE, EPWM_COUNTER_MODE_UP);	
    EPWM_disablePhaseShiftLoad(MotorControl_BASE);	
    EPWM_setPhaseShift(MotorControl_BASE, 0);	
    EPWM_setSyncOutPulseMode(MotorControl_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);	
    EPWM_setCounterCompareValue(MotorControl_BASE, EPWM_COUNTER_COMPARE_A, 10);	
    EPWM_setCounterCompareShadowLoadMode(MotorControl_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MotorControl_BASE, EPWM_COUNTER_COMPARE_B, 10);	
    EPWM_setCounterCompareShadowLoadMode(MotorControl_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(MotorControl_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(MotorControl_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(MotorControl_BASE);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(MotorControl_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(MotorControl_BASE);	
    EPWM_setTripZoneAction(MotorControl_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MotorControl_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_enableTripZoneSignals(MotorControl_BASE, EPWM_TZ_SIGNAL_OSHT5);	
}

//*****************************************************************************
//
// EQEP Configurations
//
//*****************************************************************************
void EQEP_init(){
	EQEP_motorA_init();
	EQEP_motorB_init();
}

void EQEP_motorA_init(){
	//
	// Sets the polarity of the eQEP module's input signals.
	//
	EQEP_setInputPolarity(EQEP_motorA_BASE,false,false,false,false);
	//
	// Configures eQEP module's quadrature decoder unit.
	//
	EQEP_setDecoderConfig(EQEP_motorA_BASE, (EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_1X_RESOLUTION | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE));
	//
	// Set the emulation mode of the eQEP module.
	//
	EQEP_setEmulationMode(EQEP_motorA_BASE,EQEP_EMULATIONMODE_RUNFREE);
	//
	// Configures eQEP module position counter unit.
	//
	EQEP_setPositionCounterConfig(EQEP_motorA_BASE,EQEP_POSITION_RESET_IDX,4294967295U);
	//
	// Sets the current encoder position.
	//
	EQEP_setPosition(EQEP_motorA_BASE,0U);
	//
	// Enables the eQEP module unit timer.
	//
	EQEP_enableUnitTimer(EQEP_motorA_BASE,1000000U);
	//
	// Disables the eQEP module watchdog timer.
	//
	EQEP_disableWatchdog(EQEP_motorA_BASE);
	//
	// Configures the quadrature modes in which the position count can be latched.
	//
	EQEP_setLatchMode(EQEP_motorA_BASE,(EQEP_LATCH_UNIT_TIME_OUT|EQEP_LATCH_RISING_STROBE|EQEP_LATCH_RISING_INDEX));
	//
	// Configures the mode in which the position counter is initialized.
	//
	EQEP_setPositionInitMode(EQEP_motorA_BASE,(EQEP_INIT_DO_NOTHING));
	//
	// Sets the software initialization of the encoder position counter.
	//
	EQEP_setSWPositionInit(EQEP_motorA_BASE,true);
	//
	// Sets the init value for the encoder position counter.
	//
	EQEP_setInitialPosition(EQEP_motorA_BASE,0U);
	//
	// Enables the eQEP module.
	//
	EQEP_enableModule(EQEP_motorA_BASE);
}
void EQEP_motorB_init(){
	//
	// Sets the polarity of the eQEP module's input signals.
	//
	EQEP_setInputPolarity(EQEP_motorB_BASE,false,false,false,false);
	//
	// Configures eQEP module's quadrature decoder unit.
	//
	EQEP_setDecoderConfig(EQEP_motorB_BASE, (EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_1X_RESOLUTION | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE));
	//
	// Set the emulation mode of the eQEP module.
	//
	EQEP_setEmulationMode(EQEP_motorB_BASE,EQEP_EMULATIONMODE_RUNFREE);
	//
	// Configures eQEP module position counter unit.
	//
	EQEP_setPositionCounterConfig(EQEP_motorB_BASE,EQEP_POSITION_RESET_IDX,4294967295U);
	//
	// Sets the current encoder position.
	//
	EQEP_setPosition(EQEP_motorB_BASE,0U);
	//
	// Enables the eQEP module unit timer.
	//
	EQEP_enableUnitTimer(EQEP_motorB_BASE,1000000U);
	//
	// Disables the eQEP module watchdog timer.
	//
	EQEP_disableWatchdog(EQEP_motorB_BASE);
	//
	// Configures the quadrature modes in which the position count can be latched.
	//
	EQEP_setLatchMode(EQEP_motorB_BASE,(EQEP_LATCH_UNIT_TIME_OUT|EQEP_LATCH_RISING_STROBE|EQEP_LATCH_RISING_INDEX));
	//
	// Configures the mode in which the position counter is initialized.
	//
	EQEP_setPositionInitMode(EQEP_motorB_BASE,(EQEP_INIT_DO_NOTHING));
	//
	// Sets the software initialization of the encoder position counter.
	//
	EQEP_setSWPositionInit(EQEP_motorB_BASE,true);
	//
	// Sets the init value for the encoder position counter.
	//
	EQEP_setInitialPosition(EQEP_motorB_BASE,0U);
	//
	// Enables the eQEP module.
	//
	EQEP_enableModule(EQEP_motorB_BASE);
}

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
void GPIO_init(){
	A1_init();
	A2_init();
	B1_init();
	B2_init();
}

void A1_init(){
	GPIO_setPadConfig(A1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(A1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(A1, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(A1, GPIO_CORE_CPU1);
}
void A2_init(){
	GPIO_setPadConfig(A2, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(A2, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(A2, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(A2, GPIO_CORE_CPU1);
}
void B1_init(){
	GPIO_setPadConfig(B1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(B1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(B1, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(B1, GPIO_CORE_CPU1);
}
void B2_init(){
	GPIO_setPadConfig(B2, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(B2, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(B2, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(B2, GPIO_CORE_CPU1);
}

//*****************************************************************************
//
// I2C Configurations
//
//*****************************************************************************
void I2C_init(){
	IMU_6050_init();
}

void IMU_6050_init(){
	I2C_disableModule(IMU_6050_BASE);
	I2C_initController(IMU_6050_BASE, DEVICE_SYSCLK_FREQ, IMU_6050_BITRATE, I2C_DUTYCYCLE_33);
	I2C_setConfig(IMU_6050_BASE, I2C_CONTROLLER_SEND_MODE);
	I2C_disableLoopback(IMU_6050_BASE);
	I2C_setOwnAddress(IMU_6050_BASE, IMU_6050_OWN_ADDRESS);
	I2C_setTargetAddress(IMU_6050_BASE, IMU_6050_TARGET_ADDRESS);
	I2C_setBitCount(IMU_6050_BASE, I2C_BITCOUNT_8);
	I2C_setDataCount(IMU_6050_BASE, 0);
	I2C_setAddressMode(IMU_6050_BASE, I2C_ADDR_MODE_7BITS);
	I2C_disableFIFO(IMU_6050_BASE);
	I2C_setEmulationMode(IMU_6050_BASE, I2C_EMULATION_FREE_RUN);
	I2C_enableModule(IMU_6050_BASE);
}

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************
void INTERRUPT_init(){
	
	// Interrupt Settings for INT_mainController
	// ISR need to be defined for the registered interrupts
	Interrupt_register(INT_mainController, &INT_mainController_ISR);
	Interrupt_enable(INT_mainController);
	
	// Interrupt Settings for INT_BL_PIC_RX
	// ISR need to be defined for the registered interrupts
	Interrupt_register(INT_BL_PIC_RX, &INT_BL_PIC_RX_ISR);
	Interrupt_enable(INT_BL_PIC_RX);
}
//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
void SCI_init(){
	Data_output_init();
	BL_PIC_init();
}

void Data_output_init(){
	SCI_clearInterruptStatus(Data_output_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
	SCI_clearOverflowStatus(Data_output_BASE);
	SCI_resetTxFIFO(Data_output_BASE);
	SCI_resetRxFIFO(Data_output_BASE);
	SCI_resetChannels(Data_output_BASE);
	SCI_setConfig(Data_output_BASE, DEVICE_LSPCLK_FREQ, Data_output_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
	SCI_disableLoopback(Data_output_BASE);
	SCI_performSoftwareReset(Data_output_BASE);
	SCI_enableFIFO(Data_output_BASE);
	SCI_enableModule(Data_output_BASE);
}
void BL_PIC_init(){
	SCI_clearInterruptStatus(BL_PIC_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
	SCI_clearOverflowStatus(BL_PIC_BASE);
	SCI_resetTxFIFO(BL_PIC_BASE);
	SCI_resetRxFIFO(BL_PIC_BASE);
	SCI_resetChannels(BL_PIC_BASE);
	SCI_setConfig(BL_PIC_BASE, DEVICE_LSPCLK_FREQ, BL_PIC_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
	SCI_disableLoopback(BL_PIC_BASE);
	SCI_performSoftwareReset(BL_PIC_BASE);
	SCI_enableInterrupt(BL_PIC_BASE, SCI_INT_RXFF);
	SCI_setFIFOInterruptLevel(BL_PIC_BASE, SCI_FIFO_TX0, SCI_FIFO_RX8);
	SCI_enableFIFO(BL_PIC_BASE);
	SCI_enableModule(BL_PIC_BASE);
}

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************
void SYNC_init(){
	SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);
	//
	// For EPWM1, the sync input is: SYSCTL_SYNC_IN_SRC_EXTSYNCIN1
	//
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM10, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_ECAP1, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_ECAP4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	//
	// SOCA
	//
	SysCtl_enableExtADCSOCSource(0);
	//
	// SOCB
	//
	SysCtl_enableExtADCSOCSource(0);
}
