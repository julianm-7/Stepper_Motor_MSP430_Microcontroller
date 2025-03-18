/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
#include <stdint.h>
#include <stdio.h>
#include "LcdDrivermsp430/Crystalfontz128x128_ST7735.h"
#include "LcdDrivermsp430/HAL_MSP_EXP430FR5994_Crystalfontz128x128_ST7735.h"
#include "HAL_FR5994_OPT3001.h"
#include "HAL_FR5994_I2CLIB.h"
#include "grlib.h"
#include "driverlib.h"
#define CMASK BIT5
#define definePeriod 301 //301 original value
#define OPTADDRESS 0x44
#define highLimit 30874
#define lowLimit 1095
#pragma diag_suppress=169
void configurePort6();
void configurePort4();
void initTimerA0();
void TIMER_A_configureUpMode();
void hardwareConf();
void myMotorDriver();
void myMotorController();
void FR5994_I2C_init();
void OPT3001_configReg();
void OPT3001_writeHighLimitReg();
void OPT3001_writeLowLimitReg();
void PatternModeFunction();
void PatternModeCheck();
void initializeALL();
typedef enum {OFF,LOW,HIGH,PATTERN,CW,CCW,ON,LightSensorForward,LightSensorBackward,PushButtonForward,PushButtonBackward,StartUp} ProgramMode ;
Graphics_Context g_sContext;
volatile ProgramMode SpeedMode,MotorMode,TextMode,TrackerMode,PatternModeSpeed;
volatile int32_t globalStepCount;
volatile uint16_t value,value2;
volatile uint16_t period;
volatile uint16_t configData;
volatile uint8_t PBS1,PBS2;
volatile uint16_t value3;
volatile int8_t motorSeq;
volatile ProgramMode patternTracker[40];
volatile ProgramMode patternSequence[]= {LOW,OFF,LOW,OFF,LOW,OFF,LOW,OFF};
volatile uint16_t pSteps,nSteps;
volatile int32_t arrayTracker=0;
volatile char buffer[100];
int ii,j,xx,yy;
uint8_t x=0;

void main()
{
    initializeALL();
    while(1)
    {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN1);
        __delay_cycles(30000);
        PatternModeCheck();
    }
}

#pragma vector= TIMER0_A0_VECTOR
__interrupt void timerISR()
{
    uint16_t lightReading;
    if (TrackerMode==TextMode)
    {}
    else
    {
        if (TrackerMode==PATTERN)
        {
            Graphics_clearDisplay(&g_sContext);
        }
        switch (TextMode)
        {
        case PushButtonForward:
            sprintf(buffer," Push Button ");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
            sprintf(buffer,"CW at High Speed");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
            break;
        case PushButtonBackward:
            sprintf(buffer," Push Button ");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
            sprintf(buffer,"CCW at Low Speed");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
            break;
        case LightSensorForward:
            sprintf(buffer,"Light Sensor");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
            sprintf(buffer,"CW at High Speed");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
            break;
        case LightSensorBackward:
            sprintf(buffer,"Light Sensor");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
            sprintf(buffer,"CCW at Low Speed");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
            break;
        case PATTERN:
            pSteps=0;
            nSteps=0;
            break;
            // LCD display will print within PatternModeFunction();
        }
            TrackerMode=TextMode;
    }
    Timer_A_stop (TIMER_A0_BASE);
    int i=0;
    if (x==1)
    {
        for (i=0;i<9;i++)
        {
            SpeedMode=PATTERN;
            myMotorController();
        }
    }
    else
    {
    myMotorController();
    }
    lightReading= I2C_read16(RESULT_REG);
    while ((lightReading<lowLimit) || (lightReading>highLimit))
    {
        TA0CCTL0= (TA0CCTL0 & (~BIT0));
        initTimerA0();
        while ((TA0CCTL0 & BIT0) != BIT0){}
        myMotorController();
        lightReading= I2C_read16(RESULT_REG);
    }
    TA0CCTL0= (TA0CCTL0 & (~BIT0));
}

#pragma vector = PORT6_VECTOR
__interrupt void lightSensorISR()
{
    GPIO_clearInterrupt(GPIO_PORT_P6,GPIO_PIN3);
    initTimerA0();
    configData=OPT3001_readConfigReg(OPTADDRESS);
    if ((configData&CMASK) == BIT5)
    {
        SpeedMode=LOW;
        TextMode=LightSensorBackward;
    }
    else
    {
        SpeedMode=HIGH;
        TextMode=LightSensorForward;
    }
    switch (SpeedMode)
    {
    case LOW:
        period= definePeriod;
        break;
    case HIGH:
        period= definePeriod/2;
        break;
    }
    while ((TA0CCTL0 & BIT0) != BIT0)
    {}
}

#pragma vector = PORT5_VECTOR
__interrupt void pushButtonISR()
{
    uint8_t tracker=0;
    PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
    PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
    if ((PBS2&PBS1)==0)
    {
        initTimerA0();
        tracker=1;
    }
    PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
    PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
    if ((PBS1==0) && (PBS2==0))
    {
        SpeedMode=PATTERN;
        TextMode=PATTERN;
    }
    else if (PBS1==0)
    {
        SpeedMode=LOW;
        TextMode=PushButtonBackward;
    }
    else if (PBS2==0)
    {
        SpeedMode=HIGH;
        TextMode=PushButtonForward;
    }
    switch (SpeedMode)
    {
    case LOW:
        period= definePeriod;
        break;
    case HIGH:
        period= definePeriod/2;
        break;
    }

    PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
    PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
    if (PBS1!=0)
    {
    GPIO_clearInterrupt(GPIO_PORT_P5,GPIO_PIN6);
    }
    if (PBS2!=0)
    {
    GPIO_clearInterrupt(GPIO_PORT_P5,GPIO_PIN5);
    }
    PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
    PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
    if (tracker)
    {
        while ((TA0CCTL0 & BIT0) != BIT0)
        {}
    }

}

void myMotorController()
{
    switch (SpeedMode)
    {
    case LOW:
        MotorMode=CCW;
        break;
    case HIGH:
        MotorMode=CW;
        break;
    case OFF:
        MotorMode=OFF;
        break;
    case PATTERN:
        MotorMode=PATTERN;
        break;
    }
    switch (MotorMode)
    {
    case OFF:
        myMotorDriver();
        break;
    case CW:
        if (motorSeq>7)
        {
            motorSeq=0;
        }
        myMotorDriver();
        motorSeq++;
        break;
    case CCW:
        if (motorSeq<0)
        {
            motorSeq=7;
        }
        myMotorDriver();
        motorSeq--;
        break;
    case PATTERN:
        PatternModeFunction();
        break;
    }
}
void myMotorDriver()
{
    if (MotorMode==CW)
        globalStepCount++;
    if (MotorMode==CCW)
        globalStepCount--;
    switch(motorSeq)
    {
    case 0:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
        // A RED LP
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;

    case 1:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
        // A RED LP
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 2:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 3:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 4:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 5:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 6:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 7:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
        // A RED LP
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    default:
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    }
}

void configurePort6()
{
    GPIO_setAsInputPinWithPullDownResistor (GPIO_PORT_P6,GPIO_PIN3);
    GPIO_enableInterrupt (GPIO_PORT_P6,GPIO_PIN3);
    GPIO_selectInterruptEdge(GPIO_PORT_P6,GPIO_PIN3, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P6,GPIO_PIN3);
}

void configurePort4()
{
    GPIO_selectInterruptEdge(GPIO_PORT_P4,GPIO_PIN3, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(GPIO_PORT_P4,GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(GPIO_PORT_P5,GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(GPIO_PORT_P5,GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P5,GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P5,GPIO_PIN6);
    GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN3);
    GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN2);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN3);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN2);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5,GPIO_PIN6);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5,GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN3);
    GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P5,GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P5,GPIO_PIN5);
}

void PatternModeCheck()
{
    yy=0;
    if (SpeedMode!=OFF)
    {
        arrayTracker++;
        patternTracker[arrayTracker]=SpeedMode;
        arrayTracker++;
    }
    SpeedMode=OFF;
    patternTracker[arrayTracker]=SpeedMode;
    j=7;
    for (ii=arrayTracker;ii>=(arrayTracker-7);ii--)
    {
        if (patternTracker[ii]!=patternSequence[j])
        {
            break;
        }
        else
            j--;
        if (ii==(arrayTracker-7))
        {
            SpeedMode=PATTERN;
            TextMode=PATTERN;
            x=1;
            initTimerA0();
            while (((TA0CCTL0 & BIT0) != BIT0) && (!yy))
            {}
        }
    }
}
void initTimerA0()
{
    TIMER_A_configureUpMode();
    Timer_A_clearTimerInterrupt(TIMER_A0_BASE);
    Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
}

void TIMER_A_configureUpMode()
{
    Timer_A_initUpModeParam MyTimerA0;
    MyTimerA0.clockSourceDivider=TIMER_A_CLOCKSOURCE_DIVIDER_20;
    MyTimerA0.clockSource=TIMER_A_CLOCKSOURCE_SMCLK;
    MyTimerA0.timerPeriod= period; // This count will complete in .01319 seconds if its equal to definePeriod
    MyTimerA0.timerInterruptEnable_TAIE=TIMER_A_TAIE_INTERRUPT_DISABLE;
    MyTimerA0.captureCompareInterruptEnable_CCR0_CCIE=TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    MyTimerA0.timerClear=TIMER_A_DO_CLEAR;
    MyTimerA0.startTimer= false;
    Timer_A_initUpMode(TIMER_A0_BASE, &MyTimerA0);
}


void hardwareConf()
{
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN3);
    GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN2);
}

void FR5994_I2C_init()
{
    EUSCI_B_I2C_initMasterParam i2cConfig =
       {
               EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
               10000000,
               EUSCI_B_I2C_SET_DATA_RATE_400KBPS,
       };
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7,GPIO_PIN1,GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7,GPIO_PIN0,GPIO_PRIMARY_MODULE_FUNCTION);

    EUSCI_B_I2C_disable(EUSCI_B2_BASE);
    EUSCI_B_I2C_initMaster(EUSCI_B2_BASE, &i2cConfig);
    EUSCI_B_I2C_enable(EUSCI_B2_BASE);
}

void OPT3001_configReg()
{
    value3 = 50904;
    I2C_setAddress(OPTADDRESS);
    I2C_write16(CONFIG_REG,value3);
}
void OPT3001_writeHighLimitReg()
{
    I2C_setAddress(OPTADDRESS);
    I2C_write16(HIGHLIMIT_REG,highLimit);
}

void OPT3001_writeLowLimitReg()
{
    I2C_setAddress(OPTADDRESS);
    I2C_write16(LOWLIMIT_REG,lowLimit);
}

void initializeALL()
{
    WDT_A_hold(WDT_A_BASE);
    PMM_unlockLPM5();
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(0);
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    hardwareConf();
    FR5994_I2C_init();
    OPT3001_init(OPTADDRESS);
    OPT3001_configReg();
    OPT3001_writeHighLimitReg();
    OPT3001_writeLowLimitReg();
    configurePort4();
    configurePort6();
    period=definePeriod;
    TrackerMode=OFF;
    globalStepCount=0;
    __enable_interrupt();
}

void PatternModeFunction()
{
//    globalStepCount+=4000;
    uint16_t i;
    switch (pSteps)
    {
    case 0:
        if ((globalStepCount % 400) != 0)
        {
            Graphics_clearDisplay(&g_sContext);
            SpeedMode=LOW;
            period=definePeriod;
            sprintf(buffer,"Pattern Mode");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,30,OPAQUE_TEXT);
            sprintf(buffer,"Resetting to");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
            sprintf(buffer,"12 ' o clock");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,60,OPAQUE_TEXT);
            while ((globalStepCount % 400) != 0)
            {
                myMotorController();
                initTimerA0();
                while ((TA0CCTL0 & BIT0) != BIT0){}
                TA0CCTL0= (TA0CCTL0 & (~BIT0));
            }
            PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
            PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
            if (x==0)
            {
                if ((PBS1==1) || (PBS2==1)){break;}
            }
            else
            {
                GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
                GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
            }
            Graphics_clearDisplay(&g_sContext);
            sprintf(buffer,"Reset is complete");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
            sprintf(buffer,"Step %u",pSteps);
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        }
        else
        {
            sprintf(buffer,"Pattern Mode");
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,30,OPAQUE_TEXT);
            sprintf(buffer,"Step %u",pSteps);
            Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        }
        globalStepCount=0;
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        SpeedMode=LOW;
        nSteps=133;
        for (i=0;i<=nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"Step: %u",pSteps);
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        break;
    case 1:
        SpeedMode=HIGH;
        period=definePeriod/2;
        nSteps=167;
        for (i=0;i<nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"Step: %u",pSteps);
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        break;
    case 2:
        SpeedMode=HIGH;
        nSteps=100;
        for (i=0;i<nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"Step: %u",pSteps);
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        break;
    case 3:
        SpeedMode=HIGH;
        nSteps=200;
        for (i=0;i<nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"Step: %u",pSteps);
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        break;
    case 4:
        SpeedMode=HIGH;
        nSteps=134;
        for (i=0;i<nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"Step: %u",pSteps);
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        break;
    case 5:
        SpeedMode=HIGH;
        nSteps=200;
        for (i=0;i<nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"Step: %u",pSteps);
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        break;
    case 6:
        SpeedMode=LOW;
        period=definePeriod;
        nSteps=133;
        for (i=0;i<nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"Step: %u",pSteps);
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        break;
    case 7:
        SpeedMode=LOW;
        nSteps=100;
        for (i=0;i<nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"Step: %u",pSteps);
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        break;
    case 8:
        SpeedMode=HIGH;
        period=definePeriod/2;
        nSteps=168;
        for (i=0;i<nSteps;i++)
        {
            myMotorController();
            initTimerA0();
            while ((TA0CCTL0 & BIT0) != BIT0){}
            TA0CCTL0= (TA0CCTL0 & (~BIT0));
        }
        PBS1=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN6);
        PBS2=GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN5);
        if (x==0)
        {
            if ((PBS1==1) || (PBS2==1)){break;}
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
        }
        pSteps++;
        Graphics_clearDisplay(&g_sContext);
        sprintf(buffer,"Pattern Mode");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
        sprintf(buffer,"is complete");
        Graphics_drawStringCentered(&g_sContext,buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
        pSteps=0;
        x=0;
        yy=1;
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        break;
    }
}
