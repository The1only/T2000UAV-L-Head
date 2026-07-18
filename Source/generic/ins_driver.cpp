/**
 * @file ins_driver.cpp
 * @brief Implementation of INS_driver.
 *
 * Contains the implementation details for the INS_driver class.
 */

// Main.cpp  (portable)
// Build on macOS: clang++ -std=c++17 -O2 Main.cpp Com_posix.cpp wit_c_sdk.c -lpthread -o WitSimulate

#include <cstdio>
#include <cstdint>
#include <cstring>
// #include <cmath>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "wit_c_sdk.h"

// Look for an external IMU over Bluetooth
#ifndef Q_OS_IOS
#define USE_BT_IMU
#else
#undef  USE_BT_IMU
#endif
// #undef  USE_BT_IMU


#ifndef Q_OS_IOS
// #if defined(USE_BT_IMU)
    #include "bleuart.h"
// #endif
#include "serialport.h"
#endif

typedef struct { double L, F, D; } LFD;

static char s_cDataUpdate = 0;
ulong iComPort  = 4;
int iAddress  = 0x50;

#ifdef Q_OS_IOS
#define ComBt void
#define ComQt void
#endif

#ifndef USE_BT_IMU
    #define ComBt void
#endif

static ComQt *serialPorts = nullptr;
static ComBt *serialPortb = nullptr;

bool AutoScanSensor();
void AutoSetBaud(int);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
//static void DelayMs(uint16_t ms);

using RxCallbackINS  = void(*)(void *handler, uint32_t uiReg, uint16_t uiRegNu[]); //char*, uint32_t);
static RxCallbackINS  callback_ = nullptr;
static void setINSRxCallback(RxCallbackINS cb) { callback_ = cb; }
static void *handle = nullptr;

bool INS_driver(void *handler, ComQt *serPorts, ComBt *serPortb, void *func)
{
    serialPorts = serPorts;
    serialPortb = serPortb;
    handle = handler;

    setINSRxCallback((RxCallbackINS)func);
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(CopeSensorData);
    WitDelayMsRegister(posix_delay_ms);
    s_cDataUpdate = 0;
    return 1;
}

void AutoSetBaud(int baud)
{
    (void)baud;
#ifndef Q_OS_IOS
    if(serialPorts){
        if(baud == QSerialPort::Baud9600){
            WitSetUartBaud(WIT_BAUD_9600);
            serialPorts->setBaudrate(QSerialPort::Baud9600);
        }
        if(baud == QSerialPort::Baud115200){
            WitSetUartBaud(WIT_BAUD_115200);
            serialPorts->setBaudrate(QSerialPort::Baud115200);
        }
    }
#endif
}

bool AutoScanSensor()
{
    WitReadReg(AX, 3);
    SLEEP_MS(200);
    if(s_cDataUpdate == 1){
        return 1;
    }
    else{
        return 0;
    }
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    (void)p_data;
    (void)uiSize;

#ifndef Q_OS_IOS
    if(serialPorts){
        if(serialPorts != nullptr){
            serialPorts->send(reinterpret_cast<const char*>(p_data),static_cast<unsigned short>(uiSize));
        }
        else{
#ifdef USE_BT_IMU
            if(serialPortb != nullptr)
            serialPortb->send(reinterpret_cast<const char*>(p_data),static_cast<unsigned short>(uiSize));
#endif
        }
    }
#endif
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
    (void) uiRegNum;
    s_cDataUpdate = 1;
    callback_(handle,uiReg,(uint16_t*)sReg);
}
