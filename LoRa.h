// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>
//#include <wiringPi.h>
//#include <wiringPiSPI.h>
#include <unistd.h>
#include "ToradexSPI.h"

#define AMBER_13                1
#define AMBER_20                0

#define SPI_CHANNEL_LORA        0

#if AMBER_13 == 1
#define LORA_DEFAULT_SS_PIN     6
#define LORA_DEFAULT_RESET_PIN  26
#define LORA_DEFAULT_DIO0_PIN   5
#elif AMBER_20 == 1
#define LORA_DEFAULT_SS_PIN     3
#define LORA_DEFAULT_RESET_PIN  0
#define LORA_DEFAULT_DIO0_PIN   21
#endif

#define PA_OUTPUT_RFO_PIN       0
#define PA_OUTPUT_PA_BOOST_PIN  1

typedef unsigned char byte;

class LoRaClass {
public:
    LoRaClass();

    void spi_setup();

    int begin(long frequency);
    void end();

    int beginPacket(int implicitHeader = false);
    int endPacket(bool async = false);

    int parsePacket(int size = 0);
    int packetRssi();
    float packetSnr();
    long packetFrequencyError();

    size_t write(uint8_t byte);
    size_t write(const uint8_t *buffer, size_t size);

    // from Stream
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();

#ifndef ARDUINO_SAMD_MKRWAN1300
    void onReceive(void(*callback)(int));
    void receive(int size = 0);
#endif
    void idle();
    void sleep();

    void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
    void setFrequency(long frequency);
    void setSpreadingFactor(int sf);
    void setSignalBandwidth(long sbw);
    void setCodingRate4(int denominator);
    void setPreambleLength(long length);
    void setSyncWord(int sw);
    void enableCrc();
    void disableCrc();
    void enableInvertIQ();
    void disableInvertIQ();
    bool isRX_DoneSet();

    void setOCP(uint8_t mA); // Over Current Protection control
    void setModeRx();

    // deprecated
    void crc() { enableCrc(); }
    void noCrc() { disableCrc(); }

    byte random();

    void clearRxIrq();

    void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
    void dumpRegisters();

private:
    void explicitHeaderMode();
    void implicitHeaderMode();

    void handleDio0Rise();
    bool isTransmitting();

    int getSpreadingFactor();
    long getSignalBandwidth();

    void setLdoFlag();

    //uint8_t readRegister(uint8_t address);
    //void writeRegister(uint8_t address, uint8_t value);
    //uint8_t singleTransfer(uint8_t address, uint8_t value);

    static void onDio0Rise();

private:
    int _ss;
    int _reset;
    int _dio0;
    long _frequency;
    int _packetIndex;
    int _implicitHeaderMode;
    void (*_onReceive)(int);
};
#endif //LORA_H

