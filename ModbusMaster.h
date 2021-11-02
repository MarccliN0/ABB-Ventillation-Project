/**
@file
Arduino library for communicating with Modbus slaves over RS232/485 (via RTU protocol).

@defgroup setup ModbusMaster Object Instantiation/Initialization
@defgroup buffer ModbusMaster Buffer Management
@defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
@defgroup register Modbus Function Codes for Holding/Input Registers
@defgroup constant Modbus Function Codes, Exception Codes
*/
/*

  ModbusMaster.h - Arduino library for communicating with Modbus slaves
  over RS232/485 (via RTU protocol).

  This file is part of ModbusMaster.

  ModbusMaster is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  ModbusMaster is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with ModbusMaster.  If not, see <http://www.gnu.org/licenses/>.

  Written by Doc Walker (Rx)
  Copyright © 2009-2013 Doc Walker <4-20ma at wvfans dot net>

*/


#ifndef ModbusMaster_h
#define ModbusMaster_h


/**
@def __MODBUSMASTER_DEBUG__ (1).
Set to 1 to enable debugging features within class:
  - pin 4 cycles for each byte read in the Modbus response
  - pin 5 cycles for each millisecond timeout during the Modbus response
*/
#define __MODBUSMASTER_DEBUG__ (0)


/* _____STANDARD INCLUDES____________________________________________________ */
// include types & constants of Wiring core API
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
//#include "WProgram.h"
#include <stdint.h>
#include <cstddef>
#endif

uint32_t millis();
#define BYTE 0xA5

/* _____UTILITY MACROS_______________________________________________________ */


/* _____PROJECT INCLUDES_____________________________________________________ */
// functions to calculate Modbus Application Data Unit CRC
//#include "util/crc16.h"
// moved inlcuding crc16.h to ModbusMaster.cpp

// functions to manipulate words
///#include "util/word.h"
#include "word.h"


#include "SerialPort.h"

/* _____CLASS DEFINITIONS____________________________________________________ */
/**
Arduino class library for communicating with Modbus slaves over
RS232/485 (via RTU protocol).
*/
class ModbusMaster
{
  public:
    ModbusMaster();
    ModbusMaster(uint8_t);
    ModbusMaster(uint8_t, uint8_t);

    void begin();
    void begin(uint16_t);
    void idle(void (*)());

    static const uint8_t ku8MBIllegalFunction            = 0x01;

    static const uint8_t ku8MBIllegalDataAddress         = 0x02;

    static const uint8_t ku8MBIllegalDataValue           = 0x03;

    static const uint8_t ku8MBSlaveDeviceFailure         = 0x04;

    static const uint8_t ku8MBSuccess                    = 0x00;

    static const uint8_t ku8MBInvalidSlaveID             = 0xE0;

    static const uint8_t ku8MBInvalidFunction            = 0xE1;

    static const uint8_t ku8MBResponseTimedOut           = 0xE2;

    static const uint8_t ku8MBInvalidCRC                 = 0xE3;

    uint16_t getResponseBuffer(uint8_t);
    void     clearResponseBuffer();
    uint8_t  setTransmitBuffer(uint8_t, uint16_t);
    void     clearTransmitBuffer();

    void beginTransmission(uint16_t);
    uint8_t requestFrom(uint16_t, uint16_t);
    void sendBit(bool);
    void send(uint8_t);
    void send(uint16_t);
    void send(uint32_t);
    uint8_t available(void);
    uint16_t receive(void);


    uint8_t  readCoils(uint16_t, uint16_t);
    uint8_t  readDiscreteInputs(uint16_t, uint16_t);
    uint8_t  readHoldingRegisters(uint16_t, uint16_t);
    uint8_t  readInputRegisters(uint16_t, uint8_t);
    uint8_t  writeSingleCoil(uint16_t, uint8_t);
    uint8_t  writeSingleRegister(uint16_t, uint16_t);
    uint8_t  writeMultipleCoils(uint16_t, uint16_t);
    uint8_t  writeMultipleCoils();
    uint8_t  writeMultipleRegisters(uint16_t, uint16_t);
    uint8_t  writeMultipleRegisters();
    uint8_t  maskWriteRegister(uint16_t, uint16_t, uint16_t);
    uint8_t  readWriteMultipleRegisters(uint16_t, uint16_t, uint16_t, uint16_t);
    uint8_t  readWriteMultipleRegisters(uint16_t, uint16_t);

  private:
    uint8_t  _u8SerialPort;                                      ///< serial port (0..3) initialized in constructor
    uint8_t  _u8MBSlave;                                         ///< Modbus slave (1..255) initialized in constructor
    uint16_t _u16BaudRate;                                       ///< baud rate (300..115200) initialized in begin()
    static const uint8_t ku8MaxBufferSize                = 64;   ///< size of response/transmit buffers
    uint16_t _u16ReadAddress;                                    ///< slave register from which to read
    uint16_t _u16ReadQty;                                        ///< quantity of words to read
    uint16_t _u16ResponseBuffer[ku8MaxBufferSize];               ///< buffer to store Modbus slave response; read via GetResponseBuffer()
    uint16_t _u16WriteAddress;                                   ///< slave register to which to write
    uint16_t _u16WriteQty;                                       ///< quantity of words to write
    uint16_t _u16TransmitBuffer[ku8MaxBufferSize];               ///< buffer containing data to transmit to Modbus slave; set via SetTransmitBuffer()
    uint16_t* txBuffer; // from Wire.h -- need to clean this up Rx
    uint8_t _u8TransmitBufferIndex;
    uint16_t u16TransmitBufferLength;
    uint16_t* rxBuffer; // from Wire.h -- need to clean this up Rx
    uint8_t _u8ResponseBufferIndex;
    uint8_t _u8ResponseBufferLength;

    // Modbus function codes for bit access
    static const uint8_t ku8MBReadCoils                  = 0x01; ///< Modbus function 0x01 Read Coils
    static const uint8_t ku8MBReadDiscreteInputs         = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
    static const uint8_t ku8MBWriteSingleCoil            = 0x05; ///< Modbus function 0x05 Write Single Coil
    static const uint8_t ku8MBWriteMultipleCoils         = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

    // Modbus function codes for 16 bit access
    static const uint8_t ku8MBReadHoldingRegisters       = 0x03; ///< Modbus function 0x03 Read Holding Registers
    static const uint8_t ku8MBReadInputRegisters         = 0x04; ///< Modbus function 0x04 Read Input Registers
    static const uint8_t ku8MBWriteSingleRegister        = 0x06; ///< Modbus function 0x06 Write Single Register
    static const uint8_t ku8MBWriteMultipleRegisters     = 0x10; ///< Modbus function 0x10 Write Multiple Registers
    static const uint8_t ku8MBMaskWriteRegister          = 0x16; ///< Modbus function 0x16 Mask Write Register
    static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers

    // Modbus timeout [milliseconds]
    static const uint16_t ku16MBResponseTimeout          = 2000; ///< Modbus timeout [milliseconds]

    // master function that conducts Modbus transactions
    uint8_t ModbusMasterTransaction(uint8_t u8MBFunction);

    // idle callback function; gets called during idle time between TX and RX
    void (*_idle)();
    SerialPort *MBSerial = NULL; // added by KRL
};
#endif

/**
@example examples/Basic/Basic.pde
@example examples/PhoenixContact_nanoLC/PhoenixContact_nanoLC.pde
*/
