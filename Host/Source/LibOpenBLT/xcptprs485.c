/************************************************************************************/ /**
* \file         xcptprs485.c
* \brief        XCP RS485 transport layer source file.
* \ingroup      XcpTpRs485
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2017  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include <assert.h>  /* for assertions                          */
#include <stdint.h>  /* for standard integer types              */
#include <stddef.h>  /* for NULL declaration                    */
#include <stdbool.h> /* for boolean type                        */
#include <stdlib.h>  /* for standard library                    */
#include <string.h>  /* for string library                      */
#include <unistd.h>  /* UNIX standard functions                 */
#include <stdio.h>
#include <sys/time.h>
#include <fcntl.h>      /* file control definitions                */
#include <sys/ioctl.h>  /* system I/O control                      */
#include "session.h"    /* Communication session module            */
#include "xcploader.h"  /* XCP loader module                       */
#include "xcptprs485.h" /* XCP RS485 transport layer               */
#include "util.h"       /* Utility module                          */
#include "serialport.h" /* Serial port module                      */

/****************************************************************************************
* Function prototypes
****************************************************************************************/
#define IN            0
#define OUT           1
#define BUFFER_MAX    3
#define DIRECTION_MAX 35
#define VALUE_MAX     30
/*
 * Delay value calculation macros.
 * c - number of characters
 * b - bits per character
 * s - bits per second
 */
#define DV(c, b, s) ((c) * (b)*1000000l / (s))

static int GPIOExport(int pin);
static int GPIOUnexport(int pin);
static int GPIODirection(int pin, int dir);
static int GPIOWrite(int pin, int value);

static void XcpTpRs485Init(void const *settings);
static void XcpTpRs485Terminate(void);
static bool XcpTpRs485Connect(void);
static void XcpTpRs485Disconnect(void);
static bool XcpTpRs485SendPacket(tXcpTransportPacket const *txPacket,
                                 tXcpTransportPacket *rxPacket, uint16_t timeout);

uint16_t ModbusCRC16(uint8_t *pucFrame, uint16_t usLen);
/****************************************************************************************
* Local constant declarations
****************************************************************************************/
/** \brief XCP transport layer structure filled with UART specifics. */
static const tXcpTransport rs485Transport = {
    XcpTpRs485Init,
    XcpTpRs485Terminate,
    XcpTpRs485Connect,
    XcpTpRs485Disconnect,
    XcpTpRs485SendPacket,
};

void tty_delay(int usec) {
    struct timeval tv, ttv;
    long           ts;
    gettimeofday(&tv, NULL);
    do {
        (void)gettimeofday(&ttv, NULL);
        ts = 1000000l * (ttv.tv_sec - tv.tv_sec) + (ttv.tv_usec - tv.tv_usec);
    } while (ts < usec); // && !tty_break);
}

static int GPIOExport(int pin) {
    char buffer[BUFFER_MAX];
    int  bytes_written;
    int  fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (-1 == fd) {
        //logw(2, "Failed to open export for writing!\n");
        return (-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return (0);
}

static int GPIOUnexport(int pin) {
    char buffer[BUFFER_MAX];
    int  bytes_written;
    int  fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd) {
        //logw(2, "Failed to open unexport for writing!\n");
        return (-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return (0);
}

static int GPIODirection(int pin, int dir) {
    static const char s_directions_str[] = "in\0out";
    char              path[DIRECTION_MAX];
    int               fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        //logw(2, "Failed to open gpio direction for writing!\n");
        return (-1);
    }

    if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
        //logw(2, "Failed to set direction!\n");
        return (-1);
    }

    close(fd);
    return (0);
}

static int GPIOWrite(int pin, int value) {
    static const char s_values_str[] = "01";
    char              path[VALUE_MAX];
    int               fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        //logw(2, "Failed to open gpio value for writing!\n");
        return (-1);
    }

    if (1 != write(fd, &s_values_str[value ? 1 : 0], 1)) {
        //logw(2, "Failed to write value!\n");
        return (-1);
    }

    close(fd);
    return (0);
}

/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief The settings to use in this transport layer. */
static tXcpTpRs485Settings tpRs485Settings;

/***********************************************************************************/ /**
** \brief     Obtains a pointer to the transport layer structure, so that it can be 
**            linked to the XCP protocol module.
** \return    Pointer to transport layer structure.
**
****************************************************************************************/
tXcpTransport const *XcpTpRs485GetTransport(void) {
    return &rs485Transport;
} /*** end of XcpTpUartGetTransport ***/

/************************************************************************************/ /**
** \brief     Initializes the transport layer.
** \param     settings Pointer to settings structure.
** \return    None.
**
****************************************************************************************/
static void XcpTpRs485Init(void const *settings) {
    char *uartPortName;

    /* Reset transport layer settings. */
    tpRs485Settings.portname = NULL;
    tpRs485Settings.baudrate = 0;

    /* Check parameters. */
    assert(settings != NULL);

    /* Only continue with valid parameters. */
    if (settings != NULL) /*lint !e774 */
    {
        /* Shallow copy the transport layer settings for layer usage. */
        tpRs485Settings = *((tXcpTpRs485Settings *)settings);
        /* The portname is a pointer and it is not guaranteed that it stays valid so we need
     * to deep copy this one. note the +1 for '\0' in malloc.
     */
        assert(((tXcpTpRs485Settings *)settings)->portname != NULL);
        if (((tXcpTpRs485Settings *)settings)->portname != NULL) /*lint !e774 */
        {
            uartPortName = malloc(strlen(((tXcpTpRs485Settings *)settings)->portname) + 1);
            assert(uartPortName != NULL);
            if (uartPortName != NULL) /*lint !e774 */
            {
                strcpy(uartPortName, ((tXcpTpRs485Settings *)settings)->portname);
                tpRs485Settings.portname = uartPortName;
            }
        }
    }
    /* Initialize the serial port. */
    SerialPortInit();
    GPIOExport(18);
    GPIODirection(18, OUT);
} /*** end of XcpTpUartInit ***/

/************************************************************************************/ /**
** \brief     Terminates the transport layer.
**
****************************************************************************************/
static void XcpTpRs485Terminate(void) {
    /* Terminate the serial port. */
    SerialPortTerminate();
    /* Release memory that was allocated for storing the port name. */
    if (tpRs485Settings.portname != NULL) {
        free((char *)tpRs485Settings.portname);
    }
    /* Reset transport layer settings. */
    tpRs485Settings.portname = NULL;
    tpRs485Settings.baudrate = 0;
} /*** end of XcpTpUartTerminate ***/

/************************************************************************************/ /**
** \brief     Connects to the transport layer.
** \return    True is connected, false otherwise.
**
****************************************************************************************/
static bool XcpTpRs485Connect(void) {
    bool                result = false;
    bool                baudrateSupported;
    tSerialPortBaudrate baudrate;

    /* Check if the specified baudrate is supported by the serial port driver. */
    baudrateSupported = (tpRs485Settings.baudrate == 9600) ||
                        (tpRs485Settings.baudrate == 19200) ||
                        (tpRs485Settings.baudrate == 38400) ||
                        (tpRs485Settings.baudrate == 57600) ||
                        (tpRs485Settings.baudrate == 115200);

    /* Check transport layer settings. */
    assert(tpRs485Settings.portname != NULL);
    assert(baudrateSupported);

    /* Only continue if the transport layer settings are valid. */
    if ((tpRs485Settings.portname != NULL) && (baudrateSupported)) /*lint !e774 */
    {
        /* Convert the numeric baudrate to the one supported by the serial port driver. */
        switch (tpRs485Settings.baudrate) {
        case 115200:
            baudrate = SERIALPORT_BR115200;
            break;
        case 57600:
            baudrate = SERIALPORT_BR57600;
            break;
        case 38400:
            baudrate = SERIALPORT_BR38400;
            break;
        case 19200:
            baudrate = SERIALPORT_BR19200;
            break;
        case 9600:
        default:
            baudrate = SERIALPORT_BR9600;
            break;
        }
        /* Connect to the serial port. */
        result = SerialPortOpen(tpRs485Settings.portname, baudrate);
    }
#define USE_SHC_BROADCAST 1

#if defined(USE_SHC_BROADCAST) && USE_SHC_BROADCAST
    static uint8_t rs485Buffer[XCPLOADER_PACKET_SIZE_MAX + 9];
    /* Set result value to okay and only change it from now on if an error occurred. */
    /* Prepare the XCP packet for transmission on UART. This is basically the same as
   * the XCP packet data but just the length of the packet is added to the first
   * byte.
   */
    // Slave Address: Function : code : Data Data Data : CRC  Lo : CRC Hi
    rs485Buffer[0] = (uint8_t)0;  // broadcasting
    rs485Buffer[1] = (uint8_t)5;  // write single coil
    rs485Buffer[2] = (uint8_t)0;  // ((tpRs485Settings.RegAddr >> 8) & 0xFF); //Holding register address Hi
    rs485Buffer[3] = (uint8_t)33; // (tpRs485Settings.RegAddr & 0xFF); //Holdign register address Lo
    rs485Buffer[4] = 0xff;        // true 0x00 //false
    rs485Buffer[5] = 0x00;        // true 0x00 //false

    uint16_t ui16Crc = ModbusCRC16(rs485Buffer, 6);

    rs485Buffer[6] = (uint8_t)(ui16Crc & 0xFF);        //0xA5; //CRC low byte
    rs485Buffer[7] = (uint8_t)((ui16Crc >> 8) & 0xff); // 0xA5; //CRC high byte

    /* Transmit the packet. */
    GPIOWrite(18, 1);
    tty_delay(35000000l / 9600);
    if (!SerialPortWrite(rs485Buffer, 8)) {
        result = false;
    }
    tty_delay(DV(8, 10, 9600)); //9800, 1stop, no parity, 8bit : 10bit/char
    GPIOWrite(18, 0);

#endif

    /* Give the result back to the caller. */
    return result;
} /*** end of XcpTpUartConnect ***/

/************************************************************************************/ /**
** \brief     Disconnects from the transport layer.
**
****************************************************************************************/
static void XcpTpRs485Disconnect(void) {
    /* Disconnect from the serial port. */
    SerialPortClose();
} /*** end of XcpTpUartDisconnect ***/

static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40};

uint16_t ModbusCRC16(uint8_t *pucFrame, uint16_t usLen) {
    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    int     iIndex;

    while (usLen--) {
        iIndex  = ucCRCLo ^ *(pucFrame++);
        ucCRCLo = (uint8_t)(ucCRCHi ^ aucCRCHi[iIndex]);
        ucCRCHi = aucCRCLo[iIndex];
    }
    return (uint16_t)(ucCRCHi << 8 | ucCRCLo);
}

/************************************************************************************/ /**
** \brief     Transmits an XCP packet on the transport layer and attempts to receive the
**            response packet within the specified timeout.
** \param     txPacket Pointer to the packet to transmit.
** \param     rxPacket Pointer where the received packet info is stored.
** \param     timeout Maximum time in milliseconds to wait for the reception of the
**            response packet.
** \return    True is successful and a response packet was received, false otherwise.
**
****************************************************************************************/
static bool XcpTpRs485SendPacket(tXcpTransportPacket const *txPacket,
                                 tXcpTransportPacket *rxPacket, uint16_t timeout) {
    bool     result = false;
    uint16_t byteIdx;
    /* uartBuffer is static to lower the stack load. +1 because the first byte for an XCP 
   * packet on the UART transport layer contains the packet length.
   */
    static uint8_t rs485Buffer[XCPLOADER_PACKET_SIZE_MAX + 9];
    uint32_t       responseTimeoutTime = 0;
    bool           packetReceptionComplete;

    /* Check parameters. */
    assert(txPacket != NULL);
    assert(rxPacket != NULL);

    /* Only continue with valid parameters. */
    if ((txPacket != NULL) && (rxPacket != NULL)) /*lint !e774 */
    {
#if defined(USE_SHC_BROADCAST) && USE_SHC_BROADCAST
        if (0xD2 /*XCPLOADER_CMD_PROGRAM_START*/ == txPacket->data[0]) {
            /* Set result value to okay and only change it from now on if an error occurred. */
            /* Prepare the XCP packet for transmission on UART. This is basically the same as
		   * the XCP packet data but just the length of the packet is added to the first
		   * byte.
		   */
            // Slave Address: Function : code : Data Data Data : CRC  Lo : CRC Hi
            rs485Buffer[0] = (uint8_t)0;  // broadcasting
            rs485Buffer[1] = (uint8_t)5;  // write single coil
            rs485Buffer[2] = (uint8_t)0;  // ((tpRs485Settings.RegAddr >> 8) & 0xFF); //Holding register address Hi
            rs485Buffer[3] = (uint8_t)33; // (tpRs485Settings.RegAddr & 0xFF); //Holdign register address Lo
            rs485Buffer[4] = 0xff;        // true 0x00 //false
            rs485Buffer[5] = 0x00;        // true 0x00 //false

            uint16_t ui16Crc = ModbusCRC16(rs485Buffer, 6);

            rs485Buffer[6] = (uint8_t)(ui16Crc & 0xFF);        //0xA5; //CRC low byte
            rs485Buffer[7] = (uint8_t)((ui16Crc >> 8) & 0xff); // 0xA5; //CRC high byte

            GPIOWrite(18, 1);
            tty_delay(35000000l / 9600);
            /* Transmit the packet. */
            if (!SerialPortWrite(rs485Buffer, 8)) {
                result = false;
            }
            tty_delay(DV(8, 10, 9600)); //9800, 1stop, no parity, 8bit : 10bit/char
            GPIOWrite(18, 0);
            //UtilTimeDelayMs(100); // sleep(100);
        }

        if (0xCF /*XCPLOADER_CMD_PROGRAM_RESET*/ == txPacket->data[0]) {
            /* Set result value to okay and only change it from now on if an error occurred. */
            /* Prepare the XCP packet for transmission on UART. This is basically the same as
           * the XCP packet data but just the length of the packet is added to the first
           * byte.
           */
            // Slave Address: Function : code : Data Data Data : CRC  Lo : CRC Hi
            rs485Buffer[0] = (uint8_t)0;  // broadcasting
            rs485Buffer[1] = (uint8_t)5;  // write single coil
            rs485Buffer[2] = (uint8_t)0;  // ((tpRs485Settings.RegAddr >> 8) & 0xFF); //Holding register address Hi
            rs485Buffer[3] = (uint8_t)32; // (tpRs485Settings.RegAddr & 0xFF); //Holdign register address Lo
            rs485Buffer[4] = 0xff;        // true 0x00 //false
            rs485Buffer[5] = 0x00;        // true 0x00 //false

            uint16_t ui16Crc = ModbusCRC16(rs485Buffer, 6);

            rs485Buffer[6] = (uint8_t)(ui16Crc & 0xFF);        //0xA5; //CRC low byte
            rs485Buffer[7] = (uint8_t)((ui16Crc >> 8) & 0xff); // 0xA5; //CRC high byte

            GPIOWrite(18, 1);
            tty_delay(35000000l / 9600);
            /* Transmit the packet. */
            if (!SerialPortWrite(rs485Buffer, 8)) {
                result = false;
            }
            tty_delay(DV(8, 10, 9600)); //9800, 1stop, no parity, 8bit : 10bit/char
            GPIOWrite(18, 0);
            //UtilTimeDelayMs(100);
        }

// #define XCPLOADER_CMD_PROGRAM_START   (0xD2u)    /**< XCP program start command code.  */
// #define XCPLOADER_CMD_PROGRAM_CLEAR   (0xD1u)    /**< XCP program clear command code.  */
// #define XCPLOADER_CMD_PROGRAM         (0xD0u)    /**< XCP program command code.        */
// #define XCPLOADER_CMD_PROGRAM_RESET   (0xCFu)    /**< XCP program reset command code.  */
#endif
        /* Set result value to okay and only change it from now on if an error occurred. */
        result = true;
        /* Prepare the XCP packet for transmission on UART. This is basically the same as 
     * the XCP packet data but just the length of the packet is added to the first 
     * byte.
     */
        // Slave Address: Function : code : Data Data Data : CRC  Lo : CRC Hi
        rs485Buffer[0] = (uint8_t)tpRs485Settings.SlaveAddr;
        rs485Buffer[1] = (uint8_t)tpRs485Settings.FuncCode;                // holding register
        rs485Buffer[2] = (uint8_t)((tpRs485Settings.RegAddr >> 8) & 0xFF); //Holding register address Hi
        rs485Buffer[3] = (uint8_t)(tpRs485Settings.RegAddr & 0xFF);        //Holdign register address Lo
        rs485Buffer[4] = 0x00;                                             // word(16bit) length hi
        rs485Buffer[5] = (txPacket->len + 1) / 2;                          // word(16bit) length lo
        rs485Buffer[6] = txPacket->len;                                    // byte(8bit) length (= word length * 2)
        // data from 7

        for (byteIdx = 0; byteIdx < txPacket->len; byteIdx++) {
            rs485Buffer[byteIdx + 7] = txPacket->data[byteIdx];
        }

        uint16_t ui16Crc = ModbusCRC16(rs485Buffer, txPacket->len + 7);

        rs485Buffer[txPacket->len + 7] = (uint8_t)(ui16Crc & 0xFF);        //0xA5; //CRC low byte
        rs485Buffer[txPacket->len + 8] = (uint8_t)((ui16Crc >> 8) & 0xff); // 0xA5; //CRC high byte

        /* Transmit the packet. */
        GPIOWrite(18, 1);
        tty_delay(35000000l / 9600);
        if (!SerialPortWrite(rs485Buffer, txPacket->len + 9)) {
            result = false;
        }
        tty_delay(DV((uint32_t)(txPacket->len + 9), 10, 9600)); //9600, 1stop, no parity, 8bit : 10bit/char
        GPIOWrite(18, 0);

        /* Only continue if the transmission was successful. */
        if (result) {
            /* Determine timeout time for the response packet. */
            responseTimeoutTime = UtilTimeGetSystemTimeMs() + timeout;
            /* Initialize packet reception length. */
            rxPacket->len = 0;
            /* Poll with timeout detection to receive the first byte. This one contains the packet length and cannot be zero. */
            while (UtilTimeGetSystemTimeMs() < responseTimeoutTime) {
                if (SerialPortRead(&rxPacket->len, 1)) {
                    /* Length received. Validate it before accepting it */
                    if (rxPacket->len > 0) {
                        /* Start of packet received. Stop this loop to continue with the reception of the packet. */
                        break;
                    }
                }
            }
            /* Check if a valid start of packet was received, in which case the first byte won't have a zero value. */
            if (rxPacket->len == 0) {
                /* No valid start of packet received, so a timeout occurred. */
                result = false;
            }
        }

        /* Only continue with reception if a valid pacekt lenght was received. */
        if (result) {
            /* Continue with reception of the packet. */
            packetReceptionComplete = false;
            byteIdx                 = 0;
            /* Poll with timeout detection to receive the full packet. */
            while (UtilTimeGetSystemTimeMs() < responseTimeoutTime) {
                /* Check if the next byte was received. */
                if (SerialPortRead(&rxPacket->data[byteIdx], 1)) {
                    /* Check if the packet reception is now complete. */
                    if ((byteIdx + 1) == rxPacket->len) {
                        /* Set flag and stop the loop. */
                        packetReceptionComplete = true;
                        //printf("receive complete\n");
                        break;
                    }
                    /* Increment indexer to the next byte. */
                    byteIdx++;
                }
            }
            /* Check if a timeout occurred. */
            if (!packetReceptionComplete) {
                result = false;
            }
        }
    }
    /* Give the result back to the caller. */
    return result;
} /*** end of XcpTpUartSendPacket ***/

/*********************************** end of xcptpuart.c ********************************/
