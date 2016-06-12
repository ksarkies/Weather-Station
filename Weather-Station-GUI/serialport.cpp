/*
Title:    Serial Port Subclass
*/

/***************************************************************************
 *   Copyright (C) 2007 by Ken Sarkies                                     *
 *   ksarkies@internode.on.net                                             *
 *                                                                         *
 *   This file is part of Acquisition                                      *
 *                                                                         *
 *   Acquisition is free software; you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   Acquisition is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with Acquisition if not, write to the                           *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/

#include "serialport.h"
#include <QDebug>

//-----------------------------------------------------------------------------
// Subclass of QExtSerialPort to allow use with Acquisition

SerialPort::SerialPort() : QextSerialPort()
{
}

SerialPort::SerialPort(const QString & name) : QextSerialPort(name)
{
}

SerialPort::~SerialPort()
{
}

//-----------------------------------------------------------------------------
/** @brief Initialise the serial port to the specified baudrate and open it.

This is a convenience function to set the serial port parameters to a common
set of values, and to set the baudrate from a table of standard rates. The
baudrate index has the following values:
- 0 =   2400
- 1 =   4800
- 2 =   9600
- 3 =  19200
- 4 =  38400
- 5 =  57600
- 6 = 115200

Once the settings are made, the port is then opened.
@param[in] initialBaudrate Index into the baudrate table to initialize the port.
@param[in] timeout A timeout setting per character in case of non-arrival.
@returns true if successfully opened.
*/

bool SerialPort::initPort(const uchar initialBaudrate,
                                  const ulong timeout)
{
    uchar baudrate = initialBaudrate % 7;
    if (baudrate == 0)                // Adapt baud
    {
      setBaudRate(BAUD2400);
      qDebug() << "Baudrate" << "2.4k";
    }
    else if (baudrate == 1)
    {
      setBaudRate(BAUD4800);
      qDebug() << "Baudrate" << "4.8k";
    }
    else if (baudrate == 2)
    {
      setBaudRate(BAUD9600);
      qDebug() << "Baudrate" << "9.6k";
    }
    else if (baudrate == 3)
    {
      setBaudRate(BAUD19200);
      qDebug() << "Baudrate" << "19.2k";
    }
    else if (baudrate == 4)
    {
      setBaudRate(BAUD38400);
      qDebug() << "Baudrate" << "38.4k";
    }
    else if (baudrate == 5)
    {
      setBaudRate(BAUD57600);
      qDebug() << "Baudrate" << "57.6k";
    }
    else if (baudrate == 6)
    {
      setBaudRate(BAUD115200);
      qDebug() << "Baudrate" << "115.2k";
    }
    setFlowControl(FLOW_OFF);
    setParity(PAR_NONE);
    setDataBits(DATA_8);
    setStopBits(STOP_1);
    setTimeout(timeout);
    return open(QIODevice::ReadWrite);
}

//-----------------------------------------------------------------------------
