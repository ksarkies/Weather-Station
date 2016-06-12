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

#ifndef ACQSERIALPORT_H
#define ACQSERIALPORT_H
#define _TTY_POSIX_         // Need to tell qextserialport we are in POSIX

#include <qextserialport.h>
#include <QString>

//-----------------------------------------------------------------------------
/** @brief Serial Port extensions to QextSerialPort.

This subclasses QExtSerialPort to allow addition of some useful member
functions.
*/

class SerialPort : public QextSerialPort
{
public:
    SerialPort();
    SerialPort(const QString & name);
    ~SerialPort();
    bool initPort(const uchar baudrate,const ulong timeout);
};

#endif
