/**
@mainpage Weather Station GUI
@version 1.0
@author Ken Sarkies (www.jiggerjuice.net)
@date 10 June 2016

This is the PC user interface to the Weather Station.

Call with weather-station [options]

-P   port (/dev/ttyUSB0 default)
-b   baudrate (from 2400, 4800, 9600, 19200, 38400 default, 57600, 115200)

@note
Compiler: gcc 4.8.4 (Ubuntu 4.8.4-2ubuntu1~14.04.3)
@note
Uses: Qt version 5.2.1
*/
/****************************************************************************
 *   Copyright (C) 2016 by Ken Sarkies                                      *
 *   ksarkies@internode.on.net                                              *
 *                                                                          *
 *   This file is part of Weather Station Project                           *
 *                                                                          *
 *   Weather Station is free software; you can redistribute it and/or       *
 *   modify it under the terms of the GNU General Public License as         *
 *   published by the Free Software Foundation; either version 2 of the     *
 *   License, or (at your option) any later version.                        *
 *                                                                          *
 *   Weather Station is distributed in the hope that it will be useful,     *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   You should have received a copy of the GNU General Public License      *
 *   along with Weather Station if not, write to the                        *
 *   Free Software Foundation, Inc.,                                        *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.              *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "weather-station-main.h"
#include "weather-station.h"
#include <QApplication>
#include <QMessageBox>

//-----------------------------------------------------------------------------
/** @brief Weather Station GUI Main Program

*/

int main(int argc,char ** argv)
{
/* Interpret any command line options */
    QString serialDevice = DEFAULT_SERIAL_PORT;
    uint initialBaudrate = DEFAULT_BAUDRATE;
    int baudParm;
    int c;
    opterr = 0;
    while ((c = getopt (argc, argv, "P:b:")) != -1)
    {
        switch (c)
        {
// Serial Port Device
        case 'P':
            serialDevice = optarg;
            break;
// Serial baudrate
        case 'b':
            baudParm = atoi(optarg);
            switch (baudParm)
            {
                case 2400: initialBaudrate=0;break;
                case 4800: initialBaudrate=1;break;
                case 9600: initialBaudrate=2;break;
                case 19200: initialBaudrate=3;break;
                case 38400: initialBaudrate=4;break;
                case 57600: initialBaudrate=5;break;
                case 115200: initialBaudrate=6;break;
            default:
                fprintf (stderr, "Invalid Baudrate %i.\n", baudParm);
                return false;
            }
            break;
// Unknown
        case '?':
            if ((optopt == 'P') || (optopt == 'b'))
                fprintf (stderr, "Option -%c requires an argument.\n", optopt);
            else if (isprint (optopt))
                fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            else
                fprintf (stderr,"Unknown option character `\\x%x'.\n",optopt);
            default: return false;
        }
    }

    QApplication application(argc,argv);
    WeatherStationGui weatherStationGui(serialDevice,initialBaudrate);
    if (weatherStationGui.success())
    {
        weatherStationGui.show();
        return application.exec();
    }
    else
        QMessageBox::critical(0,"Unable to connect to remote system",
              QString("%1").arg(weatherStationGui.error()));
    return false;
}
