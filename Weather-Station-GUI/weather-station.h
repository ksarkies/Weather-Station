/*          Weather Station GUI Main Window Header

@date 10 June 2016
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

#ifndef WEATHER_STATION_H
#define WEATHER_STATION_H

#include "ui_weather-station-main.h"
#include "weather-station-main.h"
#include <QDir>
#include <QFile>
#include <QTime>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QListWidgetItem>
#include <QDialog>
#include <QCloseEvent>

#define millisleep(a) usleep(a*1000)

//-----------------------------------------------------------------------------
/** @brief Weather Station Main Window.

*/

class WeatherStationGui : public QDialog
{
    Q_OBJECT
public:
    WeatherStationGui(QString device, uint parameter, QWidget* parent = 0);
    ~WeatherStationGui();
    bool success();
    QString error();
private slots:
    void on_connectButton_clicked();
    void onDataAvailable();
    void on_batteryPushButton_clicked();
    void on_saveFileButton_clicked();
    void on_closeFileButton_clicked();
//    void on_recordingButton_clicked();
//    void on_configureButton_clicked();
    void closeEvent(QCloseEvent*);
private:
// User Interface object instance
    Ui::WeatherStationMainDialog WeatherStationMainUi;
// Common code
    void initMainWindow(Ui::WeatherStationMainDialog);
    void setSourceComboBox(int index);
// Methods
    void processResponse(const QString response);
    void displayErrorMessage(const QString message);
    void saveLine(QString line);    // Save line to a file
    void ssleep(int seconds);
// Variables
    QString serialDevice;
    uint baudrate;
    bool synchronized;
    QString errorMessage;
    QString response;
    QSerialPort* port;           //!< Serial port object pointer
    QDir saveDirectory;
    QString saveFile;
    QFile* outFile;
    QTime tick;
};

#endif
