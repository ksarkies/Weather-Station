/*          Weather Station GUI Recording Window Header

@date 6 August 2016
*/

/****************************************************************************
 *   Copyright (C) 2016 by Ken Sarkies                                      *
 *   ksarkies@internode.on.net                                              *
 *                                                                          *
 *   This file is part of Weather Station GUI                               *
 *                                                                          *
 *   Weather Station GUI is free software; you can redistribute it and/or   *
 *   modify it under the terms of the GNU General Public License as         *
 *   published by the Free Software Foundation; either version 2 of the     *
 *   License, or (at your option) any later version.                        *
 *                                                                          *
 *   Weather Station GUI is distributed in the hope that it will be useful, *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   You should have received a copy of the GNU General Public License      *
 *   along with Weather Station GUI if not, write to the                    *
 *   Free Software Foundation, Inc.,                                        *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.              *
 ***************************************************************************/

#ifndef WEATHER_STATION_RECORD_H
#define WEATHER_STATION_RECORD_H

#include "weather-station.h"
#include "ui_weather-station-record.h"
#include <QDialog>
#include <QStandardItemModel>

//-----------------------------------------------------------------------------
/** @brief Weather Station Recording Window.

*/

class WeatherStationRecordGui : public QDialog
{
    Q_OBJECT
public:
#ifdef SERIAL
    WeatherStationRecordGui(SerialPort* socket, QWidget* parent = 0);
#else
    WeatherStationRecordGui(QTcpSocket* socket, QWidget* parent = 0);
#endif
    ~WeatherStationRecordGui();
private slots:
    void on_deleteButton_clicked();
    void on_startButton_clicked();
    void on_stopButton_clicked();
    void on_closeFileButton_clicked();
    void on_recordFileButton_clicked();
    void onMessageReceived(const QString &text);
    void onListItemClicked(const QModelIndex & index);
    void on_registerButton_clicked();
    void on_closeButton_clicked();
private:
// User Interface object instance
    Ui::WeatherStationRecordDialog WeatherStationRecordUi;
    int extractValue(const QString &response);
    void requestRecordingStatus();
    void refreshDirectory();
    void getFreeSpace();
    int writeFileHandle;
    int readFileHandle;
    bool recordingOn;
    bool writeFileOpen;
    bool readFileOpen;
    QStandardItemModel *model;
    int row;
    bool directoryEnded;
    bool nextDirectoryEntry;

};

#endif
