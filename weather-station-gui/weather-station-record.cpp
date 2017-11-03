/*       Weather Station Recording Window

Remote recording of history data is started here, with recording to a local
file on the attached medium, notably an SD card.

The files on the card are displayed and a new one suggested.
Recording is started and stopped, at which the file is closed.

13 October 2017
*/
/****************************************************************************
 *   Copyright (C) 2017 by Ken Sarkies                                      *
 *   ksarkies@internode.on.net                                              *
 *                                                                          *
 *   This file is part of data-acquisition                                  *
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

#include "weather-station-main.h"
#include "weather-station-record.h"
#include <QApplication>
#include <QString>
#include <QLineEdit>
#include <QLabel>
#include <QListView>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QDebug>
#include <QStandardItemModel>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

//-----------------------------------------------------------------------------
/** Recording GUI Constructor

The remote unit is queried for status of recording and storage drive statistics.
The directory listing is obtained from the remote unit. 

@param[in] socket TCP Socket object pointer
@param[in] parent Parent widget.
*/

WeatherStationRecordGui::WeatherStationRecordGui(QSerialPort* p, QWidget* parent)
                                                    : QDialog(parent)
{
    socket = p;
    WeatherStationRecordUi.setupUi(this);
// Ask for the microcontroller SD card free space (process response later)
    getFreeSpace();
    model = new QStandardItemModel(0, 2, this);
    WeatherStationRecordUi.fileTableView->setModel(model);
    WeatherStationRecordUi.fileTableView->setGridStyle(Qt::NoPen);
    WeatherStationRecordUi.fileTableView->setShowGrid(false);
    QHeaderView *verticalHeader = WeatherStationRecordUi.fileTableView->verticalHeader();
    verticalHeader->setSectionResizeMode(QHeaderView::Fixed);
    verticalHeader->setDefaultSectionSize(18);
    row = 0;
// Signal to process a click on a directory item
    connect(WeatherStationRecordUi.fileTableView,
                     SIGNAL(clicked(const QModelIndex)),
                     this,SLOT(onListItemClicked(const QModelIndex)));
// Send a command to refresh the directory
    refreshDirectory();
    writeFileHandle = 0xFF;
    requestRecordingStatus();
}

WeatherStationRecordGui::~WeatherStationRecordGui()
{
}

//-----------------------------------------------------------------------------
/** @brief Delete File.

If the delete checkbox is selected, delete the file (if it exists).
*/

void WeatherStationRecordGui::on_deleteButton_clicked()
{
    on_closeFileButton_clicked();
    QString fileName = WeatherStationRecordUi.recordFileName->text();
    if ((fileName.length() > 0) &&
        (WeatherStationRecordUi.deleteCheckBox->isChecked()))
    {
        socket->write("fX");
        socket->write(fileName.toLocal8Bit().data());
        socket->write("\n\r");
        refreshDirectory();
        getFreeSpace();
    }
}

//-----------------------------------------------------------------------------
/** @brief Open Recording File.

If a write file is not open the specified file is opened for writing. Response
is a status that indicates if the file was opened/created and the recording
started. This is processed later.
*/

void WeatherStationRecordGui::on_recordFileButton_clicked()
{
    on_registerButton_clicked();
    QString fileName = WeatherStationRecordUi.recordFileName->text();
    if (fileName.length() > 0)
    {
        socket->write("fW");
        socket->write(fileName.toLocal8Bit().data());
        socket->write("\n\r");
        requestRecordingStatus();
        refreshDirectory();
    }
}

//-----------------------------------------------------------------------------
/** @brief Start Recording.

If a write file is not open the specified file is opened for writing and a
command to start recording is sent. Response is a status that indicates if the
file was opened/created and the recording started. This is processed later.
*/

void WeatherStationRecordGui::on_startButton_clicked()
{
    if (writeFileHandle < 0xFF)
    {
        socket->write("pr+\n\r");
        requestRecordingStatus();
    }
    else WeatherStationRecordUi.errorLabel->setText("File not open");
}

//-----------------------------------------------------------------------------
/** @brief Stop Recording.

*/

void WeatherStationRecordGui::on_stopButton_clicked()
{
    socket->write("pr-\n\r");
    requestRecordingStatus();
}

//-----------------------------------------------------------------------------
/** @brief Close the write file.

If recording is in progress it is stopped and the write file is closed.
*/

void WeatherStationRecordGui::on_closeFileButton_clicked()
{
    if (writeFileHandle < 0xFF)
    {
        char command[6] = "fC0\n\r";
        command[2] = '0'+writeFileHandle;
        socket->write("pr-\n\r");
        socket->write(command);
        requestRecordingStatus();
    }
    else WeatherStationRecordUi.errorLabel->setText("File not open");
    writeFileHandle = 0xFF;
}

//-----------------------------------------------------------------------------
/** @brief Process a Message.

After a command is sent, response messages from the remote are passed here for
processing. Appropriate fields on the form are updated.

@todo This interprets response as done in main. See if this can be made
independent of formats.
*/

void WeatherStationRecordGui::onMessageReceived(const QString &response)
{
    QStringList breakdown = response.split(",");
    QString command = breakdown[0].right(1);
// Error Code
    switch (command[0].toLatin1())
    {
// Show Free Space
        case 'F':
        {
            int freeSpace = breakdown[2].toInt()*breakdown[1].toInt()/2048;
            WeatherStationRecordUi.diskSpaceAvailable->setText(QString("%1 M")\
                                                    .arg(freeSpace, 0, 10));
            break;
        }
/* Directory listing.
Fill a predefined model with strings from the response breakdown.
The response will be a comma separated list of items preceded by a type.
*/
        case 'D':
        {
            model->clear();
            if (breakdown.size() <= 1) break;
            for (int i=1; i<breakdown.size(); i++)
            {
                QChar type = breakdown[i][0];
                bool ok;
                QString fileSize = QString("%1")
                    .arg((float)breakdown[i].mid(1,8).toInt(&ok,16)/1000000,8,'f',3);
                if (type == 'd')
                    fileSize = "";
                if ((type == 'f') || (type == 'd'))
                {
                    QString fileName = breakdown[i].mid(9,breakdown[i].length()-1);
                    QFont font;
                    if (type == 'd') font.setBold(true);
                    QStandardItem *nameItem = new QStandardItem(fileName);
                    QStandardItem *sizeItem = new QStandardItem(fileSize);
                    QList<QStandardItem *> row;
                    nameItem->setFont(font);
                    nameItem->setData(Qt::AlignLeft, Qt::TextAlignmentRole);
                    sizeItem->setData(Qt::AlignRight, Qt::TextAlignmentRole);
                    row.append(nameItem);
                    row.append(sizeItem);
                    nameItem->setData(QVariant(type));
//                    item->setIcon(...);
                    model->appendRow(row);
                }
            }
            break;
        }
/* Directory listing incremental.
Fill a predefined model with a string from the response breakdown. A series of
responses will give each entry in the listing, terminated by an empty filename.
This will ease the communications load by requesting each entry only after
the previous entry has been fully received.
*/
        case 'd':
        {
            if (breakdown.size() < 1) break;
// Empty parameters received indicates the directory listing has ended.
            directoryEnded = (breakdown.size() == 1);
            if (directoryEnded) break;
            nextDirectoryEntry = true;
            for (int i=1; i<breakdown.size(); i++)
            {
                QChar type = breakdown[i][0];
                if ((type == 'e') || (type == 'n')) break;
                bool ok;
                QString fileSize = QString("%1")
                    .arg((float)breakdown[i].mid(1,8).toInt(&ok,16)/1000000,8,'f',3);
                if (type == 'd')
                    fileSize = "";
                if ((type == 'f') || (type == 'd'))
                {
                    QString fileName = breakdown[i].mid(9,breakdown[i].length()-1);
                    QFont font;
                    if (type == 'd') font.setBold(true);
                    QStandardItem *nameItem = new QStandardItem(fileName);
                    QStandardItem *sizeItem = new QStandardItem(fileSize);
                    QList<QStandardItem *> row;
                    nameItem->setFont(font);
                    nameItem->setData(Qt::AlignLeft, Qt::TextAlignmentRole);
                    sizeItem->setData(Qt::AlignRight, Qt::TextAlignmentRole);
                    row.append(nameItem);
                    row.append(sizeItem);
                    nameItem->setData(QVariant(type));
//                    item->setIcon(...);
                    model->appendRow(row);
                }
/* Request the next entry by sending another incremental directory command with
no directory name. */
                socket->write("fd\r\n");
            }
            break;
        }
// Status of recording and open files.
// The write and read file handles are retrieved from this
        case 's':
        {
            if (breakdown.size() <= 1) break;
            recordingOn = (breakdown[1].toInt() & 0x02) > 0;
            if (recordingOn)  // recording on
                WeatherStationRecordUi.startButton->
                    setStyleSheet("background-color:lightgreen;");
            else
                WeatherStationRecordUi.startButton->
                    setStyleSheet("background-color:lightpink;");
            if (breakdown.size() <= 2) break;
            writeFileHandle = breakdown[2].toInt();
            writeFileOpen = (writeFileHandle < 255);
            if (writeFileOpen)
            {
                WeatherStationRecordUi.recordFileButton->
                    setStyleSheet("background-color:lightgreen;");
                WeatherStationRecordUi.recordFileName->setText(breakdown[3]);
            }
            else
                WeatherStationRecordUi.recordFileButton->
                    setStyleSheet("background-color:lightpink;");
            if (breakdown.size() <= 3) break;
            readFileHandle = breakdown[3].toInt();
            readFileOpen = (readFileHandle < 255);
            if (readFileOpen)
                 WeatherStationRecordUi.readFileName->setText(breakdown[3]);
           break;
        }
// Open a file for recording.
        case 'W':
        {
            writeFileHandle = extractValue(response);
            break;
        }
        case 'E':
        {
            QString errorText[19] = {"Hard Disk Error",
                                     "Internal Error",
                                     "Medium Not Ready",
                                     "File not Found",
                                     "Path not Found",
                                     "Invalid Path Format",
                                     "Access denied or directory full",
                                     "File Exists",
                                     "File/directory object invalid",
                                     "Drive write protected",
                                     "Logical drive number invalid",
                                     "Volume has no work area",
                                     "No valid FAT volume",
                                     "Format aborted: parameter error",
                                     "Timeout waiting for access",
                                     "File sharing policy violation",
                                     "LFN working buffer could not be allocated",
                                     "Too many open files",
                                     "Invalid Parameter"};
            int status = breakdown[1].toInt();
            if ((status > 0) && (status < 20))
                WeatherStationRecordUi.errorLabel->setText(errorText[status-1]);
            break;
        }
    }
}

//-----------------------------------------------------------------------------
/** @brief Extract an Integer Value from a Response.

*/

int WeatherStationRecordGui::extractValue(const QString &response)
{
    int i = 4;
    QString temp;
    while (response[i] >= '0')
    {
        temp[i-4] = response[i];
        i++;
    }
    return temp.toInt();
}

//-----------------------------------------------------------------------------
/** @brief Slot to process Directory Entry Clicks.

Display filename in edit box, or enter a directory and redisplay.
*/

void WeatherStationRecordGui::onListItemClicked(const QModelIndex & index)
{
    WeatherStationRecordUi.recordFileName->clear();
    QStandardItem *item = model->itemFromIndex(index);
    QString fileName = item->text();
    QChar type = item->data().toChar();
    if (type == 'f')
    {
        WeatherStationRecordUi.recordFileName->setText(fileName);
        WeatherStationRecordUi.readFileName->setText(fileName);
    }
    if (type == 'd')
        socket->write(QString("fD%1\n\r").arg(fileName).toLocal8Bit().data());
}

//-----------------------------------------------------------------------------
/** @brief Close Window

*/

void WeatherStationRecordGui::on_closeButton_clicked()
{
    this->close();
}

//-----------------------------------------------------------------------------
/** @brief Remount the storage media

*/

void WeatherStationRecordGui::on_registerButton_clicked()
{
    socket->write("fM\n\r");
    refreshDirectory();
}

//-----------------------------------------------------------------------------
/** @brief Refresh the Directory.

This requests the first directory entry for the top directory only.
Subsequent directory entries are obtained when the response to the previous
one has been received.
*/

void WeatherStationRecordGui::refreshDirectory()
{
    model->clear();
    socket->write("fd/\n\r");
}

//-----------------------------------------------------------------------------
/** @brief Ask for Status of Recording.

*/

void WeatherStationRecordGui::requestRecordingStatus()
{
    socket->write("fs\n\r");
}

//-----------------------------------------------------------------------------
/** @brief Ask for Amount of Free Space Remaining on the Medium.

*/

void WeatherStationRecordGui::getFreeSpace()
{
    socket->write("fF\n\r");
}


