/**
@mainpage Power Management Data Processing Main Window
@version 1.0
@author Ken Sarkies (www.jiggerjuice.net)
@date 08 August 2017

Utility program to aid in analysis if BMS data files.
*/

/****************************************************************************
 *   Copyright (C) 2017 by Ken Sarkies                                      *
 *   ksarkies@internode.on.net                                              *
 *                                                                          *
 *   This program is free software; you can redistribute it and/or          *
 *   modify it under the terms of the GNU General Public License as         *
 *   published by the Free Software Foundation; either version 2 of the     *
 *   License, or (at your option) any later version.                        *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   You should have received a copy of the GNU General Public License      *
 *   along with this program if not, write to the                           *
 *   Free Software Foundation, Inc.,                                        *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.              *
 ***************************************************************************/


#include "data-processing-main.h"
#include <QApplication>
#include <QString>
#include <QLineEdit>
#include <QLabel>
#include <QMessageBox>
#include <QTextEdit>
#include <QCloseEvent>
#include <QFileDialog>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QDebug>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

//-----------------------------------------------------------------------------
/** Power Management Data Processing Main Window Constructor

@param[in] parent Parent widget.
*/

DataProcessingGui::DataProcessingGui()
{
// Build the User Interface display from the Ui class in ui_mainwindowform.h
    DataProcessingMainUi.setupUi(this);
    outFile = NULL;
}

DataProcessingGui::~DataProcessingGui()
{
}

//-----------------------------------------------------------------------------
/** @brief Successful establishment of Window

@returns TRUE if successful.
*/
bool DataProcessingGui::success()
{
    return true;
}

//-----------------------------------------------------------------------------
/** @brief Open a raw data file for Reading.

This button only opens the file for reading.
*/

void DataProcessingGui::on_openReadFileButton_clicked()
{
    QString errorMessage;
    QFileInfo fileInfo;
    QString filename = QFileDialog::getOpenFileName(this,
                                "Data File","./","Text Files (*.txt)");
    if (filename.isEmpty())
    {
        displayErrorMessage("No filename specified");
        return;
    }
    inFile = new QFile(filename);
    fileInfo.setFile(filename);
/* Look for start and end times, and determine current zero calibration */
    if (inFile->open(QIODevice::ReadOnly))
    {
        scanFile(inFile);
    }
    else
    {
        displayErrorMessage(QString("%1").arg((uint)inFile->error()));
    }
}

//-----------------------------------------------------------------------------
/** @brief Extract All to CSV.

The data files are expected to have the format of the BMS transmissions and
will be text files. 

All data is extracted to a csv file with one record per time interval. The
records subsequent to the time record are analysed and data is extracted
to the appropriate fields. The code expects the records to have a particular
order as sent by the BMS and the output has the same order without any
identification. The output format is suitable for spreadsheet analysis.
*/

void DataProcessingGui::on_dumpAllButton_clicked()
{
    QDateTime startTime = DataProcessingMainUi.startTime->dateTime();
    QDateTime endTime = DataProcessingMainUi.endTime->dateTime();
    if (! inFile->isOpen()) return;
    if (! openSaveFile()) return;
    inFile->seek(0);      // rewind input file
    combineRecords(startTime, endTime, inFile, outFile, true);
    if (saveFile.isEmpty())
        displayErrorMessage("File already closed");
    else
    {
        outFile->close();
        delete outFile;
//! Clear the name to prevent the same file being used.
        saveFile = QString();
    }
}

//-----------------------------------------------------------------------------
/** @brief Analysis of CSV files for various performance indicators.

The file is analysed for a variety of faults and other performance indicators.

The results are printed out to a report file.

- Situations where the charger is not allocated but a device is ready. To show
  this look for no device under charge and source voltage above any device.
  Print the op state, charging phase, device and source voltages, switches,
  decision and indicators.

- device current during the charging phase to show in particular how the float
  state is reached and if this is a true indication of a device being fully
  charged.

- Solar current input derived from all batteries as they are charged in the bulk
  phase. This may cut short during any one day if all batteries enter the float
  state and the charger is de-allocated.
*/

void DataProcessingGui::on_analysisFileSelectButton_clicked()
{
}

//-----------------------------------------------------------------------------
/** @brief Extract and Combine Raw Records to CSV.

Raw records are combined into single records for each time interval, and written
to a csv file. Format suitable for spreadsheet analysis.

@param[in] QDateTime start time.
@param[in] QDateTime end time.
@param[in] QFile* input file.
@param[in] QFile* output file.
*/

bool DataProcessingGui::combineRecords(QDateTime startTime, QDateTime endTime,
                                       QFile* inFile, QFile* outFile,bool header)
{
    int voltage = -1;
    int current = 0;
    float temperature = -1;
    float humidity = -1;
    float pressure = 0;
    float insolation = -1;
    int rainfall = 0;
    int windSpeed = -1;
    bool blockStart = false;
    QTextStream inStream(inFile);
    QTextStream outStream(outFile);
    if (header)
    {
        outStream << "Time,";
        outStream << "Voltage," << "Current,";
        outStream << "Temperature," << "Humidity,";
        outStream << "Pressure,";
        outStream << "Insolation,";
        outStream << "Rainfall,";
        outStream << "Wind Speed";
        outStream << "\n\r";
    }
    QDateTime time = startTime;
    QDateTime lastTime = startTime;
    int timeStep = DataProcessingMainUi.timeCorrectionSpinBox->value();
    while (! inStream.atEnd())
    {
        if  ((time > endTime) && (endTime > startTime)) break;
      	QString lineIn = inStream.readLine();
        if (lineIn.size() <= 1) break;
        QStringList breakdown = lineIn.split(",");
        int size = breakdown.size();
        if (size <= 0) break;
        QString firstText = breakdown[0].simplified();
        QString secondText;
        QString thirdText;
        if (size > 1)
        {
            secondText = breakdown[1].simplified();
        }
        if (size > 2)
        {
            thirdText = breakdown[2].simplified();
        }
        if (size > 1)
        {
// Find and extract the time record
            if (firstText == "pH")
            {
                lastTime = time;
                time = QDateTime::fromString(breakdown[1].simplified(),Qt::ISODate);
                if (time < lastTime)    // Attempt to correct for faulty time
                    time = lastTime.addSecs(timeStep);
                if ((blockStart) && (time > startTime))
                {
                    outStream << timeRecord << ",";
                    outStream << (float)voltage/256 << ",";
                    outStream << (float)current/256 << ",";
                    outStream << temperature << ",";
                    outStream << humidity << ",";
                    outStream << pressure << ",";
                    outStream << insolation << ",";
                    outStream << (float)rainfall/256 << ",";
                    outStream << (float)windSpeed/256 << ",";
                    outStream << "\n\r";
                }
                timeRecord = time.toString(Qt::ISODate);
                blockStart = true;
            }
            if (firstText == "dB")
            {
                current = secondText.toInt();
                voltage = thirdText.toInt();
            }
            if (firstText == "dT")
            {
                temperature = secondText.toFloat();
            }
            if (firstText == "dH")
            {
                humidity = secondText.toFloat();
            }
            if (firstText == "dP")
            {
                pressure = secondText.toFloat();
            }
            if (firstText == "dL")
            {
                insolation = secondText.toFloat()/3.3;
            }
            if (firstText == "dR")
            {
                rainfall = secondText.toInt();
            }
            if (firstText == "dS")
            {
                windSpeed = secondText.toInt();
            }
        }
    }
    return inStream.atEnd();
}

//-----------------------------------------------------------------------------
/** @brief Seek First Time Record.

The input file is searched record by record until the first time record is
found.

@param[in] QFile* input file.
@returns QDateTime time of first time record. Null if not found.
*/

QDateTime DataProcessingGui::findFirstTimeRecord(QFile* inFile)
{
    QDateTime time;
    QTextStream inStream(inFile);
    while (! inStream.atEnd())
    {
      	QString lineIn = inStream.readLine();
        if (lineIn.size() <= 1) break;
        QStringList breakdown = lineIn.split(",");
        int size = breakdown.size();
        if (size <= 0) break;
        QString firstText = breakdown[0].simplified();
// Find and extract the time record
        if ((size > 1) && (firstText == "pH"))
        {
            time = QDateTime::fromString(breakdown[1].simplified(),Qt::ISODate);
            break;
        }
    }
    return time;
}

//-----------------------------------------------------------------------------
/** @brief Open a Data File for Writing.

This is called from other action functions. The file is requested in a file
dialogue and opened. The function aborts if the file exists.

@returns true if file successfully created and opened.
*/

bool DataProcessingGui::openSaveFile()
{
    if (! saveFile.isEmpty())
    {
        displayErrorMessage("A save file is already open - close first");
        return false;
    }
    QString filename = QFileDialog::getSaveFileName(this,
                        "Save csv Data",
                        QString(),
                        "Comma Separated Variables (*.csv)",0,0);
    if (filename.isEmpty()) return false;
    if (! filename.endsWith(".csv")) filename.append(".csv");
    QFileInfo fileInfo(filename);
    saveDirectory = fileInfo.absolutePath();
    saveFile = saveDirectory.filePath(filename);
    outFile = new QFile(saveFile);             // Open file for output
    if (! outFile->open(QIODevice::WriteOnly))
    {
        displayErrorMessage("Could not open the output file");
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
/** @brief Scan the data file

Look for start and end times and record types. Obtain the current zeros from
records that have isolated operational status.

*/

void DataProcessingGui::scanFile(QFile* inFile)
{
    if (! inFile->isOpen()) return;
    QTextStream inStream(inFile);
    QDateTime startTime, endTime;
    while (! inStream.atEnd())
    {
      	QString lineIn = inStream.readLine();
        if (lineIn.size() <= 1) break;
        QStringList breakdown = lineIn.split(",");
        int length = breakdown.size();
        if (length <= 0) break;
        QString firstText = breakdown[0].simplified();
        QString secondText = breakdown[1].simplified();
        if ((firstText == "pH") && (length > 1))
        {
            QDateTime time = QDateTime::fromString(secondText,Qt::ISODate);
            if (startTime.isNull()) startTime = time;
            endTime = time;
        }
    }
    if (! startTime.isNull()) DataProcessingMainUi.startTime->setDateTime(startTime);
    if (! endTime.isNull()) DataProcessingMainUi.endTime->setDateTime(endTime);
}

//-----------------------------------------------------------------------------
/** @brief Print an error message.

*/

void DataProcessingGui::displayErrorMessage(QString message)
{
    DataProcessingMainUi.errorMessageLabel->setText(message);
}

//-----------------------------------------------------------------------------
/** @brief Message box for output file exists.

Checks the existence of a specified file and asks for decisions about its
use: abort, overwrite, or append.

@param[in] QString filename: name of output file.
@param[out] bool* append: true if append was selected.
@returns true if abort was selected.
*/

bool DataProcessingGui::outfileMessage(QString filename, bool* append)
{
    QString saveFile = saveDirectory.filePath(filename);
// If report filename exists, decide what action to take.
// Build a message box with options
    if (QFile::exists(saveFile))
    {
        QMessageBox msgBox;
        msgBox.setText(QString("A previous save file ").append(filename).
                        append(" exists."));
// Overwrite the existing file
        QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"),
                         QMessageBox::AcceptRole);
// Append to the existing file
        QPushButton *appendButton = msgBox.addButton(tr("Append"),
                         QMessageBox::AcceptRole);
// Quit altogether
        QPushButton *abortButton = msgBox.addButton(tr("Abort"),
                         QMessageBox::AcceptRole);
        msgBox.exec();
        if (msgBox.clickedButton() == overwriteButton)
        {
            QFile::remove(saveFile);
        }
        else if (msgBox.clickedButton() == abortButton)
        {
            return true;
        }
// Don't write the header into the appended file
        else if (msgBox.clickedButton() == appendButton)
        {
            *append = true;
        }
    }
    return false;
}

