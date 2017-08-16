/*       Weather Station Main Window

Here the data stream from the remote is received and saved to a file.

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

#include "weather-station.h"
#include "weather-station-main.h"
//#include "weather-station-record.h"
//#include "weather-station-configure.h"
#include <QApplication>
#include <QString>
#include <QLineEdit>
#include <QLabel>
#include <QMessageBox>
#include <QTextEdit>
#include <QCloseEvent>
#include <QFileDialog>
#include <QDateTime>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QDebug>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

#define NUMBAUDS 7
const qint32 bauds[NUMBAUDS] = {2400,4800,9600,19200,38400,57600,115200};
/*---------------------------------------------------------------------------*/
/** Weather Station Main Window Constructor

@param[in] parent Parent widget.
*/

WeatherStationGui::WeatherStationGui(QString device, uint parameter,
                                       QWidget* parent) : QDialog(parent)
{
/* Build the User Interface display from the Ui class in ui_mainwindowform.h */
    WeatherStationMainUi.setupUi(this);
    initMainWindow(WeatherStationMainUi);

    saveFile.clear();
    response.clear();

    port = NULL;
    baudrate = parameter;
    serialDevice = device;
    setSourceComboBox(0);           /* Pick up the top of the detected sources */
/* Create serial port if it has been specified, otherwise leave to the GUI. */
    on_connectButton_clicked();
    if (port != NULL)
    {
/* Turn on remote microcontroller communications */
        port->write("pc+\n\r");
/* This should cause the remote microcontroller to respond with all data */
        port->write("dS\n\r");
    }
}

WeatherStationGui::~WeatherStationGui()
{
/* Turn off remote microcontroller communications to save power. */
    if (port != NULL)
    {
        port->write("pc-\n\r");
        delete port;
        port = NULL;
    }
}

/*---------------------------------------------------------------------------*/
/** @brief Initialize the main window */

void WeatherStationGui::initMainWindow(Ui::WeatherStationMainDialog mainWindow)
{
    mainWindow.sourceComboBox->setEnabled(true);
    mainWindow.sourceComboBox->show();
    mainWindow.sourceComboBox->raise();
    mainWindow.baudrateComboBox->setEnabled(true);
    mainWindow.baudrateComboBox->show();
    mainWindow.baudrateComboBox->raise();

    mainWindow.battery->setMinimum(0);
    mainWindow.battery->setMaximum(100);
    mainWindow.battery->setValue(0);

    WeatherStationMainUi.batteryGroupBox->setVisible(false);
}

/*---------------------------------------------------------------------------*/
/** @brief Initialize the Serial Combo Box Items

This queries selected serial devices in /dev and fills in the source combo box
with likely candidates. The last entry is /dev/ttyS0, with ACM and USB
entries appearing in descending order. Thus if a USB serial device is present,
/dev/ttyUSB0 will appear as the first entry.
*/

void WeatherStationGui::setSourceComboBox(int index)
{
    QString serialDevice;
    WeatherStationMainUi.sourceComboBox->clear();
    WeatherStationMainUi.sourceComboBox->insertItem(0,"/dev/ttyS0");
/* Try three ACM sources */
    for (int i=3; i>=0; i--)
    {
        serialDevice = "/dev/ttyACM"+QString::number(i);
        QFileInfo checkACMFile(serialDevice);
        if (checkACMFile.exists())
            WeatherStationMainUi.sourceComboBox->insertItem(0,serialDevice);
    }
/* Try three USB sources */
    for (int i=3; i>=0; i--)
    {
        serialDevice = "/dev/ttyUSB"+QString::number(i);
        QFileInfo checkUSBFile(serialDevice);
        if (checkUSBFile.exists())
            WeatherStationMainUi.sourceComboBox->insertItem(0,serialDevice);
    }
    WeatherStationMainUi.sourceComboBox->setCurrentIndex(index);

    for (int i = 0; i < NUMBAUDS; i++)
        WeatherStationMainUi.baudrateComboBox->addItem(QString("%1").arg(bauds[i]));
    WeatherStationMainUi.baudrateComboBox->setCurrentIndex(DEFAULT_BAUDRATE);
}

/*---------------------------------------------------------------------------*/
/** @brief Handle incoming serial data

This is called when data appears in the serial buffer. Data is pulled in until
a newline occurs, at which point the assembled command QString is processed.

All incoming messages are processed here and passed to other windows as appropriate.
*/

void WeatherStationGui::onDataAvailable()
{
    QByteArray data = port->readAll();
    int n=0;
    while (n < data.size())
    {
        if ((data.at(n) != '\r') && (data.at(n) != '\n')) response += data.at(n);
        if (data.at(n) == '\n')
        {
/* The current time is saved to ms precision followed by the line. */
            tick.restart();
            processResponse(response);
            response.clear();
        }
        n++;
    }
}

/*---------------------------------------------------------------------------*/
/** @brief Process the incoming serial data

Parse the line and take action on the command received.
*/

void WeatherStationGui::processResponse(const QString response)
{
    QStringList breakdown = response.split(",");
    int size = breakdown.size();
    QString firstField;
    firstField = breakdown[0].simplified();
    QString secondField;
    if (size > 1) secondField = breakdown[1].simplified();
    QString current, voltage;
    if (! saveFile.isEmpty()) saveLine(response);
/* When the time field is received, send back a short message to keep comms
alive. Also check for calibration as time messages stop during this process. */
    if ((size > 0) && (firstField == "pH"))
    {
        port->write("pc+\n\r");
    }
/* Outside temperature is sent as fixed point floating number to scale. */
    if ((size > 0) && (firstField == "dT"))
    {
        if (size > 1) WeatherStationMainUi.outsideTemperature
            ->setText(QString("%1").arg(secondField
                .toFloat(),0,'f',1).append(QChar(0x00B0)).append("C"));
    }
/* Outside humidity is sent as fixed point floating number to scale. */
    if ((size > 0) && (firstField == "dH"))
    {
        if (size > 1) WeatherStationMainUi.outsideHumidity
            ->setText(QString("%1").arg(secondField
                .toFloat(),0,'f',1).append("%"));
    }

// TBD convert to suitable scaled value
/* Wind Speed */
    if ((size > 0) && (firstField == "dS"))
    {
        if (size > 1) WeatherStationMainUi.windSpeed
            ->setText(QString("%1").arg(secondField
                .toFloat(),0,'f',1));
    }

// TBD convert to suitable scaled value
/* Rainfall */
    if ((size > 0) && (firstField == "dR"))
    {
        if (size > 1) WeatherStationMainUi.rainfall
            ->setText(QString("%1 mm").arg(secondField
                .toFloat(),0,'f',0));
    }

/* Solar Radiance. Current converted to a percentage of maximum current (330mA)
for the Suntech STP005S12-5W */
    if ((size > 0) && (firstField == "dL"))
    {
        if (size > 1) WeatherStationMainUi.insolation
            ->setText(QString("%1%").arg(secondField
                .toFloat()/3.3,0,'f',1));
    }

// TBD convert to suitable scaled value
/* Barometric Pressure */
    if ((size > 0) && (firstField == "dP"))
    {
        if (size > 1) WeatherStationMainUi.airPressure
            ->setText(QString("%1 mbar").arg(secondField
                .toFloat(),0,'f',0));
    }

// TBD work at providing a better health measure
/* Battery Health. For now use voltage as fixed point number to scale. */
    if ((size > 0) && (firstField == "dV"))
    {
        float batteryVoltage = secondField.toFloat();
        WeatherStationMainUi.voltage
            ->setText(QString("%1 V").arg(secondField
                .toFloat(),0,'f',2));
        int batteryHealth = batteryVoltage*100/6.5;
        if (batteryHealth > 100) batteryHealth = 100;
        if (size > 1) WeatherStationMainUi.battery
            ->setValue(batteryHealth);
    }

/* Battery Current */
    if ((size > 0) && (firstField == "dI"))
    {
        WeatherStationMainUi.current
            ->setText(QString("%1 mA").arg(secondField
                .toFloat(),0,'f',0));
    }

/* This allows debug messages to be displayed on the terminal. */
    if ((size > 0) && (firstField.left(1) == "D"))
    {
        qDebug() << response;
        if (! saveFile.isEmpty()) saveLine(response);
    }
}

/*---------------------------------------------------------------------------*/
/** @brief Show battery status.

A group box is made visible to show the battery voltage and current.
*/

void WeatherStationGui::on_batteryPushButton_clicked()
{
    if (WeatherStationMainUi.batteryGroupBox->isVisible())
        WeatherStationMainUi.batteryGroupBox->setVisible(false);
    else
        WeatherStationMainUi.batteryGroupBox->setVisible(true);
}

/*---------------------------------------------------------------------------*/
/** @brief Obtain a save file name and path and attempt to open it.

The files are csv but the ending can be arbitrary to allow compatibility
with the data processing application.
*/

void WeatherStationGui::on_saveFileButton_clicked()
{
/* Make sure there is no file already open. */
    if (! saveFile.isEmpty())
    {
        displayErrorMessage("A file is already open - close first");
        return;
    }
    WeatherStationMainUi.errorLabel->clear();
    QString filename = QFileDialog::getSaveFileName(this,
                        "Acquisition Save Acquired Data",
                        QString(),
                        "Comma Separated Variables (*.csv *.txt)");
    if (filename.isEmpty()) return;
/*    if (! filename.endsWith(".csv")) filename.append(".csv"); */
    QFileInfo fileInfo(filename);
    saveDirectory = fileInfo.absolutePath();
    saveFile = saveDirectory.filePath(filename);
    outFile = new QFile(saveFile);             /* Open file for output */
    if (! outFile->open(QIODevice::WriteOnly))
    {
        displayErrorMessage("Could not open the output file");
        return;
    }
}

/*---------------------------------------------------------------------------*/
/** @brief Save a line to the opened save file.

*/
void WeatherStationGui::saveLine(QString line)
{
/* Check that the save file has been defined and is open. */
    if (saveFile.isEmpty())
    {
        displayErrorMessage("Output File not defined");
        return;
    }
    if (! outFile->isOpen())
    {
        displayErrorMessage("Output File not open");
        return;
    }
    QTextStream out(outFile);
    out << line << "\n\r";
}

/*---------------------------------------------------------------------------*/
/** @brief Close the save file.

*/
void WeatherStationGui::on_closeFileButton_clicked()
{
    WeatherStationMainUi.errorLabel->clear();
    if (saveFile.isEmpty())
        displayErrorMessage("File already closed");
    else
    {
        outFile->close();
        delete outFile;
/*! Save the name to prevent the same file being used. */
        saveFile = QString();
    }
}
/*---------------------------------------------------------------------------*/
/** @brief Show an error condition in the Error label.

@param[in] message: Message to be displayed.
*/

void WeatherStationGui::displayErrorMessage(const QString message)
{
    WeatherStationMainUi.errorLabel->setText(message);
}

/*---------------------------------------------------------------------------*/
/** @brief Error Message

@returns a message when the device didn't respond.
*/
QString WeatherStationGui::error()
{
    return errorMessage;
}

/*---------------------------------------------------------------------------*/
/** @brief Successful establishment of serial port setup

@returns TRUE if successful.
*/
bool WeatherStationGui::success()
{
    return true;
}

/*---------------------------------------------------------------------------*/
/** @brief Close off the window and deallocate resources

This may not be necessary as QT may implement it implicitly.
*/
void WeatherStationGui::closeEvent(QCloseEvent *event)
{
    event->accept();
}

/*---------------------------------------------------------------------------*/
/** @brief Sleep for a number of centiseconds but keep processing events

*/

void WeatherStationGui::ssleep(int centiseconds)
{
    for (int i=0; i<centiseconds*10; i++)
    {
        qApp->processEvents();
        millisleep(10);
    }
}

/*---------------------------------------------------------------------------*/
/** @brief Attempt to connect to the remote system.

For serial, if opening the port is successful, connect to the slot, otherwise
delete the port. Read the device name from the GUI.
*/
void WeatherStationGui::on_connectButton_clicked()
{
    if (port == NULL)
    {
        serialDevice = WeatherStationMainUi.sourceComboBox->currentText();
/* Setup serial port and attempt to open it */
        port = new QSerialPort(serialDevice);
        if (port->open(QIODevice::ReadWrite))
        {
            connect(port, SIGNAL(readyRead()), this, SLOT(onDataAvailable()));
            WeatherStationMainUi.connectButton->setText("Disconnect");
            synchronized = true;
/* Turn on microcontroller communications */
            port->write("pc+\n\r");
/* This should cause the microcontroller to respond with all data */
            port->write("dS\n\r");
            int baudrate = WeatherStationMainUi.baudrateComboBox->currentIndex();
            port->setBaudRate(bauds[baudrate]);
            port->setDataBits(QSerialPort::Data8);
            port->setParity(QSerialPort::NoParity);
            port->setStopBits(QSerialPort::OneStop);
            port->setFlowControl(QSerialPort::NoFlowControl);
        }
        else
        {
            delete port;
            port = NULL;
            errorMessage = QString("Unable to access the serial port\n"
                        "Check the connections and power.\n"
                        "You may need root privileges?");
        }
    }
    else
    {
        disconnect(port, SIGNAL(readyRead()), this, SLOT(onDataAvailable()));
        delete port;
        port = NULL;
        WeatherStationMainUi.connectButton->setText("Connect");
    }
    setSourceComboBox(WeatherStationMainUi.sourceComboBox->currentIndex());
}
