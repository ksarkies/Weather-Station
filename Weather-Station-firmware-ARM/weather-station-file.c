/* STM32F1 Weather Station for Solar Power

Defines the file management for storage of historical data.

Data is written to the storage medium as it is collected. A simple buffer is
used to store the data and when all data collection is complete it is written as
a block with a time stamp. Files of a fixed size are used, each having an
integer number of data blocks. When the storage medium becomes full, the oldest
file is deleted and reused.

Files are read for transmission over the serial port in response to requests
for data within a given time period.

Initial 6 August 2016
*/

/*
 * This file is part of the battery-management-system project.
 *
 * Copyright 2013 K. Sarkies <ksarkies@internode.on.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

/* ChaN FAT includes */
#include "integer.h"
#include "diskio.h"
#include "ff.h"

/* Project Includes */
#include "weather-station-file.h"

/* Local Prototypes */
static void initFile(void);
static void parseFileCommand(char *line);
static uint8_t findFileHandle(void);
static void deleteFileHandle(uint8_t fileHandle);

/* Local Variables */
/* ChaN FAT */
static FATFS Fatfs[_VOLUMES];
static FATFS *fs;		            /* File system object for logical drive 0 */
static FIL file[MAX_OPEN_FILES];    /* file descriptions, 2 files maximum. */
static FILINFO fileInfo[MAX_OPEN_FILES];
static bool fileUsable;
static uint8_t filemap=0;           /* map of open file handles */
static uint8_t writeFileHandle;
static uint8_t readFileHandle;

/*--------------------------------------------------------------------------*/
/* @brief File Initialization

The file system work area is initialised.
*/

static void initFile(void)
{
/* initialise the drive working area */
    FRESULT fileStatus = f_mount(&Fatfs[0],"",0);
    fileUsable = (fileStatus == FR_OK);

/* Initialise some global variables */
    writeFileHandle = 0xFF;
    readFileHandle = 0xFF;
    uint8_t i=0;
    for (i=0; i<MAX_OPEN_FILES; i++) fileInfo[i].fname[0] = 0;
    filemap = 0;
}

/*--------------------------------------------------------------------------*/
/** @brief Parse a command line and act on it.

Each command is a single character followed by the total length of the command
plus parameters.

The commands are action commands only:
W - open a file for writing and reading. Returns a file handle.
R - open a file for reading. Returns a file handle.
C - close a file.
S - store a block of data.
G - retrieve a block of data.
F - Free space on drive

All commands return a status value at the end of any other data sent.

@param[in] char *line: the command line in ASCII
*/

static void parseFileCommand(char *line)
{
    FRESULT fileStatus;

    switch (line[0])
    {
/* Open a file for read/write */
/* Parameter is a filename, 8 character plus dot plus 3 character extension */
/* Returns a file handle. On error file handle is 0xFF. */
        case 'W':
        {
            uint8_t fileHandle = 0xFF;
/* Already open */
            if (writeFileHandle < 0xFF)
                fileStatus = FR_DENIED;
            else
            {
                fileHandle = findFileHandle();
/* Unable to be allocated */
                if (fileHandle >= MAX_OPEN_FILES)
                    fileStatus = FR_TOO_MANY_OPEN_FILES;
                else
                {
/* Try to open a file write/read, creating it if necessary */
                    fileStatus = f_open(&file[fileHandle], line+2, \
                                        FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
/* Skip to the end of the file to append. */
                    if (fileStatus == FR_OK)
                        fileStatus = f_lseek(&file[fileHandle], f_size(file));
                    else
                    {
                        deleteFileHandle(fileHandle);
                        fileHandle = 0xFF;
                    }
                    writeFileHandle = fileHandle;
                    if (fileStatus == FR_OK)
                        fileStatus = f_stat(line+2, fileInfo+writeFileHandle);
                }
            }
          	xQueueSendToBack(fileReceiveQueue,&fileHandle,FILE_SEND_TIMEOUT);
            break;
        }
/* Open a file read only. No check if the file is already opened. */
/* Parameter is a filename, 8 character plus dot plus 3 character extension */
/* Returns a file handle */
        case 'R':
        {
            uint8_t fileHandle = 0xFF;
/* Already open */
            if (readFileHandle < 0xFF)
                fileStatus = FR_DENIED;
            else
            {
                fileHandle = findFileHandle();
/* Unable to be allocated */
                if (fileHandle >= MAX_OPEN_FILES)
                    fileStatus = FR_TOO_MANY_OPEN_FILES;
                else
                {
/* Try to open a file read only */
                    fileStatus = f_open(&file[fileHandle], line+2, \
                                        FA_OPEN_EXISTING | FA_READ);
                    if (fileStatus != FR_OK)
                    {
                        deleteFileHandle(fileHandle);
                        fileHandle = 0xFF;
                    }
                    readFileHandle = fileHandle;
                    if (fileStatus == FR_OK)
                        fileStatus = f_stat(line+2, fileInfo+readFileHandle);
                }
            }
          	xQueueSendToBack(fileReceiveQueue,&fileHandle,FILE_SEND_TIMEOUT);
            break;
        }
/* Close a file */
/* Parameter is a file handle that was given when opened */
        case 'C':
        {
            if (line[1] != 3)
            {
                fileStatus = FR_INVALID_PARAMETER;
                break;
            }
            uint8_t fileHandle = line[2];
            if (fileHandle >= MAX_OPEN_FILES)
            {
                fileStatus = FR_INVALID_OBJECT;
                break;
            }
            if (writeFileHandle == fileHandle) writeFileHandle = 0xFF;
            else if (readFileHandle == fileHandle) readFileHandle = 0xFF;
            else
            {
                fileStatus = FR_INVALID_OBJECT;
                break;
            }
            fileInfo[fileHandle].fname[0] = 0;
            deleteFileHandle(fileHandle);
            fileStatus = f_close(&file[fileHandle]);
            fileHandle = 0xFF;
            break;
        }
/* Store data to the file starting at the end of the file. */
/* Parameters are the filehandle, the length of the data block and the data
block of maximum length 80 bytes. Nothing is written if the line is too long.
The number of bytes to write is given by the length less 4. Returns the number
written. The number written will differ from the number requested if the disk
is full. */
        case 'P':
        {
            if (line[1] < 4) break;
            uint8_t fileHandle = line[2];
            UINT length = line[1]-3;
            UINT numWritten = 0;
            if (length < 82)
            {
                fileStatus = f_write(&file[fileHandle],line+3,length,&numWritten);
                if (numWritten != length)
                {
                    fileStatus = FR_DENIED;
                }
                if (fileStatus == FR_OK) f_sync(&file[fileHandle]);
            }
            else fileStatus = FR_INVALID_PARAMETER;
/* Send a denied status if the disk fills. The caller probably won't use this. */
            break;
        }
/* Get data from a file from the last position that data was read after opening. */
/* Parameters are filehandle followed by the number of bytes to get.
Returns the number read followed by binary byte-wise data. The number read will
differ from the number requested if EOF reached. */
        case 'G':
        {
            uint8_t buffer[80];
            uint8_t fileHandle = line[2];
            uint8_t i = 0;
            UINT length = line[3];
            UINT numRead = 0;
            if (length < 82)
                fileStatus = f_read(&file[fileHandle],buffer,length,&numRead);
            else fileStatus = FR_INVALID_PARAMETER;
            if (uxQueueSpacesAvailable(fileReceiveQueue) >= numRead+1)
            {
                xQueueSendToBack(fileReceiveQueue,&numRead,FILE_SEND_TIMEOUT);
                for (i=0; i<numRead; i++)
                    xQueueSendToBack(fileReceiveQueue,buffer+i,FILE_SEND_TIMEOUT);
            }
            break;
        }
/* Directory listing. */
/* If the name is given, the directory specified is opened and the first entry
returned. Subsequent calls with zero length name will return subsequent entries.
Returns a null terminated file name. At the end, or on error, a zero length
string is sent.
Preceding the name a type character:
 f = file, d = directory, n = error e = end
will be sent. */
        case 'D':
        {
            uint8_t i = 0;
            uint8_t numRead = 0;
            static DIR directory;
            FILINFO fileInfo;
            fileStatus = FR_OK;
            if (line[2] != 0) fileStatus = f_opendir(&directory, line+2);
            if (fileStatus == FR_OK)
            {
                fileStatus = f_readdir(&directory, &fileInfo);
                numRead = stringLength(fileInfo.fname);
            }
            char type = 'f';
            if (fileInfo.fattrib == AM_DIR) type = 'd';
            if (fileInfo.fname[0] == 0) type = 'e';
            if (fileStatus != FR_OK)
            {
                numRead = 0;
                fileInfo.fname[0] = 0;
                type = 'n';
            }
/* If space on the queue, send the type (char), four bytes of file size (MSB
first) and null terminated filename */
            if (uxQueueSpacesAvailable(fileReceiveQueue) >= numRead+2)
            {
                xQueueSendToBack(fileReceiveQueue,&type,FILE_SEND_TIMEOUT);
                uint8_t fileSizeByte = (fileInfo.fsize >> 24) & 0xFF;
                xQueueSendToBack(fileReceiveQueue,&fileSizeByte,FILE_SEND_TIMEOUT);
                fileSizeByte = (fileInfo.fsize >> 16) & 0xFF;
                xQueueSendToBack(fileReceiveQueue,&fileSizeByte,FILE_SEND_TIMEOUT);
                fileSizeByte = (fileInfo.fsize >> 8) & 0xFF;
                xQueueSendToBack(fileReceiveQueue,&fileSizeByte,FILE_SEND_TIMEOUT);
                fileSizeByte = fileInfo.fsize & 0xFF;
                xQueueSendToBack(fileReceiveQueue,&fileSizeByte,FILE_SEND_TIMEOUT);
                for (i=0; i<numRead+1; i++)
                    xQueueSendToBack(fileReceiveQueue,fileInfo.fname+i,FILE_SEND_TIMEOUT);
            }
            break;
        }
/* Read the free space on the drive. */
/* No parameters. Returns free clusters as 4 bytes (32 bit word), lowest first */
        case 'F':
        {
            DWORD freeClusters = 0;
	        fileStatus = f_getfree("", (DWORD*)&freeClusters, &fs);
            uint32_t sectorCluster = fs->csize;
            uint8_t i;
            if (uxQueueSpacesAvailable(fileReceiveQueue) >= 8)
            {
                for (i=0; i<4; i++)
                {
                    uint8_t wordBuf = (freeClusters >> 8*i) & 0xFF;
                    xQueueSendToBack(fileReceiveQueue,&wordBuf,FILE_SEND_TIMEOUT);
                }
                for (i=0; i<4; i++)
                {
                    uint8_t wordBuf = (sectorCluster >> 8*i) & 0xFF;
                    xQueueSendToBack(fileReceiveQueue,&wordBuf,FILE_SEND_TIMEOUT);
                }
            }
            break;
        }
/* Return the open status, name and size of open files. */
        case 'S':
        {
            xQueueSendToBack(fileReceiveQueue,&writeFileHandle,FILE_SEND_TIMEOUT);
            if (writeFileHandle  < 0xFF)
            {
                TCHAR *writeNameChar;
                uint8_t i=0;
                do
                {
                    writeNameChar = fileInfo[writeFileHandle].fname+i;
                    xQueueSendToBack(fileReceiveQueue,writeNameChar,FILE_SEND_TIMEOUT);
                    i++;
                }
                while (*writeNameChar > 0);
            }
            xQueueSendToBack(fileReceiveQueue,&readFileHandle,FILE_SEND_TIMEOUT);
            if (readFileHandle  < 0xFF)
            {
                TCHAR *readNameChar;
                uint8_t i=0;
                do
                {
                    readNameChar = fileInfo[readFileHandle].fname+i;
                    xQueueSendToBack(fileReceiveQueue,readNameChar,FILE_SEND_TIMEOUT);
                    i++;
                }
                while (*readNameChar > 0);
            }
            fileStatus = 0;
            break;
        }
/* Delete a file. Beware that the file be NOT open when deleting.
Checks the filename and handle for the write and read files, if they are open. */
        case 'X':
        {
            if (! ((writeFileHandle < 0xFF) &&
                stringEqual((char*)line+2, fileInfo[writeFileHandle].fname)) &&
                ! ((readFileHandle < 0xFF) &&
                stringEqual((char*)line+2, fileInfo[readFileHandle].fname)))
            {
                fileStatus = f_unlink(line+2);
            }
            else
                fileStatus = FR_DENIED;
            break;
        }
/* Reinitialize the memory card. */
        case 'M':
        {
            FRESULT fileStatus = f_mount(&Fatfs[0],"",0);
            fileUsable = (fileStatus == FR_OK);
            writeFileHandle = 0xFF;
            readFileHandle = 0xFF;
            uint8_t i=0;
            for (i=0; i<MAX_OPEN_FILES; i++) fileInfo[i].fname[0] = 0;
            filemap = 0;
            break;
        }
    }
/* Return the file status */
    xQueueSendToBack(fileReceiveQueue,&fileStatus,FILE_SEND_TIMEOUT);
}

/*--------------------------------------------------------------------------*/
/* @brief Find a file handle

The file handle map consists of bits set when a handle has been allocated.
This function searches for a free handle.

@returns 255 if no handle was allocated (too many open files), or 0..MAX_OPEN_FILES-1
*/

static uint8_t findFileHandle(void)
{
    uint8_t i=0;
    uint8_t fileHandle=0xFF;
    uint8_t mask = 0;
    while ((i<MAX_OPEN_FILES) && (fileHandle == 0xFF))
    {
        mask = (1 << i);
        if ((filemap & mask) == 0) fileHandle = i;
        i++;
    }
    if (fileHandle < 0xFF) filemap |= mask;
    return fileHandle;
}

/*--------------------------------------------------------------------------*/
/* @brief Delete a file handle

The file handle map consists of bits set when a handle has been allocated.
This function deletes a handle. Does nothing if file handle is not valid.
*/

static void deleteFileHandle(uint8_t fileHandle)
{
    if (fileHandle < MAX_OPEN_FILES)
        filemap &= ~(1 << fileHandle);
}

/*--------------------------------------------------------------------------*/
/** @brief Record a Data Record with One Integer Parameter

The data is recorded to the opened write file.

This is an asynchronous TxPDO in CANopen and therefore is nominally of fixed
length. The response parameters are converted to ASCII integer for debug.

The command is aborted in its entirety if another task is blocking access to
the filesystem. It will return an access denied status if a status byte is not
returned.

@param char* ident: an identifier string.
@param int32_t param1: first parameter.
@param int32_t param2: second parameter.
*/

uint8_t recordSingle(char* ident, int32_t param1)
{
    uint8_t fileStatus;
    if (isRecording() && (writeFileHandle < 0x7F))
    {
        if (xSemaphoreTake(fileSendSemaphore,FILE_SEND_TIMEOUT))
        {
            char record[80];
            record[0] = '0';            /* dummy in case writeFileHandle is zero */
            record[1] = 0;
            stringAppend(record, ident);
            stringAppend(record, ",");
            char buffer[20];
            intToAscii(param1, buffer);
            stringAppend(record, buffer);
            stringAppend(record, "\r\n");
            uint8_t length = stringLength(record);
            record[0] = writeFileHandle;
            if (sendFileCommand('P',length, (uint8_t*)record))
                xQueueReceive(fileReceiveQueue,&fileStatus,FILE_SEND_TIMEOUT);
            xSemaphoreGive(fileSendSemaphore);
        }
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Record a Data Record with Two Integer Parameters

The data is recorded to the opened write file.

This is an asynchronous TxPDO in CANopen and therefore is nominally of fixed
length. The response parameters are converted to ASCII integer for debug.

The command is aborted in its entirety if another task is blocking access to
the filesystem. It will return an access denied status if a status byte is not
returned.

@param char* ident: an identifier string.
@param int32_t param1: first parameter.
@param int32_t param2: second parameter.
@returns uint8_t file status.
*/

uint8_t recordDual(char* ident, int32_t param1, int32_t param2)
{
    uint8_t fileStatus;
    if (isRecording() && (writeFileHandle < 0x7F))
    {
        if (xSemaphoreTake(fileSendSemaphore,FILE_SEND_TIMEOUT))
        {
            char record[80];
            record[0] = '0';
            record[1] = 0;
            stringAppend(record, ident);
            stringAppend(record, ",");
            char buffer[20];
            intToAscii(param1, buffer);
            stringAppend(record, buffer);
            stringAppend(record, ",");
            intToAscii(param2, buffer);
            stringAppend(record, buffer);
            stringAppend(record, "\r\n");
            uint8_t length = stringLength(record);
            record[0] = writeFileHandle;
            if (sendFileCommand('P',length, (uint8_t*)record))
                xQueueReceive(fileReceiveQueue,&fileStatus,FILE_SEND_TIMEOUT);
            xSemaphoreGive(fileSendSemaphore);
        }
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Record a Data Record with a String Parameter

The string is recorded to the opened write file.

This is an asynchronous TxPDO in CANopen and therefore is nominally of fixed
length. The response parameters are converted to ASCII integer for debug.

The command is aborted in its entirety if another task is blocking access to
the filesystem. It will return an access denied status if a status byte is not
returned.

@param char* ident: an identifier string.
@param int32_t param1: first parameter.
@param int32_t param2: second parameter.
*/

uint8_t recordString(char* ident, char* string)
{
    uint8_t fileStatus = FR_DENIED;
    if (isRecording() && (writeFileHandle < 0x7F))
    {
        if (xSemaphoreTake(fileSendSemaphore,FILE_SEND_TIMEOUT))
        {
            char record[80];
            record[0] = '0';
            record[1] = 0;
            stringAppend(record, ident);
            stringAppend(record, ",");
            stringAppend(record, string);
            stringAppend(record, "\r\n");
            uint8_t length = stringLength(record);
            record[0] = writeFileHandle;
            if (sendFileCommand('P',length, (uint8_t*)record))
                xQueueReceive(fileReceiveQueue,&fileStatus,FILE_SEND_TIMEOUT);
            xSemaphoreGive(fileSendSemaphore);
        }
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Send a Command String

A convenience API function used to facilitate formation and queueing of commands.
All commands are a single character followed by the length of the parameters.

The command is aborted in its entirety if there is insufficient space available
in the queue.

@param[in] char command: Command to be sent.
@param[in] uint8_t length: Length of parameter set only.
@param[in] uint8_t *parameters: Parameter list.
@returns true if the command succeeded.
*/

bool sendFileCommand(char command, uint8_t length, uint8_t *parameters)
{
    uint8_t i;
    uint8_t totalLength = length+2;
    if (uxQueueSpacesAvailable(fileSendQueue) >= totalLength)
    {
        if (! xQueueSendToBack(fileSendQueue,&command,FILE_SEND_TIMEOUT))
            return false;
        if (! xQueueSendToBack(fileSendQueue,&totalLength,FILE_SEND_TIMEOUT))
            return false;
        for (i=0; i<length; i++)
            if (! xQueueSendToBack(fileSendQueue,parameters+i,FILE_SEND_TIMEOUT))
                return false;
    }
    return true;
}

