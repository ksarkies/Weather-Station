/* STM32F1 Weather Station for Solar Power. File Management.

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
 * This file is part of the Battery Management System project.
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

/* Global Variables */
/* ChaN FAT */
static FATFS Fatfs[_VOLUMES];
static FATFS *fs;		            /* File system object for logical drive 0 */
static FIL file[MAX_OPEN_FILES];    /* file descriptions, 2 files maximum. */
static FILINFO fileInfo[MAX_OPEN_FILES];
static bool fileUsable;
static uint8_t filemap=0;           /* map of open file handles */

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
    uint8_t i=0;
    for (i=0; i<MAX_OPEN_FILES; i++) fileInfo[i].fname[0] = 0;
    filemap = 0;
}

/*--------------------------------------------------------------------------*/
/** @brief Open a File for writing and reading.

@param[in] char* 8 character filename, plus dot plus 3 character extension.
@param[out] file handle. On error file handle is 0xFF.
@returns FRESULT* file status error code.
*/

FRESULT open_write_file(char* filename, uint8_t* fileHandle)
{
    FRESULT fileStatus;
    uint8_t fileHandle = 0xFF;
/* Already open */
    if (readFileHandle < 0xFF)
    {
        fileStatus = FR_DENIED;
        return fileStatus;
    }
    fileHandle = findFileHandle();
/* Unable to be allocated */
    if (fileHandle >= MAX_OPEN_FILES)
    {
        fileStatus = FR_TOO_MANY_OPEN_FILES;
        return fileStatus;
    }
/* Try to open a file write/read, creating it if necessary */
    fileStatus = f_open(&file[fileHandle], line+2, \
                        FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    if (fileStatus != FR_OK)
    {
        deleteFileHandle(fileHandle);
        fileHandle = 0xFF;
        return fileStatus;
    }
/* Skip to the end of the file to start appending data. */
    fileStatus = f_lseek(&file[fileHandle], f_size(file));
    if (fileStatus == FR_OK)
        fileStatus = f_stat(filename, fileInfo+readFileHandle);
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Open a File for reading only.

@param[in] char* 8 character filename, plus dot plus 3 character extension.
@param[out] file handle. On error file handle is 0xFF.
@returns FRESULT* file status error code.
*/

FRESULT open_read_file(char* filename, uint8_t* fileHandle)
{
    FRESULT fileStatus;
    fileHandle = 0xFF;
/* Already open */
    if (readFileHandle < 0xFF)
    {
        fileStatus = FR_DENIED;
        return fileStatus;
    }
    fileHandle = findFileHandle();
/* Unable to be allocated */
    if (fileHandle >= MAX_OPEN_FILES)
    {
        fileStatus = FR_TOO_MANY_OPEN_FILES;
        return fileStatus;
    }
/* Try to open a file read only */
    fileStatus = f_open(&file[fileHandle], filename, FA_OPEN_EXISTING | FA_READ);
    if (fileStatus != FR_OK)
    {
        deleteFileHandle(fileHandle);
        fileHandle = 0xFF;
        return fileStatus;
    }
    fileStatus = f_stat(filename, fileInfo+readFileHandle);
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Close a file.

@param[in] uint8_t* fileHandle: file handle.
@returns[in] FRESULT* file status error code.
*/

FRESULT close_file(uint8_t* fileHandle)
{
    FRESULT fileStatus;
    if (fileHandle >= MAX_OPEN_FILES)
    {
        fileStatus = FR_INVALID_OBJECT;
        return fileStatus;
    }
    if (writeFileHandle == fileHandle) writeFileHandle = 0xFF;
    else if (readFileHandle == fileHandle) readFileHandle = 0xFF;
    else
    {
        fileStatus = FR_INVALID_OBJECT;
        return fileStatus;
    }
    fileInfo[fileHandle].fname[0] = 0;
    deleteFileHandle(fileHandle);
    fileStatus = f_close(&file[fileHandle]);
    fileHandle = 0xFF;
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Store data to the file.

Store data to the file starting at the end of the file. The number written will
differ from that requested if the storage is full.

@param[in] uint8_t* fileHandle: file handle.
@param[in] data block.
@param[in] UINT length: the length of the data block to write.
@param[out] UINT* numWritten.
@returns[in] FRESULT* file status error code.
*/

FRESULT save_to_file(uint8_t* fileHandle, uint8_t* data, UINT length, UINT* numWritten)
{
    fileStatus = f_write(&file[fileHandle],data,length,&numWritten);
    if (fileStatus == FR_OK) f_sync(&file[fileHandle]);
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Read data from the file.

Get data from a file from the last position that data was read after opening.
Returns the number read followed by binary byte-wise data. The number read will
differ from the number requested if EOF reached.

@param[in] uint8_t* fileHandle: file handle.
@param[in] data block.
@param[in] UINT length: the length of the data block to read.
@param[out] UINT* numWritten.
@returns[in] FRESULT* file status error code.
*/

FRESULT read_from_file(uint8_t* fileHandle, uint8_t* data, UINT length, UINT* numRead)
{
    fileStatus = f_read(&file[fileHandle],data,length,&numRead);
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Directory listing.

If a directory name is given, the directory specified is opened and the first
entry is returned. Subsequent calls with zero length directory name will return
subsequent entries. Returns a null terminated file name. At the end, or on
error, a zero length string is sent.

@param[in] uint8_t* dirName: null terminated directory name.
@param[out] uint8_t* fileName: null terminated file name.
@param[out] UINT* type: a type character: f = file, d = directory, e = end.
@param[out] UINT* fileSize.
@returns FRESULT* file status error code.
*/

FRESULT read_directory(uint8_t* dirName, uint8_t* fileName, UINT* type, UINT* fileSize)
{
    static DIR directory;
    FILINFO fileInfo;
    fileStatus = FR_OK;
    if (dirName[0] != 0) fileStatus = f_opendir(&directory, line+2);
    if (fileStatus == FR_OK)
    {
        fileStatus = f_readdir(&directory, &fileInfo);
    }
    char type = 'f';
    if (fileInfo.fattrib == AM_DIR) type = 'd';
    if (fileInfo.fname[0] == 0) type = 'e';
    if (fileStatus != FR_OK)
    {
        fileInfo.fname[0] = 0;
    }
    fileSize = fileInfo.fsize;
    fileName = fileInfo.fname;
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

