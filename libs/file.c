/* File Management.

File management functions are provided.

K. Sarkies, 10 December 2016
*/

/*
 * Copyright (C) K. Sarkies <ksarkies@internode.on.net>
 *
 * This project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ff.h"
#include "file.h"
#include "buffer.h"
#include "comms.h"
#include "hardware.h"
#include "stringlib.h"

#define  _BV(bit) (1 << (bit))

/*--------------------------------------------------------------------------*/
/* Globals */

/* Local Variables */
/* ChaN FAT */
static FATFS Fatfs[_VOLUMES];
static FATFS* fs;		            /* File system object for logical drive 0 */
static FIL file[MAX_OPEN_FILES];    /* file descriptions. */
static FILINFO fileInfo[MAX_OPEN_FILES];    /* file information (open files) */
static bool fileSystemUsable;
static uint8_t filemap;             /* map of open file handles */

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

static uint8_t find_file_handle(void);
static void delete_file_handle(uint8_t fileHandle);
/*--------------------------------------------------------------------------*/
/* Helpers */
/*--------------------------------------------------------------------------*/
/** @brief File System Usable Check

The global variable is set whenever a system mount or create is done, and when
the file system status is requested.
*/

bool file_system_usable(void)
{
    return fileSystemUsable;
}

/*--------------------------------------------------------------------------*/
/** @brief File Initialization

The FreeRTOS queue and semaphore are initialised. The file system work area is
initialised.
*/

uint8_t init_file_system(void)
{
/* initialise the drive working area */
    FRESULT fileStatus = f_mount(&Fatfs[0],"",0);
    fileSystemUsable = (fileStatus == FR_OK);

/* Initialise some global variables */
    uint8_t i=0;
    for (i=0; i<MAX_OPEN_FILES; i++) fileInfo[i].fname[0] = 0;
    filemap = 0;
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Create a filesystem on the drive.

A FAT32 file system with sector size 512 bytes and cluster size 8 in common
with most SD memory cards.

NOTE: _USE_MKFS must be set to 1 in ffconf.h

@param[out] uint32_t: number of free clusters.
@param[out] uint32_t: cluster size in sectors.
@returns uint8_t: status of operation.
*/

#define SECTOR_SIZE _MIN_SS
#define CLUSTER_SIZE 8
#define WORKSPACE_SIZE SECTOR_SIZE*CLUSTER_SIZE

uint8_t make_filesystem(void)
{
    uint8_t workspace[WORKSPACE_SIZE];
    FRESULT fileStatus = FR_DENIED;
#if _USE_MKFS
    fileStatus = f_mkfs("", FM_FAT32, SECTOR_SIZE*CLUSTER_SIZE,
                            workspace, WORKSPACE_SIZE);
#endif
    fileSystemUsable = (fileStatus == FR_OK);
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Read the free space on the drive.

Sector size is nearly always 512 bytes and is limited in ffconf.h to that value.

@param[out] uint32_t: number of free clusters.
@param[out] uint32_t: cluster size in sectors.
@returns uint8_t: status of operation.
*/

uint8_t get_free_clusters(uint32_t* freeClusters, uint32_t* clusterSize)
{
    FRESULT fileStatus = f_getfree("", (DWORD*)freeClusters, &fs);
    *clusterSize = fs->csize;
    fileSystemUsable = (fileStatus == FR_OK);
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Read a directory entry.

@param[in] char*: directory name
@param[out] char*: type of entry (d = directory, f = file, n = error, e = end).
@param[out] uint32_t*: file size.
@param[out] char*: file name
@returns uint8_t: status of operation.
*/

uint8_t read_directory_entry(char* directoryName, char* type, uint32_t* size,
                             char* fileName)
{
    FRESULT fileStatus = FR_OK;
    static DIR directory;
    FILINFO fileInfo;
    if (directoryName[0] != 0) fileStatus = f_opendir(&directory, directoryName);
    if (fileStatus == FR_OK)
        fileStatus = f_readdir(&directory, &fileInfo);
    string_copy(fileName, fileInfo.fname);
    *size = fileInfo.fsize;
    *type = 'f';
    if (fileInfo.fattrib == AM_DIR) *type = 'd';
    if (fileInfo.fname[0] == 0)
    {
        *type = 'e';
        *size = 0;
    }
    if (fileStatus != FR_OK)
    {
        fileName[0] = 0;
        *type = 'n';
        *size = 0;
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Open a file for Read Only.

A valid filename must be provided. The file is opened if it exists and a file
handle is obtained and returned. The file handle is used to access file
information for opened files. A value of 0xFF indicates that a file has not
been opened. If the handle passed is already allocated, the function returns
with an ACCESS DENIED error.

Globals:
file[] an array of opened file object structures defined by ChaN FAT FS.
fileInfo[] an array of file information on open files.

@param[in] char*: file name
@param[out] uint8_t*: file handle.
@returns uint8_t: status of operation.
*/

uint8_t open_read_file(char* fileName, uint8_t* readFileHandle)
{
    FRESULT fileStatus = FR_OK;
    uint8_t fileHandle = 0xFF;
/* Null file name */
    if (fileName[0] == 0)
        fileStatus = FR_INVALID_OBJECT;
/* A file is already open */
    else if (*readFileHandle < 0xFF)
        fileStatus = FR_DENIED;
    else
    {
        fileHandle = find_file_handle();
/* Unable to be allocated */
        if (fileHandle >= MAX_OPEN_FILES)
            fileStatus = FR_TOO_MANY_OPEN_FILES;
        else
        {
/* Try to open a file read only */
            fileStatus = f_open(&file[fileHandle], fileName, \
                                FA_OPEN_EXISTING | FA_READ);
            if (fileStatus == FR_OK)
                fileStatus = f_stat(fileName, fileInfo+fileHandle);
            if (fileStatus != FR_OK)
            {
                delete_file_handle(fileHandle);
                fileHandle = 0xFF;
            }
            *readFileHandle = fileHandle;
        }
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Open a file for Writing.

A valid filename must be provided. The file is opened or created and a file
handle is obtained and returned. The file handle is used to access file
information for opened files. A value of 0xFF indicates that a file has not
been opened. If the handle passed is already allocated, the function returns
with an ACCESS DENIED error.

Globals:
file[] an array of opened file object structures defined by ChaN FAT FS.
fileInfo[] an array of file information on open files.

@param[in] char*: file name
@param[out] uint8_t*: file handle.
@returns uint8_t: status of operation.
*/

uint8_t open_write_file(char* fileName, uint8_t* writeFileHandle)
{
    FRESULT fileStatus = FR_OK;
    uint8_t fileHandle = 0xFF;
/* Null file name */
    if (fileName[0] == 0)
        fileStatus = FR_INVALID_OBJECT;
/* A file is already open */
    else if (*writeFileHandle < 0xFF)
        fileStatus = FR_DENIED;
    else
    {
        fileHandle = find_file_handle();
/* Unable to be allocated */
        if (fileHandle >= MAX_OPEN_FILES)
            fileStatus = FR_TOO_MANY_OPEN_FILES;
        else
        {
/* Try to open a file write/read, creating it if necessary.
Skip to the end of the file to append. */
            fileStatus = f_open(&file[fileHandle], fileName, \
                                FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
/* Check existence of file and get information array entry. */
            if (fileStatus == FR_OK)
                fileStatus = f_lseek(&file[fileHandle], f_size(file));
            if (fileStatus != FR_OK)
            {
                delete_file_handle(fileHandle);
                fileHandle = 0xFF;
            }
            if (fileStatus == FR_OK)
                f_stat(fileName, fileInfo+fileHandle);
            *writeFileHandle = fileHandle;
        }
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Delete a file.

A valid filename must be provided. Checks the filename against all open
file names. If a match is found then the file is not deleted.

Globals:
file[] an array of opened file object structures defined by ChaN FAT FS.
fileInfo[] an array of file information on open files.

@param[in] char*: file name
@returns uint8_t: status of operation.
*/

uint8_t delete_file(char* fileName)
{
    FRESULT fileStatus = FR_OK; 
    uint8_t fileHandle;
    bool ok = true;
    for (fileHandle = 0; fileHandle < 8; fileHandle++)
        if (valid_file_handle(fileHandle) &&
            (string_equal(fileName, fileInfo[fileHandle].fname))) ok = false;
    if (ok)
    {
        fileStatus = f_unlink(fileName);
    }
    else
        fileStatus = FR_DENIED;
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Read Block from an Open file.

Get data from a file from the last position that data was read after opening.
The read/write pointer in the file object is advanced as data is read.
Returns the number read and binary byte-wise data. The number read will
differ from the number requested if EOF is reached.

Globals:
file[] an array of opened file object structures defined by ChaN FAT FS.
fileInfo[] an array of file information on open files.

@param[in] uint8_t: file handle.
@param[in] uint8_t*: length of the data block to be read.
@param[in] uint8_t*: data block of maximum length 80 bytes
@returns uint8_t: status of operation.
*/

uint8_t read_block_from_file(uint8_t fileHandle, uint8_t* blockLength, uint8_t* data)
{
    FRESULT fileStatus = FR_OK;
    UINT numRead = 0;
    if (*blockLength < 82)
        fileStatus = f_read(&file[fileHandle],data,*blockLength,&numRead);
    else fileStatus = FR_INVALID_PARAMETER;
    *blockLength = numRead;
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Read Line from an Open file.

Get data from a file from the last position that data was read after opening.
The read/write pointer in the file object is advanced as data is read. Data is
read into a string until a carriage return is encountered. Line feeds are
stripped out.

NOTE: set _USE_STRFUNC = 1 or 2 in ffconf.h

Globals:
file[] an array of opened file object structures defined by ChaN FAT FS.
fileInfo[] an array of file information on open files.

@param[in] uint8_t: file handle.
@param[in] char*: string of maximum length 80 bytes
@returns uint8_t: status of operation.
*/

uint8_t read_line_from_file(uint8_t fileHandle, char* string)
{
    FRESULT fileStatus = FR_OK;
    char ch[2];                         /* Buffer for character */
    if (valid_file_handle(fileHandle))
    {
        UINT numRead = 0;
        uint8_t i = 0;
        do
        {
            fileStatus = f_read(&file[fileHandle],ch,1,&numRead);
            if (ch[0] != '\a')          /* Strip line feeds */
                string[i++] = ch[0];
        }
        while ((fileStatus == FR_OK) && (ch[0] != '\n'));
        if (fileStatus == FR_OK) string[i] = 0;
        else  string[0] = 0;
    }
    else fileStatus = FR_INVALID_PARAMETER;
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Write to an Open Write file.

Store data to the file starting at the end of the file.

Nothing is written if the data block is longer than 80 bytes. The number of
bytes to write is given by the length less 4. The block length parameter returns
the actual number written. The number written will be less than from the number
requested if the disk is full. 

Globals:
file[] an array of opened file object structures defined by ChaN FAT FS.
fileInfo[] an array of file information on open files.

@param[in] uint8_t: file handle.
@param[in] uint8_t*: length of the data block to be written.
@param[in] uint8_t*: data block of maximum length 80 bytes
@returns uint8_t: status of operation.
*/

uint8_t write_to_file(uint8_t fileHandle, uint8_t* blockLength, uint8_t* data)
{
    FRESULT fileStatus = FR_OK;
    UINT numWritten = 0;
    if (*blockLength < 82)
    {
        fileStatus = f_write(&file[fileHandle],data,*blockLength,&numWritten);
        if (numWritten != *blockLength)
        {
            fileStatus = FR_DENIED;
        }
/* Flush the cached data to the storage medium */
        if (fileStatus == FR_OK) f_sync(&file[fileHandle]);
    }
    else fileStatus = FR_INVALID_PARAMETER;
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Close a file.

A valid file handle must be provided.

Globals:
file[] an array of opened file object structures defined by ChaN FAT FS.
fileInfo[] an array of file information on open files.

@param[in] uint8_t*: file handle. This is returned as 0xFF if the deletion
                        is successful, otherwise unchanged.
@returns uint8_t: status of operation.
*/

uint8_t close_file(uint8_t* fileHandle)
{
    FRESULT fileStatus = FR_OK;
    if (*fileHandle >= MAX_OPEN_FILES)
    {
        fileStatus = FR_INVALID_OBJECT;
    }
    else if (! valid_file_handle(*fileHandle))
    {
        fileStatus = FR_INVALID_OBJECT;
    }
    else
    {
/* Close the file and delete the handle. */
        fileInfo[*fileHandle].fname[0] = 0;
        delete_file_handle(*fileHandle);
        fileStatus = f_close(&file[*fileHandle]);
        *fileHandle = 0xFF;
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Get filename for a file handle

A file handle is valid for open files. Returns the file name or a null string
if the file handle is not valid.

@param[in] uint8_t: file handle.
@param[out] char* file name string.
*/

void get_file_name(uint8_t fileHandle, char* fileName)
{
    if (valid_file_handle(fileHandle))
        string_copy(fileName, fileInfo[fileHandle].fname);
    else
        fileName[0] = 0;
}

/*--------------------------------------------------------------------------*/
/** @brief Find a file handle

The file handle map consists of bits set when a handle has been allocated.
This function searches for a free handle.

@returns uint8_t 0..MAX_OPEN_FILES-1 or 0xFF if no handle was allocated (too
many open files).
*/

static uint8_t find_file_handle(void)
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
/** @brief Validate a file handle

The file handle map consists of bits set when a handle has been allocated.
This function checks if the file handle refers to an open file.

@param fileHandle: uint8_t the handle for the file to be deleted.
*/

bool valid_file_handle(uint8_t fileHandle)
{
    return (filemap & (1 << fileHandle));
}

/*--------------------------------------------------------------------------*/
/** @brief Delete a file handle

The file handle map consists of bits set when a handle has been allocated.
This function deletes a handle. Does nothing if file handle is not valid.

@param fileHandle: uint8_t the handle for the file to be deleted.
*/

static void delete_file_handle(uint8_t fileHandle)
{
    if (fileHandle < MAX_OPEN_FILES)
        filemap &= ~(1 << fileHandle);
}

/*--------------------------------------------------------------------------*/
/** @brief Record a Data Record with One Integer Parameter

The data is recorded to an opened write file.

@param[in] char* ident: an identifier string.
@param[in] int32_t param1: first parameter.
@param[in] uint8_t* writeFileHandle: File handle for an open writeable file.
@returns uint8_t file status.
*/

uint8_t record_single(char* ident, int32_t param1, uint8_t writeFileHandle)
{
    uint8_t fileStatus = FR_DENIED;
    if (writeFileHandle < 0x7F)
    {
        char record[80];
        string_clear(record);
        string_append(record, ident);
        string_append(record, ",");
        char buffer[20];
        int_to_ascii(param1, buffer);
        string_append(record, buffer);
        string_append(record, "\r\n");
        uint8_t length = string_length(record);
        fileStatus = write_to_file(writeFileHandle, &length, (uint8_t*) record);
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Record a Data Record with Two Integer Parameters

The data is recorded to an opened write file.

@param[in] char* ident: an identifier string.
@param[in] int32_t param1: first parameter.
@param[in] int32_t param2: second parameter.
@param[in] uint8_t* writeFileHandle: File handle for an open writeable file.
@returns uint8_t file status.
*/

uint8_t record_dual(char* ident, int32_t param1, int32_t param2, uint8_t writeFileHandle)
{
    uint8_t fileStatus = FR_DENIED;
    if (writeFileHandle < 0x7F)
    {
        char record[80];
        string_clear(record);
        string_append(record, ident);
        string_append(record, ",");
        char buffer[20];
        int_to_ascii(param1, buffer);
        string_append(record, buffer);
        string_append(record, ",");
        int_to_ascii(param2, buffer);
        string_append(record, buffer);
        string_append(record, "\r\n");
        uint8_t length = string_length(record);
        fileStatus = write_to_file(writeFileHandle, &length, (uint8_t*) record);
    }
    return fileStatus;
}

/*--------------------------------------------------------------------------*/
/** @brief Record a Data Record with a String Parameter

The string is recorded to the opened write file.

@param[in] char* ident: an identifier string.
@param[in] char* string: string to record.
@param[in] uint8_t* writeFileHandle: File handle for an open writeable file.
@returns uint8_t file status.
*/

uint8_t record_string(char* ident, char* string, uint8_t writeFileHandle)
{
    uint8_t fileStatus = FR_DENIED;
    if (writeFileHandle < 0x7F)
    {
        char record[80];
        string_clear(record);
        string_append(record, ident);
        string_append(record, ",");
        string_append(record, string);
        string_append(record, "\r\n");
        uint8_t length = string_length(record);
        fileStatus = write_to_file(writeFileHandle, &length, (uint8_t*) record);
    }
    return fileStatus;
}

