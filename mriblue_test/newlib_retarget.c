/* Copyright 2022 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <core/mri.h>


/* Turn off the errno macro and use actual external global variable instead. */
#undef errno
extern int errno;


/* Structure used by GDB to communicate the stats for a file on the host. Each 32-bit word is returned in big endian
   format.
*/
typedef struct
{
    uint32_t    device;
    uint32_t    inode;
    uint32_t    mode;
    uint32_t    numberOfLinks;
    uint32_t    userId;
    uint32_t    groupId;
    uint32_t    deviceType;
    uint32_t    totalSizeUpperWord;
    uint32_t    totalSizeLowerWord;
    uint32_t    blockSizeUpperWord;
    uint32_t    blockSizeLowerWord;
    uint32_t    blockCountUpperWord;
    uint32_t    blockCountLowerWord;
    uint32_t    lastAccessTime;
    uint32_t    lastModifiedTime;
    uint32_t    lastChangeTime;
} GdbStats;

/* Flag set to flag that MRI hooks should be disabled when writing to stdout. */
volatile int g_mriDisableHooks = 0;


static uint32_t extractWordFromBigEndianWord(const void* pBigEndianValueToExtract);


/* File system related syscalls. */
int _open(const char *pFilename, int flags, int mode)
{
    return mriNewLib_SemihostOpen(pFilename, flags, mode);
}

int rename(const char *pOldFilename, const char *pNewFilename)
{
    return mriNewLib_SemihostRename(pOldFilename, pNewFilename);
}

int _unlink(const char *pFilename)
{
    return mriNewLib_SemihostUnlink(pFilename);
}

int _fstat(int file, struct stat *pStat)
{
    GdbStats gdbStats;

    memset(&gdbStats, 0, sizeof(gdbStats));
    int result = mriNewlib_SemihostFStat(file, &gdbStats);

    pStat->st_dev = extractWordFromBigEndianWord(&gdbStats.device);
    pStat->st_ino = extractWordFromBigEndianWord(&gdbStats.inode);
    pStat->st_mode = extractWordFromBigEndianWord(&gdbStats.mode);
    pStat->st_nlink = extractWordFromBigEndianWord(&gdbStats.numberOfLinks);
    pStat->st_uid = extractWordFromBigEndianWord(&gdbStats.userId);
    pStat->st_gid = extractWordFromBigEndianWord(&gdbStats.groupId);
    pStat->st_rdev = extractWordFromBigEndianWord(&gdbStats.deviceType);
    pStat->st_size = extractWordFromBigEndianWord(&gdbStats.totalSizeLowerWord);
    pStat->st_atim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastAccessTime);
    pStat->st_mtim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastModifiedTime);
    pStat->st_ctim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastChangeTime);
    pStat->st_blksize = extractWordFromBigEndianWord(&gdbStats.blockSizeLowerWord);
    pStat->st_blocks = extractWordFromBigEndianWord(&gdbStats.blockCountLowerWord);

    return result;
}

int _stat(const char *pFilename, struct stat *pStat)
{
    GdbStats gdbStats;

    memset(&gdbStats, 0, sizeof(gdbStats));
    int result = mriNewLib_SemihostStat(pFilename, &gdbStats);

    pStat->st_dev = extractWordFromBigEndianWord(&gdbStats.device);
    pStat->st_ino = extractWordFromBigEndianWord(&gdbStats.inode);
    pStat->st_mode = extractWordFromBigEndianWord(&gdbStats.mode);
    pStat->st_nlink = extractWordFromBigEndianWord(&gdbStats.numberOfLinks);
    pStat->st_uid = extractWordFromBigEndianWord(&gdbStats.userId);
    pStat->st_gid = extractWordFromBigEndianWord(&gdbStats.groupId);
    pStat->st_rdev = extractWordFromBigEndianWord(&gdbStats.deviceType);
    pStat->st_size = extractWordFromBigEndianWord(&gdbStats.totalSizeLowerWord);
    pStat->st_atim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastAccessTime);
    pStat->st_mtim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastModifiedTime);
    pStat->st_ctim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastChangeTime);
    pStat->st_blksize = extractWordFromBigEndianWord(&gdbStats.blockSizeLowerWord);
    pStat->st_blocks = extractWordFromBigEndianWord(&gdbStats.blockCountLowerWord);
    return result;
}

static uint32_t extractWordFromBigEndianWord(const void* pBigEndianValueToExtract)
{
    const unsigned char* pBigEndianValue = (const unsigned char*)pBigEndianValueToExtract;
    return pBigEndianValue[3]        | (pBigEndianValue[2] << 8) |
          (pBigEndianValue[1] << 16) | (pBigEndianValue[0] << 24);
}


/* File handle related syscalls. */
int _read(int file, char *ptr, int len)
{
    return mriNewlib_SemihostRead(file, ptr, len);
}

int _write(int file, char *ptr, int len)
{
    const int STDOUT_FILE_NO = 1;

    if (file == STDOUT_FILE_NO)
    {
        g_mriDisableHooks = 1;
    }

    int result = mriNewlib_SemihostWrite(file, ptr, len);

    if (file == STDOUT_FILE_NO)
    {
        g_mriDisableHooks = 0;
    }
    return result;
}

int _isatty(int file)
{
    if (file >= 0 && file <= 2)
        return 1;
    return 0;
}

int _lseek(int file, int ptr, int dir)
{
    return mriNewlib_SemihostLSeek(file, ptr, dir);
}

int _close(int file)
{
    return mriNewlib_SemihostClose(file);
}
