/* ModelicaInternal.c - External functions for Modelica.Utilities

   Copyright (C) 2002-2025, Modelica Association and contributors
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Changelog:
      Oct. 20, 2024: by Thomas Beutlich
                     Removed legacy behaviour in ModelicaInternal_stat for
                     MSVC Visual Studio >= 2015 (ticket #4473)

      Jan. 15, 2024: by Thomas Beutlich
                     Utilized ModelicaDuplicateString and
                     ModelicaDuplicateStringWithErrorReturn (ticket #3686)

      Nov. 17, 2020: by Thomas Beutlich
                     Fixed reading files with Unix-style line endings on Windows
                     for ModelicaInternal_readLine/_readFile (ticket #3631)

      Nov. 11, 2020: by Thomas Beutlich
                     Added getcwd fallback in ModelicaInternal_fullPathName if
                     realpath fails for non-existing path (ticket #3660)

      Dec. 02, 2019: by Thomas Beutlich
                     Removed call of localtime in ModelicaInternal_getTime
                     (ticket #3246)

      Nov. 13, 2019: by Thomas Beutlich
                     Utilized blockwise I/O in ModelicaInternal_copyFile
                     (ticket #3229)

      Oct. 10, 2019: by Thomas Beutlich
                     Fixed month and year correction in ModelicaInternal_getTime
                     (ticket #3143)

      Jun. 24, 2019: by Thomas Beutlich
                     Fixed uninitialized memory and realpath behaviour in
                     ModelicaInternal_fullPathName (ticket #3003)

      Jun. 28, 2018: by Hans Olsson, Dassault Systemes
                     Proper error message when out of string memory
                     in ModelicaInternal_readLine (ticket #2676)

      Oct. 23, 2017: by Thomas Beutlich, ESI ITI GmbH
                     Utilized non-fatal hash insertion, called by HASH_ADD_KEYPTR in
                     function CacheFileForReading (ticket #2097)

      Apr. 09, 2017: by Thomas Beutlich, ESI ITI GmbH
                     Fixed macOS support of ModelicaInternal_setenv
                     (ticket #2235)

      Mar. 27, 2017: by Thomas Beutlich, ESI ITI GmbH
                     Replaced localtime by re-entrant function

      Jan. 31, 2017: by Thomas Beutlich, ESI ITI GmbH
                     Fixed WIN32 support of a directory name with a trailing
                     forward/backward slash character in ModelicaInternal_stat
                     (ticket #1976)

      Mar. 02, 2016: by Thomas Beutlich, ITI GmbH
                     Fixed repeated opening of cached file in case of line miss in
                     ModelicaStreams_openFileForReading (ticket #1939)

      Dec. 10, 2015: by Martin Otter, DLR
                     Added flags NO_PID and NO_TIME (ticket #1805)

      Oct. 27, 2015: by Thomas Beutlich, ITI GmbH
                     Added nonnull attributes/annotations (ticket #1436)

      Oct. 05, 2015: by Thomas Beutlich, ITI GmbH
                     Added functions ModelicaInternal_getpid/_getTime from
                     ModelicaRandom.c of https://github.com/DLR-SR/Noise
                     (ticket #1662)

      Nov. 20, 2014: by Thomas Beutlich, ITI GmbH
                     Fixed platform dependency of ModelicaInternal_readLine/_readFile
                     (ticket #1580)

      Aug. 22, 2014: by Thomas Beutlich, ITI GmbH
                     Fixed multi-threaded access of common/shared file cache
                     (ticket #1556)

      Aug. 11, 2014: by Thomas Beutlich, ITI GmbH
                     Increased cache size of opened files and made it
                     thread-safe (ticket #1433)
                     Made getenv/putenv thread-safe for Visual Studio 2005 and
                     later (ticket #1433)

      May 21, 2013:  by Martin Otter, DLR
                     Included the improvements from DS Lund (ticket #1104):
                     - Changed implementation of print to do nothing in case of
                       missing file-system. Otherwise we just end up with an
                       error message that is not written, and the failure in
                       itself is not sufficiently fatal to just stop.
                     - Caching when reading from file

      Mar, 26, 2013: by Martin Otter, DLR
                     Changed type of variable valueStart from int to size_t
                     (ticket #1032)

      Jan. 05, 2013: by Martin Otter, DLR
                     Removed "static" declarations from the Modelica interface
                     functions

      Sep. 26, 2004: by Martin Otter, DLR
                     Added missing implementations, merged code from previous
                     ModelicaFiles and clean-up of code

      Sep. 09, 2004: by Dag Brueck, Dynasim AB
                     Further implementation and clean-up of code

      Aug. 24, 2004: by Martin Otter, DLR
                     Adapted to Dymola 5.3 with minor improvements

      Jan. 07, 2002: by Martin Otter, DLR
                     First version implemented:
                     Only tested for _WIN32, but implemented all
                     functions also for _POSIX_, with the exception of
                     ModelicaInternal_getFullPath
*/

#if defined(__gnu_linux__) && !defined(NO_FILE_SYSTEM)
#define _GNU_SOURCE 1
#endif

#include "ModelicaInternal.h"
#include "ModelicaUtilities.h"

/*
  ModelicaNotExistError never returns to the caller. In order to compile
  external Modelica C-code in most compilers, noreturn attributes need to
  be present to avoid warnings or errors.

  The following macros handle noreturn attributes according to the
  C11/C++11 standard with fallback to GNU, Clang or MSVC extensions if using
  an older compiler.
*/
#undef MODELICA_NORETURN
#undef MODELICA_NORETURNATTR
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define MODELICA_NORETURN _Noreturn
#define MODELICA_NORETURNATTR
#elif defined(__cplusplus) && __cplusplus >= 201103L
#if (defined(__GNUC__) && __GNUC__ >= 5) || \
    (defined(__GNUC__) && defined(__GNUC_MINOR__) && __GNUC__ == 4 && __GNUC_MINOR__ >= 8)
#define MODELICA_NORETURN [[noreturn]]
#define MODELICA_NORETURNATTR
#elif (defined(__GNUC__) && __GNUC__ >= 3) || \
      (defined(__GNUC__) && defined(__GNUC_MINOR__) && __GNUC__ == 2 && __GNUC_MINOR__ >= 8)
#define MODELICA_NORETURN
#define MODELICA_NORETURNATTR __attribute__((noreturn))
#elif defined(__GNUC__)
#define MODELICA_NORETURN
#define MODELICA_NORETURNATTR
#else
#define MODELICA_NORETURN [[noreturn]]
#define MODELICA_NORETURNATTR
#endif
#elif defined(__clang__)
/* Encapsulated for Clang since GCC fails to process __has_attribute */
#if __has_attribute(noreturn)
#define MODELICA_NORETURN
#define MODELICA_NORETURNATTR __attribute__((noreturn))
#else
#define MODELICA_NORETURN
#define MODELICA_NORETURNATTR
#endif
#elif (defined(__GNUC__) && __GNUC__ >= 3) || \
      (defined(__GNUC__) && defined(__GNUC_MINOR__) && __GNUC__ == 2 && __GNUC_MINOR__ >= 8) || \
      (defined(__SUNPRO_C) && __SUNPRO_C >= 0x5110)
#define MODELICA_NORETURN
#define MODELICA_NORETURNATTR __attribute__((noreturn))
#elif (defined(_MSC_VER) && _MSC_VER >= 1200) || \
       defined(__BORLANDC__)
#define MODELICA_NORETURN __declspec(noreturn)
#define MODELICA_NORETURNATTR
#else
#define MODELICA_NORETURN
#define MODELICA_NORETURNATTR
#endif

MODELICA_NORETURN static void ModelicaNotExistError(const char* name) MODELICA_NORETURNATTR;
static void ModelicaNotExistError(const char* name) {
  /* Print error message if a function is not implemented */
    ModelicaFormatError("C-Function \"%s\" is called\n"
        "but is not implemented for the actual environment\n"
        "(e.g., because there is no file system available on the machine\n"
        "as for dSPACE or xPC systems)", name);
}

#undef MODELICA_NORETURN
#undef MODELICA_NORETURNATTR

#ifdef NO_FILE_SYSTEM
void ModelicaInternal_mkdir(_In_z_ const char* directoryName) {
    ModelicaNotExistError("ModelicaInternal_mkdir"); }
void ModelicaInternal_rmdir(_In_z_ const char* directoryName) {
    ModelicaNotExistError("ModelicaInternal_rmdir"); }
int ModelicaInternal_stat(_In_z_ const char* name) {
    ModelicaNotExistError("ModelicaInternal_stat"); return 0; }
void ModelicaInternal_rename(_In_z_ const char* oldName,
    _In_z_ const char* newName) {
    ModelicaNotExistError("ModelicaInternal_rename"); }
void ModelicaInternal_removeFile(_In_z_ const char* file) {
    ModelicaNotExistError("ModelicaInternal_removeFile"); }
void ModelicaInternal_copyFile(_In_z_ const char* oldFile,
    _In_z_ const char* newFile) {
    ModelicaNotExistError("ModelicaInternal_copyFile"); }
void ModelicaInternal_readDirectory(_In_z_ const char* directory,
    int nFiles, _Out_ const char** files) {
    ModelicaNotExistError("ModelicaInternal_readDirectory"); }
int ModelicaInternal_getNumberOfFiles(_In_z_ const char* directory) {
    ModelicaNotExistError("ModelicaInternal_getNumberOfFiles"); return 0; }
const char* ModelicaInternal_fullPathName(_In_z_ const char* name) {
    ModelicaNotExistError("ModelicaInternal_fullPathName"); return NULL; }
const char* ModelicaInternal_temporaryFileName(void) {
    ModelicaNotExistError("ModelicaInternal_temporaryFileName"); return NULL; }
void ModelicaStreams_closeFile(_In_z_ const char* fileName) {
    ModelicaNotExistError("ModelicaStreams_closeFile"); }
void ModelicaInternal_print(_In_z_ const char* string,
    _In_z_ const char* fileName) {
    if ( fileName[0] == '\0' ) {
      /* Write string to terminal */
        ModelicaFormatMessage("%s\n", string);
    }
    return; }
int ModelicaInternal_countLines(_In_z_ const char* fileName) {
    ModelicaNotExistError("ModelicaInternal_countLines"); return 0; }
void ModelicaInternal_readFile(_In_z_ const char* fileName,
    _Out_ const char** string, size_t nLines) {
    ModelicaNotExistError("ModelicaInternal_readFile"); }
const char* ModelicaInternal_readLine(_In_z_ const char* fileName,
    int lineNumber, _Out_ int* endOfFile) {
    ModelicaNotExistError("ModelicaInternal_readLine"); return NULL; }
void ModelicaInternal_chdir(_In_z_ const char* directoryName) {
    ModelicaNotExistError("ModelicaInternal_chdir"); }
const char* ModelicaInternal_getcwd(int dummy) {
    ModelicaNotExistError("ModelicaInternal_getcwd"); return NULL; }
void ModelicaInternal_getenv(_In_z_ const char* name, int convertToSlash,
    _Out_ const char** content, _Out_ int* exist) {
    ModelicaNotExistError("ModelicaInternal_getenv"); }
void ModelicaInternal_setenv(_In_z_ const char* name,
    _In_z_ const char* value, int convertFromSlash) {
    ModelicaNotExistError("ModelicaInternal_setenv"); }
#else

/* The standard way to detect POSIX is to check _POSIX_VERSION,
 * which is defined in <unistd.h>
 */
#if defined(__unix__) || defined(__linux__) || defined(__APPLE_CC__)
  #include <unistd.h>
#endif
#if !defined(_POSIX_) && defined(_POSIX_VERSION)
  #define _POSIX_ 1
#endif

#include "stdint_wrap.h"
#define HASH_NO_STDINT 1
#define HASH_NONFATAL_OOM 1
#include "uthash.h"
#include "g2constructor.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#if defined(__WATCOMC__)
  #include <direct.h>
  #include <sys/types.h>
  #include <sys/stat.h>
#elif defined(__BORLANDC__)
  #include <direct.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <dirent.h>
#elif defined(_WIN32)
  #include <direct.h>
  #include <sys/types.h>
  #include <sys/stat.h>

  #if defined(__MINGW32__) || defined(__CYGWIN__) /* MinGW and Cygwin have dirent.h */
    #include <dirent.h>
  #else /* include the opendir/readdir/closedir interface for _WIN32 */
    #include "win32_dirent.h"
  #endif

#elif defined(_POSIX_) || defined(__GNUC__)
  #include <dirent.h>
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/stat.h>
#endif

#if PATH_MAX > 1024
#define BUFFER_LENGTH PATH_MAX
#else
#define BUFFER_LENGTH 1024
#endif

typedef enum {
    FileType_NoFile = 1,
    FileType_RegularFile,
    FileType_Directory,
    FileType_SpecialFile   /* pipe, FIFO, device, etc. */
} ModelicaFileType;

/* Convert to Unix directory separators: */
#if defined(_WIN32)
static void ModelicaConvertToUnixDirectorySeparator(char* string) {
    /* Convert to Unix directory separators */
    char* c = string;
    while ( *c ) {
        if ( *c == '\\' )  {
            *c = '/';
        }
        c++;
    }
}

static void ModelicaConvertFromUnixDirectorySeparator(char* string) {
    /* Convert from Unix directory separators */
    char* c = string;
    while ( *c ) {
        if ( *c == '/' )  {
            *c = '\\';
        }
        c++;
    }
}
#else
  #define ModelicaConvertToUnixDirectorySeparator(string) ;
  #define ModelicaConvertFromUnixDirectorySeparator(string) ;
#endif

static int readLine(_In_ char** buf, _In_ int* bufLen, _In_ FILE* fp) MODELICA_NONNULLATTR;
  /* Read line (of unknown and arbitrary length) from a text file */

/* --------------------- Modelica_Utilities.Internal --------------------------------- */

void ModelicaInternal_mkdir(_In_z_ const char* directoryName) {
    /* Create directory */
#if defined(__WATCOMC__) || defined(__LCC__)
    int result = mkdir(directoryName);
#elif defined(__BORLANDC__) || defined(_WIN32)
    int result = _mkdir(directoryName);
#elif defined(_POSIX_) || defined(__GNUC__)
    int result = mkdir(directoryName, S_IRUSR | S_IWUSR | S_IXUSR);
#else
   ModelicaNotExistError("ModelicaInternal_mkdir");
#endif
#if defined(__WATCOMC__) || defined(__LCC__) || defined(__BORLANDC__) || defined(_WIN32) || defined(_POSIX_) || defined(__GNUC__)
    if (result != 0) {
        ModelicaFormatError("Not possible to create new directory\n"
            "\"%s\":\n%s", directoryName, strerror(errno));
    }
#endif
}

void ModelicaInternal_rmdir(_In_z_ const char* directoryName) {
    /* Remove directory */
#if defined(__WATCOMC__) || defined(__LCC__) || defined(_POSIX_) || defined(__GNUC__)
    int result = rmdir(directoryName);
#elif defined(__BORLANDC__) || defined(_WIN32)
    int result = _rmdir(directoryName);
#else
    ModelicaNotExistError("ModelicaInternal_rmdir");
#endif
#if defined(__WATCOMC__) || defined(__LCC__) || defined(__BORLANDC__) || defined(_WIN32) || defined(_POSIX_) || defined(__GNUC__)
    if (result != 0) {
        ModelicaFormatError("Not possible to remove directory\n"
            "\"%s\":\n%s", directoryName, strerror(errno));
    }
#endif
}

static ModelicaFileType Internal_stat(_In_z_ const char* name) {
    /* Inquire type of file */
    ModelicaFileType type;
#if defined(_WIN32)
    struct _stat fileInfo;
    int statReturn = _stat(name, &fileInfo);
    if (0 != statReturn) {
        /* _stat requires "a:\" instead of "a:" */
        const char* firstSlash = strpbrk(name, "/\\");
        const char* firstColon = strchr(name, ':');
        const char c = (NULL != firstColon) ? firstColon[1] : '\0';
        size_t len = strlen(name);
        if (NULL == firstSlash && NULL != firstColon && '\0' == c) {
            char* nameTmp = (char*)malloc((len + 2)*(sizeof(char)));
            if (NULL != nameTmp) {
                strcpy(nameTmp, name);
                strcat(nameTmp, "\\");
                statReturn = _stat(nameTmp, &fileInfo);
                free(nameTmp);
            }
        }
#if defined(_MSC_VER) && _MSC_VER >= 1900
        /* _stat accepts both "a:\dir" and "a:\dir\" */
#else
        /* _stat requires "a:\dir" instead of "a:\dir\" */
        /* required for VS 2013 and earlier */
        else if (NULL != firstSlash && len > 1 &&
            ('/' == name[len - 1] || '\\' == name[len - 1])) {
            char* nameTmp = (char*)malloc(len*(sizeof(char)));
            if (NULL != nameTmp) {
                strncpy(nameTmp, name, len - 1);
                nameTmp[len - 1] = '\0';
                statReturn = _stat(nameTmp, &fileInfo);
                free(nameTmp);
            }
        }
#endif
    }
    if ( statReturn != 0 ) {
        type = FileType_NoFile;
    }
    else if ( fileInfo.st_mode & S_IFREG ) {
        type = FileType_RegularFile;
    }
    else if ( fileInfo.st_mode & S_IFDIR ) {
        type = FileType_Directory;
    }
    else {
        type = FileType_SpecialFile;
    }
#elif defined(_POSIX_) || defined(__GNUC__)
    struct stat fileInfo;
    int statReturn;
    statReturn = stat(name, &fileInfo);
    if ( statReturn != 0 ) {
        type = FileType_NoFile;
    }
    else if ( S_ISREG(fileInfo.st_mode) ) {
        type = FileType_RegularFile;
    }
    else if ( S_ISDIR(fileInfo.st_mode) ) {
        type = FileType_Directory;
    }
    else {
        type = FileType_SpecialFile;
    }
#else
    type = FileType_NoFile;
#endif
    return type;
}

int ModelicaInternal_stat(_In_z_ const char* name) {
#if defined(_WIN32) || defined(_POSIX_) || defined(__GNUC__)
    ModelicaFileType type = Internal_stat(name);
#else
    ModelicaFileType type = FileType_NoFile;
    ModelicaNotExistError("ModelicaInternal_stat");
#endif
    return type;
}

void ModelicaInternal_rename(_In_z_ const char* oldName,
                             _In_z_ const char* newName) {
    /* Change the name of a file or of a directory */
    if ( rename(oldName, newName) != 0 ) {
        ModelicaFormatError("renaming \"%s\" to \"%s\" failed:\n%s",
            oldName, newName, strerror(errno));
    }
}

void ModelicaInternal_removeFile(_In_z_ const char* file) {
    /* Remove file */
    if ( remove(file) != 0 ) {
        ModelicaFormatError("Not possible to remove file \"%s\":\n%s",
            file, strerror(errno));
    }
}

void ModelicaInternal_copyFile(_In_z_ const char* oldFile,
                               _In_z_ const char* newFile) {
    /* Copy file */
    const char* modeOld = "rb";
    const char* modeNew = "wb";
    FILE* fpOld;
    FILE* fpNew;
    ModelicaFileType type;

    /* Check file existence */
    type = Internal_stat(oldFile);
    if ( type == FileType_NoFile ) {
        ModelicaFormatError("\"%s\" cannot be copied\nbecause it does not exist", oldFile);
        return;
    }
    else if ( type == FileType_Directory ) {
        ModelicaFormatError("\"%s\" cannot be copied\nbecause it is a directory", oldFile);
        return;
    }
    else if ( type == FileType_SpecialFile ) {
        ModelicaFormatError("\"%s\" cannot be copied\n"
            "because it is not a regular file", oldFile);
        return;
    }
    type = Internal_stat(newFile);
    if ( type != FileType_NoFile ) {
        ModelicaFormatError("\"%s\" cannot be copied\nbecause the target "
            "\"%s\" exists", oldFile, newFile);
        return;
    }

    /* Copy file */
    fpOld = fopen(oldFile, modeOld);
    if ( fpOld == NULL ) {
        ModelicaFormatError("\"%s\" cannot be copied:\n%s", oldFile, strerror(errno));
        return;
    }
    fpNew = fopen(newFile, modeNew);
    if ( fpNew == NULL ) {
        fclose(fpOld);
        ModelicaFormatError("\"%s\" cannot be copied to \"%s\":\n%s",
            oldFile, newFile, strerror(errno));
        return;
    }
    {
        size_t len;
        char buf[BUFSIZ] = {'\0'};

        while ( (len = fread(buf, sizeof(char), BUFSIZ, fpOld)) > 0 ) {
            if ( len != fwrite(buf, sizeof(char), len, fpNew) ) {
                fclose(fpOld);
                fclose(fpNew);
                ModelicaFormatError("Error writing to file \"%s\".", newFile);
                return;
            }
        }
    }
    fclose(fpOld);
    fclose(fpNew);
}

void ModelicaInternal_readDirectory(_In_z_ const char* directory, int nFiles,
                                    _Out_ const char** files) {
    /* Get all file and directory names in a directory in any order
       (must be very careful, to call closedir if an error occurs)
    */
#if defined(__WATCOMC__) || defined(__BORLANDC__) || defined(_WIN32) || defined(_POSIX_) || defined(__GNUC__)
    int errnoTemp;
    int iFiles  = 0;
    char *pName;
    struct dirent *pinfo;
    DIR* pdir;

    /* Open directory information inquiry */
    pdir = opendir(directory);
    if ( pdir == NULL ) {
        ModelicaFormatError("1: Not possible to get file names of \"%s\":\n%s",
            directory, strerror(errno));
    }

    /* Read file and directory names and store them in vector "files" */
    errno = 0;
    while ( (pinfo = readdir(pdir)) != NULL ) {
        if ( (strcmp(pinfo->d_name, "." ) != 0) &&
            (strcmp(pinfo->d_name, "..") != 0) ) {
            /* Check if enough space in "files" vector */
            if ( iFiles >= nFiles ) {
                closedir(pdir);
                ModelicaFormatError("Not possible to get file names of \"%s\":\n"
                    "More files in this directory as reported by nFiles (= %i)",
                    directory, nFiles);
            }

            /* Allocate Modelica memory for file/directory name and copy name */
            pName = ModelicaDuplicateStringWithErrorReturn(pinfo->d_name);
            if ( pName == NULL ) {
                errnoTemp = errno;
                closedir(pdir);
                if ( errnoTemp == 0 ) {
                    ModelicaFormatError("Not possible to get file names of \"%s\":\n"
                        "Not enough storage", directory);
                }
                else {
                    ModelicaFormatError("Not possible to get file names of \"%s\":\n%s",
                        directory, strerror(errnoTemp));
                }
            }

            /* Save pointer to file */
            files[iFiles] = pName;
            iFiles++;
        }
    }

    if ( errno != 0 ) {
        errnoTemp = errno;
        closedir(pdir);
        ModelicaFormatError("Not possible to get file names of \"%s\":\n%s",
            directory, strerror(errnoTemp));
    }

    /* Check, whether the whole "files" vector is filled and close inquiry */
    if ( iFiles != nFiles) {
        closedir(pdir);
        ModelicaFormatError("Not possible to get file names of \"%s\":\n"
            "Less files (= %d) found as defined by argument nNames (= %d)",
             directory, iFiles, nFiles);
    }
    else if ( closedir(pdir) != 0 ) {
        ModelicaFormatError("Not possible to get file names of \"%s\":\n%s",
            directory, strerror(errno));
    }

#else
    ModelicaNotExistError("ModelicaInternal_readDirectory");
#endif
}

int ModelicaInternal_getNumberOfFiles(_In_z_ const char* directory) {
    /* Get number of files and directories in a directory */
#if defined(__WATCOMC__) || defined(__BORLANDC__) || defined(_WIN32) || defined(_POSIX_) || defined(__GNUC__)
    int nFiles = 0;
    int errnoTemp;
    struct dirent *pinfo;
    DIR* pdir;

    pdir = opendir(directory);
    if ( pdir == NULL ) {
        goto Modelica_ERROR;
    }
    errno = 0;
    while ( (pinfo = readdir(pdir)) != NULL ) {
        if ( (strcmp(pinfo->d_name, "." ) != 0) &&
                (strcmp(pinfo->d_name, "..") != 0) ) {
            nFiles++;
        }
    }
    errnoTemp = errno;
    closedir(pdir);
    if ( errnoTemp != 0 ) {
        errno = errnoTemp;
        goto Modelica_ERROR;
    }

    return nFiles;

Modelica_ERROR:
    ModelicaFormatError("Not possible to get number of files in \"%s\":\n%s",
        directory, strerror(errno));
    return 0;
#else
    ModelicaNotExistError("ModelicaInternal_getNumberOfFiles");
    return 0;
#endif
}

/* --------------------- Modelica_Utilities.Files ------------------------------------- */

_Ret_z_ const char* ModelicaInternal_fullPathName(_In_z_ const char* name) {
    /* Get full path name of file or directory */
#undef MODELICA_INTERNAL_HAVE_POSIX_REALPATH
#if defined(_BSD_SOURCE) || \
    (defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 500) || \
    (defined(_XOPEN_SOURCE) && defined(_XOPEN_SOURCE_EXTENDED)) || \
    _POSIX_VERSION >= 200112L
#define MODELICA_INTERNAL_HAVE_POSIX_REALPATH
#endif

#if defined(_WIN32)
    char* fullName;
    char localbuf[BUFFER_LENGTH];
    char* tempName = _fullpath(localbuf, name, sizeof(localbuf));
    if (tempName == NULL) {
        ModelicaFormatError("Not possible to construct full path name of \"%s\"\n%s",
            name, strerror(errno));
        return "";
    }
    fullName = ModelicaDuplicateString(tempName);
    ModelicaConvertToUnixDirectorySeparator(fullName);
    return fullName;
#elif defined(MODELICA_INTERNAL_HAVE_POSIX_REALPATH)
    char* fullName;
    char localbuf[BUFFER_LENGTH];
    size_t len;
    /* realpath availability: 4.4BSD, POSIX.1-2001. Using the behaviour of NULL: POSIX.1-2008 */
    char* tempName = realpath(name, localbuf);
    if (tempName == NULL) {
        goto FALLBACK_getcwd;
    }
    fullName = ModelicaDuplicateString(tempName);
    ModelicaConvertToUnixDirectorySeparator(fullName);
    /* Retain trailing slash to match _fullpath behaviour */
    len = strlen(name);
    if (len > 0 && '/' == name[len - 1]) {
        strcat(fullName, "/");
    }
    return fullName;
#elif defined(_POSIX_)
    char* fullName;
    char localbuf[BUFFER_LENGTH];
#else
    char* fullName = "";
    ModelicaNotExistError("ModelicaInternal_fullPathName");
    return fullName;
#endif

#if defined(MODELICA_INTERNAL_HAVE_POSIX_REALPATH)
FALLBACK_getcwd:
#endif
#if defined(MODELICA_INTERNAL_HAVE_POSIX_REALPATH) || defined(_POSIX_)
    {
        /* No such system call in _POSIX_ available (except realpath for existing paths) */
        char* cwd = getcwd(localbuf, sizeof(localbuf));
        if (cwd == NULL) {
            ModelicaFormatError("Not possible to get current working directory:\n%s",
                strerror(errno));
        }
        fullName = ModelicaAllocateString(strlen(cwd) + strlen(name) + 1);
        if (name[0] != '/') {
            /* Any name beginning with "/" is regarded as already being a full path. */
            strcpy(fullName, cwd);
            strcat(fullName, "/");
        }
        else {
            fullName[0] = '\0';
        }
        strcat(fullName, name);
    }
    return fullName;
#endif
#undef MODELICA_INTERNAL_HAVE_POSIX_REALPATH
}

_Ret_z_ const char* ModelicaInternal_temporaryFileName(void) {
    /* Get full path name of a temporary file name which does not exist */
    char* fullName;

    char* tempName = tmpnam(NULL);
    if (tempName == NULL) {
        ModelicaFormatError("Not possible to get temporary filename\n%s", strerror(errno));
        return "";
    }
    fullName = ModelicaDuplicateString(tempName);
    ModelicaConvertToUnixDirectorySeparator(fullName);

    return fullName;
}

/* --------------------- Abstract data type for stream handles --------------------- */

/* Improved for caching of the open files */
typedef struct FileCache {
    char* fileName; /* Key = File name */
    FILE* fp /* File pointer */;
    char* buf;
    int bufLen;
    int lineNumber;
    UT_hash_handle hh; /* Hashable structure */
} FileCache;

static FileCache* fileCache = NULL;
#if defined(_POSIX_) && !defined(NO_MUTEX)
#include <pthread.h>
#if defined(G2_HAS_CONSTRUCTORS)
static pthread_mutex_t m;
G2_DEFINE_CONSTRUCTOR(G2_FUNCNAME(initializeMutex))
static void G2_FUNCNAME(initializeMutex)(void) {
    if (pthread_mutex_init(&m, NULL) != 0) {
        ModelicaError("Initialization of mutex failed\n");
    }
}
G2_DEFINE_DESTRUCTOR(G2_FUNCNAME(destroyMutex))
static void G2_FUNCNAME(destroyMutex)(void) {
    if (pthread_mutex_destroy(&m) != 0) {
        ModelicaError("Destruction of mutex failed\n");
    }
}
#else
static pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;
#endif
#define MUTEX_LOCK() pthread_mutex_lock(&m)
#define MUTEX_UNLOCK() pthread_mutex_unlock(&m)
#elif defined(_WIN32) && defined(G2_HAS_CONSTRUCTORS)
#if !defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
static CRITICAL_SECTION cs;
#ifdef G2_DEFINE_CONSTRUCTOR_NEEDS_PRAGMA
#pragma G2_DEFINE_CONSTRUCTOR_PRAGMA_ARGS(G2_FUNCNAME(ModelicaInternal_initializeCS))
#endif
G2_DEFINE_CONSTRUCTOR(G2_FUNCNAME(ModelicaInternal_initializeCS))
static void G2_FUNCNAME(ModelicaInternal_initializeCS)(void) {
    InitializeCriticalSection(&cs);
}
#ifdef G2_DEFINE_DESTRUCTOR_NEEDS_PRAGMA
#pragma G2_DEFINE_DESTRUCTOR_PRAGMA_ARGS(G2_FUNCNAME(ModelicaInternal_deleteCS))
#endif
G2_DEFINE_DESTRUCTOR(G2_FUNCNAME(ModelicaInternal_deleteCS))
static void G2_FUNCNAME(ModelicaInternal_deleteCS)(void) {
    DeleteCriticalSection(&cs);
}
#define MUTEX_LOCK() EnterCriticalSection(&cs)
#define MUTEX_UNLOCK() LeaveCriticalSection(&cs)
#else
#define MUTEX_LOCK()
#define MUTEX_UNLOCK()
#endif

#if !defined(LINE_BUFFER_LENGTH)
#define LINE_BUFFER_LENGTH (64)
#endif

static void CacheFileForReading(FILE* fp, const char* fileName, int lineNumber, char* buf, int bufLen) {
    FileCache* fv;
    size_t len;
    if (fileName == NULL) {
        /* Do not add, close file */
        if (fp != NULL) {
            fclose(fp);
        }
        return;
    }
    len = strlen(fileName);
    MUTEX_LOCK();
    HASH_FIND(hh, fileCache, fileName, (unsigned)len, fv);
    if (fv != NULL) {
        fv->fp = fp;
        fv->lineNumber = lineNumber;
        fv->buf = buf;
        fv->bufLen = bufLen;
    }
    else {
        fv = (FileCache*)malloc(sizeof(FileCache));
        if (fv != NULL) {
            char* key = (char*)malloc((len + 1)*sizeof(char));
            if (key != NULL) {
                strcpy(key, fileName);
                fv->fileName = key;
                fv->fp = fp;
                fv->lineNumber = lineNumber;
                fv->buf = buf;
                fv->bufLen = bufLen;
                HASH_ADD_KEYPTR(hh, fileCache, key, (unsigned)len, fv);
                if (NULL == fv->hh.tbl) {
                   free(key);
                   free(fv);
                }
            }
            else {
                free(fv);
            }
        }
    }
    MUTEX_UNLOCK();
}

static void CloseCachedFile(const char* fileName) {
    FileCache* fv;
    size_t len = strlen(fileName);
    MUTEX_LOCK();
    HASH_FIND(hh, fileCache, fileName, (unsigned)len, fv);
    if (fv != NULL) {
        if (fv->fp != NULL) {
            fclose(fv->fp);
        }
        free(fv->buf);
        free(fv->fileName);
        HASH_DEL(fileCache, fv);
        free(fv);
    }
    MUTEX_UNLOCK();
}

static FILE* ModelicaStreams_openFileForReading(const char* fileName, int lineNumber, int* lineNumberOffset, char** buf, int* bufLen) {
    /* Open text file for reading */
    FILE* fp;
    FileCache* fv;
    size_t len = strlen(fileName);
    *lineNumberOffset = 0;
    *buf = NULL;
    *bufLen = LINE_BUFFER_LENGTH;
    MUTEX_LOCK();
    HASH_FIND(hh, fileCache, fileName, (unsigned)len, fv);
    /* Open file */
    if (fv != NULL) {
        /* Cached value */
        if (fv->fp != NULL) {
            if (lineNumber != 0 && lineNumber >= fv->lineNumber - 1) {
                *lineNumberOffset = fv->lineNumber;
                fp = fv->fp;
                *buf = fv->buf;
                *bufLen = fv->bufLen;
            }
            else {
                if ( fseek(fv->fp, 0L, SEEK_SET) == 0 ) {
                    fp = fv->fp;
                    *buf = fv->buf;
                    *bufLen = fv->bufLen;
                }
                else {
                    fclose(fv->fp);
                    fp = NULL;
                    free(fv->buf);
                }
            }
            fv->fp = NULL;
            fv->buf = NULL;
        }
        else {
            fp = NULL;
        }
    }
    else {
        fp = NULL;
    }
    MUTEX_UNLOCK();
    if (fp == NULL) {
        fp = fopen(fileName, "r");
        if ( fp == NULL ) {
            ModelicaFormatError("Not possible to open file \"%s\" for reading:\n"
                "%s\n", fileName, strerror(errno));
        }
    }
    return fp;
}

void ModelicaStreams_closeFile(_In_z_ const char* fileName) {
    /* Close file */
    CloseCachedFile(fileName); /* Closes it */
}

static FILE* ModelicaStreams_openFileForWriting(const char* fileName) {
    /* Open text file for writing (with append) */
    FILE* fp;

    /* Check fileName */
    if ( fileName[0] == '\0' ) {
        ModelicaError("fileName is an empty string.\n"
            "Opening of file is aborted\n");
    }

    /* Open file */
    ModelicaStreams_closeFile(fileName);
    fp = fopen(fileName, "a");
    if ( fp == NULL ) {
        ModelicaFormatError("Not possible to open file \"%s\" for writing:\n"
            "%s\n", fileName, strerror(errno));
    }
    return fp;
}

static int readLine(_In_ char** buf, _In_ int* bufLen, _In_ FILE* fp) {
    char* offset;
    int oldBufLen;

    if (fgets(*buf, *bufLen, fp) == NULL) {
        return EOF;
    }

    do {
        char* p;
        char* tmp;

        if ((p = strchr(*buf, '\n')) != NULL) {
            *p = '\0';
            return 0;
        }
        if ((p = memchr(*buf, 0, (size_t)(*bufLen - 1))) != NULL) {
            return 1;
        }

        oldBufLen = *bufLen;
        *bufLen *= 2;
        tmp = (char*)realloc(*buf, (size_t)*bufLen);
        if (NULL == tmp) {
            fclose(fp);
            free(*buf);
            ModelicaError("Memory allocation error\n");
            return 1;
        }
        *buf = tmp;
        offset = &((*buf)[oldBufLen - 1]);

    } while (fgets(offset, oldBufLen + 1, fp));

    return 0;
}

/* --------------------- Modelica_Utilities.Streams ----------------------------------- */

void ModelicaInternal_print(_In_z_ const char* string,
                            _In_z_ const char* fileName) {
    /* Write string to terminal or to file */
    if ( fileName[0] == '\0' ) {
        /* Write string to terminal */
        ModelicaFormatMessage("%s\n", string);
    }
    else {
        /* Write string to file */
        FILE* fp = ModelicaStreams_openFileForWriting(fileName);
        if ( fputs(string,fp) < 0 ) {
            goto Modelica_ERROR2;
        }
        if ( fputs("\n",fp)   < 0 ) {
            goto Modelica_ERROR2;
        }
        fclose(fp);
        return;

Modelica_ERROR2:
        fclose(fp);
        ModelicaFormatError("Error when writing string to file \"%s\":\n"
            "%s\n", fileName, strerror(errno));
    }
}

int ModelicaInternal_countLines(_In_z_ const char* fileName) {
    /* Get number of lines of a file */
    int lineNumberOffset;
    int bufLen;
    char* buf;
    int c;
    int nLines = 0;
    int start_of_line = 1;
    /* If true, next character starts a new line. */

    FILE* fp = ModelicaStreams_openFileForReading(fileName, 0, &lineNumberOffset, &buf, &bufLen);

    /* Count number of lines */
    while ((c = fgetc(fp)) != EOF) {
        if (start_of_line) {
            nLines++;
            start_of_line = 0;
        }
        if (c == '\n') {
            start_of_line = 1;
        }
    }
    fclose(fp);
    return nLines;
}

void ModelicaInternal_readFile(_In_z_ const char* fileName,
                               _Out_ const char** string, size_t nLines) {
    /* Read file into string vector string[nLines] */
    int lineNumberOffset;
    int bufLen;
    char* buf;
    FILE* fp = ModelicaStreams_openFileForReading(fileName, 0, &lineNumberOffset, &buf, &bufLen);
    char* line;
    size_t iLines = 1;

    if (buf == NULL) {
        buf = (char*)calloc(bufLen, sizeof(char));
        if (buf == NULL) {
            goto Modelica_OOM_ERROR1;
        }
    }

    /* Read data from file */
    while (iLines <= nLines) {
        readLine(&buf, &bufLen, fp);

        line = ModelicaDuplicateStringWithErrorReturn(buf);
        if ( line == NULL ) {
            goto Modelica_OOM_ERROR1;
        }

        string[iLines - 1] = line;
        iLines++;
    }
    fclose(fp);
    free(buf);
    return;

    /* Out-of-memory error */
Modelica_OOM_ERROR1:
    fclose(fp);
    free(buf);
    ModelicaFormatError("Error when reading line %lu from file \"%s\":\n"
        "Not enough memory to allocate string for reading line.",
        (unsigned long)iLines, fileName);
}

_Ret_z_ const char* ModelicaInternal_readLine(_In_z_ const char* fileName,
                                      int lineNumber, _Out_ int* endOfFile) {
    /* Read line lineNumber from file fileName */
    int lineNumberOffset;
    int bufLen;
    char* buf;
    FILE* fp;
    char* line;
    int iLine;

    fp = ModelicaStreams_openFileForReading(fileName, lineNumber - 1, &lineNumberOffset, &buf, &bufLen);

    if (feof(fp)) {
        goto END_OF_FILE;
    }

    if (buf == NULL) {
        buf = (char*)calloc(bufLen, sizeof(char));
        if (buf == NULL) {
            goto Modelica_OOM_ERROR2;
        }
    }

    for (iLine = 0; iLine < lineNumber - lineNumberOffset; iLine++) {
        int readError = readLine(&buf, &bufLen, fp);
        if (readError == EOF && iLine == lineNumber - lineNumberOffset - 1) {
            goto END_OF_FILE;
        }
    }

    line = ModelicaDuplicateStringWithErrorReturn(buf);
    if (line == NULL) {
        goto Modelica_OOM_ERROR2;
    }

    CacheFileForReading(fp, fileName, lineNumber, buf, bufLen);
    *endOfFile = 0;
    return line;

    /* End-of-File or error */
END_OF_FILE:
    fclose(fp);
    CloseCachedFile(fileName);
    *endOfFile = 1;
    line = ModelicaDuplicateString("");
    return line;

Modelica_OOM_ERROR2:
    fclose(fp);
    CloseCachedFile(fileName);
    ModelicaFormatError("Error when reading line %i from file \"%s\":\n"
        "Not enough memory to allocate string for reading line.", lineNumber, fileName);
    return "";
}

/* --------------------- Modelica_Utilities.System ------------------------------------ */

void ModelicaInternal_chdir(_In_z_ const char* directoryName) {
    /* Change current working directory */
#if defined(__WATCOMC__) || defined(__LCC__)
    int result = chdir(directoryName);
#elif defined(__BORLANDC__)
    int result = chdir(directoryName);
#elif defined(_WIN32)
    int result = _chdir(directoryName);
#elif defined(_POSIX_) || defined(__GNUC__)
    int result = chdir(directoryName);
#else
    ModelicaNotExistError("ModelicaInternal_chdir");
#endif
#if defined(__WATCOMC__) || defined(__LCC__) || defined(__BORLANDC__) || defined(_WIN32) || defined(_POSIX_) || defined(__GNUC__)
    if (result != 0) {
        ModelicaFormatError("Not possible to change current working directory to\n"
            "\"%s\":\n%s", directoryName, strerror(errno));
    }
#endif
}

_Ret_z_ const char* ModelicaInternal_getcwd(int dummy) {
    const char* cwd;
    char* directory;

#if defined(__WATCOMC__) || defined(__BORLANDC__) || defined(_POSIX_) || defined(__GNUC__)
    char localbuf[BUFFER_LENGTH];
    cwd = getcwd(localbuf, sizeof(localbuf));
#elif defined(_WIN32)
    char localbuf[BUFFER_LENGTH];
    cwd = _getcwd(localbuf, sizeof(localbuf));
#else
    ModelicaNotExistError("ModelicaInternal_getcwd");
    cwd = "";
#endif
#if defined(__WATCOMC__) || defined(__BORLANDC__) || defined(_WIN32) || defined(_POSIX_) || defined(__GNUC__)
    if (cwd == NULL) {
        ModelicaFormatError("Not possible to get current working directory:\n%s",
            strerror(errno));
        cwd = "";
    }
#endif
    directory = ModelicaDuplicateString(cwd);
    ModelicaConvertToUnixDirectorySeparator(directory);
    return directory;
}

void ModelicaInternal_getenv(_In_z_ const char* name, int convertToSlash,
                             _Out_ const char** content, _Out_ int* exist) {
    /* Get content of environment variable */
    char* result;
#if defined(_MSC_VER) && _MSC_VER >= 1400
    char* value;
    size_t len = 0;
    errno_t err = _dupenv_s(&value, &len, name);
    if (err) {
        value = NULL;
    }
#else
    char* value = getenv(name);
#endif

    if (value == NULL) {
        result = ModelicaDuplicateString("");
        *exist = 0;
    }
    else {
#if defined(_MSC_VER) && _MSC_VER >= 1400
        result = ModelicaDuplicateStringWithErrorReturn(value);
        if (result) {
#else
        result = ModelicaDuplicateString(value);
#endif
            if ( convertToSlash == 1 ) {
                ModelicaConvertToUnixDirectorySeparator(result);
            }
            *exist = 1;
#if defined(_MSC_VER) && _MSC_VER >= 1400
            free(value);
        }
        else {
            free(value);
            ModelicaFormatError("Not enough memory to allocate string for copying "
                "environment variable \"%s\".\n", name);
        }
#endif
    }
    *content = result;
}

#if !defined(_MSC_VER) && !defined(__WATCOMC__) && !defined(__BORLANDC__) && !defined(_WIN32) && defined(_POSIX_) && _POSIX_VERSION < 200112L
static char envBuf[BUFFER_LENGTH];
#endif

void ModelicaInternal_setenv(_In_z_ const char* name,
                             _In_z_ const char* value, int convertFromSlash) {
    /* Set environment variable */
#if defined(_MSC_VER) && _MSC_VER >= 1400
    errno_t err;
    if (1 == convertFromSlash) {
        char* buf = (char*)malloc((strlen(value) + 1)*sizeof(char));
        if (NULL != buf) {
            strcpy(buf, value);
            ModelicaConvertFromUnixDirectorySeparator(buf);
            err = _putenv_s(name, buf);
            free(buf);
        }
        else {
            ModelicaError("Memory allocation error\n");
        }
    }
    else {
        err = _putenv_s(name, value);
    }
    if (0 != err) {
        ModelicaFormatError("Not possible to set environment variable:\n%s",
        strerror(err));
    }
#elif defined(__WATCOMC__) || defined(__BORLANDC__) || defined(_WIN32)
    char* buf = (char*)malloc((strlen(name) + strlen(value) + 2)*sizeof(char));
    if (NULL != buf) {
        int result;

        strcpy(buf, name);
        strcat(buf, "=");
        strcat(buf, value);

        if (1 == convertFromSlash) {
            ModelicaConvertFromUnixDirectorySeparator(&buf[strlen(name) + 1]);
        }
#if defined(__WATCOMC__) || defined(__BORLANDC__) || defined(__LCC__)
        result = putenv(buf);
#else
        result = _putenv(buf);
#endif
        free(buf);
        if (0 != result) {
            ModelicaFormatError("Environment variable\n"
                "\"%s\"=\"%s\"\n"
                "cannot be set: %s", name, value, strerror(errno));
        }
    }
    else {
        ModelicaError("Memory allocation error\n");
    }
#elif defined(_POSIX_) && _POSIX_VERSION >= 200112L
    int result;
    if (1 == convertFromSlash) {
        char* buf = (char*)malloc((strlen(value) + 1)*sizeof(char));
        if (NULL != buf) {
            strcpy(buf, value);
            ModelicaConvertFromUnixDirectorySeparator(buf);
            result = setenv(name, buf, 1);
            free(buf);
        }
        else {
            ModelicaError("Memory allocation error\n");
        }
    }
    else {
        result = setenv(name, value, 1);
    }
    if (0 != result) {
        ModelicaFormatError("Not possible to set environment variable:\n%s",
        strerror(errno));
    }
#elif defined(_POSIX_)
    /* Restriction: This legacy implementation only works on exactly one
       environment variable since a single buffer is used. */
    if (strlen(name) + strlen(value) + 2 > sizeof(envBuf)) {
        ModelicaFormatError("Environment variable\n"
            "\"%s\"=\"%s\"\n"
            "cannot be set, because the internal buffer\n"
            "in file \"ModelicaInternal.c\" is too small (= %d)",
            name, value, sizeof(envBuf));
    }
    strcpy(envBuf, name);
    strcat(envBuf, "=");
    strcat(envBuf, value);

    if (1 == convertFromSlash) {
        ModelicaConvertFromUnixDirectorySeparator(&envBuf[strlen(name) + 1]);
    }

    if (putenv(envBuf) != 0) {
        ModelicaFormatError("Environment variable\n"
            "\"%s\"=\"%s\"\n"
            "cannot be set: %s", name, value, strerror(errno));
    }
#else
    ModelicaNotExistError("ModelicaInternal_setenv");
#endif
}

#endif

/* Low-level time and pid functions */
/* Some parts from: http://nadeausoftware.com/articles/2012/04/c_c_tip_how_measure_elapsed_real_time_benchmarking */

#if !defined(NO_PID)
  #if defined(__WATCOMC__) || defined(__BORLANDC__) || defined(_WIN32)
    #include <process.h>
  #elif defined(NO_FILE_SYSTEM) && (defined(_POSIX_) || defined(__GNUC__))
    #include <unistd.h>
    #include <sys/types.h>
  #endif
#endif

#if !defined(NO_TIME)
  #include <time.h>
  #if defined(__WATCOMC__) || defined(__BORLANDC__) || defined(_WIN32)
    #include <sys/timeb.h>
  #elif defined(_POSIX_) || defined(__GNUC__)
    #include <sys/time.h>
  #endif
#endif

int ModelicaInternal_getpid(void) {
#if defined(NO_PID)
    return 0;
#else
#if defined(_POSIX_) || defined(__GNUC__) || defined(__WATCOMC__) || defined(__BORLANDC__) || defined(__LCC__)
    return getpid();
#else
    return _getpid();
#endif
#endif
}

void ModelicaInternal_getTime(_Out_ int* ms, _Out_ int* sec, _Out_ int* min, _Out_ int* hour,
                              _Out_ int* mday, _Out_ int* mon, _Out_ int* year) {
#if defined(NO_TIME)
    *ms   = 0;
    *sec  = 0;
    *min  = 0;
    *hour = 0;
    *mday = 0;
    *mon  = 0;
    *year = 0;
#else
    struct tm* tlocal;
    int ms0;
#if defined(_POSIX_) || (defined(_MSC_VER) && _MSC_VER >= 1400)
    struct tm tres;
#endif

#if defined(_WIN32)
#if defined(__BORLANDC__)
    struct timeb timebuffer;
        ftime( &timebuffer );                   /* Retrieve ms time */
#else
    struct _timeb timebuffer;
        _ftime( &timebuffer );                  /* Retrieve ms time */
#endif
#if defined(__BORLANDC__)
    ftime(&timebuffer);
    timebuffer.time -= 60 * timebuffer.timezone;
    if (timebuffer.dstflag != 0)
        timebuffer.time += 3600;
    tlocal = gmtime(&timebuffer.time);
#elif defined(_MSC_VER) && _MSC_VER >= 1400
    _ftime_s(&timebuffer);
    timebuffer.time -= 60 * timebuffer.timezone;
    if (timebuffer.dstflag != 0) {
        int hours = 0;
        _get_daylight(&hours);
        timebuffer.time += 3600 * hours;
    }
    gmtime_s(&tres, &timebuffer.time);
    tlocal = &tres;
#else
    _ftime(&timebuffer);
    timebuffer.time -= 60 * timebuffer.timezone;
    if (timebuffer.dstflag != 0)
        timebuffer.time += 3600;
    tlocal = gmtime(&timebuffer.time);
#endif
    ms0 = (int)(timebuffer.millitm); /* Convert unsigned int to int */
#else
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    tv.tv_sec -= 60 * tz.tz_minuteswest;
    gmtime_r(&tv.tv_sec, &tres);
    tlocal = &tres;
    ms0 = (int)(tv.tv_usec / 1000); /* Convert microseconds to milliseconds */
#endif

    /* Do not memcpy as you do not know which sizes are in the struct */
    *ms = ms0;
    *sec = tlocal->tm_sec;
    *min = tlocal->tm_min;
    *hour = tlocal->tm_hour;
    *mday = tlocal->tm_mday;
    *mon = 1 + tlocal->tm_mon;      /* Correct for month starting at 1 */
    *year = 1900 + tlocal->tm_year; /* Correct for 4-digit year */
#endif
}
