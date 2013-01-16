/***************************************************************************
 *            filesysTools.h
 *
 *  Fr 02 Mär 2007 23:14:21 CET
 *  Copyright 2007 Rainer Kümmerle
 *  Email rk@raikue.net
 ****************************************************************************/

#ifndef FILESYS_TOOLS_H
#define FILESYS_TOOLS_H


/** @addtogroup utils **/
// @{

/** \file filesysTools.h
 * \brief utility functions for handling files, directory on Linux/Unix
 */

#include <string>
#include <vector>

namespace AISNavigation {

/**
 * get filename extension (the part after the last .), e.g.
 * the extension of file.txt is txt
 */
std::string getFileExtension(const std::string& filename);

/**
 * get the filename without the extension.
 * file.txt -> file
 */
std::string getPureFilename(const std::string& filename);

/**
 * change the fileextension of a given filename.
 * Only if filename contains an extension, otherwise filename is returned.
 */
std::string changeFileExtension(const std::string& filename, const std::string& newExt, bool stripDot = false);

/**
 * return the basename of the given filename
 * /etc/fstab -> fstab
 */
std::string getBasename(const std::string& filename);

/**
 * return the directory of a given filename
 * /etc/fstab -> /etc
 */
std::string getDirname(const std::string& filename);

/**
 * check if file exists (note a directory is also a file)
 */
bool fileExists(const char* filename);

/**
 * checks if file exists and is a file
 */
bool isRegularFile(const char* filename);

/**
 * is the given filename a valid direcory, e.g. exists
 */
bool isDirectory(const char* filename);

/**
 * is the given file a symbolic link
 */
bool isSymbolicLink(const char* filename);

/**
 * return the size of the file in bytes.
 * @return the size of the file (type is off_t -> if large file support, then 64 bit).
 *         -1 on error.
 */
off_t getFileSize(const char* filename);

/**
 * return the current date as a filename of the form YYYYMMDD_hhmmss
 */
std::string getCurrentDateAsFilename();

/** return the modification date of a file */
time_t getLastModificationDate(const char* filename);

/** return date of last access to a file */
time_t getLastAccessDate(const char* filename);

/** return date of last status change of a file */
time_t getLastStatusChangeDate(const char* filename);

/**
 * create a directory. if pub is true, then the rights of the dir
 * will be rwxrwxrwx (readable and writable for everyone)
 */
bool createDirectory(const char* dirName, bool pub = false);

/**
 * return all filenames in the given directory, may also be a sub directory
 * if, onlyFiles is true, only files are returned
 */
std::vector<std::string> getDirectoryElements(const char* dir, bool onlyFiles = false);

/**
 * return all files that match a given pattern, e.g., ~/blaa*.txt, /tmp/a*
 */
std::vector<std::string> getFilesByPattern(const char* pattern);

/**
 * copy a file from a to b
 * @return true, if copy was sucessful
 */
bool copyFile(const char* source, const char* dest);

} // end namespace
// @}
#endif
