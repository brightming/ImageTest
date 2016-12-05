/*
 * common.h
 *
 *  Created on: Mar 10, 2016
 *      Author: gumh
 */

#ifndef BASIC_COMMON_H_
#define BASIC_COMMON_H_
//#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;
//using namespace cv;


std::vector<std::string> split(const  std::string& s, const std::string& delim);

bool IsDir(std::string path);


/**
 * 从一个完整的带路径的文件名，截取出文件路径、文件名（不含后缀）、文件后缀
 */
void getFilePart(string& fileFullName,string& filePath,string& onlyName,string& suffix);

/**
 * 从一个不带路径的文件名，截取出文件名（不含后缀）、文件后缀
 */
void getFileNameAndSuffix(string& fileNameOnly,string& onlyName,string& suffix);

string createSaveName(string& oldName,string &dstDir,string& tag);

/**
 * 读取指定路径下的所有非目录文件
 * 返回不带路径的文件名
 */
vector<string> getAllFilesFromDir(string dirPath);

/**
 * 读取指定路径下的所有非目录文件
 * 返回带路径的文件名
 */
vector<string> getAllFilesWithPathFromDir(string dirPath);


std::string& trim(std::string &s);


int compareIgnoreCase(const string& str1,const string& str2);




/**
 * format time info into char buffer
 */
void format_current_time(char buf[]);

/**
 * format dae info into char buffer
 */
void format_current_date(char buf[]);

void get_file_content(string dir,vector<string>& lists);

#endif /* BASIC_COMMON_H_ */
