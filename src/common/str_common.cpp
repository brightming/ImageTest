/*
 * common.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: gumh
 */





#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cctype>
#include <algorithm>
#include <math.h>
#include <fstream>

#include "str_common.h"

//using namespace cv;
//using namespace std;

bool IsDir(std::string path)
{
	struct stat sb;
	if (stat(path.c_str(), &sb) == -1) return false;
	return S_ISDIR(sb.st_mode);
}


void getFilePart(string& fileFullName,string& filePath,string& onlyName,string& suffix){
	int idx=fileFullName.find_last_of('/');
	filePath=fileFullName.substr(0,idx);
	string name=fileFullName.substr(idx+1);
	idx=name.rfind('.');
	suffix=name.substr(idx+1);
	onlyName=name.substr(0,idx);
}

void getFileNameAndSuffix(string& fileNameOnly,string& onlyName,string& suffix){
	string name=fileNameOnly;
	int idx=name.rfind('.');
	suffix=name.substr(idx+1);
	onlyName=name.substr(0,idx);
}

string createSaveName(string& oldName,string &dstDir,string& tag){
	string filePath,onlyName,suffix;
	getFilePart(oldName,filePath, onlyName, suffix);


	ostringstream os;
	if(dstDir.empty()){
		os<<filePath<<"/";
	}else{
		os<<dstDir<<"/";
	}
	if(tag.length()==0){
		os<<onlyName<<"."<<suffix;//<<suffix;
	}else{
		os<<onlyName<<"_"<<tag<<"."<<suffix;
	}

	return os.str();

}


std::vector<std::string> split(const  std::string& s, const std::string& delim)
{
    std::vector<std::string> elems;
    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();
    if (delim_len == 0) return elems;
    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }
    return elems;
}

/**
 * 获取指定路径下的文件名，不包含路径名
 */
vector<string> getAllFilesFromDir(string dirPath){

	struct dirent* ptr;
	DIR *dir;
	dir=opendir(dirPath.c_str());
	vector<string> files;
	char returnFile[1024];

	while((ptr=readdir(dir))!=NULL){
		if(ptr->d_name[0]=='.'){// || ptr->d_type!= DT_REG){
			continue;
		}
		sprintf(returnFile,"%s/%s",dirPath.c_str(),ptr->d_name);
		if(IsDir(returnFile)){
			cout<<ptr->d_name<<" is directory."<<endl;
			continue;
		}
		files.push_back(ptr->d_name);
	}
	closedir(dir);

	return files;

}


/**
 * 获取指定路径下的文件名，不包含路径名
 */
vector<string> getAllFilesWithPathFromDir(string dirPath){

    cout<<"getAllFilesWithPathFromDir:dirpath="<<dirPath<<endl;

	struct dirent* ptr;
    vector<string> files;
	DIR *dir;
	dir=opendir(dirPath.c_str());
    if(dir==NULL){
        cout<<"fail to open path:"<<dirPath<<endl;
        return files;
    }



	char returnFile[1024];
	while((ptr=readdir(dir))!=NULL){
		if(ptr->d_name[0]=='.' ){//|| ptr->d_type!= DT_REG){//对于xfs文件系统，这个值总是返回0！需要另一个方法去判断
//            cout<<ptr->d_name<<" is directory."<<",ptr->d_name[0]="<<ptr->d_name[0]<<",ptr->d_type="<<(int)ptr->d_type<<",DT_REG="<<DT_REG<<endl;
			continue;
		}

		sprintf(returnFile,"%s/%s",dirPath.c_str(),ptr->d_name);
		if(IsDir(returnFile)){
			cout<<ptr->d_name<<" is directory."<<endl;
			continue;
		}
		files.push_back(returnFile);
	}
	closedir(dir);

	return files;

}

std::string& trim(std::string &s)
{
    if (s.empty())
    {
        return s;
    }

    s.erase(0,s.find_first_not_of(" "));
    s.erase(s.find_last_not_of(" ") + 1);
    return s;
}

int compareIgnoreCase(const string& str1,const string& str2){
	string s1(str1),s2(str2);
	transform(str1.begin(),str1.end(),s1.begin(),::toupper);
	transform(str2.begin(),str2.end(),s2.begin(),::toupper);
	return s1.compare(s2);
}

bool fileExists(const char *fileName)
{
    ifstream infile(fileName);
    return infile.good();
}

/**
 * format time info into char buffer
 */
void format_current_time(char buf[]){

	time_t now;
	time(&now);
	struct tm *mytm;
	mytm=localtime(&now);
	sprintf(buf,"%d%02d%02d_%02d%02d%02d",mytm->tm_year+1900,mytm->tm_mon+1,mytm->tm_mday,mytm->tm_hour,mytm->tm_min,mytm->tm_sec);

}

/**
 * format date info into char buffer
 */
void format_current_date(char buf[]){
	time_t now;
	time(&now);
	struct tm *mytm;
	mytm=localtime(&now);
	sprintf(buf,"%d%02d%02d",mytm->tm_year+1900,mytm->tm_mon+1,mytm->tm_mday);

}


void get_file_content(string dir,vector<string>& lists){
    ifstream fin;
    fin.open(dir.c_str());
    string line;
    if(fin.is_open()){
        while(getline(fin,line)){
            lists.push_back(line);
        }
    }
    fin.close();

}
