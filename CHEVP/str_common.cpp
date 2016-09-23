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
using namespace std;

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

/**
 *@audhor:du.hw
 *@time:2016-4-20
 *计算图片中的箱子区域，此区域是一个等边梯形区域
 *梯形区域的两侧边满足直线公式：y=4x+b
 *箱子实际区域区域的宽度不同，则梯形的下边界也不同
 *此处暂时认为梯形区域居于图片中正中间，且宽度是图片的一半
 */
/*vector<Point> computeLadderRegion(Mat& img){
    
    vector<Point> ladder;//梯形四个角的坐标点，顺序为从上到下，从左到右
    int img_width = img.cols;
    int img_height = img.rows;
    Point up_left,up_right,low_left,low_right;//做个坐标点，梯形的下边长度大于上边长度
    //图片左下角的像素位置为(0,0)
    //计算左下角的坐标点坐标
    low_left.x = img_width/2/2;
    low_left.y = 0;
    //计算右下角的坐标点坐标
    low_right.x = img_width - low_left.x;
    low_right.y = 0;

    int b = -(4 * low_left.x);
    //计算左上角的坐标点坐标
    up_left.x = (img_height - b)/4;
    up_left.y = img_height;
    //计算右上角的坐标点坐标
    up_right.x = img_width - up_left.x;
    up_right.y = img_height;

    ladder.push_back(up_left);
    ladder.push_back(up_right);
    ladder.push_back(low_left);
    ladder.push_back(low_right);
    
    return ladder;

}*/

/**
 *@author:du.hw
 *@time:2016-4-21
 *计算边界距离托盘距离
 *
 *
 *car_dis:机器人距离箱子区域边界的垂直距离，单位：米
 *box_px:箱子的位置在图片中像素的位置（高度位置）
 *return:箱子距离区域边界的支持距离，单位:米
 *计算公式：y=k1*x^3+k2x^2+k3*x(x是像素位置，y是实际距离)
 *注：以上计算出的y是机器人距离箱子的位置，不是区域边界距离箱子的位置
 *    x是像素位置值除以20后的值
 */

float computeBoxDistance(float car_dis,int box_px){
//   float k1 = 0.7798049205;
//   float k2 =  -2.085876494;
//   float k3 = 1.871932833;
//  // float k4 = -13014.96489;
// //  int x = box_px;
//   float x = (float)box_px/200.0;
//   printf("x is %f",x);
//   float y = k1*x*x*x + k2*x*x + k3*x;
//   y = y - car_dis + 0.2;
//   return y;
     float yo;
     int x = box_px;
     printf("============computeBoxDistance x=%d\n",x);
	 
     //yo = 7.02546412730535*pow(10,-22)*pow(x,10)-2.54902051836874*pow(10,-18)*pow(x,9)+4.05164466830614*pow(10,-15)*pow(x,8) -3.70338484448721*pow(10,-12)*pow(x,7)+2.14820436458012*pow(10,-9)*pow(x,6)-8.23007498626568*pow(10,-7)*pow(x,5)+2.09899902084658*pow(10,-4)*pow(x,4)-3.49774386329614*pow(10,-2)*pow(x,3)+3.6156*1*pow(x,2) -2.06561297732062*pow(10,2)*x+ 4.90908081207753*pow(10,3);
    //yo=7.02546412730535*pow10^(-22)*x^10-2.54902051836874*10^(-18)*x^9+(float)4.05164466830614*10^(-15)*x^8-3.70338484448721*10^(-12)*x^7+(float)2.14820436458012*10^(-9)*x^6-8.23007498626568*10^(-7)*x^5+(float)2.09899902084658*10^(-4)*x^4-(float)3.49774386329614*10^(-2)*x^3+(float)3.6156*x^2 -(float)2.06561297732062*10^(2)*x+(float)4.90908081207753*10^(3);
    //yo=3.09894790197607*pow(10,-23)*pow(x,10)-1.08560622500827*pow(10,-19)*pow(x,9)+1.65548494775805*pow(10,-16)*pow(x,8) -1.43606709710332*pow(10,-13)*pow(x,7)+7.80533687239046*pow(10,-11)*pow(x,6)-2.76601805418811*pow(10,-8)*pow(x,5)+6.45499248783512*pow(10,-6)*pow(x,4)-9.77309496977205*pow(10,-4)*pow(x,3)+ 9.20179495129394*pow(10,-2)*pow(x,2) -4.53742403301325*x+1.31557642302486*pow(10,2);
    //yo=1.67403572751943*pow(10,-23)*pow(x,10)-5.60774871871039*pow(10,-20)*pow(x,9)+8.11246898207535*pow(10,-17)*pow(x,8) -6.57636267605817*pow(10,-14)*pow(x,7)+3.26655723965187*pow(10,-11)*pow(x,6)-1.02500494450350*pow(10,-8)*pow(x,5)+2.02745918412708*pow(10,-6)*pow(x,4)-2.44234998150935*pow(10,-4)*pow(x,3)+ 1.68315461024263*pow(10,-2)*pow(x,2) -2.49427693787303*pow(10,-1)*x+2.88410987134135*pow(10,1);
   yo=2.83139219506041*pow(10,-27)*pow(x,10)-2.63508929976161*pow(10,-23)*pow(x,9)+1.06308876993501*pow(10,-19)*pow(x,8) -2.42380664358360*pow(10,-16)*pow(x,7)+3.41736721519929*pow(10,-13)*pow(x,6)-3.06444791197076*pow(10,-10)*pow(x,5)+1.73140491908804*pow(10,-7)*pow(x,4)-5.88482053314492*pow(10,-5)*pow(x,3)+ 1.09917410748580*pow(10,-2)*pow(x,2)-8.30821276232102*pow(10,-1)*x+4.94323217534324*pow(10,1);
   //yo=3.05251121047050*pow(10,-28)*pow(x,10)-1.03018952058874*pow(10,-24)*pow(10,9)+8.82876746910476*pow(10,-22)*pow(10,8) +1.59202368293061*pow(10,-24)*pow(10,7)+1.91386375789108*pow(10,-27)*pow(10,6)+1.91732055238343*pow(10,-30)*pow(x,5)+1.72871913015953*pow(10,-33)*pow(x,4)+1.45477030361726*pow(10,-36)*pow(x,3)+ 1.16594650332714*pow(10,-39)*pow(x,2)+9.01098078956630*pow(10,-43)*x+6.77071538379634*pow(10,-46);
	 return yo;
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
    fin.open(dir);
    string line;
    if(fin.is_open()){
        while(getline(fin,line)){
            lists.push_back(line);
        }
    }
    fin.close();

}
