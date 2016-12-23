
#include <opencv2/opencv.hpp>

#include <iostream>

#include <string>

using namespace cv;
using namespace std;



//-------------helper to  convert enum to string--------//
// Search and remove whitespace from both ends of the string
static std::string trim_enum_string(const std::string &s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && isspace(*it)) { it++; }
    std::string::const_reverse_iterator rit = s.rbegin();
    while (rit.base() != it && isspace(*rit)) { rit++; }
    return std::string(it, rit.base());
}
static void split_enum_args(const char* szArgs, std::string Array[], int nMax)
{
    std::stringstream ss(szArgs);
    std::string strSub;
    int nIdx = 0;
    while (ss.good() && (nIdx < nMax)) {
        getline(ss, strSub, ',');
         Array[nIdx]= trim_enum_string(strSub);


        nIdx++;

    }
};
#define DECLARE_ENUM(ename, ...) \
    enum ename { __VA_ARGS__, ename##_OTHERS,MAX_NUMBER_OF_##ename }; \
    static std::string ename##_strings[MAX_NUMBER_OF_##ename]; \
    static void initialize_##ename##_strings() { \
    split_enum_args(#__VA_ARGS__, ename##_strings, MAX_NUMBER_OF_##ename); \
    for(int i=0;i<MAX_NUMBER_OF_##ename;i++){ \
        string full=ename##_strings[i]; \
        int idx=full.find_first_of('_'); \
        if(idx>=0){ \
        string pure=full.substr(idx+1); \
        ename##_strings[i]=pure.c_str(); \
        } \
    } \
    ename##_strings[MAX_NUMBER_OF_##ename-1]="others"; \
    } \
    static const char* ename##_to_string(ename e) { \
    if (ename##_strings[0].empty()) { \
    initialize_##ename##_strings(); \
    } \
    return ename##_strings[e].c_str(); \
    } \
    static ename string_to_##ename(const char* szEnum) { \
    if (ename##_strings[0].empty()) { \
        initialize_##ename##_strings(); \
    } \
    for (int i = 0; i < MAX_NUMBER_OF_##ename; i++) {  \
        string full=ename##_strings[i]; \
        std::string szEnum_str(szEnum); \
        int income_para_idx=szEnum_str.find_first_of('_'); \
        std::string pure; \
        if(income_para_idx>=0){ \
        pure=szEnum_str.substr(income_para_idx+1); \
        } \
        if (ename##_strings[i] == szEnum || strcmp(ename##_strings[i].c_str(),(const char*)pure.c_str())==0) {return (ename)i; } \
    } \
    return ename##_OTHERS ; \
    }


DECLARE_ENUM(robot,a,robot_b,c)

int main(int argc, char **argv){




//    const char* name=robot_to_string(robot_b);
//    std::cout<<"name="<<name<<endl;


    robot x=string_to_robot("robot_dd");
    std::cout<<"x="<<x<<endl<<endl<<endl;


    std::cout<<"x.str="<<robot_to_string(x)<<endl;

    return 0;



    string file="/home/gumh/Pictures/mmexport1482203528148.jpg";
    Mat src=imread(file);



    Mat smooth_pic;
    int m_blur=1;
    GaussianBlur( src, smooth_pic, Size( 3, 3 ), 0, 0 );

    threshold(smooth_pic,smooth_pic,150,200,CV_THRESH_BINARY);

imshow("src",src);
imshow("smooth_pic",smooth_pic);

    waitKey(0);



}
