#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <ctime>

#include "ipm/IPM.h"
#include "str_common.h"

using namespace std;
using namespace cv;

int main(){

    char name[256];

    string path="/home/gumh/TrainData/owncar/tmp6_workspace";

    vector<string> files=getAllFilesWithPathFromDir(path);

    string filePath;
    string onlyName;
    string suffix;

    for(string file:files){
        Mat img=imread(file);

        imshow("img",img);

        char key=waitKey(0);

        if(key=='n'){
            cout<<"get negtive picture: "<<file<<endl;
            getFilePart(file,filePath, onlyName, suffix);
            sprintf(name,"/home/gumh/TrainData/owncar/tmp6_neg/%s.%s",onlyName.c_str(),suffix.c_str());
            cout<<"name="<<name<<endl;
            imwrite(name,img);
        }else{

            getFilePart(file,filePath, onlyName, suffix);
            sprintf(name,"/home/gumh/TrainData/owncar/tmp6_positive/%s.%s",onlyName.c_str(),suffix.c_str());

            imwrite(name,img);
        }
        //删除已经处理的
        remove( file.c_str() );
    }

    return 0;
}
