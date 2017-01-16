
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <fstream>
#include <stdio.h>

using namespace std;
using namespace cv;


#include "str_common.h"


struct BBox{
    int cls;
    float x;
    float y;
    float w;
    float h;
};
void draw_bbox(Mat& pict,vector<BBox> bboxs);
void read_bbox(string file,vector<BBox> &bboxs);
void save_bbox(string file,vector<BBox> &bboxs);

string classes[] = {"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

vector<BBox> bboxs;
Mat pict,for_draw;
Point start_p,end_p;
bool can_drag;
int current_cls;

/**
 * @brief mouseHandler
 * @param event
 * @param x
 * @param y
 * @param flags
 * @param data_ptr
 */
void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        start_p=Point(x,y);
        can_drag=true;
        cout<<"lbtn down"<<endl;

    }else if(event==EVENT_MOUSEMOVE){

        if(can_drag){
//            cout<<"can drag mouse mouse move..."<<endl;
            end_p=Point(x,y);
        }
    }else if(event==EVENT_LBUTTONUP){
        if(can_drag==true){
            cout<<"confirm to add a box.start_p=("<<start_p.x<<","<<start_p.y<<"),"<<
                  "end_p=("<<end_p.x<<","<<end_p.y<<")"<<
                  endl;
            //确认结果
            can_drag=false;
            int left_x,right_x,top_y,bot_y,w,h;
            if(end_p.x<start_p.x){
                left_x=end_p.x;
                right_x=start_p.x;
            }else{
                left_x=start_p.x;
                right_x=end_p.x;
            }
            if(end_p.y<start_p.y){
                top_y=end_p.y;
                bot_y=start_p.y;
            }else{
                top_y=start_p.y;
                bot_y=end_p.y;
            }
            cout<<"left_x="<<left_x<<",right_x="<<right_x<<
                  ",top_y="<<top_y<<",bottom_y="<<bot_y<<endl;
            w=right_x-left_x;
            h=bot_y-top_y;
            BBox box;
            box.cls=current_cls;
            box.x=left_x*1.0/pict.cols;
            box.y=top_y*1.0/pict.rows;
            box.w=w*1.0/pict.cols;
            box.h=h*1.0/pict.rows;

            bboxs.push_back(box);

            //redraw
            for_draw=pict.clone();
            draw_bbox(for_draw,bboxs);
            cout<<"box size="<<bboxs.size()<<endl;
            imshow("box",for_draw);
        }

    }else if(event==EVENT_RBUTTONDOWN){
        //右键按下，取消
        cout<<"rbtn down,cancel"<<endl;
        can_drag=false;
    }

}




void read_bbox(string file,vector<BBox> &bboxs){
    bboxs.resize(0);
    vector<string> conts;
    get_file_content(file,conts);
    for(string line:conts){
        line=trim(line);
        if(!line.empty()){
            //11 0.344192634561 0.611 0.416430594901 0.262
            vector<string> fields=split(line," ");
            if(fields.size()!=5){
                cout<<"error labels lien!"<<line<<endl;
                continue;
            }
            BBox box;
            box.cls=atoi(fields[0].c_str());
            box.x=atoi(fields[1].c_str());
            box.y=atoi(fields[2].c_str());
            box.w=atoi(fields[3].c_str());
            box.h=atoi(fields[4].c_str());

            bboxs.push_back(box);
        }
    }
}

void save_bbox(string file,vector<BBox> &bboxs){
    //如果已有，先删除
    remove(file.c_str());
    ofstream write(file,ios::out|ios::trunc);


    for(BBox box:bboxs){
        write<<box.cls<<" "<<box.x<<" "<<box.y<<" "<<box.w<<" "<<box.h<<endl;
    }
    write.close();

}

void draw_bbox(Mat& pict,vector<BBox> bboxs){
    if(bboxs.size()==0){
        return;
    }
    int width=pict.cols;
    int height=pict.rows;

    int i=0;
    for(BBox box:bboxs){
        int x=box.x*width;
        int y=box.y*height;
        int w=box.w*width;
        int h=box.h*height;

        cout<<"draw_bbox :#"<<i++<<",x="<<x<<",y="<<y<<",width="<<w<<",height="<<h<<endl;

        rectangle(pict,Rect(x,y,w,h),Scalar(0,10+box.cls*10,10+box.cls*8),1);
    }
}

/**
 * @brief main
 * 1、指定工作目录，目录下有:images,labels文件夹。
 * 2、初始化时，读取images下的所有图片，读入列表
 * 3、显示图片，同时，读取labels下的相应的文件（名字同名，不过后缀是.txt)
 * 4、如果有label文件，则读取里面的内容，并根据box的内容，在图片上显示相应的框
 * 5、如果没有label文件，则没有bbox显示
 * 6、可以对已有的图片，增加bbox
 * 7、用户通过按下0-9，q-p，共20类进行切换类别，不同的数字（或字母）对应不同的固定类别，相应的bbox也会用不同的颜色显示
 * 8、用户可以按下n进行切换到下一张图片，切换时，需将当前处理的图片的bbox保存到labels文件
 * 9、双击图片的某个bbox框内部时，该bbox框会用虚线显示，用户按下d按键，会将该bbox框删除，按下esc将会回复正常显示状态。
 *
 * @return
 */
void make_bbox(string path){
    //setMouseCallback("output", mouseHandler_ipm, &outputImg);
    //    void getFileNameAndSuffix(string& fileNameOnly,string& onlyName,string& suffix)

    namedWindow("box",2);
    setMouseCallback("box", mouseHandler,NULL);

    string fileNameOnly, onlyName, suffix;

    char name[64];
    sprintf(name,"%s/images/",path.c_str());

    vector<string> all_imgs=getAllFilesFromDir(name);

    cout<<"path="<<name<<",image size="<<all_imgs.size()<<endl;

    for(int i=0;i<all_imgs.size();i++){
        can_drag=false;
        string img_file=all_imgs[i];

        sprintf(name,"%s/images/%s",path.c_str(),img_file.c_str());
        pict=imread(name);

        //get label content
        getFileNameAndSuffix(img_file,onlyName,suffix);
        sprintf(name,"%s/lables/%s.txt",onlyName.c_str());
        bboxs.resize(0);
        if(fileExists(name)){
            read_bbox(name,bboxs);
        }
        //draw bbox
        draw_bbox(pict,bboxs);

        imshow("box",pict);

        char key=waitKey(0);
        if((int)key==27){
            break;
        }
    }

}


int main(){

    string path="/home/gumh/Videos/for_bbox";
    make_bbox(path);
}
