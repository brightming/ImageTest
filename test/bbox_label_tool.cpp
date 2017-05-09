#include <sys/types.h>
#include <sys/stat.h>
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
    Rect rect;
    string name;
};
void draw_bbox(Mat& ori_pict,vector<BBox> bboxs);
void read_bbox(string file,vector<BBox> &bboxs);
void save_bbox(string file,vector<BBox> &bboxs);

vector<string> all_classes = {"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
vector<string> classes={"rL","rS","rR","rC",
                        "yL","yS","yR","yC",
                        "gL","gS","gR","gC",
                        "black",
                        "back"
                       };
//vector<string> classes = {"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

vector<BBox> bboxs;
Mat ori_pict,for_draw;
Point start_p,end_p;
bool can_drag;
int current_cls;
string cur_img_file;

bool delete_state=false;
int current_selected_box=-1;


void reshow(){
    char text[64];


    for_draw=ori_pict.clone();

//    cout<<"cur_img_file="<<cur_img_file<<endl;
    sprintf(text,"%s",cur_img_file.c_str());
    putText(for_draw,text,Point(for_draw.cols/3,10),cv::FONT_HERSHEY_PLAIN,0.4,Scalar(0,0,255),1);

    draw_bbox(for_draw,bboxs);

    int step=ori_pict.rows/(classes.size()+1);
    for(int i=0;i<classes.size();i++){
        sprintf(text,"%s",classes[i].c_str());
        Scalar color;
        if(i==current_cls){
            color=Scalar(0,0,255);
        }
        else{
            color=Scalar(255,0,0);
        }

        putText(for_draw,string(text),Point(10,15+i*step),FONT_HERSHEY_PLAIN,0.4,color,1,8);
    }


    imshow("box",for_draw);
}

/**
 * @brief get_selected_box
 * 根据选择的点，确定从start_idx开始，第一个包含该点的box
 * @param start_idx
 * bboxs的序号
 * @param pt
 * @return
 */
int get_selected_box(int start_idx,Point pt){
    //    bool delete_state=false;
    //    int current_selected_box=-1;

    if(start_idx<0){
        start_idx=0;
    }
    if(start_idx>=bboxs.size()){
        start_idx=bboxs.size()-1;
    }

    int idx=-1;
    for(int i=start_idx;i<bboxs.size();i++){
        BBox box=bboxs[i];
        if(box.rect.contains(start_p)){
            idx=i;
        }
    }
    if(idx==-1){
        //循环搜索
        for(int i=0;i<=start_idx;i++){
            BBox box=bboxs[i];
            if(box.rect.contains(start_p)){
                idx=i;
            }
        }
    }

    cout<<"get_selected_box="<<idx<<endl;

    return idx;
}

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
        end_p=Point(x,y);

        can_drag=true;
        //        cout<<"lbtn down"<<endl;

    }else if(event==EVENT_MOUSEMOVE){

        if(can_drag){
            //            cout<<"can drag mouse mouse move..."<<endl;
            end_p=Point(x,y);

            reshow();

            //加上新框

        }
    }else if(event==EVENT_LBUTTONUP){
        if(start_p.x==end_p.x && start_p.y==end_p.y){
            can_drag=false;

            //判断落在哪个框，然后那个框就处于待删除状态，如果是有多个框，则按space按键，切换另一个框处于待删除状态
            //按下esc按键，则退出删除状态。所以esc按键有两个作用，另一个是退出程序
            //还可右键退出删除模式
            current_selected_box=get_selected_box(0,start_p);
            if(current_selected_box>=0){
                delete_state=true;
            }else{
                delete_state=false;
            }
            reshow();
        }
        else if(can_drag==true){
            //            cout<<"confirm to add a box.start_p=("<<start_p.x<<","<<start_p.y<<"),"<<
            //                  "end_p=("<<end_p.x<<","<<end_p.y<<")"<<
            //                  endl;
            //确认结果
            can_drag=false;

            //太小的框，或者只是单击了一下鼠标，则不认为是box
            double dist=sqrt(pow(end_p.x-start_p.x,2)+pow(end_p.y-start_p.y,2));
            //            cout<<"dist="<<dist<<endl;
            if(dist<10){
                return;
            }

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
            //            cout<<"left_x="<<left_x<<",right_x="<<right_x<<
            //                  ",top_y="<<top_y<<",bottom_y="<<bot_y<<endl;
            w=right_x-left_x;
            h=bot_y-top_y;
            BBox box;
            box.cls=current_cls;
            box.x=(right_x+left_x)/2.0/ori_pict.cols;//中心x
            box.y=(bot_y+top_y)/2.0/ori_pict.rows;//中心y
            box.w=w*1.0/ori_pict.cols;
            box.h=h*1.0/ori_pict.rows;
            box.name=classes[box.cls];

            box.rect=Rect(left_x,top_y,
                          w,h);

            bboxs.push_back(box);

            //redraw
            reshow();
        }

    }else if(event==EVENT_RBUTTONDOWN){
        //右键按下，取消
        cout<<"rbtn down,cancel"<<endl;
        can_drag=false;

        //取消删除模式
        delete_state=false;
        current_selected_box=-1;
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
            box.x=atof(fields[1].c_str());
            box.y=atof(fields[2].c_str());
            box.w=atof(fields[3].c_str());
            box.h=atof(fields[4].c_str());

            box.rect=Rect((box.x-box.w/2)*ori_pict.cols,(box.y-box.h/2)*ori_pict.rows,
                          box.w*ori_pict.cols,box.h*ori_pict.rows);

            bboxs.push_back(box);
        }
    }
}

void save_bbox(string file,vector<BBox> &bboxs){
//    cout<<"save bbox.file="<<file<<",bboxs.size="<<bboxs.size()<<endl;
    //如果已有，先删除
    remove(file.c_str());

    if(bboxs.size()==0){
        return;
    }
    ofstream write(file,ios::out|ios::trunc);


    for(BBox box:bboxs){
        //        cout<<"save one bbox.cls="<<box.cls<<",x="<<box.x<<",y="<<box.y<<",w="<<box.w<<",h="<<box.h<<endl;
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
    char text[64];

    int i=0;
    for(i=0;i<bboxs.size();i++){
        BBox box=bboxs[i];

        //        int x=box.x*width;
        //        int y=box.y*height;
        //        int w=box.w*width;
        //        int h=box.h*height;

        //        x-=w/2;
        //        y-=h/2;

        //        cout<<"draw_bbox :#"<<i++<<",pict.width="<<width<<",pict.height="<<height<<",cls="<<box.cls<<",box.x="<<box.x<<",x="<<x<<",box.y="<<box.y<<",y="<<y<<",width="<<w<<",height="<<h<<endl;

        int thickness=1;
        Scalar color;
        if(i==current_selected_box){
            thickness=2;
            color=Scalar(0,0,255);
        }else{
            //
//            color=Scalar(0,20+box.cls*10,10+box.cls*8);
            color=Scalar(255,0,0);
        }
        rectangle(pict,box.rect,color,thickness);

        sprintf(text,"%s",classes[box.cls].c_str());
        putText(pict,text,Point(box.rect.x+1,box.rect.y+1),cv::FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));
    }
}

/**
 * @brief compstr
 * 字符串排序。
 * 格式为：
 * type-seq.jpg
 * @param a
 * @param b
 * @return
 */
bool compstr(const string &a, const string &b) {
    int a_idx=a.find_first_of('-');
    int b_idx=b.find_first_of('-');

    int a_dot_idx=a.find_last_of('.');
    int b_dot_idx=b.find_last_of('.');

    if(a_idx>=0 && b_idx>=0){
        string a_type=a.substr(0,a_idx);
        string b_type=b.substr(0,b_idx);

        string a_seq=a.substr(a_idx+1,a_dot_idx-a_idx-1);
        string b_seq=b.substr(b_idx+1,b_dot_idx-b_idx-1);

        if(a_type==b_type){
            return atoi(a_seq.c_str())< atoi(b_seq.c_str());
        }else{
            return a_type<b_type;
        }
    }else{
        return a<b;
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
void make_bbox(string path,string img_path,string dst_label_path="labels",bool remove_from_src=false){
    //setMouseCallback("output", mouseHandler_ipm, &outputImg);
    //    void getFileNameAndSuffix(string& fileNameOnly,string& onlyName,string& suffix)

    namedWindow("box",2);
    setMouseCallback("box", mouseHandler,NULL);

    string fileNameOnly, onlyName, suffix;

    char name[256];
    char label_name[256];
    char text[64];
    char handled[256];
    sprintf(name,"%s/%s/",path.c_str(),img_path.c_str());

    cout<<"name="<<name<<endl;
    vector<string> all_imgs=getAllFilesFromDir(name);
    stable_sort(all_imgs.begin(),all_imgs.end(),compstr);

    //    cout<<"path="<<name<<",image size="<<all_imgs.size()<<endl;
    bool quit=false;
    for(int i=0;i<all_imgs.size() && quit==false;i++){
        //        current_cls=0;

        can_drag=false;

        delete_state=false;
        current_selected_box=-1;

        cur_img_file=all_imgs[i];


        //get images
        sprintf(name,"%s/%s/%s",path.c_str(),img_path.c_str(),cur_img_file.c_str());
//        cout<<"get image:"<<name<<endl;
        ori_pict=imread(name);
        for_draw=ori_pict.clone();
        sprintf(handled,"%s/JPEGImages/%s",path.c_str(),cur_img_file.c_str());

        //get label content
        getFileNameAndSuffix(cur_img_file,onlyName,suffix);
        onlyName=trim(onlyName);
        sprintf(label_name,"%s/%s/%s.txt",path.c_str(),dst_label_path.c_str(),onlyName.c_str());
        bboxs.resize(0);
        if(fileExists(label_name)){
            read_bbox(label_name,bboxs);
        }

//        cout<<"img file="<<name<<",label file="<<label_name<<endl;
        //draw bbox
        reshow();

        do{
            char key=waitKey(0);
            //            cout<<"key="<<(int)key<<endl;
            int old_current_cls=current_cls;

            if((int)key==27){
                save_bbox(label_name,bboxs);

                //移到已处理
                if(remove_from_src){
                    if(bboxs.size()>0){
                        imwrite(handled,ori_pict);
                        remove(name);
                    }
                }

                quit=true;
                break;
            }else if(key=='n'){
//                cout<<"next picture"<<endl;
                cout<<"pict #"<<i+1<<"/"<<all_imgs.size()<<endl;
                save_bbox(label_name,bboxs);


                //移到已处理
                if(remove_from_src){
                    if(bboxs.size()>0){
                        imwrite(handled,ori_pict);
                        remove(name);
                    }
                }

                break;
            }else if((int)key==-1){//delete key
                //delete the selected box
                if(bboxs.size()>0 && delete_state && current_selected_box>=0){
                    for(int k=current_selected_box;k<bboxs.size()-1;k++){
                        bboxs[k]=bboxs[k+1];
                    }
                    bboxs.resize(bboxs.size()-1);
                    reshow();

                    delete_state=false;
                    current_selected_box=-1;

                }
            }
            else if(key>='0' && key<='9'){
                current_cls=key-'0';
            }
            else if(key=='q'){
                current_cls=10;
            }
            else if(key=='w'){
                current_cls=11;
            }
            else if(key=='e'){
                current_cls=12;
            }
            else if(key=='r'){
                current_cls=13;
            }
            else if(key=='t'){
                current_cls=14;
            }
            else if(key=='y'){
                current_cls=15;
            }
            else if(key=='u'){
                current_cls=16;
            }
            else if(key=='i'){
                current_cls=17;
            }
            else if(key=='o'){
                current_cls=18;
            }
            else if(key=='p'){
                current_cls=19;
            }

            if(current_cls>=classes.size()){
                current_cls=old_current_cls;
            }
            cout<<"change to cls idx="<<current_cls<<",name="<<classes[current_cls]<<endl;

            reshow();

        }while(true);
    }

}

/**
 * @brief trim_pre_space
 * 去除图片名称中，开始的空格字符
 */
void trim_pre_space_of_img_name(string path){
    char name[256];
    sprintf(name,"%s/images/",path.c_str());

    string fileNameOnly, onlyName, suffix;


    //get all labels txt
    vector<string> all_imgs=getAllFilesFromDir(name);

    for(int i=0;i<all_imgs.size();i++){
        fileNameOnly=all_imgs[i];
        getFileNameAndSuffix(fileNameOnly,onlyName,suffix);

        if(onlyName.find_first_of(' ')==0){

            sprintf(name,"%s/images/%s",path.c_str(),fileNameOnly.c_str());
            Mat img=imread(string(name));
            remove(name);

            //去除名字前导的空格
            onlyName=trim(onlyName);
            sprintf(name,"%s/images/%s.%s",path.c_str(),onlyName.c_str(),suffix.c_str());
            imwrite(name,img);
        }

    }
}

/**
 * @brief generate_train_txt
 * 通过存在的label文件，决定train的列表
 * @param path
 */
void generate_train_txt(string path){
    char name[256];
    sprintf(name,"%s/labels/",path.c_str());

    char img_name[256];


    string fileNameOnly, onlyName, suffix;


    //get all labels txt
    vector<string> all_labels=getAllFilesFromDir(name);

    //train file
    sprintf(name,"%s/train.txt",path.c_str());
    ofstream train_file(name);
    for(int i=0;i<all_labels.size();i++){
        fileNameOnly=all_labels[i];
        getFileNameAndSuffix(fileNameOnly,onlyName,suffix);
        onlyName=trim(onlyName);

        sprintf(img_name,"%s/JPEGImages/%s.jpeg",path.c_str(),onlyName.c_str());
        ifstream tmpread(img_name);
        if(tmpread.is_open()==false){
            sprintf(img_name,"%s/JPEGImages/%s.jpg",path.c_str(),onlyName.c_str());
        }else{
            tmpread.close();
        }
        train_file<<img_name<<endl;

    }
    train_file.close();
}

void print_boxs(vector<BBox> boxs){
    for(BBox b:boxs){
        cout<<b.cls<<" "<<b.x<<" "<<b.y<<" "<<b.w<<" "<<b.h<<endl;
    }
}

/**
 * @brief translate_to_need_class_id
 * 将原来按照全20类的id，按照需要的类别的id，重新生成label文件
 * 由于第一次保存的cls的id是按照全20类的，需要转换。
 * 这个代码，只执行一次就好了，为了解决上一次生成的label文件的问题。
 * 后面的代码产生的label的cls都是对的。
 * 只用一次转换。
 * @param path
 */
void translate_to_need_class_id(string path){
    char name[256];
    sprintf(name,"%s/labels/",path.c_str());

    string fileNameOnly, onlyName, suffix;
    char label_name[256];

    vector<string> need_classes={"bus","car"};

    //get all labels txt
    vector<string> all_labels=getAllFilesFromDir(name);


    vector<BBox> boxs;
    vector<BBox> new_boxs;
    for(int i=0;i<all_labels.size();i++){
        fileNameOnly=all_labels[i];
        getFileNameAndSuffix(fileNameOnly,onlyName,suffix);
        sprintf(label_name,"%s/labels/%s.txt",path.c_str(),onlyName.c_str());

        read_bbox(label_name,boxs);
        for(BBox box:boxs){
            string name=all_classes[box.cls];
            box.cls=-1;
            for(int i=0;i<need_classes.size();i++){
                if(name==need_classes[i]){
                    box.cls=i;
                    break;
                }
            }
            if(box.cls>=0){
                new_boxs.push_back(box);
            }


        }
        print_boxs(new_boxs);

        save_bbox(label_name,new_boxs);

    }

}


void resize_img(string path,float scale){
    namedWindow("box",2);
    setMouseCallback("box", mouseHandler,NULL);

    string fileNameOnly, onlyName, suffix;

    char name[256];
    char label_name[256];
    char text[64];
    sprintf(name,"%s/images/",path.c_str());

    vector<string> all_imgs=getAllFilesWithPathFromDir(name);
    for(string img_p:all_imgs){
        Mat img=imread(img_p);
        resize(img,img,Size(img.cols/scale,img.rows/scale));
        imwrite(img_p,img);
    }


}

/**
 * @brief decode_video_to_pict
 * 将视频解码为图片
 * 图片存放目录为与video视频文件同目录下的同名文件夹下
 * @param video_file
 */
void decode_video_to_pict(string video_file,string dstfolder="JPEGImages",int skip_frame=0,float resize_scale=1){

    cv::VideoCapture cap(video_file);//打开视频
    if(!cap.isOpened())
    {
        std::cout<<"open the video :"<<video_file<<" failed!"<<std::endl;
        return ;
    }
    Mat frame;
    char name[256];
    string filePath,fileNameOnly, onlyName, suffix;
    getFilePart(video_file, filePath, onlyName, suffix);

    int seq=0;
    while(cap.read(frame)){

        if(resize_scale!=1){
            cv::resize(frame,frame,cv::Size(frame.cols/resize_scale,frame.rows/resize_scale));
        }
        sprintf(name,"%s/%s/%s-%03d.jpg",filePath.c_str(),dstfolder.c_str(),onlyName.c_str(),seq++);
        imwrite(name,frame);

        for(int i=0;i<skip_frame;i++){
            if(!cap.read(frame)){
                break;
            }
        }

    }

}

void resize_video(string src_video,float scale){



    //generate dst video file name
    string dst_video;
    char name[256];
    string filePath,fileNameOnly, onlyName, suffix;
    getFilePart(src_video, filePath, onlyName, suffix);
    sprintf(name,"%s/%s_resize_%f.%s",filePath.c_str(),onlyName.c_str(),scale,suffix.c_str());
    dst_video.assign(name);


    Mat frame;
    VideoCapture cap(src_video); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
        cout << "Cannot open the video file" << endl;

    //    cap.set(CV_CAP_PROP_POS_MSEC, 100000); //start the video at 300ms

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Input video's Frame per seconds : " << fps << endl;

    cap.read(frame);
    std::cout<<"cols="<<frame.cols<<",rows="<<frame.rows<<endl;

    VideoWriter outputvideo(dst_video, CV_FOURCC('D', 'I', 'V', 'X'),fps, Size(frame.cols/scale,frame.rows/scale), true);
    if (!outputvideo.isOpened())	{
        cout << "initialize outputvideo fail!" << endl;
    }

    while(1)
    {
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess)                   //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }
        resize(frame,frame,Size(frame.cols/scale,frame.rows/scale));
        outputvideo.write(frame);
    }
}

/**
 * @brief translate_car_truck_bus_to_car
 * 将class=[bus,car,truck,person]时候产生的
 * bus,car,truck 转换为car,cls id=0，person的cls id=1
 * 那么就变成了两类了：car,person
 * @param src_label_dir_path
 * @param dst_label_dir_path
 */
void translate_car_truck_bus_to_car(string src_label_dir_path,string dst_label_dir_path){

    vector<string> all_old_labels=getAllFilesWithPathFromDir(src_label_dir_path);
    vector<BBox> tmp_bboxs;
    char name[256];
    string filePath,fileNameOnly, onlyName, suffix;


    for(string old_label_p:all_old_labels){
        read_bbox(old_label_p,tmp_bboxs);
        for(BBox box:tmp_bboxs){
            if(box.cls==1 || box.cls==2){
                box.cls=0;//car truck
            }else if(box.cls=3){
                box.cls=1; //person
            }
        }
        getFilePart(old_label_p, filePath, onlyName, suffix);
        sprintf(name,"%s/%s.txt",dst_label_dir_path.c_str(),onlyName.c_str());

        save_bbox(name,tmp_bboxs);
    }
}

/**
 * @brief statistic_sample_distribute
 * 统计样本的分布
 * @param path
 * @param img_path
 * @param label_path
 */
void statistic_sample_distribute(string path,string img_path="JPEGImages",string label_path="labels"){

    int classes_cnt[classes.size()];
    for(int idx=0;idx<classes.size();idx++){
        classes_cnt[idx]=0;
    }

    string label_full_path=path+"/"+label_path;
    vector<string> labels=getAllFilesWithPathFromDir(label_full_path);

    for(string lab:labels){
        vector<BBox> bboxs;
        read_bbox(lab,bboxs);
        for(BBox box:bboxs){
            classes_cnt[box.cls]=classes_cnt[box.cls]+1;
        }
    }

    cout<<"--sample count----"<<endl;
    for(int idx=0;idx<classes.size();idx++){
        cout<<"label:"<<classes[idx]<<" ,count="<<classes_cnt[idx]<<endl;
    }
}


/**
 * @brief combine_imgs_and_labels
 * 将几个分布在不同路径的图片及其对应的label统一汇合到一个地方．
 *
 * 一张图片对应多个label
 *
 * 适用于：几个人对同样的图片进行标注，每个人都对所有图片进行，但是标注的类别不一样;
 * 或者几个人对不同的图片集合进行标注，然后汇总
 * @param img_folders
 * 全路径的图片文件夹
 * 里面的图片如果名字是一样的，就是同样的图片
 * @param label_folders
 * 全路径的ｌａｂｅｌ文件夹
 * @param output_img_folder
 * 全路径的图片输出文件夹
 * @param output_label_folder
 * 全路径的ｌａｂｅｌ输出文件夹
 */
void combine_imgs_and_labels(vector<string> &img_folders,vector<string> &label_folders,
                             string &output_img_folder,string &output_label_folder){


    vector<string> all_img_paths;
    vector<string> all_label_paths;
    map<string,string> img_file_name_to_fullpath;//图片不会重名，如果不同路径下的图片的名字是一样的，那么就是同一张图片

    map<string,vector<string>> label_name_to_fullpaths;//一张图片的ｌａｂｅｌ会有多个

    string path,name,suffix;
    for(string fold:img_folders){
        vector<string> imgs=getAllFilesWithPathFromDir(fold);
        cout<<"imgs.size="<<imgs.size()<<endl;
        std::copy(imgs.begin(),imgs.end(),back_inserter(all_img_paths));
        for(string img:imgs){
            getFilePart(img,path,name,suffix);
            img_file_name_to_fullpath[name]=img;
        }
    }
    for(string fold:label_folders){
        vector<string> labels=getAllFilesWithPathFromDir(fold);
        cout<<"labels.size="<<labels.size()<<endl;
        std::copy(labels.begin(),labels.end(),back_inserter(all_label_paths));

        for(string label:labels){
            getFilePart(label,path,name,suffix);
            vector<string> paths=label_name_to_fullpaths[name];
            paths.push_back(label);
            label_name_to_fullpaths[name]=paths;
        }
    }

    cout<<"all_img_paths.size="<<all_img_paths.size()<<endl;
    cout<<"all_label_paths.size="<<all_label_paths.size()<<endl;

    //对于每张图片，寻找相同名字的label
    char save_name[256];
    for(map<string,string>::iterator it=img_file_name_to_fullpath.begin();it!=img_file_name_to_fullpath.end();it++){
        //找同名的ｌａｂｅｌ的情况
        for(map<string,vector<string>>::iterator it_lab=label_name_to_fullpaths.begin();it_lab!=label_name_to_fullpaths.end();it_lab++){
            if(it->first==it_lab->first){
                //找到
                vector<string> labels=it_lab->second;
                //读取所有的ｌａｂｅｌｓ的ｂｂｏｘ
                vector<BBox> all_bboxs;
                if(labels.size()>0){
                    for(string lab:labels){
                        vector<BBox> bboxs;
                        read_bbox(lab,bboxs);
                        std::copy(bboxs.begin(),bboxs.end(),back_inserter(all_bboxs));
                    }
                    //保存图片到图片输出路径
                    cv::Mat pict=imread(it->second);
                    sprintf(save_name,"%s/%s.jpg",output_img_folder.c_str(),it->first.c_str());
                    imwrite(save_name,pict);

                    //保存ｂｂｏｘ到ｌａｂｅｌ输出路径
                    sprintf(save_name,"%s/%s.txt",output_label_folder.c_str(),it->first.c_str());
                    save_bbox(save_name,all_bboxs);
                }
                break;
            }
        }
    }

}



//refer to : https://github.com/NVIDIA/DIGITS/blob/digits-4.0/digits/extensions/data/objectDetection/README.md
class DetectNetDataFormat{
public:
    string type;
    int truncated=0;
    int occluded=0;
    float alpha=0.0;
    int left=0;
    int top=0;
    int right=0;
    int bottom=0;

    int dimensions_height=0;
    int dimensions_width=0;
    int dimensions_length=0;

    int location_x=0;
    int location_y=0;
    int location_z=0;

    float rotation_y=0;

    float score=0;

    void ConvertFromBBox(BBox& box,cv::Size &pict_size){
        cout<<"box.cls="<<box.cls<<endl;
        type=classes[box.cls];
        left=(box.x-box.w/2)*pict_size.width;
        top=(box.y-box.h/2)*pict_size.height;

        right=(box.x+box.w/2)*pict_size.width;
        bottom=(box.y+box.h/2)*pict_size.height;
    }

    void Save(ofstream &write){
        write<<type<<" "<<truncated<<" "
            <<occluded<<" "<<alpha<<" "
           <<left<<" "<<top<<" "<<right<<" "<<bottom<<" "
          <<dimensions_height<<" "<<dimensions_width<<" "<<dimensions_length<<" "
         <<location_x<<" "<<location_y<<" "<<location_z<<" "
        <<rotation_y<<" "<<score<<"\n";

    }

};

/**
 * @brief convert_to_detectnet_format
 * 转换自己的ｂｂｏｘ的格式为detectnet所需要的格式
 * 参考:http://lncohn.com/digits/nvidia_object_detection.html
 * @param pict_dir
 * @param label_dir
 * @param output_label_dir
 */
void convert_to_detectnet_format(string pict_dir,string label_dir,string output_img_dir,string output_label_dir){
    char name[256];
    //get all labels txt
    vector<string> labels=getAllFilesWithPathFromDir(label_dir);


    string filePath,onlyName,suffix;

    string pict_file;
    for(string lab:labels){
        cout<<"label file="<<lab<<endl;
        getFilePart(lab,filePath,onlyName,suffix);
        pict_file=pict_dir+"/"+onlyName+".jpg";
        cv::Mat pict=cv::imread(pict_file);
        cv::Size pict_size(pict.cols,pict.rows);
        vector<BBox> bboxs,need_boxs;
        read_bbox(lab,bboxs);

        for(BBox box:bboxs){
            if(box.cls>=1){//only one class
                continue;
            }
            need_boxs.push_back(box);

        }

        if(need_boxs.size()==0){
            continue;
        }

        string dstfile=output_label_dir+"/"+onlyName+"."+suffix;
        ofstream write(dstfile,ios::out|ios::trunc);
        for(BBox box:need_boxs){
            DetectNetDataFormat dnd;
            dnd.ConvertFromBBox(box,pict_size);
            dnd.Save(write);
        }
        write.close();
        string dst_pict_file=output_img_dir+"/"+onlyName+".jpg";
        cv::imwrite(dst_pict_file,pict);
    }
}



class TrafficLightWindow{
public:
    cv::Point2i cent_p_;//中心点
    cv::Rect rect;
};

class TrafficLightWindowWrap{
public:
    vector<TrafficLightWindow> wins;
    int width=250;
    int height=100;
    bool delete_state=false;
    int cur_select_win=-1;
    cv::Mat orig_pict;
    cv::Mat for_draw;
    string parent_path="";//包含图片，window定义的父路径
    string win_pict_path="";//保存window的图片文件夹
    string cur_pic_only_name="";//图片的名称，没有路径与后缀
};


bool check_and_fill_window(TrafficLightWindowWrap &wrap,TrafficLightWindow &window){

    int st_x,st_y,e_x,e_y;

    st_x=window.cent_p_.x-wrap.width/2;
    st_y=window.cent_p_.y-wrap.height/2;

    if(st_x<0){
        st_x=0;
    }
    if(st_x>=wrap.orig_pict.cols){
        st_x=wrap.orig_pict.cols;
    }
    if(st_y<0){
        st_y=0;
    }
    if(st_y>=wrap.orig_pict.rows){
        st_y=wrap.orig_pict.rows;
    }

    if(st_x>wrap.orig_pict.cols-20
            ||
            st_y>wrap.orig_pict.rows-20
            ){
        cout<<"st_x("<<st_x<<
             ") > wrap.orig_pict.cols-20("<<wrap.orig_pict.cols-20<<") || "
            "st_y("<<st_y<<") > wrap.orig_pict.rows-20 ("<<wrap.orig_pict.rows-20<<")!!!"<<endl;
        return false;
    }

    window.rect.x=st_x;
    window.rect.y=st_y;
    window.rect.width=wrap.width;
    window.rect.height=wrap.height;
    if(st_x+wrap.width>=wrap.orig_pict.cols){
        window.rect.width=wrap.orig_pict.cols-st_x;
    }
    if(st_y+wrap.height>=wrap.orig_pict.rows){
        window.rect.height=wrap.orig_pict.rows-st_y;
    }

    return true;
}

void read_traffic_light_window(string file,TrafficLightWindowWrap &wrap){
    wrap.wins.clear();
    vector<string> conts;
    get_file_content(file,conts);

    bool ok=false;

    for(string line:conts){
        line=trim(line);
//        cout<<"line="<<line<<endl;
        if(!line.empty()){
            //100,200
            vector<string> fields=split(line," ");
            if(fields.size()!=2){
                cout<<"error labels lien!"<<line<<endl;
                continue;
            }
//            cout<<"field[0]="<<fields[0]
//               <<" [1]="<<fields[1]
//              <<endl;
            TrafficLightWindow window;
            window.cent_p_.x=atoi(fields[0].c_str());//x
            window.cent_p_.y=atoi(fields[1].c_str());//y

            ok=check_and_fill_window(wrap,window);
            if(ok){
                wrap.wins.push_back(window);
            }
        }
    }
}

void show_windows(TrafficLightWindowWrap* wrap){

    cout<<"show_windows.wrap.wins.size="<<wrap->wins.size()<<endl;
    wrap->for_draw=wrap->orig_pict.clone();
    int idx=0;
    for(TrafficLightWindow window:wrap->wins){
        cout<<"loop"<<endl;
        if(check_and_fill_window(*wrap,window)==false){
            cout<<"continue"<<endl;
            continue;
        }
        cout<<"wind:rect.x="<<window.rect.x<<",y="<<window.rect.y<<",width="<<window.rect.width<<",height="<<window.rect.height<<endl;
        if(idx==wrap->cur_select_win && wrap->delete_state==true){
            cv::rectangle(wrap->for_draw,window.rect,cv::Scalar(0,0,255),2);
        }else{
             cv::rectangle(wrap->for_draw,window.rect,cv::Scalar(255,0,0),2);
        }

        idx++;
    }
    imshow("window",wrap->for_draw);
}

int get_selected_window(int start_idx,Point pt,TrafficLightWindowWrap wrap){
    //    bool delete_state=false;
    //    int current_selected_box=-1;

    cout<<"get_selected_window pt=("<<pt.x<<","<<pt.y<<")"<<endl;
    cv::circle(wrap.for_draw,pt,3,cv::Scalar(0,255,255),-2);
    if(start_idx<0){
        start_idx=0;
    }
    if(start_idx>=wrap.wins.size()){
        start_idx=wrap.wins.size()-1;
    }

    int idx=-1;
    for(int i=start_idx;i<wrap.wins.size();i++){
        TrafficLightWindow win=wrap.wins[i];
        cout<<"win.rect.x="<<win.rect.x<<",y="<<win.rect.y<<",width="<<win.rect.width<<",height="<<win.rect.height<<endl;
        if(win.rect.contains(pt)){
            idx=i;
        }
    }
    if(idx==-1){
        //循环搜索
        for(int i=0;i<=start_idx;i++){
            TrafficLightWindow win=wrap.wins[i];
            if(win.rect.contains(pt)){
                idx=i;
            }
        }
    }

    cout<<"get_selected_window="<<idx<<endl;

    return idx;
}

/**
 * @brief mouseHandler
 * @param event
 * @param x
 * @param y
 * @param flags
 * @param data_ptr
 */
void mouse_handle_window(int event, int x, int y, int flags, void* data_ptr)
{
    TrafficLightWindowWrap* wrap=(TrafficLightWindowWrap*)data_ptr;

    cv::Point2i pt(x,y);
    if  ( event == EVENT_LBUTTONDBLCLK )//双击选择中心点
    {
        cout<<"dbclk"<<endl;

        TrafficLightWindow win;
        win.cent_p_.x=x;
        win.cent_p_.y=y;

        if(check_and_fill_window(*wrap,win)){
            wrap->wins.push_back(win);
            show_windows(wrap);
        }
    }else if(event==EVENT_LBUTTONDOWN){
        //选择
        int idx=get_selected_window(0,pt,*wrap);
        if(idx>=0){
            wrap->cur_select_win=idx;
            wrap->delete_state=true;
            show_windows(wrap);
        }
    }else if(event==EVENT_RBUTTONDOWN){
        //右键按下，取消
        cout<<"rbtn down,cancel"<<endl;
       //取消删除模式
        wrap->cur_select_win=-1;
        wrap->delete_state=false;
        show_windows(wrap);
    }

}

void save_windows(string label_file,TrafficLightWindowWrap &wrap){
    cout<<"save label_file.file="<<label_file<<",winds.size="<<wrap.wins.size()<<endl;
    //如果已有，先删除
    remove(label_file.c_str());

    if(wrap.wins.size()==0){
        return;
    }
    ofstream write(label_file,ios::out|ios::trunc);


    char rect_file_name[256];
    for(TrafficLightWindow win:wrap.wins){
        //        cout<<"save one bbox.cls="<<box.cls<<",x="<<box.x<<",y="<<box.y<<",w="<<box.w<<",h="<<box.h<<endl;
        write<<win.cent_p_.x<<" "<<win.cent_p_.y<<endl;
        cv::Mat rect=wrap.orig_pict(win.rect);
        sprintf(rect_file_name,"%s/%s_%d_%d.jpg",wrap.win_pict_path.c_str(),wrap.cur_pic_only_name.c_str()
                ,win.cent_p_.x,win.cent_p_.y);
        cv::imwrite(rect_file_name,rect);
    }
    write.close();
}

/**
 * @brief create_traffic_light_window
 * 创建包含红绿灯的小窗口．
 * 红绿灯的检测不是在整个图片中检索的，而是在某个小范围内检测的．
 * 训练的时候，也是从原图中取出窗口，在这个窗口里进行样本框定．
 */
void create_traffic_light_window(string parent_path,string img_folder){

//    vector<TrafficLightWindow> windows;
    TrafficLightWindowWrap wrap;

    namedWindow("window",2);
    setMouseCallback("window", mouse_handle_window,&wrap);

    string fileNameOnly, onlyName, suffix;

    char name[256];
    char label_name[256];
    char text[64];
    char handled[256];

    //创建用于保存ｗｉｎｄｏｗ信息的文件夹
    char window_folder_name[256];
    sprintf(window_folder_name,"%s/%s_window_txt/",parent_path.c_str(),img_folder.c_str());
    int status;
    status = mkdir(window_folder_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    //创建保存截取出来的window图片的文件夹
    char win_pict_folder_name[256];
    sprintf(win_pict_folder_name,"%s/%s_window_rect/",parent_path.c_str(),img_folder.c_str());
    status = mkdir(win_pict_folder_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    wrap.parent_path=parent_path;
    wrap.win_pict_path=win_pict_folder_name;

    //加载图片
    sprintf(name,"%s/%s/",parent_path.c_str(),img_folder.c_str());

    cout<<"name="<<name<<endl;
    vector<string> all_imgs=getAllFilesFromDir(name);
    stable_sort(all_imgs.begin(),all_imgs.end(),compstr);

    //    cout<<"path="<<name<<",image size="<<all_imgs.size()<<endl;
    bool quit=false;

    for(int i=0;i<all_imgs.size() && quit==false;i++){
        //        current_cls=0;

        can_drag=false;

        delete_state=false;
        current_selected_box=-1;

        cur_img_file=all_imgs[i];



        //get images
        sprintf(name,"%s/%s/%s",parent_path.c_str(),img_folder.c_str(),cur_img_file.c_str());
//        cout<<"get image:"<<name<<endl;
        ori_pict=imread(name);
        wrap.orig_pict=ori_pict.clone();

        //get label content
        getFileNameAndSuffix(cur_img_file,onlyName,suffix);
        onlyName=trim(onlyName);
        wrap.cur_pic_only_name=onlyName;
        sprintf(label_name,"%s/%s.txt",window_folder_name,onlyName.c_str());
        wrap.wins.resize(0);
        if(fileExists(label_name)){
            read_traffic_light_window(label_name,wrap);
        }

//        cout<<"img file="<<name<<",label file="<<label_name<<endl;
        //draw bbox
        show_windows(&wrap);

//        save_windows(label_name,wrap);
//        continue;

        do{
            char key=waitKey(0);
            //            cout<<"key="<<(int)key<<endl;

            if((int)key==27){
                save_windows(label_name,wrap);
                quit=true;
                break;
            }else if(key=='n'){
                cout<<"next picture"<<endl;
                save_windows(label_name,wrap);

                break;
            }else if((int)key==-1){//delete key
                //delete the selected box
                if(wrap.wins.size()>0 && wrap.delete_state && wrap.cur_select_win>=0){
                    for(int k=wrap.cur_select_win;k<wrap.wins.size()-1;k++){
                        wrap.wins[k]=wrap.wins[k+1];
                    }
                    wrap.wins.resize(wrap.wins.size()-1);
                    wrap.delete_state=false;
                    wrap.cur_select_win=-1;

                    show_windows(&wrap);

                }
            }

        }while(true);
    }

}



void test_combine_labels(){
    vector<string> img_folders;
    vector<string> label_folders;
    string output_img_folder;
    string output_label_folder;

    img_folders.push_back("/home/gumh/Videos/for_combine/3429799873058_src_window_rect");
    img_folders.push_back("/home/gumh/Videos/for_combine/4239794161862_src_window_rect");
    img_folders.push_back("/home/gumh/Videos/for_combine/JPEGImages");

    label_folders.push_back("/home/gumh/Videos/for_combine/3429799873058_src_window_rect_labels");
    label_folders.push_back("/home/gumh/Videos/for_combine/4239794161862_src_window_rect_labels");
    label_folders.push_back("/home/gumh/Videos/for_combine/labels");

    output_img_folder="/home/gumh/Videos/for_combine/output_imgs/";
    output_label_folder="/home/gumh/Videos/for_combine/output_labels/";
    combine_imgs_and_labels(img_folders,label_folders,output_img_folder,output_label_folder);

}

int main(int argc,char** argv){

    string path;
    if(argc==1){
        path.assign("/home/gumh/Videos/for_combine/");
    }else{
        path.assign(argv[1]);
    }

//    convert_to_detectnet_format("/home/gumh/TrainData/person_car/JPEGImages",
//                                "/home/gumh/TrainData/person_car/labels",
//                                "/home/gumh/TrainData/person_car/detectnet_train_imgs",
//                                "/home/gumh/TrainData/person_car/detectnet_train_labels");

    //    decode_video_to_pict("/home/gumh/Videos/bbox样本/machangroad.mp4",2);
    //    decode_video_to_pict("/home/gumh/TrainData/AirPlane.2017.3.30/left2017033d16150767_left.avi","left2017033d16150767_left");

    //    trim_pre_space_of_img_name(path);

//        make_bbox(path,"3429799873058_src_window_rect","3429799873058_src_window_rect_labels");

//        statistic_sample_distribute(path,"output_imgs","output_labels");

    //generate_train_txt
        generate_train_txt(path);

    //    resize_img(path,4);

    //    string src_video="/home/gumh/Videos/bbox样本/huachengdadao.mp4";
    //    float scale=4;
    //    resize_video(src_video,scale);

    //    decode_video_to_pict("/home/gumh/Videos/bbox样本/huangpuroad1.mp4","fortest");

    //    translate_car_truck_bus_to_car("/home/gumh/Videos/bbox样本/labels","/home/gumh/Videos/bbox样本/2class_labels");


//    create_traffic_light_window("/home/gumh/Videos/sunny-outside/pict_with_traffic_light",
//                                "3429799873058_src");


//       test_combine_labels();
}
