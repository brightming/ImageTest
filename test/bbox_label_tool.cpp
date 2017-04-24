
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
vector<string> classes={"r","y","g","b"};
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

    cout<<"cur_img_file="<<cur_img_file<<endl;
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

        putText(for_draw,string(text),Point(10,20+i*step),FONT_HERSHEY_SIMPLEX,0.4,color,2,8);
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
    cout<<"save bbox.file="<<file<<",bboxs.size="<<bboxs.size()<<endl;
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
            color=Scalar(0,10+box.cls*10,10+box.cls*8);
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
void make_bbox(string path,string img_path,bool remove_from_src){
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
        cout<<"get image:"<<name<<endl;
        ori_pict=imread(name);
        for_draw=ori_pict.clone();
        sprintf(handled,"%s/JPEGImages/%s",path.c_str(),cur_img_file.c_str());

        //get label content
        getFileNameAndSuffix(cur_img_file,onlyName,suffix);
        onlyName=trim(onlyName);
        sprintf(label_name,"%s/labels/%s.txt",path.c_str(),onlyName.c_str());
        bboxs.resize(0);
        if(fileExists(label_name)){
            read_bbox(label_name,bboxs);
        }

        cout<<"img file="<<name<<",label file="<<label_name<<endl;
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
                cout<<"next picture"<<endl;
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


int main(int argc,char** argv){

    string path;
    if(argc==1){
        path.assign("/home/gumh/TrainData/pict_with_traffic_light/all/");
    }else{
        path.assign(argv[1]);
    }

//    decode_video_to_pict("/home/gumh/Videos/bbox样本/machangroad.mp4",2);
//    decode_video_to_pict("/home/gumh/TrainData/AirPlane.2017.3.30/left2017033d16150767_left.avi","left2017033d16150767_left");

//    trim_pre_space_of_img_name(path);

//    make_bbox(path,"images",false);

    statistic_sample_distribute(path);

    //generate_train_txt
    generate_train_txt(path);

//    resize_img(path,4);

//    string src_video="/home/gumh/Videos/bbox样本/huachengdadao.mp4";
//    float scale=4;
//    resize_video(src_video,scale);

//    decode_video_to_pict("/home/gumh/Videos/bbox样本/huangpuroad1.mp4","fortest");

//    translate_car_truck_bus_to_car("/home/gumh/Videos/bbox样本/labels","/home/gumh/Videos/bbox样本/2class_labels");
}
