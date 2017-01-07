#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

#include "str_common.h"

using namespace cv;
using namespace cv::ml;

using namespace std;

Mat img,image;
Mat targetData, backData;
bool flag = true;
string wdname = "image";

void on_mouse(int event, int x, int y, int flags, void* ustc); //鼠标取样本点
void getTrainData(Mat &train_data, Mat &train_label);  //生成训练数据 
void train_svm(string pict_name,string svm_name); //svm分类
void test_svm(string folder);

string svm_xml="svm.xml";

bool use_hsv=true;

int main(int argc, char** argv)
{

    string svm_name="car.xml";
//    string pict_name="/home/gumh/Videos/20161222_IMG_8571/180.jpg";
    string pict_name="/home/gumh/Pictures/1.jpeg";

//    if(argc==2){
        train_svm(pict_name,svm_name);
//    }else{
//        test_svm("/home/gumh/Pictures/stereo_calibrate1/");
//    }
    return 0;
}

//鼠标在图像上取样本点，按q键退出
void on_mouse(int event, int x, int y, int flags, void* ustc)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        Point pt = Point(x, y);
        Vec3b point = img.at<Vec3b>(y, x);  //取出该坐标处的像素值，注意x,y的顺序
        Mat tmp = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
        if (flag)
        {
            targetData.push_back(tmp); //加入正样本矩阵
            circle(img, pt, 2, Scalar(0, 255, 255), -1, 8); //画圆，在图上显示点击的点 

        }

        else
        {
            backData.push_back(tmp); //加入负样本矩阵
            circle(img, pt, 2, Scalar(255, 0, 0), -1, 8); 

        }
        imshow(wdname, img);
    }
}


void getTrainData(Mat &train_data, Mat &train_label)
{
    int m = targetData.rows;
    int n = backData.rows;
    cout << "正样本数：:" << m << endl;
    cout << "负样本数：" << n << endl;
    vconcat(targetData, backData, train_data); //合并所有的样本点，作为训练数据
    train_label = Mat(m + n, 1, CV_32S, Scalar::all(1)); //初始化标注
    for (int i = m; i < m + n; i++)
        train_label.at<int>(i, 0) = -1;
}

void train_svm(string pict_name,string svm_name)
{

    string path = pict_name;//"/home/gumh/Pictures/3.jpeg";
    img = imread(path);
    Mat rbg_img;
    img.copyTo(image);
    img.copyTo(rbg_img);
    if(use_hsv){
        cvtColor(image,image,CV_BGR2HSV);
    }
    if (img.empty())
    {
        cout << "Image load error";
        return ;
    }
    namedWindow(wdname,2);
    setMouseCallback(wdname, on_mouse, 0);

    for (;;)
    {
        imshow("image", img);

        int c = waitKey(0);
        cout<<"key="<<(char)c<<endl;
        if ((c & 255) == 27)
        {
            cout << "Exiting ...\n";
            break;
        }
        if ((char)c == 'c')
        {
            flag = false;
        }
        if ((char)c == 'q')
        {
            destroyAllWindows();
            break;
        }
    }

    Mat train_data, train_label;
    getTrainData(train_data, train_label); //获取鼠标选择的样本训练数据

    // 设置参数
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);

    // 训练分类器
    Ptr<TrainData> tData = TrainData::create(train_data, ROW_SAMPLE, train_label);
    svm->train(tData);
    svm->save(svm_name);

    Vec3b color(0, 0, 0);
    // Show the decision regions given by the SVM
    for (int i = 0; i < image.rows; ++i)
    for (int j = 0; j < image.cols; ++j)
    {
        Vec3b point = img.at<Vec3b>(i, j);  //取出该坐标处的像素值
        Mat sampleMat = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
        float response = svm->predict(sampleMat);  //进行预测，返回1或-1,返回类型为float
        if ((int)response != 1)
            rbg_img.at<Vec3b>(i, j) = color;  //将背景点设为黑色
    }

    imshow("SVM Simple Example", rbg_img); // show it to the user
    waitKey(0);
}

void test_svm(string folder){
    namedWindow("SVM Simple Example",2);
    namedWindow("SVMBINARY",2);
    // 设置参数
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm=svm->load(svm_xml);

    cout<<"svm->getVarCount()="<<svm->getVarCount()<<endl;

   vector<string> files;

   if(IsDir(folder)){
       files=getAllFilesWithPathFromDir(folder);
   }else{
       files.push_back(folder);
   }

   for(int k=0;k<files.size();k++){
       std::cout<<"predict : "<<k<<",total picture count:"<<files.size()<<endl;
       string file=files[k];

       Mat image=imread(file);
       Mat orig_img=image.clone();
       resize(image,image,Size(image.cols/4,image.rows/4));
       if(use_hsv){
           cvtColor(image,image,CV_BGR2HSV);
       }

       int64_t st=getTickCount();
       Vec3b color(0, 0, 0);
       Vec3b positive_color(255,255,255);
       Mat binary_pict=Mat::zeros(image.cols,image.rows,CV_8UC1);

       // Show the decision regions given by the SVM
       for (int i = 0; i < image.rows; ++i)
       for (int j = 0; j < image.cols; ++j)
       {
           Vec3b point = image.at<Vec3b>(i, j);  //取出该坐标处的像素值
           Mat sampleMat = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
           float response = svm->predict(sampleMat);  //进行预测，返回1或-1,返回类型为float
           if ((int)response != 1){
//               image.at<Vec3b>(i, j) = color;  //将背景点设为黑色
           }else{
               binary_pict.at<uchar>(i, j) = 255;
           }
       }



       vector< vector<Point> > contours;
       vector<Vec4i> hierarchy;
       findContours(binary_pict, contours,
                    hierarchy, CV_RETR_CCOMP,
                    CV_CHAIN_APPROX_SIMPLE);


       RotatedRect rotated_rect;
       Size2f sz;
       float  contour_area;
       float blob_angle_deg;
       float bounding_width;
       float bounding_length;
       char name[64];
       for(int i=0;i<contours.size();i++){
           rotated_rect    = minAreaRect(contours[i]);
           sz              = rotated_rect.size;
           bounding_width  = sz.width;
           bounding_length = sz.height;
           blob_angle_deg = rotated_rect.angle;
           contour_area=contourArea(contours[i]);

           if(contour_area<500){
               continue;
           }

           if (bounding_width < bounding_length){
               blob_angle_deg = 90 + blob_angle_deg;
           }

           if(abs(blob_angle_deg)>10){
               continue;
           }

//           if(bounding_width/bounding_length<3
//                   && bounding_length/bounding_width<3){
//               continue;
//           }

           Point2f rect_points[4];
           rotated_rect.points( rect_points );


           for( int j = 0; j < 4; j++ ){
               line( image, rect_points[j], rect_points[(j+1)%4], Scalar(255,0,0), 1, 8 );
//               std::cout<<"rectPoint "<<j<<"="<<rect_points[j]<<std::endl;
           }

           Point ct((rect_points[0].x+rect_points[2].x)/2*2/3,(rect_points[0].y+rect_points[2].y)/2);
           sprintf(name,"%f,%f",blob_angle_deg,contour_area);
           putText(image,name,rect_points[3],CV_FONT_HERSHEY_COMPLEX,0.5,1);

//           drawContours(image, contours,i, Scalar(255,0,0),1);
       }
       int64_t et=getTickCount();
       cout<<"time="<<(et-st)*1000./getTickFrequency()<<"ms"<<endl;

       imshow("SVM Simple Example", image); // show it to the user
       imshow("SVMBINARY", binary_pict); // show it to the user
       int c=waitKey(0);
       if ((char)c == 'q')
       {
           destroyAllWindows();
           break;
       }else if((char)c=='a'){
           cout<<"false positive:k="<<k<<",file="<<file<<endl;
           string filepath,onlyname,suffix;
           getFilePart(file,filepath,onlyname,suffix);
           sprintf(name,"/home/gumh/Pictures/false_positive/%s.%s",onlyname.c_str(),suffix.c_str());
           imwrite(name,orig_img);
       }
   }

}
