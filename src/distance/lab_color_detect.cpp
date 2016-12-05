
#include "distance/lab_color_detect.h"

void colorDetect::SetMinDistance(int dist)
{
    this->minDist=dist;
}

void colorDetect::SetTargetColor(uchar blue, uchar green, uchar red)
{
    target[0]=blue;
    target[1]=green;
    target[2]=red;

}

void colorDetect::SetTargetColor(cv::Vec3b color)
{
    this->target=color;

}

cv::Mat colorDetect::process(const cv::Mat& image,cv::Mat* output)
{
    cv::Mat ImageLab=image.clone();

    if(output->empty()){
        output->create(image.rows,image.cols,CV_8U);
    }
//    result.create(image.rows,image.cols,CV_8U);

    //将image转换为Lab格式存储在ImageLab中
    cv::cvtColor(image,ImageLab,CV_BGR2Lab);
    //将目标颜色由BGR转换为Lab
    cv::Mat temp(1,1,CV_8UC3);
    temp.at<cv::Vec3b>(0,0)=target;//创建了一张1*1的临时图像并用目标颜色填充
    cvtColor(temp,temp,CV_BGR2Lab);
    target=temp.at<cv::Vec3b>(0,0);//再从临时图像的Lab格式中取出目标颜色

    // 创建处理用的迭代器
    cv::Mat_<cv::Vec3b>::iterator it=ImageLab.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::iterator itend=ImageLab.end<cv::Vec3b>();
    cv::Mat_<uchar>::iterator itout=output->begin<uchar>();
    int cnt=0;
    while(it!=itend)
    {
        //两个颜色值之间距离的计算
        int dist=static_cast<int>(cv::norm<int,3>(cv::Vec3i((*it)[0]-target[0],
                                  (*it)[1]-target[1],(*it)[2]-target[2])));
//        cout<<dist<<" ";
//        ++cnt;
//        if(cnt%image.rows==0){
//            cout<<endl;
//        }
        if(dist<minDist)
            (*itout)=255;
        else
            (*itout)=0;
        it++;
        itout++;
    }
    return *output;
}

