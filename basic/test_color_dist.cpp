
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;



class colorDetect{
private:
    int minDist; //minium acceptable distance
    Vec3b target;//target color;
    Mat result; //the result
public:
    colorDetect(){};
    void SetMinDistance(int dist);
    void SetTargetColor(uchar blue,uchar green,uchar red);
    void SetTargetColor(Vec3b color); //set the target color
    Mat process(const Mat& image); //main process
};


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

void colorDetect::SetTargetColor(Vec3b color)
{
    this->target=color;

}

Mat colorDetect::process(const Mat& image)
{
    Mat ImageLab=image.clone();
    result.create(image.rows,image.cols,CV_8U);

    //将image转换为Lab格式存储在ImageLab中
    cvtColor(image,ImageLab,CV_BGR2Lab);
    //将目标颜色由BGR转换为Lab
    Mat temp(1,1,CV_8UC3);
    temp.at<Vec3b>(0,0)=target;//创建了一张1*1的临时图像并用目标颜色填充
    cvtColor(temp,temp,CV_BGR2Lab);
    target=temp.at<Vec3b>(0,0);//再从临时图像的Lab格式中取出目标颜色

    // 创建处理用的迭代器
    Mat_<Vec3b>::iterator it=ImageLab.begin<Vec3b>();
    Mat_<Vec3b>::iterator itend=ImageLab.end<Vec3b>();
    Mat_<uchar>::iterator itout=result.begin<uchar>();
    int cnt=0;
    while(it!=itend)
    {
        //两个颜色值之间距离的计算
        int dist=static_cast<int>(norm<int,3>(Vec3i((*it)[0]-target[0],
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
    return result;
}




void test_basic_color_fun(){
    cv::Vec3b color_a(243,63,30);
    cv::Vec3b color_b(255,27,0);

    Mat one(100,100,CV_8UC3);
    Mat two(100,100,CV_8UC3);


    unsigned char* one_ptr=(unsigned char*)one.data;
    unsigned char* two_ptr=(unsigned char*)two.data;

    for(int r=0;r<one.rows;r++){
        for(int c=0;c<one.cols;c++){
            one.at<cv::Vec3b>(r,c)[0]=30;
            one.at<cv::Vec3b>(r,c)[1]=63;
            one.at<cv::Vec3b>(r,c)[2]=243;
        }
    }

    for(int r=0;r<two.rows;r++){
        for(int c=0;c<two.cols;c++){
            two.at<cv::Vec3b>(r,c)[0]=0;
            two.at<cv::Vec3b>(r,c)[1]=27;
            two.at<cv::Vec3b>(r,c)[2]=255;
        }
    }

//    double dist=cv::norm(color_a,color_b);
//    cout<<"dist="<<dist<<endl;


    Mat one_lab,two_lab;
    cvtColor(one, one_lab, CV_BGR2Lab);
    cvtColor(two, two_lab, CV_BGR2Lab);


    imshow("one",one);
    imshow("two",two);
    imshow("o-l",one_lab);
    imshow("t-l",two_lab);

    Vec3b one_target=one_lab.at<Vec3b>(0,0);
    Vec3b two_target=two_lab.at<Vec3b>(0,0);


    int lab_dist=static_cast<int>(norm<int,3>(Vec3i(one_target[0]-two_target[0],
                              one_target[1]-two_target[1],one_target[2]-two_target[2])));

    int bgr_dist=static_cast<int>(norm<int,3>(Vec3i(color_a[0]-color_b[0],
                                  color_a[1]-color_b[1],color_a[2]-color_b[2])));
    cout<<"lab_dist="<<lab_dist<<",rbg_dist="<<bgr_dist<<endl;

    waitKey(0);
}

void test_color_dist(Mat &img,Vec3b target_color,int min_dist){
    colorDetect cdect;
    cdect.SetMinDistance(min_dist);
    cdect.SetTargetColor(target_color);
    Mat res=cdect.process(img);

    imshow("ori",img);
    imshow("res",res);
    waitKey(0);

}

void test_floodfill(){
    Mat image=imread("picture/9.png");

    cv::Vec3b white(255, 255, 255);

    for(int y=0; y<image.rows; y++)
    {
        cv::Vec3b* row = image.ptr<cv::Vec3b>(y);
        for(int x=0; x<image.cols; x++)
        {

            if(row[x] == white)
            {
                cv::floodFill(image, cv::Point(x,y), cv::Scalar(255,0,0), (cv::Rect*)0, cv::Scalar(), cv::Scalar(200,200,200));
            }

        }
    }
    imshow("img",image);
    waitKey(0);
}

int main(int argc, char* argv[])
{

    Mat img=imread("picture/9.png");

//    Vec3b lane_color(193,195,194);//bgr
//    Vec3b road_color(106,108,108);
//    Vec3b tree_color(94,98,88);
//    int min_dist=60;
//    Vec3b target_color(lane_color);
//    test_color_dist(img,target_color,min_dist);


    test_floodfill();

    return 0;
}
