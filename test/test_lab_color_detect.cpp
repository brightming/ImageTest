
#include "distance/lab_color_detect.h"



void test_basic_color_fun(){
    cv::Vec3b color_a(243,63,30);
    cv::Vec3b color_b(255,27,0);

    cv::Mat one(100,100,CV_8UC3);
    cv::Mat two(100,100,CV_8UC3);


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


    cv::Mat one_lab,two_lab;
    cvtColor(one, one_lab, CV_BGR2Lab);
    cvtColor(two, two_lab, CV_BGR2Lab);


    cv::imshow("one",one);
    cv::imshow("two",two);
    cv::imshow("o-l",one_lab);
   cv:: imshow("t-l",two_lab);

    cv::Vec3b one_target=one_lab.at<cv::Vec3b>(0,0);
    cv::Vec3b two_target=two_lab.at<cv::Vec3b>(0,0);


    int lab_dist=static_cast<int>(cv::norm<int,3>(cv::Vec3i(one_target[0]-two_target[0],
                              one_target[1]-two_target[1],one_target[2]-two_target[2])));

    int bgr_dist=static_cast<int>(cv::norm<int,3>(cv::Vec3i(color_a[0]-color_b[0],
                                  color_a[1]-color_b[1],color_a[2]-color_b[2])));
    cout<<"lab_dist="<<lab_dist<<",rbg_dist="<<bgr_dist<<endl;

    cv::waitKey(0);
}

void test_color_dist(cv::Mat &img,cv::Vec3b target_color,int min_dist){
    colorDetect cdect;
    cdect.SetMinDistance(min_dist);
    cdect.SetTargetColor(target_color);
    cv::Mat res=cdect.process(img);

    cv::imshow("ori",img);
    cv::imshow("res",res);
    cv::waitKey(0);

}

void test_floodfill(){
    cv::Mat image=cv::imread("picture/9.png");

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
    cv::imshow("img",image);
    cv::waitKey(0);
}

int main(int argc, char* argv[])
{

    cv::Mat img=cv::imread("/home/gumh/tmp/owndata/img/195.jpg");

//    img.resize(cv::Size(img.rows,img.cols));

    cv::resize(img,img,cv::Size(480,360));

    cv::imwrite("/home/gumh/195-480-360.png",img);

//    Vec3b lane_color(193,195,194);//bgr
//    Vec3b road_color(106,108,108);
//    Vec3b tree_color(94,98,88);
//    int min_dist=60;
//    Vec3b target_color(lane_color);
//    test_color_dist(img,target_color,min_dist);




    test_floodfill();

    return 0;
}
