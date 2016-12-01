#include <opencv2/opencv.hpp>
#include <iostream>
#include <glog/logging.h>

#include"test_hist.h"
#include<glog/logging.h>

using namespace std;
using namespace cv;


 //* 将一个输入图片进行分解，得到不同区域，然后计算不同区域的直方图，对比其相似度。
 //* 找出不一样的第一个区域。
 //*
 //* 后面再输入背景模板的直方图，供计算对比，快速得到第一个不同区域，该区域表示上面放有东西。

void get_grey_hist(Mat& src,Mat& out_hist,bool show_img=true);
void get_hsv_hist(Mat& src,MatND &hist,bool show_img=true);
void get_rbg_hist_3(Mat& src,vector<Mat>& output,bool show_img=true);

void get_rbg_hist(Mat& src,Mat& output,bool show_img=true);

void draw_hist_img(const Mat &src, int hist_size,Mat &dst,Scalar color=Scalar(0,0,255));
bool split_picture(Mat& src,vector<Mat>& result_mat);

//----------------------------------------------------------
void test_compare_region_hist(string input_name);
void test_transform_img(string input_name,string output_name);

//----------------------------------------------------------
void show_gaussion(string input_name);

int main( int argc, char** argv )
{

    Mat src, hsv;
//    if( argc < 2 || !(src=imread(argv[1], 1)).data ){
//        cout<<"usage: " <<argv[0]<<" <image path>"<<endl;
//        return -1;
//    }

    google::InitGoogleLogging(argv[0]);
    google::LogToStderr();


   //Mat hist;
   //get_grey_hist(src,hist);

    Mat src1=imread("/home/gumh/Pictures/blue2.jpg");
    Mat src2=imread("/home/gumh/feirui/test-roi/image-838.jpeg");

//    int64_t st=getTickCount();
//    MatND hsv_hist1;
//    get_hsv_hist(src1,hsv_hist1,true);

//    MatND hsv_hist2;
//    get_hsv_hist(src2,hsv_hist2,true);

   // double dist=compareHist(hsv_hist1,hsv_hist2,CV_COMP_BHATTACHARYYA);
   // int64_t et=getTickCount();
   // LOG(INFO)<<"dist="<<dist<<",cost time="<<(et-st)*1.0f/getTickFrequency()<<"s";


    //vector<Mat> out_hists;
    //get_rbg_hist(src,out_hists);

//    string input_name="/home/gumh/feirui/20160808/侧面/mypic31.jpeg";
//    string output_name="/home/gumh/tmp/affine2.jpeg";

//    test_transform_img(input_name,output_name);
//    test_compare_region_hist(output_name);


////    test_transform_img();

    show_gaussion("/home/gumh/feirui/test-roi/test_middle.jpeg");
//    waitKey(0);
}


void test_transform_img(string input_name,string output_name){
    Mat src;
    src=imread(input_name);
    Mat dst(src.cols,src.rows,src.type());

    transpose(src,dst);
    imwrite("/home/gumh/tmp/b4-affine.jpeg",src);
    imwrite("/home/gumh/tmp/affine1.jpeg",dst);
    flip(dst,dst,0);
   // Point2f center(src.cols/2,src.rows/2);
   // Mat rot_mat=getRotationMatrix2D(center, 90, 1);
   // warpAffine(src,dst,rot_mat,Size(dst.cols,dst.rows));

//    imwrite("/home/gumh/tmp/affine2.jpeg",dst);
    imwrite(output_name,dst);
}

void test_compare_region_hist(string input_name){

    //string input_name="/home/gumh/tmp/affine2.jpeg";
    Mat input_img=imread(input_name);

    Rect mid_rect(310,90,300,1700);
    imwrite("/home/gumh/feirui/test-roi/mid_part.jpeg",input_img(mid_rect));

    input_img=input_img(mid_rect);
    //return;
    //Mat middle_part=src()

    vector<Mat> out_region;

    int64_t st=getTickCount();
    int stand_idx=2;
    split_picture(input_img,out_region);
    for(int i=0;i<out_region.size();i++){
        ostringstream os;
        os<<"/home/gumh/feirui/test-roi/"<<i<<".jpg";
        imwrite(os.str(),out_region[i]);
    }
    MatND hist_0,hist_other;
    get_hsv_hist(out_region[0],hist_0,false);
    for(int method=0;method<=3;method++){

        string method_n;
        switch(method){
        case 0:
            method_n="CV_COMP_CORREL";
            break;
        case 1:
            method_n="CV_COMP_CHISQR";
            break;
        case 2:
            method_n="CV_COMP_INTERSECT";
            break;
        case 3:
            method_n="CV_COMP_BHATTACHARYYA";
            break;
        }

        LOG(INFO)<<method_n<<"----------------- compare with 0";
        //compare with region 0
        for(int i=0;i<out_region.size();i++){
            if(i==stand_idx){
                continue;
            }
            get_hsv_hist(out_region[i],hist_other,false);
            double dist=compareHist(hist_0,hist_other,method);
            LOG(INFO)<<method_n<<":("<<stand_idx<<" AND "<<i<<" ) dist="<<dist;
        }
        //alwayrs compare with next one
//         LOG(INFO)<<method_n<<"----------------- compare with next one";
//        for(int i=0;i<out_region.size()-1;i++){
//            get_hsv_hist(out_region[i],hist_0,false);
//            get_hsv_hist(out_region[i+1],hist_other,false);
//            double dist=compareHist(hist_0,hist_other,method);
//            LOG(INFO)<<method_n<<":("<<i<<" AND "<<(i+1)<<" ) dist="<<dist;
//        }
        int64_t et=getTickCount();
        LOG(INFO)<<method_n<<" cost time="<<(et-st)*1.0f/getTickFrequency()<<"s";
        st=getTickCount();
    }


}

bool split_picture(Mat& src,vector<Mat>& result_mat){

    if(src.empty()){
        cout<<"split_picture fail!picture is empty!"<<endl;
        return false;
    }
//    CHECK_EQ(src.rows,1920)<<"split_picture fail!src.rows!=1920";
//    CHECK_EQ(src.cols,1080)<<"split_picture fail!src.cols!=1080";
//    if(src.rows!=1920 || src.cols!=1080){
//        cout<<"split_picture fail!src.rows!=1920 || src.cols!=1080"<<endl;
//        return false;
//    }

    int width=src.cols;
    int height=src.rows;

    int roi_cnt=10;
    int roi_width=src.cols/3;
    int roi_height=src.rows/roi_cnt;
    Rect raw_roi(roi_width,0,roi_width,height);
    imwrite("/home/gumh/Pictures/roi/raw.jpeg",src(raw_roi));
    for(int i=0;i<roi_cnt;i++){
        Rect src_rect(roi_width,(roi_cnt-i-1)*roi_height,roi_width,roi_height);
        result_mat.push_back(src(src_rect));
    }


//    for(int i=0;i<recordNumber;i++){
//        Rect src_rect(resizeParamTab[i].startX,resizeParamTab[i].startY,resizeParamTab[i].width,resizeParamTab[i].height);  // 0.3m~10.0m
//        result_mat.push_back(src(src_rect));
//    }

    return true;
}



void draw_hist_img(const Mat &src, int hist_size,Mat &dst,Scalar color){

    LOG(INFO)<<"in draw_hist_img...src.type==CV_32F="<<(src.type()==CV_32F?"1":"0")<<",dst.type==CV_32F="<<(dst.type()==CV_32F?"1":"0");
    float histMaxValue = 0;
    for(int i=0; i<hist_size; i++){
        float tempValue = src.at<float>(i);
        if(histMaxValue < tempValue){
            histMaxValue = tempValue;
        }
    }


    int bin_w=cvRound(dst.cols/hist_size);
    float scale = (0.9*dst.rows)/histMaxValue;
    for(int i=0; i<hist_size; i++){
        int intensity = static_cast<int>(src.at<float>(i)*scale);
        line(dst,Point(bin_w*i,dst.rows),Point(bin_w*i,dst.rows-intensity),color);
    }
}

void get_rbg_hist(Mat& src,Mat& output_hist,bool show_img){
    int channels[]={0,1,2};
    int hist_size[]={16,16,16};
    float brange[]={0,256};
    float grange[]={0,256};
    float rrange[]={0,256};
    const float *hist_range[] = { brange,grange,rrange };

    calcHist(&src,1,channels,Mat(),output_hist,3,hist_size,hist_range,true,false);

    normalize(output_hist,output_hist);//使用L2范数将RoiHist直方图原地归一化
}

void get_rbg_hist_3(Mat& src,vector<Mat>& output,bool show_img){

    vector<Mat> rgb_planes;
    split(src,rgb_planes);

    int hist_size=256;
    float range[]={0,256};
    const float* hist_range = { range };

    Mat r_hist,g_hist,b_hist;
    calcHist(&rgb_planes[2],1,0,Mat(),r_hist,1,&hist_size,&hist_range,true,false);
    calcHist(&rgb_planes[1],1,0,Mat(),g_hist,1,&hist_size,&hist_range,true,false);
    calcHist(&rgb_planes[0],1,0,Mat(),b_hist,1,&hist_size,&hist_range,true,false);

    if(show_img){
        int hist_h=600;
        int hist_w=400;

        Mat r_draw_hist(hist_h, hist_w, CV_8UC3, Scalar::all(0));
        draw_hist_img(r_hist,hist_size,r_draw_hist,Scalar(0,0,255));

        Mat g_draw_hist(hist_h, hist_w, CV_8UC3, Scalar::all(0));
        draw_hist_img(g_hist,hist_size,g_draw_hist,Scalar(0,255,0));

        Mat b_draw_hist(hist_h, hist_w, CV_8UC3, Scalar::all(0));
        draw_hist_img(b_hist,hist_size,b_draw_hist,Scalar(255,0,0));


        int64_t ti=getTickCount();

        ostringstream os1,os2,os3,os4;
        os1<<"Source_rgb"<<ti;
        imshow(os4.str(),src);

        os2<<"Red"<<ti;
        imshow(os4.str(),r_draw_hist);

        os3<<"Green"<<ti;
        imshow(os4.str(),g_draw_hist);

        os4<<"Blue"<<ti;
        imshow(os4.str(),b_draw_hist);

    }

}

void get_hsv_hist(Mat& src,MatND &hist,bool show_img){
    Mat hsv;
    int64_t ti=getTickCount();
    cvtColor(src, hsv, CV_BGR2HSV);
    if(show_img){
        ostringstream os,os_h,os_s,os_v;
        os<<"hsv_hsv"<<ti;
        //imshow(os.str(),hsv);
        
        //
        vector<Mat> hsv_channels;
        split(hsv,hsv_channels);
        
        Mat img_h=hsv_channels[0];
        Mat img_s=hsv_channels[1];
        Mat img_v=hsv_channels[2];
        
        
//        os_h<<"hsv_hsv_h"<<ti;
//        imshow(os_h.str(),img_h);
//        os_s<<"hsv_hsv_s"<<ti;
//        imshow(os_s.str(),img_s);
//        os_v<<"hsv_hsv_v"<<ti;
//        imshow(os_v.str(),img_v);
    }

    // Quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30, sbins = 32;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    //MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    calcHist( &hsv, 1, channels, Mat(), // do not use mask
             hist, 2, histSize, ranges,
             true, // the histogram is uniform
             false );

    if(show_img){
        double maxVal=0;
        minMaxLoc(hist, 0, &maxVal, 0, 0);

        int scale = 10;
        Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

        for( int h = 0; h < hbins; h++ )
            for( int s = 0; s < sbins; s++ )
            {
                float binVal = hist.at<float>(h, s);
                int intensity = cvRound(binVal*255/maxVal);
                rectangle( histImg, Point(h*scale, s*scale),
                           Point( (h+1)*scale - 1, (s+1)*scale - 1),
                           Scalar::all(intensity),
                           CV_FILLED );
            }


        ostringstream os,os2;
        
        os<<"source_hsv"<<ti;
        imshow( os.str(), src );

        os2<<"Hist_hsv"<<ti;
        imshow( os2.str(), histImg );
    }
}

void get_grey_hist(Mat& src,Mat& out_hist,bool show_img){
    Mat tmp_src=src;
    if(src.channels()>1){
        cvtColor(src,tmp_src,CV_BGR2GRAY);
    }

    int channels[] = {0};
    MatND hist;
    int bins=256;
    int hist_size[]={bins};
    float grey_range[]={0,255};
    const float* ranges[]={grey_range};

    calcHist(&tmp_src,1,0,Mat(),hist,1,hist_size,ranges,true,false);

    if(show_img){
        //draw the histogram
        int hist_w=600;
        int hist_h=800;
        int bin_w=cvRound(hist_w/bins);

        Mat hist_image(hist_h, hist_w, CV_8UC3, Scalar(0,0,0));

        draw_hist_img(hist,bins,hist_image);


        ostringstream os1,os2;
        int64_t ti=getTickCount();
        os1<<"Source_grey"<<ti;
        imshow(os1.str(),tmp_src);
        os2<<"Hist_grey"<<ti;
        imshow(os2.str(), hist_image );

    }

}

void show_gaussion(string input_name){
    Mat src=imread(input_name);
    Mat dst;
    GaussianBlur( src, dst, Size( 3, 3 ), 0);
    imshow("src",src);
    imshow("dst",dst);
    waitKey(0);
}
