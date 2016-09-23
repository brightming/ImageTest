
#include"opencv2/opencv.hpp"
#include <iostream>
#include <math.h>
using namespace cv;
using namespace std;



int main(int argc,char* argv[]){
    Mat pic_mat=imread("/home/gumh/feirui/捷普/80072933141411909.jpg");
    if(pic_mat.empty()){
        cout<<"empty pic!"<<endl;
        return -1;
    }
    transpose(pic_mat,pic_mat);
    flip(pic_mat,pic_mat,1);
//    imshow("dd",pic_mat);
//    waitKey(0);
    vector<Vec4i> lines;
    //---整张图片-----//
    Mat whole_blur_img,whole_blur_gray_img,whole_blur_canny_img;

    blur(pic_mat, whole_blur_img, Size(3,3), Point(-1,-1));
    cvtColor(whole_blur_img,whole_blur_gray_img,CV_BGR2GRAY);
//    auto_canny(whole_blur_gray_img,whole_blur_canny_img);
    Canny(whole_blur_gray_img,whole_blur_canny_img,50,200,3);


    HoughLinesP(whole_blur_canny_img, lines, 1, CV_PI/180, 50, 80, 10 );

    float k;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];

        if(l[0]!=l[2]){
            k=(l[1]-l[3])/(l[0]-l[2]);
        }else{
            k=100000;
        }
        if(k==0){//if(k>=0 && k<=0.01 || k<0 && k>=-0.01){
            line( pic_mat, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
        }else{
            cout<<"omit....line,k="<<k<<endl;
        }
    }
    imwrite("/home/gumh/tmp/with_line.jpeg",pic_mat);
    cout<<"0000"<<endl;
return 0;
}
