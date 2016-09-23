
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


void get_vp(Mat& src,Mat& gray,Mat& edge,vector<Vec4i> &lines){

    Mat erode_pic,dilate_pic;
    blur(src,src,Size(3,3),Point(-1,-1));
    erode(src, erode_pic,Mat());
    imshow("erode_pic",erode_pic);
//    dilate(erode_pic,dilate_pic,Mat());
//    imshow("dilate_pic",dilate_pic);

    cvtColor(erode_pic,gray,CV_BGR2GRAY);
    Canny(gray,edge,100,100);




    HoughLinesP(edge,lines,1,CV_PI/180,50,30,10);
}
void draw_lines(Mat& src,vector<Vec4i>& lines,float min_k,float max_k){
    float k;
    cout<<"min_k="<<min_k<<",max_k="<<max_k<<endl;
    for(size_t i=0;i<lines.size();i++){
        Vec4i l=lines[i];
        k=tan((double)(l[3]-l[1])*1.0/(l[2]-l[0]));
        if(k<0){
            k*=-1;
        }
        if(k>=min_k && k<=max_k){

            cout<<"k="<<k<<endl;
            line(src,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,0,255),3,CV_AA);
        }
    }
}

int main(int argc,char* argv[]){
    Mat gray,edge;
    Mat src=imread("/home/gumh/TrainData/KITTI/04/image_0/000000.png");
    if(src.empty()){
        cout<<"wrong picture!"<<endl;
        return -1;
    }
    vector<Vec4i> lines;
    get_vp(src,gray,edge,lines);

    float min_k=tan((double)CV_PI*45/180);
    float max_k=tan((double)89*CV_PI/180);
    draw_lines(src,lines,min_k,max_k);
    imshow("src",src);
    imshow("edge",edge);
    waitKey(0);

    return 0;


}
