
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

#include "str_common.h"
#include "MSAC.h"

void preprocess(Mat& src,Mat& gray,Mat& edge){
    Mat erode_pic,dilate_pic;
//    blur(src,src,Size(5,5),Point(-1,-1));
    GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );
    //    erode(src, erode_pic,Mat());
    //    imshow("erode_pic",erode_pic);
    //    dilate(erode_pic,dilate_pic,Mat());
    //    imshow("dilate_pic",dilate_pic);


    cvtColor(src,gray,CV_BGR2GRAY);
//    Laplacian( gray, gray, CV_8UC1, 3, 1, 0, BORDER_DEFAULT );
    Canny(gray,edge,100,100);


}

void get_lines(Mat& edge,float min_k_pos,float max_k_pos,float min_k_neg,float max_k_neg,vector<Vec4i> &lines){


    vector<Vec4i> tmp_lines,tmp_lines2;
    HoughLinesP(edge,tmp_lines,1,CV_PI/180,50,6,2);

    float k;
    int neg_slope_max_x=0;
    int pos_slope_min_x=0;
    for(size_t i=0;i<tmp_lines.size();i++){
        Vec4i l=tmp_lines[i];
        if(l[2]==l[0]){
            cout<<"vertical line..."<<endl;

            continue;
        }
        k=(double)(l[3]-l[1])*1.0/(l[2]-l[0]);

        if(k<0 && fabs(k)>=min_k_neg && fabs(k)<=max_k_neg
                ||
           k>=0 && fabs(k)>=min_k_pos && fabs(k)<=max_k_pos){
            tmp_lines2.push_back(l);

            if(k<0){
                if(l[2]>neg_slope_max_x){
                    neg_slope_max_x=l[2];
                }
                if(l[0]>neg_slope_max_x){
                    neg_slope_max_x=l[0];
                }
            }
            if(k>0){
                if(l[3]<pos_slope_min_x){
                    pos_slope_min_x=l[3];
                }
                if(l[1]<pos_slope_min_x){
                    pos_slope_min_x=l[1];
                }
            }
        }
    }

    cout<<"valid_line_cnt="<<lines.size()<<endl;

}


void draw_lines(Mat& src,vector<Vec4i>& lines,Point offset_p){

    int fontFace = FONT_HERSHEY_PLAIN;//FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 1;
    int thickness = 2;

    RNG rng;

    int valid_line_cnt=0;
    char text[128];
    float k;
    for(size_t i=0;i<lines.size();i++){
        Vec4i l=lines[i];

        k=(double)(l[3]-l[1])*1.0/(l[2]-l[0]);
        sprintf(text,"%f",atan(k)* 180 / CV_PI);
        cv::putText(src, text,Point(l[0]+offset_p.x,l[1]+offset_p.y), fontFace, fontScale, Scalar(255,0,0), 1,8);

        line(src,Point(l[0]+offset_p.x,l[1]+offset_p.y),Point(l[2]+offset_p.x,l[3]+offset_p.y),Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)),thickness,CV_AA);

        ++valid_line_cnt;

    }

}

int estimate_vps(Mat& src,vector<Vec4i> &lines,Point offset_p){
    // Multiple vanishing points
    std::vector<cv::Mat> vps;			// vector of vps: vps[vpNum], with vpNum=0...numDetectedVps
    std::vector<std::vector<int> > CS;	// index of Consensus Set for all vps: CS[vpNum] is a vector containing indexes of lineSegments belonging to Consensus Set of vp numVp
    std::vector<int> numInliers;
    std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;


    vector<vector<Point>> lineSegments;
    for(Vec4i line:lines){
        Point sp(line[0]+offset_p.x,line[1]+offset_p.y);
        Point ep(line[2]+offset_p.x,line[3]+offset_p.y);
        vector<Point> seg;
        seg.push_back(sp);
        seg.push_back(ep);
        lineSegments.push_back(seg);
    }

    Size procSize = src.size();
    int mode = MODE_LS;
    bool verbose = false;
    MSAC msac;
    msac.init(mode, procSize, verbose);
    msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, 1);

    for(size_t v=0; v<vps.size(); v++)
    {
        //        printf("VP %d (%.3f, %.3f, %.3f)", v, vps[v].at<float>(0,0), vps[v].at<float>(1,0), vps[v].at<float>(2,0));
        //        fflush(stdout);

        //        vps[v].at<float>(1,0)=vps[v].at<float>(1,0)+ROI_y;
        //        vps[v].at<float>(0,0)=vps[v].at<float>(0,0)+ROI_x;

        //        double vpNorm = cv::norm(vps[v]);
        //        if(fabs(vpNorm - 1) < 0.001)
        //        {
        //            printf("(INFINITE)");
        //            fflush(stdout);
        //        }
        //        printf("\n");
    }
    // Draw line segments according to their cluster
    msac.drawCS(src, lineSegmentsClusters, vps);
}

//自定义排序函数
bool SortByName( const string &v1, const string &v2)//注意：本函数的参数的类型一定要与vector中元素的类型一致
{
    return v1 < v2;//升序排列
}
int main(int argc,char* argv[]){
    Mat gray,edge;
    Mat src;

    string dir="/home/gumh/TrainData/KITTI/04/image_0/";//home/gumh/Pictures/road/";//
    vector<string> all_files=getAllFilesWithPathFromDir(dir);
    std::sort(all_files.begin(),all_files.end(),SortByName);

    float min_k_pos=tan((double)CV_PI/180*45);
    float max_k_pos=tan((double)CV_PI/180*80);

    float min_k_neg=tan((double)CV_PI/180*40);
    float max_k_neg=tan((double)CV_PI/180*50);

    string filePath,OnlyName,suffix;


    int m=0;
    int width,height;
    while(true){
        if(m>=all_files.size()){
            m=0;
        }
        string file=all_files[m];
        cout<<"file="<<file<<endl;
        src=imread(file);
        if(src.empty()){
            cout<<"wrong picture!"<<endl;
            ++m;
            continue;
        }
        width=src.cols;
        height=src.rows;

        preprocess(src,gray,edge);

        Point offset_p(width/4,height/3);
        int roi_w=width/2;
        Rect roi(offset_p.x,offset_p.y,roi_w,src.rows-offset_p.y);


        vector<Vec4i> lines;
        Mat roi_part=edge(roi);
        get_lines(roi_part,min_k_pos,max_k_pos,min_k_neg,max_k_pos,lines);
        draw_lines(src,lines,offset_p);
        rectangle(src,roi,1);

        //vp
        estimate_vps(src,lines,offset_p);

        getFilePart(file,filePath,OnlyName,suffix);


        imshow("src",src);
        imshow("edge",edge);

        int key=waitKey(0);
//        cout<<"key="<<key<<endl;
        if(key==65361 || key==65362){
            m--;
            if(m<0){
                m=0;
            }
        }
        else if(key==27){
            break;
        }else{
            ++m;
            if(m>=all_files.size()){
                m=0;
            }
        }

    }

    return 0;


}
