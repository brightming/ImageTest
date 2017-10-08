#include "lane/lanemodule.h"
#include <glog/logging.h>

using namespace gumh;



void drawLine(cv::Mat &image, double theta, double rho, cv::Scalar color)
{
    if (theta < PI/4. || theta > 3.*PI/4.)// ~vertical line
    {
        cv::Point pt1(rho/cos(theta), 0);
        cv::Point pt2((rho - image.rows * sin(theta))/cos(theta), image.rows);
        cv::line( image, pt1, pt2, cv::Scalar(255), 1);
    }
    else
    {
        cv::Point pt1(0, rho/sin(theta));
        cv::Point pt2(image.cols, (rho - image.cols * cos(theta))/sin(theta));
        cv::line(image, pt1, pt2, color, 1);
    }
}
/**
计算出的直线信息存放在 line 中，为 cv::Vec4f 类型。line[0]、line[1] 存放的是直线的方向向量。line[2]、line[3] 存放的是直线上一个点的坐标。

如果直线用 y=kx+b 来表示，那么 k = line[1]/line[0]，b = line[3] - k * line[2]。

如果直线用 ρ=xcosθ+ysinθ 来表示， 那么 θ=arctank+π/2
*/

void test_fitline(){

    cv::Mat image=cv::Mat::zeros(300,300,CV_8UC3);

    cv::Vec4f line;
    cv::Point p1(-100,-100),p2(-300,-100),p3(-200,-100);
    vector<cv::Point2f> points;
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);

    cv::fitLine(points,
                line,
                CV_DIST_HUBER   ,
                0,
                0.01,
                0.01);
    cout<<"line[0]="<<line[0]<<","
          <<"line[1]="<<line[1]<<","
            <<"line[2]="<<line[2]<<","
              <<"line[3]="<<line[3]<<endl;

    double cos_theta = line[0];
    double sin_theta = line[1];
    double x0 = line[2], y0 = line[3];

    double phi = atan2(sin_theta, cos_theta) + PI / 2.0;
    double rho = y0 * cos_theta - x0 * sin_theta;

    std::cout << "phi = " << phi / PI * 180 << std::endl;
    std::cout << "rho = " << rho << std::endl;

    drawLine(image, phi, rho, cv::Scalar(0));
    double k = sin_theta / cos_theta;

    double b = y0 - k * x0;

    double x = 0;
    double y = k * x + b;
    std::cout << "k="<<k << std::endl;
    std::cout << "b="<<b << std::endl;
    std::cout<<"y="<<k<<"x+"<<b<<std::endl;
    std::cout<<"y="<<p1.y<<",=== kx+b="<<k*p1.x+b<<endl;


    float phi_degree=phi/PI * 180;
    cout<<"phi_degree="<<phi_degree<<endl;
    cout<<"fabs(phi_degree)="<<fabs(phi_degree)<<endl;
    cout<<"fabs(fabs(phi_degree)-180)="<<fabs(fabs(phi_degree)-180)<<endl;
    cout<<"fabs(fabs(phi_degree)-90)="<<fabs(fabs(phi_degree)-90)<<endl;
    cout<<"fabs(fabs(phi_degree)-270)="<<fabs(fabs(phi_degree)-270)<<endl;
    if(fabs(phi_degree)<0.01 ||
            fabs(fabs(phi_degree)-180)<0.01){
        cout<<"垂直线"<<endl;
    }else if( fabs(fabs(phi_degree)-90)<0.01
               || fabs(fabs(phi_degree)-270)<0.01){
          cout<<"水平线"<<endl;
    }

    StraightLine sl(points);


    cv::imshow("image",image);
    cv::waitKey(0);

}

void draw_straightline(cv::Mat& pict,StraightLine& line,cv::Scalar color=cv::Scalar(255,0,0),int thickness=1){
    if(line.is_vertical){
        cv::Point p1(line.rho,0);
        cv::Point p2(line.rho,pict.rows);

        cv::line(pict,p1,p2,color,thickness);
    }else if(line.is_horizontal){
        cv::Point p1(0,line.rho);
        cv::Point p2(pict.cols,line.rho);

        cv::line(pict,p1,p2,color,thickness);
    }else {
        float x,y;
        bool isok=line.GetX(0,x);

        float x2;
        isok=line.GetX(pict.rows,x2);
        cv::Point p1(x,0);
        cv::Point p2(x2,pict.rows);

        cv::line(pict,p1,p2,color,thickness);
    }
}

void test_intercept(){

    cv::Mat pict=cv::Mat::zeros(500,500,CV_8UC3);

    StraightLine l1,l2;
    vector<cv::Point2f> pts1,pts2;

    pts1.push_back(cv::Point(-100.001,0));
    pts1.push_back(cv::Point(-100,200));
    pts1.push_back(cv::Point(-100.02,200));

    pts2.push_back(cv::Point(300,100));
    pts2.push_back(cv::Point(500,100));

    l1.SetData(pts1);
    l2.SetData(pts2);

    cv::Point2f pt;
    int result=l1.GetInterceptPoint(l2,pt);
    cout<<"result="<<result<<",pt="<<pt<<endl;


    draw_straightline(pict,l1,cv::Scalar(0,255,0));
    draw_straightline(pict,l2);

    if(result)
        cv::circle(pict,pt,8,cv::Scalar(0,0,255),1);


    cv::imshow("pict",pict);

    cv::waitKey(0);


}

void test_cancombine(){
    cv::Mat pict=cv::Mat::zeros(720,1280,CV_8UC3);

    //-----------------1-----------------------//
    cv::Point2f pts1[4];
    pts1[0]=cv::Point2f(54.7732, 466.99);
    pts1[1]=cv::Point2f(51.5567, 459.753);
    pts1[2]=cv::Point2f(84.5876, 445.072);
    pts1[3]=cv::Point2f(87.8041, 452.309);


    for(int i=0;i<4;i++){
        cv::line(pict,pts1[i],pts1[(i+1)%4],cv::Scalar(255,0,0),1);
    }
    //    k=2.25045,b=299.219
    StraightLine l1;
    l1.b=299.219;
    l1.k=2.25045;
    l1.is_valid=true;
    draw_straightline(pict,l1);

    //------------------------2------------------------//
    cv::Point2f pts2[4];
    pts2[0]=cv::Point2f(17.1865, 432.39);
    pts2[1]=cv::Point2f(15.6332, 429.145);
    pts2[2]=cv::Point2f(214.525, 333.919);
    pts2[3]=cv::Point2f(216.078, 337.163);

//  //k=2.09195,b=140.79

    for(int i=0;i<4;i++){
        cv::line(pict,pts2[i],pts2[(i+1)%4],cv::Scalar(0,255,0),1);
    }
    StraightLine l2;
    l2.b=140.79;
    l2.k=2.09195;
    l2.is_valid=true;
    draw_straightline(pict,l2);



    cv::imshow("pict",pict);

    cv::waitKey(0);

}


void test_split_contour(){

    cv::Mat original=cv::imread("/home/gumh/Videos/blobs.jpeg",CV_LOAD_IMAGE_UNCHANGED);
    cv::threshold(original,original,200,255,THRESH_BINARY);
    cv::Mat tmp=original.clone();


    LOG(INFO)<<"channel="<<original.channels()<<endl;


    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(tmp, contours,
                 hierarchy, CV_RETR_CCOMP,
                 CV_CHAIN_APPROX_SIMPLE);

    LOG(INFO)<<"contours size = " <<contours.size();

    char name[64];
    for(int i=0;i<contours.size();i++){
        cv::Mat temp2=cv::Mat::zeros(original.rows,original.cols,CV_8UC1);
        drawContours(temp2, contours,i, Scalar(255), CV_FILLED, 8);
        sprintf(name,"/home/gumh/Videos/blobs/blob-%d.jpeg",i);
        cv::imwrite(name,temp2);
    }
}

void test_sub_lanemarkseg(){
    cv::Mat original=cv::imread("/home/gumh/Videos/blobs/blob-5.jpeg",CV_LOAD_IMAGE_UNCHANGED);
    cv::threshold(original,original,200,255,THRESH_BINARY);
    cv::Mat tmp=original.clone();




    vector< vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(tmp, contours,
                 hierarchy, CV_RETR_EXTERNAL,
                 CHAIN_APPROX_NONE);

    LOG(INFO)<<"contours size = " <<contours.size();


    //只看contour点
    cv::Mat only_cont=original.clone();
    only_cont-=only_cont;
    for(cv::Point2f p:contours[0]){
        only_cont.at<uchar>(p.y,p.x)=255;
    }

    RoadContext *ctx=new RoadContext();
    Road road;
    road.context=ctx;

    LaneMarkingSeg seg(ctx,contours[0]);

    cv::Mat for_draw=original.clone();
    cvtColor(for_draw,for_draw,CV_GRAY2BGR);
    road.DrawOneRectangle(for_draw,seg);
    //求每个子旋转矩形的外接矩形，求其拟合线
    for(int i=100;i<seg.sub_rects.size();i++){
        RotatedRect &r=seg.sub_rects[i];
        Rect r1=r.boundingRect();
        if(r1.tl().x+r1.width>=original.cols){
            r1.width=original.cols-r1.tl().x;
        }
        if(r1.tl().y+r1.height>=original.rows){
            r1.height=original.rows-r1.tl().y;
        }
        cv::rectangle(for_draw,r1,cv::Scalar(0,0,255));


        if(i!=3){
            continue;
        }
        vector< vector<Point>> contours_sub;
        findContours(original(r1), contours_sub,
                     hierarchy, CV_RETR_EXTERNAL,
                     CHAIN_APPROX_NONE);

        vector<cv::Point2f> pts_sub;
        for(cv::Point &p:contours_sub[0]){
            p.x+=r1.x;
            p.y+=r1.y;
            pts_sub.push_back(p);
        }
//        std::copy(contours_sub[0].begin(),contours_sub[0].end(),back_inserter(pts_sub));

        StraightLine line_sub;
        line_sub.SetData(pts_sub);
        line_sub.Draw(for_draw,cv::Scalar(255,255,0),2);


    }




    //整个contour进行拟合
    vector<cv::Point2f> pts;
    std::copy(contours[0].begin(),contours[0].end(),back_inserter(pts));

    StraightLine line;
    line.SetData(pts);
    line.Draw(for_draw,cv::Scalar(255,255,0));



    //在查找过contour的二值图上拟合
    Rect tmp_rc=seg.rotated_rect.boundingRect();




    Rect br=boundingRect(pts);
    cv::Mat br_pict=original(br);


//    cv::imshow("original",original);
    cv::imshow("for_draw",for_draw);
    cv::imshow("only_cont",only_cont);
//    cv::imshow("br_pict",br_pict);
    cv::imshow("tmp",tmp);
    cv::waitKey(0);

}

int main(int argc, char *argv[])
{
    test_sub_lanemarkseg();
    return 0;
}

