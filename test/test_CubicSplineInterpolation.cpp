
#include <fstream>
#include "lanedetect/CubicSplineInterpolation.h"

#include "math/img_math.h"
#include <opencv2/opencv.hpp>

void test_catmull_rom(){
    IplImage* img = cvCreateImage(cvSize(300,300), 8, 1);
    for (int i = 0; i < img->height; ++i)
    {
        for (int j = 0; j < img->width; ++j)
        {
            ((char *)(img->imageData + img->widthStep * (i)))[j] = 0;
        }
    }
    CvPoint point0,point1,point2,point3,point4;//3个控制点来做
    point1.x = 50;
    point1.y = 50;
    point2.x = 90;
    point2.y = 120;
    point3.x = 70;
    point3.y = 200;
    point0.x = point1.x+(point1.x-point2.x);
    point0.y = point1.y+(point1.y-point2.y);
    point4.x = point3.x+(point3.x-point2.x);
    point4.y = point3.y+(point3.y-point2.y);

    ((char *)(img->imageData + img->widthStep * (point1.y)))[point1.x] = 255;
    ((char *)(img->imageData + img->widthStep * (point2.y)))[point2.x] = 255;
    ((char *)(img->imageData + img->widthStep * (point3.y)))[point3.x] = 255;

    for (int i = 1; i < 500 ; i++) {

        float t = (float) i * (1.0f / (float) 500);
        float tt = t * t;
        float ttt = tt * t;
        CvPoint pi;
        pi.x = 0.5 * (2*point1.x+(point2.x-point0.x)*t + (2*point0.x-5*point1.x+4*point2.x-point3.x)*tt + (3*point1.x-point0.x-3*point2.x+point3.x)*ttt);
        pi.y = 0.5 * (2*point1.y+(point2.y-point0.y)*t + (2*point0.y-5*point1.y+4*point2.y-point3.y)*tt + (3*point1.y-point0.y-3*point2.y+point3.y)*ttt);
        ((char *)(img->imageData + img->widthStep * (pi.y)))[pi.x] = 255;
    }

    for (int i = 1; i < 500 ; i++) {

        float t = (float) i * (1.0f / (float) 500);
        float tt = t * t;
        float ttt = tt * t;
        CvPoint pi;
        pi.x = 0.5 * (2*point2.x+(point3.x-point1.x)*t + (2*point1.x-5*point2.x+4*point3.x-point4.x)*tt + (3*point2.x-point1.x-3*point3.x+point4.x)*ttt);
        pi.y = 0.5 * (2*point2.y+(point3.y-point1.y)*t + (2*point1.y-5*point2.y+4*point3.y-point4.y)*tt + (3*point2.y-point1.y-3*point3.y+point4.y)*ttt);
        ((char *)(img->imageData + img->widthStep * (pi.y)))[pi.x] = 255;
    }
    cvShowImage("scr", img);
    cvWaitKey(0);
    return ;
}


int test_cubicspline()
{
    double x[22] = {
        926.500000,
        928.000000,
        929.500000,
        931.000000,
        932.500000,
        934.000000,
        935.500000,
        937.000000,
        938.500000,
        940.000000,
        941.500000,
        943.000000,
        944.500000,
        946.000000,
        977.500000,
        980.500000,
        982.000000,
        983.500000,
        985.000000,
        986.500000,
        988.000000,
        989.500000};

    double y[22] = {
        381.732239,
        380.670530,
        380.297786,
        379.853896,
        379.272647,
        378.368584,
        379.319757,
        379.256485,
        380.233150,
        378.183257,
        377.543639,
        376.948999,
        376.253935,
        198.896327,
        670.369434,
        374.273702,
        372.498821,
        373.149402,
        372.139661,
        372.510891,
        372.772791,
        371.360553};

    std::vector<double> input_x(22), input_y(22);
    for ( int i=0; i<22; i++)
    {
        input_x[i] = x[i];
        input_y[i] = y[i];
    }


    CubicSplineCoeffs *cubicCoeffs;
    CubicSplineInterpolation cubicSpline;
    cubicSpline.calCubicSplineCoeffs(input_x, input_y, cubicCoeffs, CUBIC_NATURAL, CUBIC_MEDIAN_FILTER);

    std::vector<double> output_x, output_y;
    cubicSpline.cubicSplineInterpolation( cubicCoeffs, input_x, output_x, output_y );

    double xx(946.0), yy(0.0);
    cubicSpline.cubicSplineInterpolation2(cubicCoeffs, input_x, xx, yy);
    std::cout<<yy<<std::endl;

    std::ofstream outfile( "test.txt", std::ios::out );
    if ( outfile )
    {
        for ( int i=0; i<output_y.size(); i++ )
        {
            outfile<<std::fixed<<setprecision(3)<<output_x[i]<<" "<<output_y[i]<<std::endl;
        }
    }
    outfile.close();

    return 0;
}


void test_cubicspline_mat(){
    double x[7]={
        10,
        20,
        30,
        40,
        50,
        60,
        70
    };
    double y[7]={
        60,50,40,20,17,14,10
    };


    Mat src=Mat::zeros(100,100,CV_8UC3);


    std::vector<double> input_x(7), input_y(7);
    for ( int i=0; i<7; i++)
    {
        input_x[i] = x[i];
        input_y[i] = y[i];

        circle(src,Point(x[i],y[i]),3,Scalar(0,0,255),2);
    }

    CubicSplineCoeffs *cubicCoeffs;
    CubicSplineInterpolation cubicSpline;
    cubicSpline.calCubicSplineCoeffs(input_x, input_y, cubicCoeffs, CUBIC_NATURAL, CUBIC_MEDIAN_FILTER);

    std::vector<double> output_x, output_y;
    cubicSpline.cubicSplineInterpolation( cubicCoeffs, input_x, output_x, output_y );


    for(int i=0;i<output_x.size()-1;i++){
        line(src,Point(output_x[i],output_y[i]),Point(output_x[i+1],output_y[i+1]),Scalar(255,0,0),1);
    }

    imshow("src",src);
    waitKey(0);

}


void test_least_square_mat(){
    double x[7]={
        10,
        20,
        30,
        40,
        50,
        60,
        70
    };
    double y[7]={
        60,50,40,20,17,14,10
    };


    Mat src=Mat::zeros(100,100,CV_8UC3);


    std::vector<double> input_x(7), input_y(7);
    for ( int i=0; i<7; i++)
    {
        input_x[i] = x[i];
        input_y[i] = y[i];

        circle(src,Point(x[i],y[i]),3,Scalar(0,0,255),2);
    }




    int sizenum;
    double P[6];
    int dimension = 2;	//5次多项式拟合
    sizenum = sizeof(x)/ sizeof(x[0]);	//	拟合数据的维数
    polyfit(sizenum, y, y, dimension, P);

    printf("拟合系数, 按升序排列如下:\n");
    for (int i=0;i<dimension+1;i++)				//这里是升序排列，Matlab是降序排列
    {
        printf("P[%d]=%lf\n",i,P[i]);
    }
    double test_y[sizenum];
    for(int i=0;i<sizenum;i++){
        test_y[i]=P[0]+P[1]*pow(x[i],1)+P[2]*pow(x[i],2)+P[3]*pow(x[i],3)+P[4]*pow(x[i],4)+P[5]*pow(x[i],5);
        printf("yy[%d]=%f,test_y=%f\n",i,y[i],test_y[i]);
    }


    //在图上画线
    std::vector<double> output_x, output_y;
    for(int i=0;i<src.rows;i++){
        int j=P[0]+P[1]*pow(i,1)+P[2]*pow(i,2)+P[3]*pow(i,3)+P[4]*pow(i,4)+P[5]*pow(i,5);
        output_x.push_back(i);
        output_y.push_back(j);

    }

    for(int i=0;i<output_x.size()-1;i++){
        line(src,Point(output_x[i],output_y[i]),Point(output_x[i+1],output_y[i+1]),Scalar(255,0,0),1);
    }

    imshow("src",src);
    waitKey(0);

}

void test_polyfit(){
    int i, sizenum;
    double P[6];
    int dimension = 5;	//5次多项式拟合
    //	要拟合的数据
    double xx[]=  {0.995119, 2.001185, 2.999068, 4.001035, 4.999859, 6.004461, 6.999335,
                   7.999433, 9.002257, 10.003888, 11.004076, 12.001602, 13.003390, 14.001623, 15.003034,
                   16.002561, 17.003010, 18.003897, 19.002563, 20.003530};
    double yy[] = {-7.620000, -2.460000, 108.760000, 625.020000, 2170.500000, 5814.580000,
                   13191.840000, 26622.060000, 49230.220000, 85066.500000, 139226.280000, 217970.140000, 328843.860000,
                   480798.420000, 684310.000000, 951499.980000, 1296254.940000, 1734346.660000, 2283552.120000, 2963773.500000};

    sizenum = sizeof(xx)/ sizeof(xx[0]);	//	拟合数据的维数
    polyfit(sizenum, xx, yy, dimension, P);

    printf("拟合系数, 按升序排列如下:\n");
    for (i=0;i<dimension+1;i++)				//这里是升序排列，Matlab是降序排列
    {
        printf("P[%d]=%lf\n",i,P[i]);
    }
    double test_y[sizenum];
    for(i=0;i<sizenum;i++){
        test_y[i]=P[0]+P[1]*pow(xx[i],1)+P[2]*pow(xx[i],2)+P[3]*pow(xx[i],3)+P[4]*pow(xx[i],4)+P[5]*pow(xx[i],5);
        printf("yy[%d]=%f,test_y=%f\n",i,yy[i],test_y[i]);
    }

}

int main(){
//    test_polyfit();
    test_least_square_mat();
}
