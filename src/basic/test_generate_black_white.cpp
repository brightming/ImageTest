#include <iostream>
#include <cv.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


static Scalar randomColor(RNG& rng)
{
    int icolor = (unsigned)rng;
    return Scalar(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
}

void fill_poly(Mat& image){
    char wndname[] = "Drawing Demo";
    const int NUMBER = 100;
    const int DELAY = 5;
    int lineType = 4;//CV_AA; // change it to 8 to see non-antialiased graphics
    int i, width = image.cols, height = image.rows;
    int x1 = -width/2, x2 = width*3/2, y1 = -height/2, y2 = height*3/2;
    RNG rng(0xFFFFFFFF);



    for (i = 0; i< NUMBER; i++)
    {


        Point pt[2][3];
        pt[0][0].x = rng.uniform(x1, x2);
        pt[0][0].y = rng.uniform(y1, y2);
        pt[0][1].x = rng.uniform(x1, x2);
        pt[0][1].y = rng.uniform(y1, y2);
        pt[0][2].x = rng.uniform(x1, x2);
        pt[0][2].y = rng.uniform(y1, y2);
        pt[1][0].x = rng.uniform(x1, x2);
        pt[1][0].y = rng.uniform(y1, y2);
        pt[1][1].x = rng.uniform(x1, x2);
        pt[1][1].y = rng.uniform(y1, y2);
        pt[1][2].x = rng.uniform(x1, x2);
        pt[1][2].y = rng.uniform(y1, y2);



//        const Point* ppt[2] = {pt[0], pt[1]};
//        int npt[] = {3, 3};

//        fillPoly(image, ppt, npt, 2, randomColor(rng), lineType);



        vector<Point> contour;
        contour.push_back(pt[0][0]);
        contour.push_back(pt[0][1]);
        contour.push_back(pt[1][0]);
        contour.push_back(pt[1][1]);
        Mat mat_con(contour);

        int b=(int)rng.uniform(0,255);
        int g=(int)rng.uniform(0,255);
        int red=(int)rng.uniform(0,255);
        for(int r=0;r<image.rows;r++){
            for(int c=0;c<image.cols;c++){
                if(pointPolygonTest(mat_con,Point(r,c),true)>0){
                    image.at<cv::Vec3b>(r,c)[0]=b;
                    image.at<cv::Vec3b>(r,c)[1]=g;
                    image.at<cv::Vec3b>(r,c)[2]=red;
                }
            }
        }

    }



//    Point a,b;
//    a.x=10;
//    a.y=10;
//    b.x=100;
//    b.y=300;
//    line(image,a,b,randomColor(rng),2,4);
}

int test_point_in_polygon(){
    // create a RGB colour image (set it to a black background)

        Mat img = Mat::zeros(400, 400, CV_8UC3);

        // define a polygon (as a vector of points)

        vector<Point> contour;
        contour.push_back(Point(50,50));
        contour.push_back(Point(300,50));
        contour.push_back(Point(350,200));
        contour.push_back(Point(300,150));
        contour.push_back(Point(150,350));
        contour.push_back(Point(100,100));

        // create a pointer to the data as an array of points (via a conversion to
        // a Mat() object)

        const cv::Point *pts = (const cv::Point*) Mat(contour).data;
        int npts = Mat(contour).rows;

        std::cout << "Number of polygon vertices: " << npts << std::endl;

        // draw the polygon

        polylines(img, &pts,&npts, 1,
                    true, 			// draw closed contour (i.e. joint end to start)
                    Scalar(0,255,0),// colour RGB ordering (here = green)
                    3, 		        // line thickness
                    CV_AA, 0);


        // do point in polygon test (by conversion/cast to a Mat() object)
        // define and test point one (draw it in red)

        Point2f test_pt;
        test_pt.x = 150;
        test_pt.y = 75;

        rectangle(img, test_pt, test_pt, Scalar(0, 0, 255), 3, 8, 0); // RED point

        if (pointPolygonTest(Mat(contour), test_pt, true) > 0){
            std::cout << "RED {" << test_pt.x << "," << test_pt.y
                      << "} is in the polygon (dist. "
                      << pointPolygonTest(Mat(contour), test_pt, 1) << ")"
                      << std::endl;
        }

        // define and test point two (draw it in blue)

        test_pt.x = 50;
        test_pt.y = 350;

        rectangle(img, test_pt, test_pt, Scalar(255, 0, 0), 3, 8, 0); // BLUE point

        if (pointPolygonTest(Mat(contour), test_pt, true) < 0){
            std::cout << "BLUE {" << test_pt.x << "," << test_pt.y
                      << "} is NOT in the polygon (dist. "
                      << pointPolygonTest(Mat(contour), test_pt, 1) << ")"
                      << std::endl;
        }

        // pointPolygonTest :-
        // "The function determines whether the point is inside a contour, outside,
        // or lies on an edge (or coincides with a vertex). It returns positive
        // (inside), negative (outside) or zero (on an edge) value, correspondingly.
        // When measureDist=false , the return value is +1, -1 and 0, respectively.
        // Otherwise, the return value it is a signed distance between the point
        // and the nearest contour edge." - OpenCV Manual version 2.1

        // create an image and display the image

        namedWindow("Polygon Test", 0);
        imshow( "Polygon Test", img );
        waitKey(0);

        imwrite("/home/gumh/img.png",img);

        return 0;
}



int main(int argc, char *argv[])
{

//    int height=600;
//    int width=400;
//    Mat image = Mat::zeros(height, width, CV_8UC3);

//    fill_poly(image);
//    imshow("img",image);
//    imwrite("/home/gumh/img.png",image);
//    waitKey(0);

//    test_point_in_polygon();


    char *a;
    a=new char[10];

    *a='b';
    *(a+1)='c';

    std::cout<<"a="<<&a[0]<<endl;

    printf("a=%p ,*a=%c,*(a+1)=%c\n",a,*a,*(a+1));

}
