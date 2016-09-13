
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int erosion_elem = 0;
int erosion_size = 2;
/**  @function Erosion  */
void Erosion( Mat& src,Mat& erosion_dst)
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// 腐蚀操作
  erode( src, erosion_dst, element );
//  imshow( "Erosion Demo", erosion_dst );
}



int main(int argc, char* argv[])
{
    Mat img = imread(argv[1], -1);
    if (img.empty())
    {
        cout <<"Error: Could not load image" <<endl;
        return 0;
    }

    Mat gray;
    cvtColor(img, gray, CV_BGR2GRAY);
    blur(gray,gray,Size(4,4),Point(-1,-1));
    Erosion(gray,gray);
    blur(gray,gray,Size(4,4),Point(-1,-1));

    Mat dst;
    threshold(gray, dst, 0, 255, CV_THRESH_OTSU);

    imshow("src", img);
    imshow("gray", gray);
    imshow("dst", dst);
    waitKey(0);

    return 0;
}
