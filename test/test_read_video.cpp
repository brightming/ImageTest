#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

/* 
show 2 methods to read a video stream from online-camera or offline-file
    C++ Class: VideoCapture
*/

int main()
{

    std::cout << getBuildInformation();

    // VideoCapture打开视频流方法1：直接在创建类时指定参数
//    VideoCapture capFromCamera(0);
    VideoCapture capFromFile("/home/gumh/tmp/hi3519sn10_20161201-150914-026.mp4");

    // VideoCapture打开视频流方法2：通过默认构造函数创建对象，然后通过open方法打开视频流
    //VideoCapture capFromFile;
    //capFromFile.open("E:\\video\\1.avi");

    if (/*!capFromCamera.isOpened() ||*/ !capFromFile.isOpened())
    {
        std::cerr<<"fail to open file~~"<<std::endl;
        return -1;
    }
//    namedWindow("capFromCamera", 1);
    namedWindow("capFromFile", 1);

    cout << "对视频窗口按下任意键退出" << endl;
    while (1)
    {
        Mat frameFromCamera, frameFromFile;
        // get a new frame from camera
//        bool frameA = capFromCamera.read(frameFromCamera);
        // get a new frame from file 
        bool frameB = capFromFile.read(frameFromFile); 

        if (/*!frameA ||*/ !frameB)
        {
            cout << "摄像头关闭或视频文件读取到结尾" << endl;
            break;
        }

//        imshow("capFromCamera", frameFromCamera);
        imshow("capFromFile", frameFromFile);
        // 当前帧被显示后，程序等待33ms,如果期间用户触发一个按键，循环退出，视频读入停止 
        if (waitKey(33) >= 0) 
        {
            cout << "用户按下按键，退出程序" << endl;
            break;
        }
    }
    // 这两句可以省略，在程序结束前，VideoCapture会调用析构函数进行资源的释放
//    capFromCamera.release();
    capFromFile.release();
    destroyAllWindows();
    cin.get();
    return 0;

}
