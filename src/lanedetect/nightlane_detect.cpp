/*TODO
 * improve edge linking
 * remove blobs whose axis direction doesnt point towards vanishing pt
 * Parallelisation
 * lane prediction
*/


#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <time.h>

#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cctype>

using namespace std;
using namespace cv;
clock_t start, stop;



#include "lanedetect/nightlane_detect.h"
#include "str_common.h"



void makeFromFolder(string path){

    vector<string> all_pics=getAllFilesWithPathFromDir(path);
    std::sort(all_pics.begin(),all_pics.end(),less<string>());
    Mat frame=imread(all_pics[0]);
    LaneDetect detect(frame);

    int idx=0;
    while(1)
    {
        if(idx>=all_pics.size()){
            idx=0;
        }
        frame=imread(all_pics[idx++]);

//        cvtColor(frame, frame, CV_BGR2GRAY);

        //start = clock();
        detect.nextFrame(frame);
        //stop =clock();
        // cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;

        int key=waitKey(33);
        if(key==32){ //enter space to pause
            key=waitKey(0);
        }
        if(key==27 || (char)key=='q'){
            break;
        }
//        if(waitKey(10) == 27) //wait for 'esc' key press for 10 ms. If 'esc' key is pressed, break loop
//        {
//            cout<<"video paused!, press q to quit, any other key to continue"<<endl;
//            if(waitKey(0) == 'q')
//            {
//                cout << "terminated by user" << endl;
//                break;
//            }
//        }
    }
}

void makeFromVid(string path)
{
    Mat frame;
    VideoCapture cap(path); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
        cout << "Cannot open the video file" << endl;

    //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Input video's Frame per seconds : " << fps << endl;

    cap.read(frame);
    LaneDetect detect(frame);

    while(1)
    {
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess)                   //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

//        cvtColor(frame, frame, CV_BGR2GRAY);

        //start = clock();
        detect.nextFrame(frame);
        //stop =clock();
        // cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;

        if(waitKey(10) == 27) //wait for 'esc' key press for 10 ms. If 'esc' key is pressed, break loop
        {
            cout<<"video paused!, press q to quit, any other key to continue"<<endl;
            if(waitKey(0) == 'q')
            {
                cout << "terminated by user" << endl;
                break;
            }
        }
    }
}

