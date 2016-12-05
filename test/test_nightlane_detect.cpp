
#include "lanedetect/nightlane_detect.h"

int main()
{
//    makeFromVid("/home/gumh/qtcreator-workspace/lanedetectsrc/Vehicle-Lane-Detection/sample/wandaor.mov");
    // makeFromVid("/home/yash/opencv-2.4.10/programs/road.m4v");
    makeFromFolder("/home/gumh/Videos/challenge");
    waitKey(0);
    destroyAllWindows();
}
