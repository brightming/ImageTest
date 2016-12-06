#ifndef __LANE_C___H__
#define __LANE_C___H__


#ifdef  __cplusplus
extern "C" {
#endif

#include <opencv2/core/types_c.h>

typedef struct vec4i_c {
    int used;
    int len;
    int* pts;
}vec4i_c;


int hough_color_detect_img_c(IplImage *shrink,vec4i_c *lines,int draw_lines);

int night_lane_detect_img_c(IplImage *shrink,vec4i_c *lines,int draw_lines);


#ifdef  __cplusplus
}
#endif


#endif
