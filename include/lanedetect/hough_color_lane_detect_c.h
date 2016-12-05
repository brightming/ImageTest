#ifndef __HOUGH_COLOR_LANE_DETECT_C__
#define __HOUGH_COLOR_LANE_DETECT_C__

#include <opencv2/core/types_c.h>

#ifdef  __cplusplus
extern "C" {
#endif

typedef struct vec4i_c {
    int used;
    int len;
    int* pts;
}vec4i_c;

int hough_color_detect_img_c(IplImage *shrink,vec4i_c *lines,int draw_lines);

#ifdef  __cplusplus
}
#endif


#endif //__HOUGH_COLOR_LANE_DETECT_C__
