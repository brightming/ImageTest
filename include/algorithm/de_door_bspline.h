#ifndef __DE_DOOR_BSPLINE__
#define __DE_DOOR_BSPLINE__
#include <vector>


/**
 * @brief basic_de_door_spline_coefficient
 * 未优化的算法
 * 参考：http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve-coef.html
 * 计算u在p degree的各系数
 * @param contrl_point_cnt
 * 控制点数量.
 * @param p
 * degree
 * @param knots
 * 节点数据，长度为：contrl_point_cnt+p+1。
 * knots数量，控制点数量，degree的关系，参考：http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve.html
 * @param u
 * 输入的计算值
 * @param coefficients
 * 返回的对应各控制点的系数，长度等于contrl_point_cnt
 */
void basic_de_door_spline_coefficient(int contrl_point_cnt,int p,std::vector<float>& knots,float u,std::vector<float>& coefficients);


#endif ///__DE_DOOR_BSPLINE__
