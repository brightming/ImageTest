#include <iostream>
#include <assert.h>
#include "math/b_spline.h"

/**
 * @brief calc_b_spline_coefficient
 * @param max_control_point_cnt_idx
 * control point的最大下标，从0开始，所以，control point的数量为 max_control_point_cnt_idx+1
 * @param p
 * degree
 * @param max_knots_idx
 * knots的最大下标，所以，knots的数量为 max_knots_idx+1
 * @param knots
 * 具体的konts数量
 * @param u
 * @param coeffis
 * 计算返回的系数 ，长度为 max_control_point_cnt_idx+1
 */
void calc_b_spline_coefficient(int max_control_point_cnt_idx,int p,int max_knots_idx,float* knots,float u,std::vector<float>& coeffis){


//    std::cout<<"max_knots_idx="<<max_knots_idx<<",max_control_point_cnt_idx="<<max_control_point_cnt_idx<<",p="<<p<<std::endl;
    assert(max_knots_idx==max_control_point_cnt_idx+p+1 && "m=n+1+p");


    for(int i=0;i<=max_control_point_cnt_idx;i++){
        coeffis[i]=0;
    }

    int m=max_knots_idx+1;
    float um=knots[m-1];
    float u0=knots[0];

    if(u==u0){
        coeffis[0]=1.0;
        return;
    }
    if(u==um){
        coeffis[max_control_point_cnt_idx]=1.0;
        return;
    }

    assert(u>=u0&&u<=um && "u is invalid!");

    //find u in which segment
    int k=-1;

    for(int i=0;i<m-1;i++){
        if(u>=knots[i] && u<knots[i+1]){
            k=i;
            break;
        }
    }
//    std::cout<<"control_point_cnt="<<max_control_point_cnt_idx+1<<",m="<<m<<",u0="<<u0<<",um="<<um<<",u="<<u<<",k="<<k<<std::endl;

    coeffis[k]=1.0;

    for(int d=1;d<=p;d++){
//        std::cout<<"coeffis[0]="<<coeffis[0]<<
//                ",coeffis[1]="<<coeffis[1]<<",k="<<k<<",d="<<d<<",k+d="<<k+d<<",k-d="<<k-d<<",k+1="<<k+1<<",k-d+1="<<k-d+1<<"\n";
        if(k-d>=0){
            coeffis[k-d]=(knots[k+1]-u)/(knots[k+1]-knots[k-d+1])*coeffis[k-d+1];// right (south-west corner) term only
//            std::cout<<"1 d="<<d<<",k-d="<<k-d<<",coeffis["<<k-d<<"]="<<coeffis[k-d]<<std::endl;
        }
        for(int i=k-d+1;i<=k-1;i++){
            coeffis[i]=(u-knots[i])/(knots[i+d]-knots[i])*coeffis[i]+(knots[i+d+1]-u)/(knots[i+d+1]-knots[i+1])*coeffis[i+1];
//            std::cout<<"2 d="<<d<<",coeffis["<<i<<"]="<<coeffis[i]<<std::endl;
        }
        coeffis[k]=(u-knots[k])/(knots[k+d]-knots[k])*coeffis[k];//  left (north-west corner) term only
//        std::cout<<"3 d="<<d<<",coeffis["<<k<<"]="<<coeffis[k]<<std::endl;
    }
}
