
#pragma once

#include "qsort.h"
#include "global_data_structure.h"

namespace weighted_fit {

#define MAX_FITPOINTS_CNT 2000
#define K   5.0 //(4.685 / 0.6745)     /// 加权拟合中的系数


//typedef struct
//{
//    double x;
//    double y;
//} iPoint;


//typedef struct{
//    double a;   //y = a*x + b
//    double b;
//    double Rho; // 该段直线的倾角
//    iPoint startPoint;
//    iPoint endPoint;
//} LinePara;


double med(double r[], int count);// 求取中值
int calW(double x[], double y[], int count, LinePara* estLinePara, double w[]);
int fitPara(double x[], double y[], int count, LinePara* estLinePara, double w[]);

void weightedFit(double x[], double y[], int count, LinePara* estLinePara);
//return variance about the least_square_best_fit line

#define cmp_pts( x, y )   ( x < y )    //  用于快速排序比较x < y , 得到的结果升序排列

}
