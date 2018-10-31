
#include "weighted_fit.h"

#include <cmath>

using namespace std;

namespace weighted_fit {


CV_IMPLEMENT_QSORT(IntQSort, double, cmp_pts)  // 该宏利用声明并定义函数IntQSort用于快速排序




void weightedFit(double x[], double y[], int count, LinePara* estLinePara)
{
    double wValue[MAX_FITPOINTS_CNT];   // 权值系数
    // 加权最小二乘法
    // count: 数据点计数
    // estLinePara : 直线拟合的估计值，可以利用最小二乘法计算得到
    // 利用最小二乘进行估计
    double* temp;
    int flagFlip = 0;// 是否对X,Y进行翻转过
    //FitPara(X , Y , Cnt , EstLinePara , NULL);
    //if(abs(EstLinePara->a) > 1 || EstLinePara->a == NAN || EstLinePara->a == -NAN)
    if(fabs(x[0] - x[count - 1]) < fabs(y[0] - y[count - 1]))
    {
        // 该段直线为斜率大于1
        // 沿45度线进行翻转
        // 将 X 和 Y 进行翻转
        temp = x;
        x = y;
        y = temp;
        flagFlip = 1;  // 翻转
    }


    int i = 0;
    if(wValue == NULL)
        return;
    // 迭代20次
    for(i = 0 ; i < 20 ; i++)
    {
        // 计算权值
        calW(x, y, count, estLinePara, wValue);
        fitPara(x, y, count, estLinePara, wValue);// 根据权值拟合参数
    }
    //free(W);
    // EstLinePara->Dis = abs(EstLinePara->b)/(sqrt(EstLinePara->a * EstLinePara->a + EstLinePara->b * EstLinePara->b));
    if(flagFlip == 0)
    {
        // 未翻转
        estLinePara->Rho = atan(estLinePara->a);
    }
    else
    {
        // 翻转过
        if(fabs(estLinePara->a) < 0.00001)
        {
            estLinePara->a = 100000;
        }
        else
        {
            estLinePara->a = 1.0/ estLinePara->a;
        }
        estLinePara->b = -estLinePara->b * (estLinePara->a);
        estLinePara->Rho = atan(estLinePara->a);
    }

    //X Y若翻转过，再翻转回去
    if(flagFlip == 1)
    {
        // 该段直线为斜率大于1
        // 沿45度线进行翻转
        // 将 X 和 Y 进行翻转
        temp = x;
        x = y;
        y = temp;
    }

    //计算线段的两个端点
    if (abs(estLinePara->a) >= 30000)
    {
        estLinePara->startPoint.y = y[0];
        estLinePara->startPoint.x = (y[0] - estLinePara->b)/estLinePara->a;

        estLinePara->endPoint.y = y[count-1];
        estLinePara->endPoint.x = (y[count-1] - estLinePara->b)/estLinePara->a;
    }
    else
    {
        estLinePara->startPoint.x = x[0];
        estLinePara->startPoint.y = estLinePara->a* x[0] + estLinePara->b;

        estLinePara->endPoint.x = x[count-1];
        estLinePara->endPoint.y = estLinePara->a* x[count-1] + estLinePara->b;
    }


    // ===== calculate Variance about the Least_square_best_fit Line. ==============
    double deltaX = estLinePara->endPoint.x - estLinePara->startPoint.x;
    double deltaY = estLinePara->endPoint.y - estLinePara->startPoint.y;

    double dis = sqrt(pow(deltaX, 2.0) + pow(deltaY, 2.0));
    double cosTheta =  deltaX/ dis;
    double sinTheta = -deltaY/ dis;

    double dDis;
    double sum = 0;
    for (int i = 0; i < count; i++)
    {
        dDis = fabs((y[i] - y[0])*cosTheta + (x[i] - x[0])*sinTheta);
        sum += dDis * dDis;
    }

    estLinePara->standardDeviation = sqrt(sum/count);

    //xiwrong-->todo //should delete this return
//    return sqrt(sum/count);

}


double med(double r[], int count)// 求取中值
{
    //qsort(R , Cnt , sizeof(R[0]) , Cmp);
    IntQSort(r , count , 0);
    return r[count/2];
}

int calW(double x[] , double y[] , int count, LinePara * estLinePara , double w[])
{
    int i = 0;
    double a = (double)estLinePara->a;
    double b = (double)estLinePara->b;

    int median = 0;
    double u;
    double tmp;
    for(i = 0; i < count ; i++)
    {
        tmp = (int)fabs(y[i] - a* x[i] - b );
        w[i] = tmp;
    }
    median = med(w , count);
    median = median > 2 ? median : 2;

    for(i = 0 ; i < count ; i++)
    {
        u =(double)(w[i]/(K * median));

        if(u < 1)
        {
            w[i] =(double)((1 - u * u) * (1 - u * u) * 100);   //将W范围限制在0-100
            //W[i] = (int)((1-u)*(1-u)*100);
        }
        else{
            w[i] = 0;
        }
    }

    return 0;
}

int fitPara(double x[], double y[], int count, LinePara* estLinePara, double w[])
{

    int i = 0;
    double P1 = 0; // sum(wi*xi*yi);
    double P2 = 0; // sum(wi * xi * xi)
    double P3 = 0; // sum(wi * xi)
    double P4 = 0; // sum(wi * yi)
    double P5 = 0; // sum(wi)
    if(w == NULL) // 直接进行最小二乘拟合，即所有数据的权值相等
    {
        //
        for( i = 0 ; i < count ;i++)
        {
            P1 += x[i] * y[i];
            P2 += x[i] * x[i];
            P3 += x[i];
            P4 += y[i];
            P5 += 1;
        }
    }
    else{ //加权最小二乘拟合
        for( i = 0 ; i < count ;i++)
        {
            P1 += w[i] * x[i] * y[i];
            P2 += w[i] * x[i] * x[i];
            P3 += w[i] * x[i];
            P4 += w[i] * y[i];
            P5 += w[i];
        }
    }

    estLinePara->a = ((P1*P5 - P4*P3)/ (P2*P5 - P3*P3));
    estLinePara->b = (P1 - P2 * estLinePara->a)/P3;
    return 0;
}


}
