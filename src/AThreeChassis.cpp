#include "chassis/AThreeChassis.h"
#include <cmath>

//定义要用到的常量，方便计算
const double M_PI_6 = M_PI/6;

void AThreeChassis::setSpeed(double vx,double vy,double vw)
{
    //需要将速度进行分解到三个轮子上
    double vb = -vy + vw * L;
    double vl = -vx * cos(M_PI_6) + vy * sin(M_PI_6) + vw * L;
    double vr = vx * cos(M_PI_6) + vy * sin(M_PI_6) + vw * L;

    //设置轮速
    setWheelSpeed(vb,vr,vl);
}
void AThreeChassis::getSpeed(double& vx,double& vy,double& vw)
{
    //需要获取三个轮子的轮速，合成出轮子速度

    //获取三个轮子速度
    double vf,vl,vr;
    getWheelSpeed(vf,vl,vr);

    //速度合成
    vx = (vl - vr) * (sqrt(3) / 3);
    vy = (-2 * vf + vl + vr) / 3;
    vw = (vf + vl + vr) / (3*L);
}