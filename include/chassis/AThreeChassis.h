#pragma once
#include <ros/ros.h>
#include "./AChassis.h"

/**
 * @brief 抽象的三轮底盘
 * 
 */
class AThreeChassis:public AChassis
{
private:
    double L;   //底盘运动半径L

public:
    /**
     * @brief 设置底盘的运动半径
     * 
     * @param radius 底盘的运动半径
     */
    void setChassisRadius(double radius){
        this->L = radius;
        ROS_INFO("Set Chassis Radius: %f",radius);
    }



protected:
    /**
     * @brief 设置车轮的期望线速度，具体通过怎样的方式下发速度指令，继续由子类实现，
     * 车轮线速度单位为 m/s
     * 
     * @param v1 1号车轮的线速度
     * @param v2 2号车轮的线速度
     * @param v3 3号车轮的线速度
     */
    virtual void setWheelSpeed(double v1,double v2,double v3) = 0;

    /**
     * @brief 获取当前轮速
     * @return 当前轮速
     */
    virtual void getWheelSpeed(double& v1,double& v2,double& v3) = 0;

public:
    //实现父类纯虚函数
    void setSpeed(double vx,double vy,double vw) override;
    void getSpeed(double& vx,double& vy,double& vw) override;

    explicit AThreeChassis(double radius):L(radius){}
};

