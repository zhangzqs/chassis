#pragma once
#include "./AThreeChassis.h"
#include "./SerialCommunicator.h"

/**
 * @brief 串口通讯实现的三轮底盘
 * 
 */
class SerialThreeChassis:public AThreeChassis
{
private:
    SerialCommunicator& serialCommunicator;
    double K = 1;      //比例系数k

public:
    void setK(double k){
        this->K = k;
        ROS_INFO("Set wheel K: %f",k);
    }

protected:
    void setWheelSpeed(double v1,double v2,double v3) override;
    void getWheelSpeed(double& v1,double& v2,double& v3) override;

private:
    /**
     * @brief 设置电机原始转速
     * 每个电机速度取值[-5000,5000]
     * 
     * @param v1 1号电机原始转速
     * @param v2 2号电机原始转速
     * @param v3 3号电机原始转速
     */
    void setPrimitiveSpeed(int v1, int v2, int v3);

    /**
     * @brief 获取电机原始转速
     * 每个电机速度取值[-5000,5000]
     * 
     * @param v1 1号电机原始转速
     * @param v2 2号电机原始转速
     * @param v3 3号电机原始转速
     */
    void getPrimitiveSpeed(int &v1,int &v2,int &v3);

    void requestSpeed();

public:

    explicit SerialThreeChassis(double radius,SerialCommunicator& serialCommunicator);

};