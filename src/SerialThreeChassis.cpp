
#include <serial/serial.h>
#include <ros/ros.h>
#include "chassis/SerialThreeChassis.h"
SerialThreeChassis::SerialThreeChassis(double radius,SerialCommunicator& serialCommunicator)
:AThreeChassis(radius),serialCommunicator(serialCommunicator){

}

void SerialThreeChassis::setPrimitiveSpeed(int v1, int v2, int v3)
{
    //ROS_INFO("Set wheel speed [%d %d %d]",v1,v2,v3);
    
    //消除负数
    v1 += 5000;
    v2 += 5000;
    v3 += 5000;

    //待发送的串口数据包
    CmdDataFrame frame{0xA6,{
            uint8_t(v1 >> 8),
            uint8_t(v1 & 0xff),
            uint8_t(v2 >> 8),
            uint8_t(v2 & 0xff),
            uint8_t(v3 >> 8),
            uint8_t(v3 & 0xff),
    }};
    serialCommunicator.sendFrame(frame);
}

//发送请求速度指令
void SerialThreeChassis::requestSpeed(){
    CmdDataFrame frame{0xA5};
    serialCommunicator.sendFrame(frame);
}

void SerialThreeChassis::getPrimitiveSpeed(int &v1,int &v2,int &v3)
{
    CmdDataFrame frame;
    do{
        do{
            requestSpeed();
        } while (!serialCommunicator.receivable());
        frame = serialCommunicator.receive();
    } while (frame.cmd != 0x5A);

    //此时已经获取到了轮速在数组pkg中
    v1 = (frame.data[0] << 8 | frame.data[1]) - 5000;
    v2 = (frame.data[2] << 8 | frame.data[3]) - 5000;
    v3 = (frame.data[4] << 8 | frame.data[5]) - 5000;
    //ROS_INFO("Received wheel speed [%d %d %d]",v1,v2,v3);
}

void SerialThreeChassis::setWheelSpeed(double v1,double v2,double v3)
{
    setPrimitiveSpeed(int(v1*K),int(v2*K),int(v3*K));
}

void SerialThreeChassis::getWheelSpeed(double& v1,double& v2,double& v3)
{
    int ps1,ps2,ps3;
    getPrimitiveSpeed(ps1,ps2,ps3);
    v1 = ps1/K;
    v2 = ps2/K;
    v3 = ps3/K;
}