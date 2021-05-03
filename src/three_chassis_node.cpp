#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include "chassis/SerialThreeChassis.h"
#include "chassis/SerialCommunicator.h"
#include "chassis/ImuReader.h"

serial::Serial ser; //串口对象
serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); //设置串口超时时间1s
SerialCommunicator sc(ser);
SerialThreeChassis chassis(154.3 / 1000, sc);
ImuReader imuReader(sc);

double cmd_vx, cmd_vy, cmd_vw;
/**
 * @brief 速度控制指令接收回调函数
 * 
 * @param ptr 速度控制消息
 */
void cmdCallback(const geometry_msgs::Twist::ConstPtr& ptr) {
    cmd_vx = ptr->linear.x;
    cmd_vy = ptr->linear.y;
    cmd_vw = ptr->angular.z;
}

bool initSerial(const std::string& serial_port,uint32_t baud_rate){

    ROS_INFO("Try Serial port: %s", serial_port.c_str());
    ROS_INFO("Serial baud rate: %d", baud_rate);

    ser.setPort(serial_port);   //设置串口端口
    ser.setBaudrate(baud_rate); //设置串口波特率
    ser.setTimeout(timeout);    //设置串口超时时间

    try {
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR("Unable to open port %s", serial_port.c_str());
        return false;
    }
    if (!ser.isOpen()) {
        ser.close();
        return false;
    }
    ROS_INFO("Serial port initialized");
    return true;
}

geometry_msgs::TwistStamped getSpeedMsg(){
    static int seq = 0;
    double rx_vx, rx_vy, rx_vw;   //从底盘接收的速度
    chassis.getSpeed(rx_vx, rx_vy, rx_vw);
    geometry_msgs::TwistStamped speedMsg;
    speedMsg.header.frame_id = "/base_link";
    speedMsg.header.seq = seq++;
    speedMsg.header.stamp = ros::Time::now();
    speedMsg.twist.linear.x = rx_vx;
    speedMsg.twist.linear.y = rx_vy;
    speedMsg.twist.angular.z = rx_vw;
    return speedMsg;
}

void publishImuTf(){
    tf2_ros::StaticTransformBroadcaster pub;
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();    //获取时间
    tfs.header.frame_id = "/base_link";   //参考系
    tfs.child_frame_id = "/imu_link";   //子坐标系，imu坐标系
    pub.sendTransform(tfs);
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "three_chassis");   //初始化三轮底盘节点
    ros::NodeHandle nh("~");


    if (nh.hasParam("k")) {
        double k;
        nh.getParam("k", k);
        chassis.setK(k);
    }
    if (nh.hasParam("radius")) {
        double radius;
        nh.getParam("radius", radius);
        chassis.setChassisRadius(radius);
    }

    std::string serial_port; //串口端口号
    int baud_rate = 115200;  //串口波特率

    std::vector<std::string> vs;
    nh.getParamNames(vs);
    for(auto s:vs){
        ROS_INFO("%s",s.c_str());
    }
    if (nh.hasParam("serial_port")) {
        nh.getParam("serial_port", serial_port);
    }
    if (nh.hasParam("baud_rate")) {
        nh.getParam("baud_rate", baud_rate);
    }

//    if(!initSerial(serial_port,baud_rate)){
//        return -1;
//    }

    if(serial_port.empty()){
        ROS_INFO("Not found device!!! Maybe you forgot run device_detector_node to detect device.");
        return -1;
    }

    initSerial(serial_port,baud_rate);

    ros::Subscriber cmd_subber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdCallback);   //速度订阅
    ros::Publisher speed_pubber = nh.advertise<geometry_msgs::TwistStamped>("/real_speed", 10);  //编码器速度发布
    //ros::Publisher imu_pubber = nh.advertise<sensor_msgs::Imu>("/imuData",10);    //IMU数据发布


    for (ros::Rate rate(20); ros::ok(); rate.sleep()) {
        speed_pubber.publish(getSpeedMsg());
        //auto imuMsg = imuReader.getImuData();
        //imu_pubber.publish(imuMsg);
        chassis.setSpeed(cmd_vx, cmd_vy, cmd_vw);   //定周期设置速度
        ros::spinOnce();
    }

    ser.close();
    return 0;
}