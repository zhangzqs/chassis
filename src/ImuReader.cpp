//
// Created by zzq on 5/1/21.
//

#include "chassis/ImuReader.h"
#include <ros/ros.h>
#include "chassis/ImuData.h"

ImuReader::ImuReader(SerialCommunicator &serialCommunicator) : serialCommunicator(serialCommunicator) {

}

void ImuReader::requestImuData() {
    CmdDataFrame frame{0xA8};
    serialCommunicator.sendFrame(frame);
}

sensor_msgs::Imu ImuReader::getImuData() {
    static uint32_t seq = 0;


    CmdDataFrame frame;
    do {
        do {
            requestImuData();
        } while (!serialCommunicator.receivable());
        frame = serialCommunicator.receive();
    } while (frame.cmd != 0x8A);



    ImuData imuData{};

    memcpy(&imuData, frame.data.data(), 52);

    sensor_msgs::Imu imuMsg;
    imuMsg.header.frame_id = "/imu_link";
    imuMsg.header.stamp = ros::Time::now();
    imuMsg.header.seq = ++seq;
    imuMsg.orientation.x = imuData.quaternion.x;
    imuMsg.orientation.y = imuData.quaternion.y;
    imuMsg.orientation.z = imuData.quaternion.z;
    imuMsg.orientation.w = imuData.quaternion.w;

    ROS_INFO("");

    imuMsg.angular_velocity.x = -imuData.angular_vel.x;
    imuMsg.angular_velocity.y = -imuData.angular_vel.y;
    imuMsg.angular_velocity.z = imuData.angular_vel.z;
    imuMsg.linear_acceleration.x = imuData.linear_acc.x;
    imuMsg.linear_acceleration.y = imuData.linear_acc.y;
    imuMsg.linear_acceleration.z = imuData.linear_acc.z;

    return imuMsg;
}