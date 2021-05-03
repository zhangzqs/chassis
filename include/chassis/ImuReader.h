#pragma once

#include <sensor_msgs/Imu.h>
#include "SerialCommunicator.h"


class ImuReader{
private:
    SerialCommunicator& serialCommunicator;
    void requestImuData();

public:
    explicit ImuReader(SerialCommunicator &serialCommunicator);
    sensor_msgs::Imu getImuData();


};