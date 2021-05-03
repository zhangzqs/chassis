//
// Created by zzq on 4/24/21.
//

#pragma once

#include "DataFrameParser.h"
#include "CmdDataFrame.h"
#include <numeric>
#include <ros/ros.h>

class ACommunicator{
private:
    virtual void writeByte(uint8_t byte) = 0;
    virtual uint8_t readByte() = 0;
    virtual uint8_t available() = 0;

    virtual bool readable() {
        return available() > 0;
    };

    virtual std::vector<uint8_t> readAvailableBytes(){
        std::vector<uint8_t> bytes;
        while(readable()){
            bytes.push_back(readByte());
        }
        return bytes;
    }

    virtual void writeBytes(uint8_t *byteArr,int len) {
        for(int i=0;i<len;i++){
            writeByte(byteArr[i]);
        }
    }
private:
    DataFrameParser dataFrameParser;

public:
    /**
     * 发送数据帧
     * @param frame
     */
    void sendFrame(CmdDataFrame& frame){
        std::vector<uint8_t> bytes = frame.generateBytes();
        writeBytes(bytes.data(),int(bytes.size()));
        //ROS_INFO("Has Send %s",frame.toString().c_str());
    }

    /**
     * 是否能够接收数据帧
     * @return 是否能够
     */
    bool receivable(){
        std::vector<uint8_t> bytes = readAvailableBytes();
        dataFrameParser.pushBytes(bytes.data(),int(bytes.size()));
        return !dataFrameParser.empty();
    }

    /**
     * 接收一帧数据
     * @return 一帧数据
     */
    CmdDataFrame receive(){
        auto frame =  dataFrameParser.popFrame();
        //ROS_INFO("Received: %s",frame.toString().c_str());
        return frame;
    }
};
