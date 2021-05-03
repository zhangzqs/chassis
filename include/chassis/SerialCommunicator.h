//
// Created by sit on 2021/4/25.
//
#pragma once

#include "ACommunicator.h"
#include <serial/serial.h>

class SerialCommunicator:public ACommunicator{

private:
    serial::Serial& ser;
public:
    explicit SerialCommunicator(serial::Serial &ser):ser(ser){}

    void writeByte(uint8_t byte) override{
        ser.write(&byte,1);
    }
    uint8_t readByte() override{
        uint8_t b;
        ser.read(&b,1);
        return b;
    }


    std::vector<uint8_t> readAvailableBytes() override{
        int len = int(ser.available());
        std::vector<uint8_t> bytes;
        bytes.reserve(len);
        ser.read(bytes,len);
        return bytes;
    }

    uint8_t available() override{
        return ser.available();
    }

    void writeBytes(uint8_t *byteArr,int len) override{
        ser.write(byteArr,len);
    }
};
