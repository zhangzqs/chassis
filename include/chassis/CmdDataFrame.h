//
// Created by zzq on 4/24/21.
//

#pragma once

#include <vector>
#include <string>
#include <sstream>

/**
 * 数据帧
 */
struct CmdDataFrame{
    uint8_t cmd{};    //指令
    std::vector<uint8_t> data{};  //数据

    std::vector<uint8_t> generateBytes(){
        std::vector<uint8_t> bytes;
        bytes.push_back(0x55);
        bytes.push_back(0xAA);
        bytes.push_back(cmd);
        bytes.push_back(data.size());
        for(uint8_t i : data){
            bytes.push_back(i);
        }
        uint8_t sum = std::accumulate(bytes.begin()+2,bytes.end(),0);
        bytes.push_back(sum);
        bytes.push_back(0xBB);
        bytes.push_back(0xCC);
        return bytes;
    }
    std::string toString(){
        std::stringstream ss;
        ss<<"{cmd:"<<std::hex<<int(cmd)<<"; data:[";
        for(uint8_t dat:data){
            ss<<std::hex<<int(dat)<<" ";
        }
        ss<<"]}";
        return ss.str();
    }
};
