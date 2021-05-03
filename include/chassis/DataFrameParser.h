//
// Created by zzq on 4/24/21.
//

#pragma once
#include <vector>
#include <queue>
#include <numeric>
#include "CmdDataFrame.h"


/**
 * 数据帧解析器
 */
class DataFrameParser {
private:
    std::vector<uint8_t> buffer{};        //字节缓冲队列
    std::queue<CmdDataFrame> frameBuffer{};   //已解析出的帧缓冲队列

private:

    bool checkDataFrame(){
        //数据帧头校验
        if(!(buffer[0] == 0x55 && buffer[1] == 0xAA)) {   //数据帧头错误
            return false;
        }
        //数据帧尾校验
        if(!(buffer[buffer.size() - 2] == 0xBB && buffer[buffer.size() - 1] == 0xCC)){ //数据帧尾错误
            return false;
        }
        //数据长度校验
        int len = buffer[3];
        if(buffer.size() != (2+1+1+len+1+2)){ //若数据长度不一致
            //包头+命令+数据长度n+n字节的数据+校验和+包尾
            return false;
        }

        //数据校验和校验
        //0,1包头 2命令 3数据长度 4数据首字节 4+len-1数据尾字节 4+len校验和 size-2,size-1数据包尾

        //校验和为(命令+数据长度+所有数据之和)&0xff
        uint8_t sum = std::accumulate(buffer.begin()+2,buffer.begin()+4+len,0);
        if(*(buffer.end()-3) != sum){   //若校验和失败
            return false;
        }
        return true;    //校验成功
    }


    CmdDataFrame parseFrame(){  //解析帧数据,此时假设buffer的校验正确
        CmdDataFrame frame;
        frame.cmd = buffer[2];
        int len = buffer[3];
        frame.data.reserve(len);
        for(int i=4;i<=(4+len-1);i++){
            frame.data.push_back(buffer[i]);
        }
        return frame;
    }
public:

    /**
     * 放入一个字节
     */
    void pushByte(uint8_t byte){
        buffer.push_back(byte);
        if(buffer.size() >= 2){    //包头接收完毕
            if(buffer[0] == 0x55 && buffer[1] == 0xAA){   //收到数据帧头
                if(buffer[buffer.size() - 2] == 0xBB && buffer[buffer.size() - 1] == 0xCC){ //收到数据尾
                    //数据包处理逻辑
                    if(checkDataFrame()){   //如果校验正确，那就解析
                        frameBuffer.push(parseFrame());
                    }
                    buffer.clear();
                }
            }else{
                buffer.clear();
            }
        }
    }

    void pushBytes(uint8_t *bytes,int len){
        for(int i=0;i<len;i++){
            pushByte(bytes[i]);
        }
    }

    /**
     * 取出一帧数据和指令
     * @return 一个数据帧
     */
    CmdDataFrame popFrame(){
        CmdDataFrame dataFrame = frameBuffer.front();
        frameBuffer.pop();
        return dataFrame;
    }

    bool empty(){
        return frameBuffer.empty();
    }
};