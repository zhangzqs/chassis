//
// Created by zzq on 5/1/21.
//

#pragma once

struct ImuData{
    struct {
        float y,p,r;
    } euler;    //欧拉角

    struct {
        float x,y,z,w;
    } quaternion;   //四元数

    struct {
        float x,y,z;
    } angular_vel;  //角速度

    struct {
        float x,y,z;
    } linear_acc;   //线加速度
};
