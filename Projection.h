//
// Created by Chen Sihang on 2018/11/26.
//


// 这个类进行投影，将原图像投影到柱面坐标上
#ifndef PROJECTION_H
#define PROJECTION_H
#include "CImg.h"
#include "math.h"
#include <cstdio>
#define CHANNEL_NUM 3
#define ANGLE 15
using namespace std;
using namespace cimg_library;

// 存储像素的RGB值
typedef struct Pixel{
    unsigned char val[CHANNEL_NUM];
    Pixel(){}
    Pixel(unsigned char _r, unsigned char _g, unsigned char _b){
        val[0] = _r;
        val[1] = _g;
        val[2] = _b;
    }
} Pixel;

class Projection{
public:
    // 将图像投影到柱面坐标上
    static CImg<unsigned char> imageProjection(const CImg<unsigned char> &);
    // 进行双线性插值计算
    // cimg : 输入图像
    // float : x
    // float : y
    // int : channel
    static unsigned char bilinearInterpolation(const CImg<unsigned char>&, float, float, int);
};

#endif