#ifndef _EQUALIZATION_H_
#define _EQUALIZATION_H_
#include <iostream>
#include <cmath>
#include <time.h>
#include "CImg.h"
using namespace std;
using namespace cimg_library;

// 多线程传参
typedef struct PT{
    int i;
    string fileDic;
    PT(){}
}PT;

class equalization
{
private:
    // 转换为灰度图像
    void toGrayScale();

    // 灰度图像的均衡化
    void grayHistogramEqualization();
    
    // 彩色图像的均衡化
    void colorHistogramEqualization();

    // 均衡化的过程
    void equalizationStep(CImg<unsigned char> &);

    // 输入图像
    CImg<unsigned char> inputImg;

    // 灰度图像
    CImg<unsigned char> grayScale;
    CImg<unsigned char> grayOutput;

    // 彩色图像
    CImg<unsigned char> colorOutput;
public:
    equalization(CImg<unsigned char>&, int);
    ~equalization(){}
};


#endif