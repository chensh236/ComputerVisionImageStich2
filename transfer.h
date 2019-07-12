#ifndef _TRANSFER_H_
#define _TRANSFER_H_
#include <iostream>
#include <time.h>
#include <cmath>
#include "CImg.h"

using namespace std;
using namespace cimg_library;

typedef struct para{
    CImg<float> *input;
    int start;
    int end;
    //false 为RGB->Lab，true为相反
    bool mode;
} para;

class transfer
{
private:
    CImg<float> RGBtoLab(CImg<float>);
    CImg<float> LabToRGB(CImg<float>);
    CImg<float> transferTo(CImg<float>, CImg<float>);
    float meanSrc[3];
    float meanTemplate[3];
    float variableSrc[3];
    float variableTemplate[3];
public:
    transfer(CImg<unsigned char>& src, CImg<unsigned char>& tem, CImg<unsigned char>& output);
    static void RGBtoLab(float, float, float, float*, float*, float*);
    static void LabToRGB(float, float, float, float*, float*, float*);
    // 进行多线程转化
    static void ThreadUser(para *);
    ~transfer(){}
};

#endif