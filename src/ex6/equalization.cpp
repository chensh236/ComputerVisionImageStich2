#include "equalization.h"
#define MAX 256

equalization::equalization(CImg<unsigned char>& src, int mode){
    CImg<unsigned char> temp(src);
    colorOutput = inputImg = temp;
    CImg<int> hist;

    // 判断彩色均衡化和灰度均衡化
    switch(mode){
        case 0:
        // 灰度图像
                toGrayScale();                //Grayscale the image
                grayHistogramEqualization();
                break;
        case 1:
        // 彩色图像
                colorHistogramEqualization();
                break;
        default:
                cout<<"ERROR mode input!"<<endl;
                break;
    }
    src = colorOutput;
}


void equalization::toGrayScale(){
    grayScale = CImg<unsigned char>(inputImg.width(), inputImg.height(), 1, 1, 0); //To one channel
    cimg_forXY(inputImg, x, y)
    {
        int b = inputImg(x, y, 0);
        int g = inputImg(x, y, 1);
        int r = inputImg(x, y, 2);

        double newValue = (r * 0.2126 + g * 0.7152 + b * 0.0722);
        grayScale(x, y) = newValue;
    }
}

void equalization::grayHistogramEqualization(){
    // 对灰度值进行统计
    int grayNum[MAX] = {0};

    cimg_forXY(grayScale, x, y){
        grayNum[ grayScale(x, y) ] += 1;
    }

    // 归一化
    double probabilityDis[MAX] = {0};
    
    for(int i = 0; i < MAX; i++)
    {
        probabilityDis[i] = (double)grayNum[i] / (double)(grayScale.width() * grayScale.height());
    }

    // 求出累计函数以及映射函数
    double sumDis[MAX] = {0};
    int mapped[MAX] = {0};
    sumDis[0] = probabilityDis[0];
    mapped[0] = round(255.0 * sumDis[0]);
    for(int i = 1; i < MAX; i++){
        sumDis[i] = sumDis[i - 1] + probabilityDis[i];
        mapped[i] = round(255.0 * sumDis[i]);
    }
    grayOutput = CImg<unsigned char>(grayScale.width(), grayScale.height(), 1, 1, 0); //To one channel
    // 映射操作
    cimg_forXY(grayScale, x, y){
        int mapped_index = grayScale(x, y) > 0 ? (grayScale(x, y) < 256 ? grayScale(x, y) : 255) : 0;
        grayOutput(x, y) = mapped[mapped_index];
    }
}

void equalization::colorHistogramEqualization(){
    colorOutput = CImg<unsigned char>(inputImg.width(), inputImg.height(), 1, 3, 0); //To one channel
    // 转换YCbCr
    CImg<unsigned char> yCbCr = colorOutput;
    cimg_forXY(inputImg, x, y){
        float Y = 0.299 * (float)inputImg(x, y, 0) + 0.857 * (float)inputImg(x, y, 1) + 0.114 * (float)inputImg(x, y, 2);
        float Cb = 128.0 - 0.168736 * (float)inputImg(x, y, 0) - 0.331264 * (float)inputImg(x, y, 1) + 0.5 * (float)inputImg(x, y, 2);
        float Cr = 128.0 + 0.5 * (float)inputImg(x, y, 0) - 0.418688 * (float)inputImg(x, y, 1) - 0.081312 * (float)inputImg(x, y, 2);
        yCbCr(x, y, 0) = Y > 0 ? (Y < 256 ? Y : 255) : 0;
        yCbCr(x, y, 1) = Cb > 0 ? (Cb < 256 ? Cb : 255) : 0;
        yCbCr(x, y, 2) = Cr > 0 ? (Cr < 256 ? Cr : 255) : 0;
    }

    colorOutput = yCbCr;

    // 只需要对Y进行均衡化
    equalizationStep(yCbCr);
    // 转换RGB
     cimg_forXY(colorOutput, x, y){
        float R = (float)colorOutput(x, y, 0) + 1.402 * ((float)colorOutput(x, y, 2) - 128.0);
        float G = (float)colorOutput(x, y, 0) - 0.34414 * ((float)colorOutput(x, y, 1) - 128.0) - 0.71414 * ((float)colorOutput(x, y, 2) - 128.0);
        float B = (float)colorOutput(x, y, 0) + 1.772 * ((float)colorOutput(x, y, 1) - 128.0);
        colorOutput(x, y, 0) = R > 0 ? (R < 256 ? R : 255) : 0;
        colorOutput(x, y, 1) = G > 0 ? (G < 256 ? G : 255) : 0;
        colorOutput(x, y, 2) = B > 0 ? (B < 256 ? B : 255) : 0;
    }
}

void equalization::equalizationStep(CImg<unsigned char> &yCbCr){
    // 对亮度值进行统计
    int channelNum[MAX] = {0};
    cimg_forXY(yCbCr, x, y){
        channelNum[ yCbCr(x, y, 0) ] += 1;
    }

    // 归一化
    double probabilityDis[MAX] = {0};

    for(int i = 0; i < MAX; i++)
    {
        probabilityDis[i] = (double)channelNum[i] / (double)(yCbCr.width() * yCbCr.height());
    }

    // 求出累计函数以及映射函数
    double sumDis[MAX] = {0};
    int mapped[MAX] = {0};
    sumDis[0] = probabilityDis[0];
    mapped[0] = round(255.0 * sumDis[0]);
    for(int i = 1; i < MAX; i++){
        sumDis[i] = sumDis[i - 1] + probabilityDis[i];
        mapped[i] = round(255.0 * sumDis[i]);
    }
    // 映射操作
    cimg_forXY(yCbCr, x, y){
       int mapped_index = yCbCr(x, y, 0) > 0 ? (yCbCr(x, y, 0) < 256 ? yCbCr(x, y, 0) : 255) : 0;
       colorOutput(x, y, 0) = mapped[mapped_index];
    }
}