#include "transfer.h"
#define MAX_THREAD 16

transfer::transfer(CImg<unsigned char>& src, CImg<unsigned char>& tem, CImg<unsigned char>& output){
    CImg<float> inputImg(src);
    // 程序计时
    CImg<float> LabInput = RGBtoLab(inputImg);
    CImg<float> templateImg(tem);
    CImg<float> LabTemplate = RGBtoLab(templateImg);
    CImg<float> LabResult = transferTo(LabInput, LabTemplate);
    CImg<float> result = LabToRGB(LabResult);
    output = result;
}

void transfer::ThreadUser(para *parameter){ //线程入口
    // 转换为Lab
    if(parameter->mode == false){
        for(int i = parameter->start; i <= parameter->end; ++i){
        for(int j = 0; j < (*(parameter->input)).width(); ++j){
            float L, a, b;
            // x,y != i, j 列行顺序不能颠倒
            transfer::RGBtoLab( (*(parameter->input))(j, i, 0), (*(parameter->input))(j, i, 1), (*(parameter->input))(j, i, 2), &L, &a, &b);
            (*(parameter->input))(j, i, 0) = L;
            (*(parameter->input))(j, i, 1) = a;
            (*(parameter->input))(j, i, 2) = b;
        }
    }
    // 转换为RGB
    }else if(parameter->mode == true){
        for(int i = parameter->start; i <= parameter->end; ++i){
            for(int j = 0; j < (*(parameter->input)).width(); ++j){
                float X, Y, Z;
                transfer::LabToRGB( (*(parameter->input))(j, i, 0), (*(parameter->input))(j, i, 1), (*(parameter->input))(j, i, 2), &X, &Y, &Z);
                (*(parameter->input))(j, i, 0) = X;
                (*(parameter->input))(j, i, 1) = Y;
                (*(parameter->input))(j, i, 2) = Z;
            }
        }
    }
}

CImg<float> transfer::LabToRGB(CImg<float> input){
    CImg<float> result = input;
    // 满足进行多线程浮点运算的条件
    if(result.height() > MAX_THREAD){
        HANDLE h[MAX_THREAD]; //线程句柄
        para parameter[MAX_THREAD]; // 参数
        int index = 0; //第几个线程
        int step = result.height() / MAX_THREAD + 1;
        for(int i = 0; i < result.height(); i += step, index++){
            // 判断是否溢出
            bool flag = true;
            parameter[index].input = &result;
            parameter[index].start = i;
            parameter[index].mode = true; // 说明转换方向
            if(i + step <= result.height()){
            parameter[index].end = i + step - 1;
            }else{
                parameter[index].end = result.height() - 1;
                flag = false;
            }
            h[index] = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadUser, &parameter[index],1,0); //创建子线程
            if(!flag) break;
        }
        WaitForMultipleObjects(MAX_THREAD, h, TRUE, INFINITE);
        for (int i = 0; i < MAX_THREAD; i++)
        {
            CloseHandle(h[i]);
        }
    } else{
        cimg_forXY(input, x, y){
            float R, G, B;
            LabToRGB( input(x, y, 0), input(x, y, 1), input(x, y, 2), &R, &G, &B);
            result(x, y, 0) = R;
            result(x, y, 1) = G;
            result(x, y, 2) = B;
        }
    }
    return result;
}

CImg<float> transfer::RGBtoLab(CImg<float> input){

    CImg<float> LabImg = input;
    // 满足进行多线程浮点运算的条件
    if(input.height() > MAX_THREAD){
        HANDLE h[MAX_THREAD]; //线程句柄
        para parameter[MAX_THREAD]; // 参数
        int index = 0; //第几个线程
        // 先将图片转换位XYZ格式
        int step = LabImg.height() / MAX_THREAD + 1;
        for(int i = 0; i < LabImg.height(); i += step, index++){
            // 判断是否溢出
            bool flag = true;
            parameter[index].input = &LabImg;
            parameter[index].start = i;
            parameter[index].mode = false; // 说明转换方向
            if(i + step <= LabImg.height()){
            parameter[index].end = i + step - 1;
            }else{
                parameter[index].end = LabImg.height() - 1;
                flag = false;
            }
            h[index] = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadUser, &parameter[index],1,0); //创建子线程
            if(!flag) break;
        }
        WaitForMultipleObjects(MAX_THREAD, h, TRUE, INFINITE);
        for (int i = 0; i < MAX_THREAD; i++)
        {
            CloseHandle(h[i]);
        }
    } else{
        // 图片转换位Lab格式
        cimg_forXY(LabImg, x, y){
            float L, a, b;
            RGBtoLab( LabImg(x, y, 0), LabImg(x, y, 1), LabImg(x, y, 2), &L, &a, &b);
            LabImg(x, y, 0) = L;
            LabImg(x, y, 1) = a;
            LabImg(x, y, 2) = b;
        }
    }
    return LabImg;
}

CImg<float> transfer::transferTo(CImg<float> srcImg, CImg<float> templateImg){
    
    // 求均值
    meanSrc[0] = meanSrc[1] = meanSrc[2] = 0;
    meanTemplate[0] = meanTemplate[1] = meanTemplate[2] = 0;
    cimg_forXY(srcImg, x, y){
        meanSrc[0] += srcImg(x, y, 0);
        meanSrc[1] += srcImg(x, y, 1);
        meanSrc[2] += srcImg(x, y, 2);
    }
    cimg_forXY(templateImg, x, y){
        meanTemplate[0] += templateImg(x, y, 0);
        meanTemplate[1] += templateImg(x, y, 1);
        meanTemplate[2] += templateImg(x, y, 2);
    }

    for(int i = 0; i < 3; ++i){
        meanSrc[i] = meanSrc[i] / (float)(srcImg.width() * srcImg.height());
        meanTemplate[i] = meanTemplate[i] / (float)(templateImg.width() * templateImg.height());
    }

    // 求方差
    variableSrc[0] = variableSrc[1] = variableSrc[2] = 0;
    variableTemplate[0] = variableTemplate[1] = variableTemplate[2] = 0;
    cimg_forXY(srcImg, x, y){
        variableSrc[0] += (srcImg(x, y, 0) - meanSrc[0]) * (srcImg(x, y, 0) - meanSrc[0]);
        variableSrc[1] += (srcImg(x, y, 1) - meanSrc[1]) * (srcImg(x, y, 1) - meanSrc[1]);
        variableSrc[2] += (srcImg(x, y, 2) - meanSrc[2]) * (srcImg(x, y, 2) - meanSrc[2]);
    }

    cimg_forXY(templateImg, x, y){
        variableTemplate[0] += (templateImg(x, y, 0) - meanTemplate[0]) * (templateImg(x, y, 0) - meanTemplate[0]);
        variableTemplate[1] += (templateImg(x, y, 1) - meanTemplate[1]) * (templateImg(x, y, 1) - meanTemplate[1]);
        variableTemplate[2] += (templateImg(x, y, 2) - meanTemplate[2]) * (templateImg(x, y, 2) - meanTemplate[2]);
    }

    for(int i = 0; i < 3; ++i){
        variableSrc[i] = sqrt(variableSrc[i] / (float)(srcImg.width() * srcImg.height()));
        variableTemplate[i] = sqrt(variableTemplate[i] / (float)(templateImg.width() * templateImg.height()));
    }
    CImg<float> result = srcImg;
    // 颜色转换操作
    cimg_forXY(result, x, y) {
        result(x, y, 0) = (result(x, y, 0) - meanSrc[0]) * variableTemplate[0] / variableSrc[0] + meanTemplate[0];
        result(x, y, 1) = (result(x, y, 1) - meanSrc[1]) * variableTemplate[1] / variableSrc[1] + meanTemplate[1];
        result(x, y, 2) = (result(x, y, 2) - meanSrc[2]) * variableTemplate[2] / variableSrc[2] + meanTemplate[2];
    }
    return result;
}

void transfer::RGBtoLab(float R, float G, float B, float *L, float *a, float *b)
{
    // 转化为LMS空间
    float l, m, s;
    l = 0.3811 * R + 0.5783 * G + 0.0402 * B;
    m = 0.1967 * R + 0.7244 * G + 0.0782 * B;
    s = 0.0241 * R + 0.1288 * G + 0.8444 * B;

    if(l == 0) l = 1;
    if(m == 0) m = 1;
    if(s == 0) s = 1;

    l = log(l) / log(10);
    m = log(m) / log(10);
    s = log(s) / log(10);

    // 转换为Lab空间
    const float paraA = 1.0 / sqrt(3);
    const float paraB = 1.0 / sqrt(6);
    const float paraC = 1.0 / sqrt(2);
    *L = paraA * (l + m + s);
    *a = paraB * l + paraB * m - 2.0 * paraB * s;
    *b = paraC * l - paraC * m;
}

void transfer::LabToRGB(float L, float a, float b, float *R, float *G, float *B)
{
     // 转换为LMS空间
     const float paraA = sqrt(3) / 3.0;
     const float paraB = sqrt(6) / 6.0;
     const float paraC = sqrt(2) / 2.0;

     float l, m, s;
     l = paraA * L + paraB * a + paraC * b;
     m = paraA * L + paraB * a - paraC * b;
     s = paraA * L - 2.0 * paraB * a;

     l = pow(10, l);
     m = pow(10, m);
     s = pow(10, s);
    

     // 转化为RGB空间
     *R = 4.4679 * l - 3.5873 * m + 0.1193 * s;
     *G = (-1.2186) * l + 2.3809 * m - 0.1624 * s;
     *B = 0.0497 * l - 0.2439 * m + 1.2045 * s;
    
    *R = *R > 0.0f? (*R < 255.0f? *R : 255.0f) : 0.0f;
    *G = *G > 0.0f? (*G < 255.0f? *G : 255.0f) : 0.0f;
    *B = *B > 0.0f? (*B < 255.0f? *B : 255.0f) : 0.0f;
    
}