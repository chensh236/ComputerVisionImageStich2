#pragma once
#include "CImg.h"
#include <queue>
#include <set>
#include <thread>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include "equalization.h"
#include <cmath>
#include <iostream>
#include <map>
#include <iostream>
#include <cstring>
#include <vector>
#include "Projection.h"
// 用于sift
#define NOTAVES_NUM 4
#define LEVEL_NUM 2
//描述子数量
#define DESCRIPTOR_SUM 128
// KD树查找的距离阈值
#define RATIO_THRESHOLD 0.5
//解出H的随机数对
#define NUM_OF_PAIR 4
#define RANSAC_THRESHOLD 4.0

extern "C"
{
#include "vl/generic.h"
#include "vl/sift.h"
#include "vl/kdtree.h"
};
using namespace std;
using namespace cimg_library;

// 用于存储关键点的对
typedef struct ImgPair
{
    VlSiftKeypoint src;
    VlSiftKeypoint dst;
    ImgPair(VlSiftKeypoint _src, VlSiftKeypoint _dst) : src(_src), dst(_dst) {}
} ImgPair;

// 存储图片的信息，比如cimg 关键点等
typedef struct Image
{
    // 投影后的图像
    CImg<unsigned char> projectedSrc;
    // 存储关键点的特征描述子
    map<vector<float>, VlSiftKeypoint> features;
    Image() {}
} Image;

struct Homography
{
    // 单应性矩阵
    double H[3][3];
    Homography(double w11, double w12, double w13, double w21, double w22, double w23, double w31, double w32)
    {
        H[0][0] = w11;
        H[0][1] = w12;
        H[0][2] = w13;
        H[1][0] = w21;
        H[1][1] = w22;
        H[1][2] = w23;
        H[2][0] = w31;
        H[2][1] = w32;
        H[2][2] = 1;
    }
    Homography() {}
};

// 这个类进行图像处理，包括读取图像、进行sift特征提取等
class ImageProcess
{
  public:
    ImageProcess(string, const int);
    ~ImageProcess()
    {
        delete[] imgs;
    }

    // 这个类进行文件读取操作
    // string : filename
    // int : sum of the pictures
    void readFile(string, const int);

    static CImg<unsigned char> toGrayScale(const CImg<unsigned char> &);

    // 利用sift获取特征点
    // int : 图片的序号
    static map<vector<float>, VlSiftKeypoint> siftAlgorithm(const CImg<unsigned char> &);

  private:
    // 进行图像匹配
    void matching();
    // 求出匹配的关键点
    vector<ImgPair> getImgPair(Image &, Image &);

    //RANSAC
    Homography RANSAC(const vector<ImgPair> &);

    // 求出单应性矩阵
    Homography getHomographyMat(const vector<ImgPair> &);

    // 求出最小距离的索引
    vector<int> getInlinerIndex(const vector<ImgPair> &, Homography &, set<int>);

    // 求解最小距离的H
    Homography getInlinerHomography(const vector<ImgPair>, vector<int>);

    // 求出变换Homography变换后的xy坐标
    float getXAfterWarping(float, float, Homography &);
    float getYAfterWarping(float, float, Homography &);

    // 求出变换后xy的最大值和最小值
    float getMaxXAfterWarping(const CImg<unsigned char> &, Homography &);
    float getMaxYAfterWarping(const CImg<unsigned char> &, Homography &);

    //求出变换到目标图像的值
    void warpingImageByHomography(const CImg<unsigned char> &, CImg<unsigned char> &, Homography &, float, float);
    void movingImageByOffset(const CImg<unsigned char> &, CImg<unsigned char> &, int, int);

    // 多图像拼接
    void updateFeaturesByHomography(map<vector<float>, VlSiftKeypoint> &, Homography &, float, float);
    void updateFeaturesByOffset(map<vector<float>, VlSiftKeypoint> &, int, int);
    // 平滑图像
    CImg<unsigned char> blend(const CImg<unsigned char> &, const CImg<unsigned char> &);

    Image *imgs;
    int picSum;

    // 多线程调优
    static void *readFile_Thread(void *, int, string);
    static void *RANSAC_Thread(void *__this, vector<ImgPair>, bool);
    static void *toYCbCr_Thread(void *, int);
    void readFile_Single(int, string);
    // 结果图像
    CImg<unsigned char> result;
    CImg<float> YCbCrResult;
    CImg<unsigned char> balanced;
    CImg<float> YCbCrBalanced;

    Homography forward_H;
    Homography backward_H;
};