#include "Projection.h"

unsigned char Projection::bilinearInterpolation(const CImg<unsigned char> &src, float x, float y, int channel)
{
    int x_floor = floor(x), y_floor = floor(y);
    int x_ceil = ceil(x) >= (src.width() - 1) ? (src.width() - 1) : ceil(x);
    int y_ceil = ceil(y) >= (src.height() - 1) ? (src.height() - 1) : ceil(y);

    float a = x - x_floor, b = y - y_floor;

    //choice为false，左边图像作为来源，否则右边图像作为来源
    Pixel leftdown = Pixel(src(x_floor, y_floor, 0), src(x_floor, y_floor, 1), src(x_floor, y_floor, 2));
    Pixel lefttop = Pixel(src(x_floor, y_ceil, 0), src(x_floor, y_ceil, 1), src(x_floor, y_ceil, 2));
    Pixel rightdown = Pixel(src(x_ceil, y_floor, 0), src(x_ceil, y_floor, 1), src(x_ceil, y_floor, 2));
    Pixel righttop = Pixel(src(x_ceil, y_ceil, 0), src(x_ceil, y_ceil, 1), src(x_ceil, y_ceil, 2));
    return ((1 - a) * (1 - b) * leftdown.val[channel] + a * (1 - b) * rightdown.val[channel] +
            a * b * righttop.val[channel] + (1 - a) * b * lefttop.val[channel]);
}

CImg<unsigned char> Projection::imageProjection(const CImg<unsigned char> &src)
{
    
    CImg<unsigned char> res(src.width(), src.height(), 1, src.spectrum(), 0);
    bool flag = (src.width() > src.height());
    int width = flag? src.height() : src.width();
    int height = flag? src.width() : src.height();
    const float tanVal = tan(ANGLE * cimg::PI / 180.0);
    float r = (width / 2.0) / tanVal;

    if(flag)
    {
        cimg_forXY(src, x, y)
        {
            float dst_x = y - width / 2;
            float dst_y = x - height / 2;

            float k = r / sqrt(pow(r, 2) + pow(dst_x, 2));
            float src_x = dst_x / k;
            float src_y = dst_y / k;

            if (src_x + width / 2 >= 0 && src_x + width / 2 < src.height() && src_y + height / 2 >= 0 && src_y + height / 2 < src.width())
            {
                for (int k = 0; k < res.spectrum(); k++)
                {
                    res(x, y, k) = Projection::bilinearInterpolation(src, src_y + height / 2, src_x + width / 2, k);
                }
            }
        }
    }
    else
    {
        cimg_forXY(src, x, y)
        {
            float dst_x = x - width / 2;
            float dst_y = y - height / 2;

            float k = r / sqrt(pow(r, 2) + pow(dst_x, 2));
            float src_x = dst_x / k;
            float src_y = dst_y / k;

            if (src_x + width / 2 >= 0 && src_x + width / 2 < src.width() && src_y + height / 2 >= 0 && src_y + height / 2 < src.height())
            {
                for (int k = 0; k < res.spectrum(); k++)
                {
                    res(x, y, k) = Projection::bilinearInterpolation(src, src_x + width / 2, src_y + height / 2, k);
                }
            }
        }
    }

    return res;
    // 数学公式：https://blog.csdn.net/weixinhum/article/details/50611750
}