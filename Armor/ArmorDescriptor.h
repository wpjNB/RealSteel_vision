#ifndef ARMORDESCRIPTOR_H
#define ARMORDESCRIPTOR_H

#include <LightDescriptor.h>
#include <ArmorParam.h>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
enum ArmorType
{

    SMALL_ARMOR = 0,
    BIG_ARMOR = 1

};

class ArmorDescriptor
{
public:
    // 全部初始化为0
    ArmorDescriptor();

    // 计算装甲板的各个数据，包括四个定点的坐标、得分和保存装甲板的图像，写入到这个对象中
    ArmorDescriptor(const LightDescriptor &lLight, const LightDescriptor &rLight, const ArmorType armorType, const cv::Mat &srcImg, const float rotationScore, ArmorParam param);

    // 清空装甲板信息
    void clear()
    {
        rotationScore = 0;
        sizeScore = 0;
        distScore = 0;
        finalScore = 0;
        for (int i = 0; i < 4; i++)
        {
            vertex[i] = cv::Point2f(0, 0);
        }
        type = SMALL_ARMOR;
    }

public:
    std::array<cv::RotatedRect, 2> lightPairs; // 0 left, 1 right
    float sizeScore;                           // S1 = e^(size)
    float distScore;                           // S2 = e^(-offset)
    float rotationScore;                       // S3 = -(ratio^2 + yDiff^2)
    float finalScore;

    std::vector<cv::Point2f> vertex; // 装甲板的四个顶点，不包括光条区域
    cv::Mat frontImg;                // 单通道的装甲板图案

    int ArmorNum; // number on armor(recognized by SVM) 装甲板上的数字（用SVM识别得到）
    Mat armorROI; // 数字所在的ROI图像
    Mat armorImg; // image of armor set by getArmorImg() from ArmorNumClassifier() 装甲板的图片（透射变换获得）
    ArmorType type;
    Point2f center;
};

#endif // ARMORDESCRIPTOR_H
