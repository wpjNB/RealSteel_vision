#include "ArmorDescriptor.h"

#define debug
ArmorDescriptor::ArmorDescriptor()
{
    rotationScore = 0;
    sizeScore = 0;
    vertex.resize(4);
    for (int i = 0; i < 4; i++)
    {
        vertex[i] = cv::Point2f(0, 0);
    }
    type = SMALL_ARMOR;
}

ArmorDescriptor::ArmorDescriptor(const LightDescriptor &lLight, const LightDescriptor &rLight, const ArmorType armorType, const cv::Mat &srcimg, float rotaScore, ArmorParam _param)
{
    // 获取两个灯条的旋转矩形
    lightPairs[0] = lLight.rec();
    lightPairs[1] = rLight.rec();

    cv::Size exLSize(int(lightPairs[0].size.width), int(lightPairs[0].size.height * 2));
    cv::Size exRSize(int(lightPairs[1].size.width), int(lightPairs[1].size.height * 2));
    cv::RotatedRect exLLight(lightPairs[0].center, exLSize, lightPairs[0].angle);
    cv::RotatedRect exRLight(lightPairs[1].center, exRSize, lightPairs[1].angle);

    cv::Point2f pts_l[4];
    exLLight.points(pts_l);
    cv::Point2f upper_l = pts_l[2];
    cv::Point2f lower_l = pts_l[3];

    cv::Point2f pts_r[4];
    exRLight.points(pts_r);
    cv::Point2f upper_r = pts_r[1];
    cv::Point2f lower_r = pts_r[0];

    vertex.resize(4);
    vertex[0] = upper_l;
    vertex[1] = upper_r;
    vertex[2] = lower_r;
    vertex[3] = lower_l;

    // Rect ROI = boundingRect(vertex);
    // armorROI = srcimg(ROI);

    // 设置装甲板类型
    type = armorType;
    // // 保存装甲板的图像

    // rotationScore = rotaScore;

    // // 计算尺寸分数
    // float normalized_area = contourArea(vertex) / _param.area_normalized_base;
    // sizeScore = exp(normalized_area);

    // 计算装甲板和roi图像中心区域的距离分数
    // Point2f srcImgCenter(grayImg.cols / 2, grayImg.rows / 2);
    // 灰度图中心点与装甲板中心点的距离
    // float sightOffset = cvex::distance(srcImgCenter, cvex::crossPointOf(array<Point2f, 2>{vertex[0],vertex[2]}, array<Point2f, 2>{vertex[1],vertex[3]}));
    // distScore = exp(-sightOffset / _param.sight_offset_normalized_base);
}
