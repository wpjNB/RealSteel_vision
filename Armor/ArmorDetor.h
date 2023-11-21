#ifndef ARMORDETOR_H
#define ARMORDETOR_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <vector>
#include <ArmorParam.h>
#include <ArmorDescriptor.h>
#include <LightDescriptor.h>
#include "ArmorNumClassifier.h"
using namespace std;
using namespace cv;

enum DetectorState
{
    LIGHTS_NOT_FOUND = 0,
    LIGHTS_FOUND = 1,
    ARMOR_NOT_FOUND = 2,
    ARMOR_FOUND = 3
};

#define debug
class ArmorDetector
{
public:
    // 初始化各个参数和我方颜色
    void init(int selfColor, int gray_mode)
    {
        if (selfColor == RED)
        {
            _enemy_color = BLUE;
            _self_color = RED;
        }
        this->_gray_mode = gray_mode;
    }

    void loadImg(Mat &img);
    // 预处理
    void preprocessing();
    // 识别装甲板的主程序，
    void detect();
    void solve_angle();
    // 分离色彩，提取我们需要（也就是敌人）的颜色，返回灰度图
    Mat separateColors();

    // 筛选符合条件的轮廓
    // 输入存储轮廓的矩阵，返回存储灯条信息的矩阵
    void filterContours(vector<vector<Point>> &lightContours, vector<LightDescriptor> &lightInfos);

    // 绘制旋转矩形

    void drawLightInfo(Mat &img, vector<LightDescriptor> &LD);
    void draw_armor(Mat &img, const vector<ArmorDescriptor> &armors);

    // 匹配灯条，筛选出装甲板
    vector<ArmorDescriptor> matchArmor(vector<LightDescriptor> &lightInfos);

    void adjustRec(cv::RotatedRect &rec);
    // 设置世界坐标
    void SetPoint3D(int armor_type, float weight, float height);
    // 设置相机参数
    int SetCamParam(const char *path);
    void SetCamParam(const Mat &camera_matrix, const cv::Mat &distortion_coeff);
    // 提供获取角度的接口
    void GetAngle(float &pitch, float &yaw, float &distance);

    // 是否有装甲板
    bool GetDetectState();

    // 设置目标装甲板
    void SetTargetArmor();
    bool armorCompare();

    void loadSVMModel(const char *model_path);
    // 展示图像
    void ShowImg(bool showSrcON, bool showDebugON, bool showBinaryON);
    // 展示结算信息
    void showDebuginfo();

private:
    int _enemy_color;
    int _self_color;
    int _gray_mode;

    cv::Rect _roi; // ROI区域

    cv::Mat _srcImg;    // 载入的图片保存于该成员变量中
    cv::Mat _roiImg;    // 从上一帧获得的ROI区域
    cv::Mat _grayImg;   // ROI区域的灰度图
    cv::Mat _debugImg;  // 标记测试图
    cv::Mat _max_color; // 二值化图

    vector<Point3f> BigArmorPoint3D;
    vector<Point3f> SmallArmorPoint3D;
    vector<Point2f> Point2D;

    Mat camera_matrix = Mat(3, 3, CV_32FC1);
    Mat distortion_coefficients;

    DetectorState detectstate;

    ArmorDescriptor TargetArmor;
    ArmorDescriptor LastArmor;
    vector<ArmorDescriptor> _armors; // 导入装甲版的类
    ArmorParam _param;               // 导入装甲板参数类
    ArmorNumClassifier NumClassifier;

    float pitch;
    float yaw;
    float dis;
};
#endif