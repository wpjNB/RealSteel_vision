#include "ArmorDetor.h"

ArmorType armortype = SMALL_ARMOR;

template <typename T>
float distance(const cv::Point_<T> &pt1, const cv::Point_<T> &pt2)
{
    return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2));
}

void ArmorDetector::SetPoint3D(int armor_type, float weight, float height)
{
    float x = weight / 2;
    float y = height / 2;
    switch (armortype)
    {
    case BIG_ARMOR:
        BigArmorPoint3D = vector<Point3f>{
            Point3f(-x / 2, y / 2, 0), // lu
            Point3f(x / 2, y / 2, 0),  // ru
            Point3f(x / 2, -y / 2, 0), // rd
            Point3f(-x / 2, -y / 2, 0) // ld
        };
        break;
    case SMALL_ARMOR:
        SmallArmorPoint3D = vector<Point3f>{
            Point3f(-x / 2, y / 2, 0), // lu
            Point3f(x / 2, y / 2, 0),  // ru
            Point3f(x / 2, -y / 2, 0), // rd
            Point3f(-x / 2, -y / 2, 0) // ld
        };
    default:
        break;
    }
}
void ArmorDetector::SetCamParam(const Mat &camera_matrix, const cv::Mat &distortion_coeff) // 使用传引用的方式，传递引用通常更高效，因为它避免了数据的复制
{
    this->camera_matrix = camera_matrix;
    this->distortion_coefficients = distortion_coeff;
}
int ArmorDetector::SetCamParam(const char *filepath) // 使用传引用的方式，传递引用通常更高效，因为它避免了数据的复制
{
    FileStorage fsread;
    fsread.open(filepath, FileStorage::READ);
    if (!fsread.isOpened())
    {
        cout << "Failed to open xml";
        return -1;
    }
    Mat camera_matrix0;
    Mat distortion_coeff0;

    fsread["camera_matrix"] >> camera_matrix0;
    fsread["distortion_coefficients"] >> distortion_coeff0;

#if 0
    cout<<camera_matrix<<endl;
    cout<<distortion_coefficients<<endl;
#endif

    SetCamParam(camera_matrix0, distortion_coeff0);

    fsread.release();
    return 0;
}
void ArmorDetector::loadImg(Mat &img)
{
    _srcImg = img;

    Rect imgBound = Rect(cv::Point(0, 0), Point(_srcImg.cols - 0, _srcImg.rows - 0));

    _roi = imgBound;
    _roiImg = _srcImg(_roi).clone(); // 注意一下，对_srcImg进行roi裁剪之后，原点坐标也会移动到裁剪后图片的左上角
}
void ArmorDetector::detect()
{
    // 预处理
    preprocessing();
    // 找轮廓
    vector<vector<Point>>
        lightContours;
    findContours(_max_color.clone(), lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // 筛选灯条
    _debugImg = _roiImg.clone();
    vector<LightDescriptor> lightInfos;
    filterContours(lightContours, lightInfos);
    // 没找到灯条就返回没找到
    if (detectstate == LIGHTS_FOUND)
    {
        drawLightInfo(_debugImg, lightInfos);
        // 匹配装甲板
        _armors = matchArmor(lightInfos);
        if (detectstate == ARMOR_FOUND)
        {
            draw_armor(_debugImg, _armors);

            solve_angle();
        }
    }
}
void ArmorDetector::solve_angle()
{

    Mat rVec = Mat::zeros(3, 1, CV_64FC1); // 旋转矩阵
    Mat tVec = Mat::zeros(3, 1, CV_64FC1); // 平移矩阵
    vector<Point2f> D2 = vector<Point2f>{
        _armors[0].vertex[0],
        _armors[0].vertex[1],
        _armors[0].vertex[2],
        _armors[0].vertex[3]};
    switch (armortype)
    {
    case BIG_ARMOR:
        solvePnP(BigArmorPoint3D, D2, this->camera_matrix, this->distortion_coefficients, rVec, tVec);
        break;
    case SMALL_ARMOR:
        solvePnP(SmallArmorPoint3D, D2, this->camera_matrix, this->distortion_coefficients, rVec, tVec);
        break;
    default:
        break;
    }

    double x_pos = tVec.at<double>(0, 0);
    double y_pos = tVec.at<double>(1, 0);
    double z_pos = tVec.at<double>(2, 0);
    double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
    double tan_yaw = x_pos / z_pos;
    pitch = -atan(tan_pitch) * 180 / CV_PI;
    yaw = atan(tan_yaw) * 180 / CV_PI;
    dis = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
}
// 从这个接口获取角度
void ArmorDetector::GetAngle(float &pitch, float &yaw, float &distance)
{
    pitch = this->pitch;
    yaw = this->yaw;
    distance = this->dis;
}
void ArmorDetector::adjustRec(cv::RotatedRect &rec)
{
    using std::swap;

    float &width = rec.size.width;
    float &height = rec.size.height;
    float &angle = rec.angle;

    while (angle >= 90.0)
        angle -= 180.0;
    while (angle < -90.0)
        angle += 180.0;

    if (angle >= 45.0)
    {
        swap(width, height);
        angle -= 90.0;
    }
    else if (angle < -45.0)
    {
        swap(width, height);
        angle += 90.0;
    }
}
void ArmorDetector::filterContours(vector<vector<Point>> &lightContours, vector<LightDescriptor> &lightInfos)
{
    RotatedRect lightRec;
    for (const auto &contour : lightContours)
    {
        // 得到面积
        float lightContourArea = contourArea(contour);
        // 面积太小的不要
        if (lightContourArea < _param.light_min_area)
            continue;
        // 椭圆拟合区域得到外接矩形

        lightRec = minAreaRect(contour);

        // 矫正灯条的角度，将其约束为-45~45
        adjustRec(lightRec);
        // 宽高比、凸度筛选灯条  注：凸度=轮廓面积/外接矩形面积
        if (lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
            lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity)
            continue;
        // 对灯条范围适当扩大
        lightRec.size.width *= _param.light_color_detect_extend_ratio;
        lightRec.size.height *= _param.light_color_detect_extend_ratio;

        // 因为颜色通道相减后己方灯条直接过滤，不需要判断颜色了,可以直接将灯条保存
        lightInfos.push_back(LightDescriptor(lightRec));
    }

    if (lightInfos.empty())
        detectstate = LIGHTS_NOT_FOUND;
    else
        detectstate = LIGHTS_FOUND;
}
void ArmorDetector::drawLightInfo(Mat &img, vector<LightDescriptor> &LD)
{

    vector<std::vector<cv::Point>> cons;

    int i = 0;
    for (auto &lightinfo : LD)
    {
        RotatedRect rotate = lightinfo.rec();
        auto vertices = new cv::Point2f[4];
        rotate.points(vertices);
        vector<Point> con;
        for (int i = 0; i < 4; i++)
        {
            con.push_back(vertices[i]);
        }
        cons.push_back(con);
        drawContours(img, cons, i, Scalar(0, 255, 255), 1, 8);
        i++;
    }
}
//
void ArmorDetector::draw_armor(Mat &img, const vector<ArmorDescriptor> &armors)
{

    // 绘制装甲板区域
    for (size_t i = 0; i < armors.size(); i++)
    {
        vector<Point2i> points;
        for (int j = 0; j < 4; j++)
        {
            points.push_back(Point(static_cast<int>(armors[i].vertex[j].x), static_cast<int>(armors[i].vertex[j].y)));
        }

        polylines(img, points, true, Scalar(0, 255, 0), 2, 8, 0); // 绘制两个不填充的多边形
    }
}
// 匹配灯条，筛选出装甲板
vector<ArmorDescriptor> ArmorDetector::matchArmor(vector<LightDescriptor> &lightInfos)
{
    vector<ArmorDescriptor> armors;
    if (lightInfos.size() < 2)
    {
        detectstate = ARMOR_NOT_FOUND;
        return armors;
    }
    // 按灯条中心x从小到大排序
    sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor &ld1, const LightDescriptor &ld2)
         {
        //Lambda函数,作为sort的cmp函数
        return ld1.center.x < ld2.center.x; });
    for (size_t i = 0; i < lightInfos.size(); i++)
    {
        // 遍历所有灯条进行匹配
        for (size_t j = i + 1; (j < lightInfos.size()); j++)
        {
            const LightDescriptor &leftLight = lightInfos[i];
            const LightDescriptor &rightLight = lightInfos[j];

            // 角差
            float angleDiff_ = abs(leftLight.angle - rightLight.angle);
            // 长度差比率
            float LenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
            // 筛选
            if (angleDiff_ > _param.light_max_angle_diff_ ||
                LenDiff_ratio > _param.light_max_height_diff_ratio_)
            {

                continue;
            }
            // 左右灯条相距距离
            float dis = distance(leftLight.center, rightLight.center);
            // 左右灯条长度的平均值
            float meanLen = (leftLight.length + rightLight.length) / 2;
            // 左右灯条中心点y的差值
            float yDiff = abs(leftLight.center.y - rightLight.center.y);
            // y差比率
            float yDiff_ratio = yDiff / meanLen;
            // 左右灯条中心点x的差值
            float xDiff = abs(leftLight.center.x - rightLight.center.x);
            // x差比率
            float xDiff_ratio = xDiff / meanLen;
            // 相距距离与灯条长度比值
            float ratio = dis / meanLen;
            // 筛选
            if (yDiff_ratio > _param.light_max_y_diff_ratio_ ||
                xDiff_ratio < _param.light_min_x_diff_ratio_ ||
                ratio > _param.armor_max_aspect_ratio_ ||
                ratio < _param.armor_min_aspect_ratio_)
            {
                continue;
            }

            // 按比值来确定大小装甲
            ArmorType armorType = ratio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;
            // 计算旋转得分
            float ratiOff = (armorType == BIG_ARMOR) ? max(_param.armor_big_armor_ratio - ratio, float(0)) : max(_param.armor_small_armor_ratio - ratio, float(0));
            float yOff = yDiff / meanLen;
            float rotationScore = -(ratiOff * ratiOff + yOff * yOff);
            // 得到匹配的装甲板
            ArmorDescriptor armor(leftLight, rightLight, armorType, _roiImg, rotationScore, _param);
            // imshow("ROI", armor.armorROI);

            NumClassifier.loadImg(_srcImg);
            NumClassifier.getArmorImg(armor);
            // imshow("armorImg", armor.armorImg);
            NumClassifier.setArmorNum(armor);

            armors.emplace_back(armor);
            break;
        }
    }
    if (armors.empty())
        detectstate = ARMOR_NOT_FOUND;
    else
        detectstate = ARMOR_FOUND;
    return armors;
}
// 预处理
void ArmorDetector::preprocessing()
{
    Mat element0 = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
    Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    Mat bin;
    // 阈值化
    cvtColor(_roiImg, _grayImg, CV_BGR2GRAY);
    threshold(_grayImg, bin, 120, 255, THRESH_BINARY);
    // 通道相间
    Mat color;
    vector<Mat> splited;
    split(_roiImg, splited);
    subtract(splited[0], splited[2], color);
    threshold(color, color, 46, 255, THRESH_BINARY); // red
    dilate(color, color, element0);
    // 进行与运算
    _max_color = bin & color; // _max_color获得了清晰的二值图
    dilate(_max_color, _max_color, element2);
}
Mat ArmorDetector::separateColors()
{
    Mat grayImg;
    if (_gray_mode == Channel_subtraction) // 通道相减法
    {
        vector<Mat> channels;
        // 把一个3通道图像转换成3个单通道图像
        split(_roiImg, channels); // 分离色彩通道

        // imshow("split_B", channels[0]);
        // imshow("split_G", channels[1]);
        // imshow("split_R", channels[2]);
        // 剔除我们不想要的颜色
        // 对于图像中红色的物体来说，其rgb分量中r的值最大，g和b在理想情况下应该是0，同理蓝色物体的b分量应该最大,将不想要的颜色减去，剩下的就是我们想要的颜色
        if (_enemy_color == RED)
        {
            grayImg = channels.at(2) - channels.at(0); // R-B
        }
        else
        {
            grayImg = channels.at(0) - channels.at(2); // B-R
        }
    }
    else if (_gray_mode == bgr2gray)
    {
        cvtColor(_roiImg, grayImg, COLOR_BGR2GRAY);
    }
    else if (_gray_mode == bgr2hsv)
    {
        Mat HsvImg;
        cvtColor(_roiImg, HsvImg, COLOR_BGR2HSV);
        inRange(HsvImg, Scalar(100, 43, 46), Scalar(124, 255, 255), grayImg);
    }
    return grayImg;
}
//
void ArmorDetector::SetTargetArmor()
{
    if (detectstate == ARMOR_NOT_FOUND)
        TargetArmor = ArmorDescriptor();
    else if (detectstate == ARMOR_FOUND)
    {
        ArmorDescriptor mva = _armors[0];
        for (int i = 1; i < _armors.size(); i++)
        {
            if (armorCompare())
                mva = _armors[i];
        }
        TargetArmor = mva;
    }
    LastArmor = TargetArmor; // 将上一帧的targetArmor设置为本帧的lastArmor
}
// 比较装甲板的得分
bool ArmorDetector::armorCompare()
{

    return 0;
}
void ArmorDetector::loadSVMModel(const char *model_path)
{
    NumClassifier.loadSvmModel(model_path);
}
bool ArmorDetector::GetDetectState()
{
    if (detectstate == ARMOR_FOUND)
        return 1;
    else
        return 0;
}
void ArmorDetector::ShowImg(bool showSrcON, bool showDebugON, bool showBinaryON)
{
    if (showSrcON)
        imshow("Src", _srcImg);
    if (showDebugON)
        imshow("Debug", _debugImg);
    if (showBinaryON)
        imshow("Binary", _max_color);
}
void ArmorDetector::showDebuginfo()
{
    Mat img = Mat::zeros(250, 400, CV_8UC3);
    // if (detectstate == ARMOR_FOUND)
    //     putText(img, "ARMOR_FOUND", Point(100, 35), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
    // else
    //     putText(img, "ARMOR_NOFOUND", Point(100, 35), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
    putText(img, format("Dis: %.1f", dis), Point(100, 140), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
    putText(img, format("pitch: %.1f", pitch), Point(100, 70), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
    putText(img, format("yaw: %.1f", yaw), Point(100, 105), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
    putText(img, format("TargetNum: %d", _armors[0].armorNum), Point(100, 175), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 2, 8, false);
    imshow("AngleSolver", img);
}
