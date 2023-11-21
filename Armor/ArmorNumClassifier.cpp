#include "ArmorNumClassifier.h"

ArmorNumClassifier::ArmorNumClassifier()
{
    svm = ml::SVM::create();
    armorImgSize = Size(40, 40);
    p = Mat();

    warpPerspective_mat = Mat(3, 3, CV_32FC1);
    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(armorImgSize.width, 0);
    dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
    dstPoints[3] = Point2f(0, armorImgSize.height);
}

ArmorNumClassifier::~ArmorNumClassifier() {}

void ArmorNumClassifier::loadSvmModel(const char *model_path, Size armorsize)
{
    svm = StatModel::load<SVM>(model_path);
    if (svm.empty())
    {
        cout << "erro";
        exit(0);
    };
    this->armorImgSize = armorsize;
    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(armorsize.width, 0);
    dstPoints[2] = Point2f(armorsize.width, armorsize.height);
    dstPoints[3] = Point2f(0, armorsize.height);
}
void ArmorNumClassifier::loadImg(Mat &srcImg)
{

    // copy srcImg as warpPerspective_src
    (srcImg).copyTo(warpPerspective_src);

    // preprocess srcImg for the goal of acceleration
    cvtColor(warpPerspective_src, warpPerspective_src, 6); // CV_BGR2GRAY=6
    threshold(warpPerspective_src, warpPerspective_src, 10, 255, THRESH_BINARY);
}
void ArmorNumClassifier::getArmorImg(ArmorDescriptor &armor)
{
    // set the armor vertex as srcPoints
    for (int i = 0; i < 4; i++)
        srcPoints[i] = armor.vertex[i];

    // get the armor image using warpPerspective
    warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);                                                                     // get perspective transform matrix  透射变换矩阵
    warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, armorImgSize, INTER_NEAREST, BORDER_CONSTANT, Scalar(0)); // warpPerspective to get armorImage
    warpPerspective_dst.copyTo(armor.armorImg);                                                                                              // copyto armorImg
}
void ArmorNumClassifier::setArmorNum(ArmorDescriptor &armor)
{

    // adapt armorImg to the SVM model sample-size requirement
    p = armor.armorImg.reshape(1, 1);
    p.convertTo(p, CV_32FC1);

    // set armor number according to the result of SVM
    armor.ArmorNum = (int)svm->predict(p);
}