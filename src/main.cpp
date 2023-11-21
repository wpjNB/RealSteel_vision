#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <vector>
#include "ArmorDetor.h"
using namespace std;
using namespace cv;
int TargetNum = 0;
Color Enemy_color = BLUE;
int main(int argc, char *argv[])
{

    ArmorDetector detector;
    detector.loadSVMModel("/home/wpj/123svm.xml");
    detector.SetCamParam("/home/wpj/test_opencv (1)/src/out_camera_data.xml");
    detector.SetPoint3D(SMALL_ARMOR, 200, 150);
    VideoCapture cap("/home/wpj/1234567.mp4"); // 打开摄像头
    // cap.set(CAP_PROP_FRAME_WIDTH, 640);  // 宽度
    // cap.set(CAP_PROP_FRAME_HEIGHT, 480); // 高度
    // cap.set(CAP_PROP_EXPOSURE, 50);      // 曝光 50

    if (!cap.isOpened())
        return false;
    Mat frame;
    while (1)
    {
        cap >> frame;
        if (frame.empty())
            break;
        detector.init(Enemy_color, bgr2gray);
        double t1 = (double)getTickCount();
        detector.loadImg(frame);
        detector.detect();
        detector.ShowImg(1, 1, 1); // 原图，测试图，二值化
        double yaw = 0, pitch = 0, distance = 0;
        double t2 = (double)getTickCount();
        int fps = getTickFrequency() / (t2 - t1);
        if (detector.GetDetectState() == 1) // 显示pitch yaw
            detector.showDebuginfo();
        cout << "fps:" << fps << endl;
        waitKey(29);
        char key = waitKey(1);
        switch ((key))
        {
        case '1':
            TargetNum = 1;
            break;

        case '2':
            TargetNum = 2;
            break;
        case '3':
            TargetNum = 3;
            break;
        case '4':
            TargetNum = 4;
            break;
        case 'r':
        case 'R':
            Enemy_color = RED;
            break;
        case 'b':
        case 'B':
            Enemy_color = BLUE;
            break;
        default:
            break;
        }
    }

    cap.release();
    destroyAllWindows();
}
