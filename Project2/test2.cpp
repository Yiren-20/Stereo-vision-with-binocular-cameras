/*
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    // 创建VideoCapture对象，参数为0表示打开默认摄像头
    VideoCapture cap(1);

    // 检查摄像头是否成功打开
    if (!cap.isOpened())
    {
        cout << "Failed to open camera!" << endl;
        return -1;
    }

    // 创建窗口
    namedWindow("Camera", WINDOW_NORMAL);

    // 循环读取摄像头捕捉到的帧并显示
    while (true)
    {
        // 读取一帧图像
        Mat frame;
        cap.read(frame);

        // 检查是否成功读取到一帧图像
        if (frame.empty())
        {
            cout << "Failed to read frame from camera!" << endl;
            break;
        }

        // 显示图像
        imshow("Camera", frame);

        // 按下q键退出循环
        if (waitKey(1) == 'q')
            break;
    }

    // 释放摄像头资源和所有窗口
    cap.release();
    destroyAllWindows();

    return 0;
}
*/
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // 创建VideoCapture对象，参数为1表示打开第二个摄像头
    // 增加cv::CAP_DSHOW作为后端
    VideoCapture cap(1 + cv::CAP_DSHOW);

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        cout << "Failed to open camera!" << endl;
        return -1;
    }

    // 设置摄像头分辨率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // 创建窗口
    namedWindow("All Camera", WINDOW_NORMAL);
    namedWindow("Left Camera", WINDOW_NORMAL);
    namedWindow("Right Camera", WINDOW_NORMAL);

    Mat frame, left_frame, right_frame;
    while (true) {
        // 读取一帧图像
        cap.read(frame);

        // 检查是否成功读取到一帧图像
        if (frame.empty()) {
            cout << "Failed to read frame from camera!" << endl;
            break;
        }

        // 裁剪左右画面
        // 定义左右画面的感兴趣区域
        Rect leftROI(0, 0, 1280, 720);
        Rect rightROI(1280, 0, 1280, 720);

        // 根据ROI裁剪图像
        left_frame = frame(leftROI);
        right_frame = frame(rightROI);

        // 显示图像
        imshow("All Camera", frame);
        imshow("Left Camera", left_frame);
        imshow("Right Camera", right_frame);

        // 按下'q'键退出循环
        char key = (char)waitKey(1);
        if (key == 'q') {
            break;
        }
    }

    // 释放摄像头资源和所有窗口
    cap.release();
    destroyAllWindows();

    return 0;
}