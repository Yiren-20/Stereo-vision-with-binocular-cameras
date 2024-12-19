/*
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    // ����VideoCapture���󣬲���Ϊ0��ʾ��Ĭ������ͷ
    VideoCapture cap(1);

    // �������ͷ�Ƿ�ɹ���
    if (!cap.isOpened())
    {
        cout << "Failed to open camera!" << endl;
        return -1;
    }

    // ��������
    namedWindow("Camera", WINDOW_NORMAL);

    // ѭ����ȡ����ͷ��׽����֡����ʾ
    while (true)
    {
        // ��ȡһ֡ͼ��
        Mat frame;
        cap.read(frame);

        // ����Ƿ�ɹ���ȡ��һ֡ͼ��
        if (frame.empty())
        {
            cout << "Failed to read frame from camera!" << endl;
            break;
        }

        // ��ʾͼ��
        imshow("Camera", frame);

        // ����q���˳�ѭ��
        if (waitKey(1) == 'q')
            break;
    }

    // �ͷ�����ͷ��Դ�����д���
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
    // ����VideoCapture���󣬲���Ϊ1��ʾ�򿪵ڶ�������ͷ
    // ����cv::CAP_DSHOW��Ϊ���
    VideoCapture cap(1 + cv::CAP_DSHOW);

    // �������ͷ�Ƿ�ɹ���
    if (!cap.isOpened()) {
        cout << "Failed to open camera!" << endl;
        return -1;
    }

    // ��������ͷ�ֱ���
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // ��������
    namedWindow("All Camera", WINDOW_NORMAL);
    namedWindow("Left Camera", WINDOW_NORMAL);
    namedWindow("Right Camera", WINDOW_NORMAL);

    Mat frame, left_frame, right_frame;
    while (true) {
        // ��ȡһ֡ͼ��
        cap.read(frame);

        // ����Ƿ�ɹ���ȡ��һ֡ͼ��
        if (frame.empty()) {
            cout << "Failed to read frame from camera!" << endl;
            break;
        }

        // �ü����һ���
        // �������һ���ĸ���Ȥ����
        Rect leftROI(0, 0, 1280, 720);
        Rect rightROI(1280, 0, 1280, 720);

        // ����ROI�ü�ͼ��
        left_frame = frame(leftROI);
        right_frame = frame(rightROI);

        // ��ʾͼ��
        imshow("All Camera", frame);
        imshow("Left Camera", left_frame);
        imshow("Right Camera", right_frame);

        // ����'q'���˳�ѭ��
        char key = (char)waitKey(1);
        if (key == 'q') {
            break;
        }
    }

    // �ͷ�����ͷ��Դ�����д���
    cap.release();
    destroyAllWindows();

    return 0;
}