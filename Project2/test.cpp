#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>
#include <cmath>
#include "stdafx.h"

#include "stereoCameraParams.h"
#include "SemiGlobalMatching.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;
using namespace cv;



// Ԥ����
void preprocess(Mat& img1, Mat& img2)
{
    // ��ɫͼ->�Ҷ�ͼ
    if (img1.channels() == 3)
    {
        cvtColor(img1, img1, COLOR_BGR2GRAY); // ͨ��OpenCV���ص�ͼ��ͨ��˳����BGR
    }
    if (img2.channels() == 3)
    {
        cvtColor(img2, img2, COLOR_BGR2GRAY);
    }
    // ��ͼ�����ֱ��ͼ���⻯������ǿͼ��ĶԱȶȡ�
    equalizeHist(img1, img1);
    equalizeHist(img2, img2);
}

// ʹ��������ڲξ���ͻ���ϵ�����л���У����
Mat undistortion(Mat image, Mat camera_matrix, Mat dist_coeff)
{
    Mat undistortion_image;
    undistort(image, undistortion_image, camera_matrix, dist_coeff);

    return undistortion_image;
}

// ��ȡ����У��������У����ӳ��任����Q���ڽ��Ӳ�ͼת��Ϊ���ͼ
void getRectifyTransform(int height, int width, StereoCameraParams::stereoconfig& config, Mat& map1x, Mat& map1y, Mat& map2x, Mat& map2y, Mat& Q)
{
    // ��ȡ�ڲκ����
    Mat left_K = config.cam_matrix_left;
    Mat right_K = config.cam_matrix_right;
    Mat left_distortion = config.distortion_l;
    Mat right_distortion = config.distortion_r;
    Mat R = config.R;
    Mat T = config.T;

    // ���㼫��У��
   
    Mat R1, R2, P1, P2;
    Rect roi1, roi2;

    stereoRectify(left_K, left_distortion, right_K, right_distortion, Size(width, height), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, Size(), &roi1, &roi2);
  //���������ͼ��ӳ��ľ��� map1 �� map2
    initUndistortRectifyMap(left_K, left_distortion, R1, P1, Size(width, height), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(right_K, right_distortion, R2, P2, Size(width, height), CV_32FC1, map2x, map2y);
}


void rectifyImages(Mat& img1, Mat& img2, Mat& map1x, Mat& map1y, Mat& map2x, Mat& map2y)
{
    // ��ͼ�����У��
    Mat rectifyImg1, rectifyImg2;
    remap(img1, rectifyImg1, map1x, map1y, INTER_LINEAR);
    remap(img2, rectifyImg2, map2x, map2y, INTER_LINEAR);

    // ����ͼ��
    img1 = rectifyImg1;
    img2 = rectifyImg2;
}

//������ͼ��֮�����ƽ���ߣ������Ӿ�Ч��
Mat drawLine(Mat image1, Mat image2)
{
    // �������ͼ��
    int height = max(image1.rows, image2.rows);
    int width = image1.cols + image2.cols;

    Mat output(height, width, CV_8UC3, Scalar(0, 0, 0));
    Mat roi1(output, Rect(0, 0, image1.cols, image1.rows));
    image1.copyTo(roi1);
    Mat roi2(output, Rect(image1.cols, 0, image2.cols, image2.rows));
    image2.copyTo(roi2);

    // ���Ƶȼ��ƽ����
    int line_interval = 50;  // ֱ�߼����50
    for (int k = 0; k < height / line_interval; k++)
    {
        line(output, Point(0, line_interval * (k + 1)), Point(2 * width, line_interval * (k + 1)), Scalar(0, 255, 0), 2, LINE_AA);
    }

    return output;
}

cv::Mat stereoMatchSGBM(cv::Mat left_image, cv::Mat right_image, bool down_scale = false)
{
    // SGBMƥ���������
    int img_channels = left_image.channels();
    int blockSize = 3;
    //���� 0, 128 �ֱ�������С�Ӳ������Ӳ
    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
        //��С�Ӳ������Ӳ�,�ͷ�������Ԥ������
        0, 128, blockSize, 8 * img_channels * blockSize * blockSize,
        32 * img_channels * blockSize * blockSize, 1, 63, 15, 100, 1, cv::StereoSGBM::MODE_SGBM_3WAY
    );
    cv::Ptr<cv::StereoSGBM> right_matcher = cv::StereoSGBM::create(
        0, 128, blockSize, 8 * img_channels * blockSize * blockSize,
        32 * img_channels * blockSize * blockSize, 1, 63, 15, 100, 1, cv::StereoSGBM::MODE_SGBM_3WAY
    );
    //������ƥ��������С�Ӳ��ȷ������ƥ�������ӲΧ�Գ�
    right_matcher->setMinDisparity(-left_matcher->getNumDisparities());

    // �����Ӳ�ͼ
    cv::Mat disparity;
    cv::Size size = left_image.size();
    //down_scale Ϊ false����ֱ����ԭʼͼ���ϼ����Ӳ�ͼ
    if (down_scale == false)
    {
        left_matcher->compute(left_image, right_image, disparity);
        cv::Mat disparity_right;
        right_matcher->compute(right_image, left_image, disparity_right);
    }
    else
    {
        cv::Mat left_image_down, right_image_down;
        cv::pyrDown(left_image, left_image_down);
        cv::pyrDown(right_image, right_image_down);
        float factor = static_cast<float>(left_image.cols) / left_image_down.cols;
        cv::Mat disparity_left_half, disparity_right_half;
        left_matcher->compute(left_image_down, right_image_down, disparity_left_half);
        right_matcher->compute(right_image_down, left_image_down, disparity_right_half);
        cv::resize(disparity_left_half, disparity, size, 0, 0, cv::INTER_AREA);
        cv::Mat disparity_right;
        cv::resize(disparity_right_half, disparity_right, size, 0, 0, cv::INTER_AREA);
        disparity = factor * disparity;
        disparity_right = factor * disparity_right;
    }

    // ��ʵ�Ӳ��ΪSGBM�㷨�õ����Ӳ��ǡ�16�ģ�
    cv::Mat trueDisp_left, trueDisp_right;
    disparity.convertTo(trueDisp_left, CV_32F, 1.0 / 16.0);
    //disparity_right.convertTo(trueDisp_right, CV_32F, 1.0 / 16.0);

    return trueDisp_left;
}


// ���Ӳ�ͼ�������ͼ
Mat computeDepthMap(Mat& disparity, Mat& Q)
{
    Mat points_3d;
    
    //Q����
    reprojectImageTo3D(disparity, points_3d, Q);
    Mat depthMap = Mat::zeros(disparity.rows, disparity.cols, CV_32FC1);
    for (int y = 0; y < disparity.rows; y++)
    {
        for (int x = 0; x < disparity.cols; x++)
        {
            Vec3f point = points_3d.at<Vec3f>(y, x);
            //Z ����
            float depth = point[2];
            //��Ч
            if (depth > 0 && depth < 65535) { 
                depthMap.at<float>(y, x) = depth;
            }
        }
    }

    return depthMap;
}

// ����˫Ŀ�궨�����������ͼ
Mat computeDepthMapcofig(StereoCameraParams::stereoconfig& config, Mat& disparity)
{
    float fb = config.cam_matrix_left.at<double>(0, 0) * (-config.T.at<double>(0));
    float doffs = config.doffs;

    Mat depthMap = Mat::zeros(disparity.rows, disparity.cols, CV_32FC1);

    for (int y = 0; y < disparity.rows; y++)
    {
        for (int x = 0; x < disparity.cols; x++)
        {
            //fb�ǻ�������ĳ˻���������Ľ���ͻ��ߣ������ľ��룩�й�
            float disparityValue = disparity.at<float>(y, x);
            float depth = fb / (disparityValue + doffs);
            if (depth > 0 && depth < 65535) { // ���ֵ��Ч
                depthMap.at<float>(y, x) = depth;
            }
        }
    }


    return depthMap;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr create_point_cloud(cv::Mat depth_map, cv::Mat left_image, cv::Mat Q, StereoCameraParams::stereoconfig& config)
{
    //������һ�����ƶ��� cloud������ʼ�������Ŀ�ȡ��߶Ⱥ͵������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = depth_map.cols;
    cloud->height = depth_map.rows;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    //����ڲ�
    double fx = config.cam_matrix_left.at<double>(0, 0);
    double fy = config.cam_matrix_left.at<double>(1, 1);
    double cx = config.cam_matrix_left.at<double>(0, 2);
    double cy = config.cam_matrix_left.at<double>(1, 2);
    //�������ͼ����������
    for (int y = 0; y < depth_map.rows; y++)
    {
        for (int x = 0; x < depth_map.cols; x++)
        {
            pcl::PointXYZRGB& point = cloud->points[y * depth_map.cols + x];
            float depth = depth_map.at<float>(y, x);
            if (depth > 0)
            {//// �������ͼ��λΪ���ף�ת��Ϊ��
                //����ά�ռ��е�λ��
                point.z = depth / 1000.0;
                point.x = (x - cx) * point.z / fx;
                point.y = (y - cy) * point.z / fy;
                point.r = left_image.at<cv::Vec3b>(y, x)[2];
                point.g = left_image.at<cv::Vec3b>(y, x)[1];
                point.b = left_image.at<cv::Vec3b>(y, x)[0];
            }
            else
            {
                point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    // ʹ�� Q ����Ե��ƽ�����ӳ�䵽��������ϵ
    Eigen::Matrix4f projection_matrix;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            projection_matrix(i, j) = Q.at<double>(i, j);
        }
    }
    projection_matrix(3, 0) = projection_matrix(3, 1) = projection_matrix(3, 3) = 0;
    projection_matrix(3, 2) = 1;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_remapped(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *cloud_remapped, projection_matrix);

    return cloud;
}


int main(int argc, char** argv)
{
    // ��ȡͼ��
 // Mat img1 = imread("D:/��/ͼ����/���ݼ�/im0.png");
   //Mat img2 = imread("D:/��/ͼ����/���ݼ�/im1.png");
   Mat img1 = imread("D:/��/ͼ����/IPMV����ҵ1/camera_calibration_tool/camera_calibration_tool/left/left14.jpg");
   Mat img2 = imread("D:/��/ͼ����/IPMV����ҵ1/camera_calibration_tool/camera_calibration_tool/right/right14.jpg");
   //Mat img1 =imread("D:/��/ͼ����/IPMV����ҵ1/Project1/Project1/back1.png");
   //Mat img2 = imread("D:/��/ͼ����/IPMV����ҵ1/Project1/Project1/back2.png");
    // Ԥ����
    preprocess(img1, img2);

    // �����������
    StereoCameraParams::stereoconfig config = StereoCameraParams::config;
    cout << config.cam_matrix_left << endl;
    // ��������
    Mat undistortionImg1 = undistortion(img1, config.cam_matrix_left, config.distortion_l);
    Mat undistortionImg2 = undistortion(img2, config.cam_matrix_right, config.distortion_r);

    // ��ȡ����У��������У����ӳ��任������ͶӰ����
    Mat map1x, map1y, map2x, map2y, Q;
    getRectifyTransform(img1.rows, img1.cols, config, map1x, map1y, map2x, map2y, Q);
    cout <<"Q����"<< Q << endl;

    // ��ͼ�����У��
    rectifyImages(undistortionImg1, undistortionImg2, map1x, map1y, map2x, map2y);
    Mat line;
    line = drawLine(undistortionImg1, undistortionImg2);
    //imshow("line", line);
    //waitKey(0);
    preprocess(undistortionImg1, undistortionImg2);

    const sint32 width = static_cast<uint32>(img1.cols);
    const sint32 height = static_cast<uint32>(img2.rows);

    // ����Ӱ��ĻҶ�����
    auto bytes_left = new uint8[width * height];
    auto bytes_right = new uint8[width * height];
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            bytes_left[i * width + j] = img1.at<uint8>(i, j);
            bytes_right[i * width + j] = img2.at<uint8>(i, j);
        }
    }
    // SGMƥ��������
    SemiGlobalMatching::SGMOption sgm_option;
    // �ۺ�·����
    sgm_option.num_paths = 4;
    // ��ѡ�ӲΧ
    sgm_option.min_disparity = 0;
    sgm_option.max_disparity = 128;
    // census��������
    sgm_option.census_size = SemiGlobalMatching::Census5x5;
    // һ���Լ��
    sgm_option.is_check_lr = true;
    sgm_option.lrcheck_thres = 1.0f;
    // Ψһ��Լ��
    sgm_option.is_check_unique = true;
    sgm_option.uniqueness_ratio = 0.99;
    // �޳�С��ͨ��
    sgm_option.is_remove_speckles = true;
    sgm_option.min_speckle_aera = 30;
    // �ͷ���P1��P2
    sgm_option.p1 = 72;
    sgm_option.p2_init = 288;
    // �Ӳ�ͼ���
    sgm_option.is_fill_holes = false;

    printf("w = %d, h = %d, d = [%d,%d]\n\n", width, height, sgm_option.min_disparity, sgm_option.max_disparity);

    // ����SGMƥ����ʵ��
    SemiGlobalMatching sgm;

    // ��ʼ��
    auto start = std::chrono::steady_clock::now();
    if (!sgm.Initialize(width, height, sgm_option)) {
        cout << "SGM��ʼ��ʧ�ܣ�" << std::endl;
        return -2;
    }
    auto end = std::chrono::steady_clock::now();
    auto tt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printf("SGM Initializing Timing : %lf s\n\n", tt.count() / 1000.0);

    // ƥ��
    printf("SGM Matching...\n");
    start = std::chrono::steady_clock::now();
    // disparity���鱣�������ص��Ӳ���
    auto disparity = new float32[uint32(width * height)]();
    if (!sgm.Match(bytes_left, bytes_right, disparity)) {
        cout << "SGMƥ��ʧ�ܣ�" << std::endl;
        return -2;
    }
    end = std::chrono::steady_clock::now();
    tt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printf("\nSGM Matching Timing :   %lf s\n", tt.count() / 1000.0);

    // ��ʾ�Ӳ�ͼ
    Mat disp_mat = Mat(height, width, CV_8UC1);
    float min_disp = width, max_disp = -width;
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = disparity[i * width + j];
            if (disp != Invalid_Float) {
                min_disp = std::min(min_disp, disp);
                max_disp = std::max(max_disp, disp);
            }
        }
    }
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = disparity[i * width + j];
            if (disp == Invalid_Float) {
                disp_mat.data[i * width + j] = 0;
            }
            else {
                disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
            }
        }
    }
    imwrite("�Ӳ�ͼ.jpg", disp_mat);
    imshow("�Ӳ�ͼ", disp_mat);
    waitKey(0);
    cv::Mat disp_color;
    applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
    cv::imshow("�Ӳ�ͼ��", disp_color);
    imwrite("�Ӳ�ͼ��.jpg", disp_color);
    waitKey(0);

    
    Mat disparity1 = stereoMatchSGBM(undistortionImg1, undistortionImg2, true);
    imshow("disparity", disparity1);
    imwrite("disparity.jpg", disparity1);
    waitKey(0);
    Mat disp8;
    disparity1.convertTo(disp8, CV_8U, 255 / (StereoSGBM::create()->getNumDisparities() * 16.));
    cv::Mat disparity1_color;
    applyColorMap(disp8, disparity1_color, cv::COLORMAP_JET);
    cv::imshow("disparitycolor", disparity1_color);
    imwrite("disparitycolor.jpg", disparity1_color);
    waitKey(0);
    /**/
    //// ���Ӳ�ͼ�������ͼ
   Mat depthMap = computeDepthMap(disp_mat, Q);
   // Mat disp_mat2 = imread("D:/��/ͼ����/IPMV����ҵ1/�Ӳ�_ground_truth.png");
    //Mat depthMap=computeDepthMapcofig(config, disp_mat);
    float maxValue = *max_element(depthMap.begin<float>(), depthMap.end<float>());
    float minValue = *min_element(depthMap.begin<float>(), depthMap.end<float>());
    cout << maxValue << " " << minValue << endl;
    // ���ӻ����ͼ
    Mat depthMapVis = 2 * (depthMap - minValue) / (maxValue - minValue);
    double minValue1 = 10.0; // ��Сֵ
    double maxValue1 = 1000.0; // ���ֵ
    double brightnessFactor = 1.02;
    Mat depthMapVis_brighten=2* (depthMap - minValue1) / (maxValue1 - minValue1);
    // Ӧ���������ϵ����ȷ��ֵ���ᳬ�� [0, 255] �ķ�Χ
    depthMapVis_brighten = (depthMapVis * brightnessFactor) * 255 / 2;

    // ����������ת��ΪCV_8UC1����ȷ������ʾ�ͱ���ʱ��ȷ����
    depthMapVis_brighten.convertTo(depthMapVis, CV_8UC1);

    // �������ʾͼ��
    imwrite("Depth Map Brighter.png", depthMapVis_brighten);
    imshow("Depth Map Brighter", depthMapVis_brighten);
    imwrite("���ͼ.png", depthMapVis);
    imshow("Depth Map", depthMapVis);
    waitKey(0);
   cv::Mat rgb;
  rgb = imread("D:/��/ͼ����/IPMV����ҵ1/camera_calibration_tool/camera_calibration_tool/left/left13.jpg");
    //rgb = imread("D:/��/ͼ����/���ݼ�/im0.png");
    //rgb = imread("D:/��/ͼ����/IPMV����ҵ1/Project1/Project1/back1.png");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = create_point_cloud(depthMapVis_brighten, rgb, Q, config);
    cout << "point cloud size = " << cloud->points.size() << endl;
    // Save point cloud to PCD file
    pcl::io::savePCDFile("output_cloud.pcd", *cloud);

    std::cout << "Point cloud saved" << std::endl;
    /**/
    return 0;

}
