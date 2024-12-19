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



// 预处理
void preprocess(Mat& img1, Mat& img2)
{
    // 彩色图->灰度图
    if (img1.channels() == 3)
    {
        cvtColor(img1, img1, COLOR_BGR2GRAY); // 通过OpenCV加载的图像通道顺序是BGR
    }
    if (img2.channels() == 3)
    {
        cvtColor(img2, img2, COLOR_BGR2GRAY);
    }
    // 对图像进行直方图均衡化，以增强图像的对比度。
    equalizeHist(img1, img1);
    equalizeHist(img2, img2);
}

// 使用相机的内参矩阵和畸变系数进行畸变校正。
Mat undistortion(Mat image, Mat camera_matrix, Mat dist_coeff)
{
    Mat undistortion_image;
    undistort(image, undistortion_image, camera_matrix, dist_coeff);

    return undistortion_image;
}

// 获取畸变校正和立体校正的映射变换矩阵、Q用于将视差图转换为深度图
void getRectifyTransform(int height, int width, StereoCameraParams::stereoconfig& config, Mat& map1x, Mat& map1y, Mat& map2x, Mat& map2y, Mat& Q)
{
    // 读取内参和外参
    Mat left_K = config.cam_matrix_left;
    Mat right_K = config.cam_matrix_right;
    Mat left_distortion = config.distortion_l;
    Mat right_distortion = config.distortion_r;
    Mat R = config.R;
    Mat T = config.T;

    // 计算极线校正
   
    Mat R1, R2, P1, P2;
    Rect roi1, roi2;

    stereoRectify(left_K, left_distortion, right_K, right_distortion, Size(width, height), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, Size(), &roi1, &roi2);
  //计算出用于图像映射的矩阵 map1 和 map2
    initUndistortRectifyMap(left_K, left_distortion, R1, P1, Size(width, height), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(right_K, right_distortion, R2, P2, Size(width, height), CV_32FC1, map2x, map2y);
}


void rectifyImages(Mat& img1, Mat& img2, Mat& map1x, Mat& map1y, Mat& map2x, Mat& map2y)
{
    // 对图像进行校正
    Mat rectifyImg1, rectifyImg2;
    remap(img1, rectifyImg1, map1x, map1y, INTER_LINEAR);
    remap(img2, rectifyImg2, map2x, map2y, INTER_LINEAR);

    // 更新图像
    img1 = rectifyImg1;
    img2 = rectifyImg2;
}

//在两幅图像之间绘制平行线，用于视觉效果
Mat drawLine(Mat image1, Mat image2)
{
    // 建立输出图像
    int height = max(image1.rows, image2.rows);
    int width = image1.cols + image2.cols;

    Mat output(height, width, CV_8UC3, Scalar(0, 0, 0));
    Mat roi1(output, Rect(0, 0, image1.cols, image1.rows));
    image1.copyTo(roi1);
    Mat roi2(output, Rect(image1.cols, 0, image2.cols, image2.rows));
    image2.copyTo(roi2);

    // 绘制等间距平行线
    int line_interval = 50;  // 直线间隔：50
    for (int k = 0; k < height / line_interval; k++)
    {
        line(output, Point(0, line_interval * (k + 1)), Point(2 * width, line_interval * (k + 1)), Scalar(0, 255, 0), 2, LINE_AA);
    }

    return output;
}

cv::Mat stereoMatchSGBM(cv::Mat left_image, cv::Mat right_image, bool down_scale = false)
{
    // SGBM匹配参数设置
    int img_channels = left_image.channels();
    int blockSize = 3;
    //参数 0, 128 分别设置最小视差和最大视差。
    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
        //最小视差和最大视差,惩罚参数的预处理项
        0, 128, blockSize, 8 * img_channels * blockSize * blockSize,
        32 * img_channels * blockSize * blockSize, 1, 63, 15, 100, 1, cv::StereoSGBM::MODE_SGBM_3WAY
    );
    cv::Ptr<cv::StereoSGBM> right_matcher = cv::StereoSGBM::create(
        0, 128, blockSize, 8 * img_channels * blockSize * blockSize,
        32 * img_channels * blockSize * blockSize, 1, 63, 15, 100, 1, cv::StereoSGBM::MODE_SGBM_3WAY
    );
    //设置右匹配器的最小视差，以确保左右匹配器的视差范围对称
    right_matcher->setMinDisparity(-left_matcher->getNumDisparities());

    // 计算视差图
    cv::Mat disparity;
    cv::Size size = left_image.size();
    //down_scale 为 false，则直接在原始图像上计算视差图
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

    // 真实视差（因为SGBM算法得到的视差是×16的）
    cv::Mat trueDisp_left, trueDisp_right;
    disparity.convertTo(trueDisp_left, CV_32F, 1.0 / 16.0);
    //disparity_right.convertTo(trueDisp_right, CV_32F, 1.0 / 16.0);

    return trueDisp_left;
}


// 从视差图生成深度图
Mat computeDepthMap(Mat& disparity, Mat& Q)
{
    Mat points_3d;
    
    //Q矩阵
    reprojectImageTo3D(disparity, points_3d, Q);
    Mat depthMap = Mat::zeros(disparity.rows, disparity.cols, CV_32FC1);
    for (int y = 0; y < disparity.rows; y++)
    {
        for (int x = 0; x < disparity.cols; x++)
        {
            Vec3f point = points_3d.at<Vec3f>(y, x);
            //Z 坐标
            float depth = point[2];
            //有效
            if (depth > 0 && depth < 65535) { 
                depthMap.at<float>(y, x) = depth;
            }
        }
    }

    return depthMap;
}

// 根据双目标定参数计算深度图
Mat computeDepthMapcofig(StereoCameraParams::stereoconfig& config, Mat& disparity)
{
    float fb = config.cam_matrix_left.at<double>(0, 0) * (-config.T.at<double>(0));
    float doffs = config.doffs;

    Mat depthMap = Mat::zeros(disparity.rows, disparity.cols, CV_32FC1);

    for (int y = 0; y < disparity.rows; y++)
    {
        for (int x = 0; x < disparity.cols; x++)
        {
            //fb是基础矩阵的乘积，与相机的焦距和基线（相机间的距离）有关
            float disparityValue = disparity.at<float>(y, x);
            float depth = fb / (disparityValue + doffs);
            if (depth > 0 && depth < 65535) { // 深度值有效
                depthMap.at<float>(y, x) = depth;
            }
        }
    }


    return depthMap;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr create_point_cloud(cv::Mat depth_map, cv::Mat left_image, cv::Mat Q, StereoCameraParams::stereoconfig& config)
{
    //创建了一个点云对象 cloud，并初始化了它的宽度、高度和点的数量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = depth_map.cols;
    cloud->height = depth_map.rows;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    //相机内参
    double fx = config.cam_matrix_left.at<double>(0, 0);
    double fy = config.cam_matrix_left.at<double>(1, 1);
    double cx = config.cam_matrix_left.at<double>(0, 2);
    double cy = config.cam_matrix_left.at<double>(1, 2);
    //遍历深度图并创建点云
    for (int y = 0; y < depth_map.rows; y++)
    {
        for (int x = 0; x < depth_map.cols; x++)
        {
            pcl::PointXYZRGB& point = cloud->points[y * depth_map.cols + x];
            float depth = depth_map.at<float>(y, x);
            if (depth > 0)
            {//// 假设深度图单位为毫米，转换为米
                //在三维空间中的位置
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

    // 使用 Q 矩阵对点云进行重映射到世界坐标系
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
    // 读取图像
 // Mat img1 = imread("D:/课/图像处理/数据集/im0.png");
   //Mat img2 = imread("D:/课/图像处理/数据集/im1.png");
   Mat img1 = imread("D:/课/图像处理/IPMV大作业1/camera_calibration_tool/camera_calibration_tool/left/left14.jpg");
   Mat img2 = imread("D:/课/图像处理/IPMV大作业1/camera_calibration_tool/camera_calibration_tool/right/right14.jpg");
   //Mat img1 =imread("D:/课/图像处理/IPMV大作业1/Project1/Project1/back1.png");
   //Mat img2 = imread("D:/课/图像处理/IPMV大作业1/Project1/Project1/back2.png");
    // 预处理
    preprocess(img1, img2);

    // 加载相机参数
    StereoCameraParams::stereoconfig config = StereoCameraParams::config;
    cout << config.cam_matrix_left << endl;
    // 消除畸变
    Mat undistortionImg1 = undistortion(img1, config.cam_matrix_left, config.distortion_l);
    Mat undistortionImg2 = undistortion(img2, config.cam_matrix_right, config.distortion_r);

    // 获取畸变校正和立体校正的映射变换矩阵、重投影矩阵
    Mat map1x, map1y, map2x, map2y, Q;
    getRectifyTransform(img1.rows, img1.cols, config, map1x, map1y, map2x, map2y, Q);
    cout <<"Q矩阵"<< Q << endl;

    // 对图像进行校正
    rectifyImages(undistortionImg1, undistortionImg2, map1x, map1y, map2x, map2y);
    Mat line;
    line = drawLine(undistortionImg1, undistortionImg2);
    //imshow("line", line);
    //waitKey(0);
    preprocess(undistortionImg1, undistortionImg2);

    const sint32 width = static_cast<uint32>(img1.cols);
    const sint32 height = static_cast<uint32>(img2.rows);

    // 左右影像的灰度数据
    auto bytes_left = new uint8[width * height];
    auto bytes_right = new uint8[width * height];
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            bytes_left[i * width + j] = img1.at<uint8>(i, j);
            bytes_right[i * width + j] = img2.at<uint8>(i, j);
        }
    }
    // SGM匹配参数设计
    SemiGlobalMatching::SGMOption sgm_option;
    // 聚合路径数
    sgm_option.num_paths = 4;
    // 候选视差范围
    sgm_option.min_disparity = 0;
    sgm_option.max_disparity = 128;
    // census窗口类型
    sgm_option.census_size = SemiGlobalMatching::Census5x5;
    // 一致性检查
    sgm_option.is_check_lr = true;
    sgm_option.lrcheck_thres = 1.0f;
    // 唯一性约束
    sgm_option.is_check_unique = true;
    sgm_option.uniqueness_ratio = 0.99;
    // 剔除小连通区
    sgm_option.is_remove_speckles = true;
    sgm_option.min_speckle_aera = 30;
    // 惩罚项P1、P2
    sgm_option.p1 = 72;
    sgm_option.p2_init = 288;
    // 视差图填充
    sgm_option.is_fill_holes = false;

    printf("w = %d, h = %d, d = [%d,%d]\n\n", width, height, sgm_option.min_disparity, sgm_option.max_disparity);

    // 定义SGM匹配类实例
    SemiGlobalMatching sgm;

    // 初始化
    auto start = std::chrono::steady_clock::now();
    if (!sgm.Initialize(width, height, sgm_option)) {
        cout << "SGM初始化失败！" << std::endl;
        return -2;
    }
    auto end = std::chrono::steady_clock::now();
    auto tt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printf("SGM Initializing Timing : %lf s\n\n", tt.count() / 1000.0);

    // 匹配
    printf("SGM Matching...\n");
    start = std::chrono::steady_clock::now();
    // disparity数组保存子像素的视差结果
    auto disparity = new float32[uint32(width * height)]();
    if (!sgm.Match(bytes_left, bytes_right, disparity)) {
        cout << "SGM匹配失败！" << std::endl;
        return -2;
    }
    end = std::chrono::steady_clock::now();
    tt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printf("\nSGM Matching Timing :   %lf s\n", tt.count() / 1000.0);

    // 显示视差图
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
    imwrite("视差图.jpg", disp_mat);
    imshow("视差图", disp_mat);
    waitKey(0);
    cv::Mat disp_color;
    applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
    cv::imshow("视差图彩", disp_color);
    imwrite("视差图彩.jpg", disp_color);
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
    //// 从视差图生成深度图
   Mat depthMap = computeDepthMap(disp_mat, Q);
   // Mat disp_mat2 = imread("D:/课/图像处理/IPMV大作业1/视差_ground_truth.png");
    //Mat depthMap=computeDepthMapcofig(config, disp_mat);
    float maxValue = *max_element(depthMap.begin<float>(), depthMap.end<float>());
    float minValue = *min_element(depthMap.begin<float>(), depthMap.end<float>());
    cout << maxValue << " " << minValue << endl;
    // 可视化深度图
    Mat depthMapVis = 2 * (depthMap - minValue) / (maxValue - minValue);
    double minValue1 = 10.0; // 最小值
    double maxValue1 = 1000.0; // 最大值
    double brightnessFactor = 1.02;
    Mat depthMapVis_brighten=2* (depthMap - minValue1) / (maxValue1 - minValue1);
    // 应用亮度提高系数，确保值不会超出 [0, 255] 的范围
    depthMapVis_brighten = (depthMapVis * brightnessFactor) * 255 / 2;

    // 将数据类型转换为CV_8UC1，以确保在显示和保存时正确处理
    depthMapVis_brighten.convertTo(depthMapVis, CV_8UC1);

    // 保存和显示图像
    imwrite("Depth Map Brighter.png", depthMapVis_brighten);
    imshow("Depth Map Brighter", depthMapVis_brighten);
    imwrite("深度图.png", depthMapVis);
    imshow("Depth Map", depthMapVis);
    waitKey(0);
   cv::Mat rgb;
  rgb = imread("D:/课/图像处理/IPMV大作业1/camera_calibration_tool/camera_calibration_tool/left/left13.jpg");
    //rgb = imread("D:/课/图像处理/数据集/im0.png");
    //rgb = imread("D:/课/图像处理/IPMV大作业1/Project1/Project1/back1.png");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = create_point_cloud(depthMapVis_brighten, rgb, Q, config);
    cout << "point cloud size = " << cloud->points.size() << endl;
    // Save point cloud to PCD file
    pcl::io::savePCDFile("output_cloud.pcd", *cloud);

    std::cout << "Point cloud saved" << std::endl;
    /**/
    return 0;

}
