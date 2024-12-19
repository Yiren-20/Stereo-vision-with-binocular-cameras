#pragma once
#ifndef STEREO_CAMERA_PARAMS_H
#define STEREO_CAMERA_PARAMS_H

#include <opencv2/core/core.hpp>

namespace StereoCameraParams 
{
    
    // 左相机内参
    /* 
    const cv::Mat cam_matrix_left = (cv::Mat_<double>(3, 3) << 3997.684, 0, 225.0,
        0., 3997.684, 187.5,
        0, 0, 1);*/
    const cv::Mat cam_matrix_left = (cv::Mat_<double>(3, 3) << 5.05816940e+03, 0.00000000e+00, 6.65178485e+02,
        0.00000000e+00, 5.06976631e+03, 3.52056887e+02,
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
    // 右相机内参
   /* const cv::Mat cam_matrix_right = (cv::Mat_<double>(3, 3) << 3997.684, 0, 225.0,
        0., 3997.684, 187.5,
        0, 0, 1);*/
    const cv::Mat cam_matrix_right = (cv::Mat_<double>(3, 3) << 5.05816940e+03, 0.00000000e+00, 6.52920958e+02,
        0.00000000e+00, 5.06976631e+03, 3.31894461e+02,
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00);

    // 左右相机畸变系数:[k1, k2, p1, p2, k3]
    const cv::Mat distortion_l = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
    const cv::Mat distortion_r = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
    // 旋转矩阵
    const cv::Mat R = (cv::Mat_<double>(3, 3) << 9.98295296e-01,  5.08756709e-04,  5.83630254e-02,
        -1.64558015e-03,  9.99809825e-01,  1.94320798e-02,
        -5.83420400e-02, - 1.94949949e-02,  9.98106283e-01);
    // 平移矩阵
    const cv::Mat T = (cv::Mat_<double>(3, 1) << -10.95479209,-2.54286062,-0.74905974);
    // 主点列坐标的差
    const double doffs = 131.111;
    // 指示上述内外参是否为经过立体校正后的结果
    const bool isRectified = true;
    
    struct stereoconfig
    {
        cv::Mat cam_matrix_left;
        cv::Mat cam_matrix_right;
        cv::Mat distortion_l;
        cv::Mat distortion_r;
        cv::Mat R;
        cv::Mat T;
        double doffs;
        bool isRectified;
    };
    const stereoconfig config = {
        cam_matrix_left,
        cam_matrix_right,
        distortion_l,
        distortion_r,
        R,
        T,
        131.111,//10.1492,
        true
    };
    
}

#endif // STEREO_CAMERA_PARAMS_H