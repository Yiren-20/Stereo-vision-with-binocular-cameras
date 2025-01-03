from glob import glob
import numpy as np
import os
import cv2

class Calib3D:

    def __init__(self, imgs_dir):
        assert os.path.exists(imgs_dir)
      
        self.imgs = [cv2.imread(i, cv2.IMREAD_GRAYSCALE) for i in glob(os.sep.join([imgs_dir, '*.jpg']))]
        
        assert 0 == len(self.imgs) % 2
        self.length = len(self.imgs) // 2

        self.img_size = self.imgs[0].shape[::-1] # (w, h)
        self.board_size = (14, 11)

        self.img_points = [self.detect_corners(img).reshape(-1, 2) for img in self.imgs]
    
    def detect_corners(self, img):
        ok, corners = cv2.findChessboardCorners(img, self.board_size)
       
        cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        return corners
    
    def obj_ponits(self):
        points = np.zeros((np.prod(self.board_size), 3), np.float32)
        points[:, :2] = np.indices(self.board_size).T.reshape(-1, 2)
        return self.length*[points]

    def camera_matrix(self):
        K0 = cv2.initCameraMatrix2D(self.obj_ponits(), self.img_points[:self.length], self.img_size, 0)
        K1 = cv2.initCameraMatrix2D(self.obj_ponits(), self.img_points[self.length:], self.img_size, 0)
        return K0, K1
    
    def stereo_calibrate(self):
        K0, K1 = self.camera_matrix()
        rms, K0, D0, K1, D1, R, T, E, F = cv2.stereoCalibrate(
            self.obj_ponits(), self.img_points[:self.length], self.img_points[self.length:],
            K0, None, K1, None, self.img_size,
            flags = cv2.CALIB_FIX_ASPECT_RATIO
                  | cv2.CALIB_ZERO_TANGENT_DIST
                  | cv2.CALIB_USE_INTRINSIC_GUESS
                  | cv2.CALIB_SAME_FOCAL_LENGTH
                  | cv2.CALIB_RATIONAL_MODEL
                  | cv2.CALIB_FIX_K3
                  | cv2.CALIB_FIX_K4
                  | cv2.CALIB_FIX_K5,
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-5)
        )
        return K0, D0, K1, D1, R, T

    def stereo_rectify(self):
        # R T 左到右的旋转/投影矩阵，源码注释是 first to second，这里第一个参数就是左相机
        K0, D0, K1, D1, R, T = self.stereo_calibrate()
        # R0 P0 左到右的旋转/投影矩阵，R1 P1 右到左的旋转/投影矩阵
        R0, R1, P0, P1, Q, roi0, roi1 = cv2.stereoRectify(K0, D0, K1, D1, self.img_size, R, T, flags = cv2.CALIB_ZERO_DISPARITY, alpha = 0)
        print("左相机")
        print(K0)
        print("右相机")
        print(K1)
        print("平移")
        print(T)
        print("旋转")
        print(R)
        # K0 D0 校正失真，R0 P0 校正垂直视差
        m0a, m0b = cv2.initUndistortRectifyMap(K0, D0, R0, P0, self.img_size, cv2.CV_16SC2)
        img0 = cv2.remap(self.imgs[0], m0a, m0b, cv2.INTER_LINEAR)
        img0c = cv2.cvtColor(img0, cv2.COLOR_GRAY2BGR)

        m1a, m1b = cv2.initUndistortRectifyMap(K1, D1, R1, P1, self.img_size, cv2.CV_16SC2)
        img1 = cv2.remap(self.imgs[self.length], m1a, m1b, cv2.INTER_LINEAR)
        img1c = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)

        rectify = np.concatenate((img0c, img1c), axis = 1)
        rectify[::40, :] = (0, 255, 0)
        cv2.imshow('rectify', rectify)

        return img0, img1, Q
    
   

    
    def Run(self):
        self.stereo_rectify()


        

Calib3D(os.sep.join([os.path.abspath(os.curdir), 'imgs\\'])).Run()