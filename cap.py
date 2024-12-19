#!/usr/bin/env python
# coding=utf-8
import cv2
from PyCameraList.camera_device import test_list_cameras, list_video_devices, list_audio_devices
import time
cameras = list_video_devices()
print(dict(cameras))
#return: {0: 'Intel(R) RealSense(TM) 3D Camera (Front F200) RGB', 1: 'NewTek NDI Video', 2: 'Intel(R) RealSense(TM) 3D Camera Virtual Driver', 3: 'Intel(R) RealSense(TM) 3D Camera (Front F200) Depth', 4: 'OBS-Camera', 5: 'OBS-Camera2', 6: 'OBS-Camera3', 7: 'OBS-Camera4', 8: 'OBS Virtual Camera'}
 
audios = list_audio_devices()
print(dict(audios))
	

cap = cv2.VideoCapture(1+cv2.CAP_DSHOW) #这里增加一个这，具体不知道是啥
# 设置摄像头分辨率
cap.set(6,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

data_path="./"
with open(data_path+"left/num.txt","r") as f:
    data = f.readline()
    left_count=int(data)
with open(data_path+"right/num.txt","r") as f:
    right_count=int(data)

    
while True:
        # 读取摄像头数据
    ret, frame = cap.read()
    #裁剪坐标为[y0:y1, x0:x1]  HEIGHT * WIDTH
    left_frame = frame[0:720, 0:1280]
    right_frame = frame[0:720, 1280:2560]
    cv2.imshow("all", frame)
    cv2.imshow("left", left_frame)
    cv2.imshow("right", right_frame)
    key=cv2.waitKey(1)
    if key==27:
        with open(data_path+"left/num.txt","w") as f:
            f.write(str(left_count))  # 自带文件关闭功能，不需要再写f.close()
        with open(data_path+"right/num.txt","w") as f:
            f.write(str(right_count))  # 自带文件关闭功能，不需要再写f.close()
        break
    if key==108: #left
        ret=cv2.imwrite(data_path+"left/left"+str(left_count)+".jpg",left_frame)
        print("已存储左相机第"+str(left_count)+"张")
        left_count=left_count+1
    if key==114: #left
        ret=cv2.imwrite(data_path+"right/right"+str(right_count)+".jpg",right_frame)
        print("已存储右相机第"+str(right_count)+"张")
        right_count=right_count+1
    if key==32: #left
        ret=cv2.imwrite(data_path+"left/left"+str(left_count)+".jpg",left_frame)
        print("已存储左相机第"+str(left_count)+"张")
        left_count=left_count+1
    
        ret=cv2.imwrite(data_path+"right/right"+str(right_count)+".jpg",right_frame)
        print("已存储右相机第"+str(right_count)+"张")
        right_count=right_count+1
        
        
   
    
    # cv2.imshow("right", right_frame)
    