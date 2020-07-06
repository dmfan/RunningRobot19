#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" camera 模块 fisheye 鱼眼相机校正"""
import numpy as np
import cv2



def undistorted(image_pic):
    img = image_pic.copy()
    _img_shape = img.shape[:2]
    DIM = _img_shape[::-1]
    # print("DIM:{0}".format(DIM))
    K =np.array([[408.667787832136,0,301.034900498098],[0,408.714628092275,239.871569259691],[0,0,1]])
    D =np.array([-0.344907133322046,0.0980694024203472,-0.000517980545844449,-0.000773632039523929])
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img




fx = 209.943915050074
cx = 321.082510891168
fy = 211.595689060520
cy = 253.576489719712
k1, k2, k3, p1, p2 = 0.275359728841924, -0.242725469668622, 0.0589753730315234, 0.00644875280063650, -0.00228158263778931

#相机坐标系到像素坐标系的转换矩阵
k = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
])
#畸变系数
d = np.array([
    k1, k2, p1, p2, k3
])

#璜老板的三行去畸变
def undistort(img):
    global k,d
    h, w = img.shape[:2]
    mapx, mapy = cv2.initUndistortRectifyMap(k, d, None, k, (w, h), 5)
    return cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)



if __name__ == '__main__':
    cap = cv2.VideoCapture(0) #创建内置摄像头变量
    img_path = "00headphoto_save23.jpg"

    while(1):
        ret,img =cap.read() #把摄像头获取的图像信息保存之img变量

        ret = True
        img = cv2.imread(img_path)

        if ret == True:
            cv2.imshow("Image",img)
            imgdis = undistort(img)
            cv2.imshow('undis',imgdis)
            key = cv2.waitKey(0)
            if key == 27:
                cv2.destroyAllWindows()
                break
            elif key == ord('s'):
                print("save trtr123")
                cv2.imwrite("picTR123.jpg",Horg_img) #保存图片
        
