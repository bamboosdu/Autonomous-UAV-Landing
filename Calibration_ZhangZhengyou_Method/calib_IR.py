# -*- coding: utf-8 -*-
"""
Homework: Calibrate the Camera with ZhangZhengyou Method.
Picture File Folder: ".\pic\IR_camera_calib_img", With Distort. 

By YouZhiyuan 2019.11.18
"""

import os
import numpy as np
import cv2
import glob


def calib(inter_corner_shape, size_per_grid, img_dir,img_type):
    # criteria: only for subpix calibration, which is not used here.
    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    w,h = inter_corner_shape
    # cp_int: corner point in int form, save the coordinate of corner points in world sapce in 'int' form
    # like (0,0,0), (1,0,0), (2,0,0) ....,(10,7,0).
    cp_int = np.zeros((w*h,3), np.float32)
    cp_int[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
    # cp_world: corner point in world space, save the coordinate of corner points in world space.
    cp_world = cp_int*size_per_grid
    
    obj_points = [] # the points in world space
    img_points = [] # the points in image space (relevant to obj_points)
    images = glob.glob(img_dir + os.sep + '**.' + img_type)
    for fname in images:
        img = cv2.imread(fname)
        gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # find the corners, cp_img: corner points in pixel space.
        ret, cp_img = cv2.findChessboardCorners(gray_img, (w,h), None)
        # if ret is True, save.
        if ret == True:
            # cv2.cornerSubPix(gray_img,cp_img,(11,11),(-1,-1),criteria)
            obj_points.append(cp_world)
            img_points.append(cp_img)
            # view the corners
            cv2.drawChessboardCorners(img, (w,h), cp_img, ret)
            cv2.imshow('FoundCorners',img)
            cv2.waitKey()
    cv2.destroyAllWindows()
    # calibrate the camera
    ret, mat_inter, coff_dis, v_rot, v_trans = cv2.calibrateCamera(obj_points, img_points, gray_img.shape[::-1], None, None)
    print (("ret:"),ret)
    print (("internal matrix:\n"),mat_inter)
    # in the form of (k_1,k_2,p_1,p_2,k_3)
    print (("distortion cofficients:\n"),coff_dis)  
    print (("rotation vectors:\n"),v_rot)
    print (("translation vectors:\n"),v_trans)
    # calculate the error of reproject
    total_error = 0
    for i in range(len(obj_points)):
        img_points_repro, _ = cv2.projectPoints(obj_points[i], v_rot[i], v_trans[i], mat_inter, coff_dis)
        error = cv2.norm(img_points[i], img_points_repro, cv2.NORM_L2)/len(img_points_repro)
        total_error += error
    print(("Average Error of Reproject: "), total_error/len(obj_points))
    
    return mat_inter, coff_dis
    
def dedistortion(inter_corner_shape, img_dir,img_type, save_dir, mat_inter, coff_dis):
    w,h = inter_corner_shape
    images = glob.glob(img_dir + os.sep + '**.' + img_type)
    for fname in images:
        img_name = fname.split(os.sep)[-1]
        img = cv2.imread(fname)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mat_inter,coff_dis,(w,h),0,(w,h)) # 自由比例参数
        dst = cv2.undistort(img, mat_inter, coff_dis, None, newcameramtx)
        # clip the image
        # x,y,w,h = roi
        # dst = dst[y:y+h, x:x+w]
        cv2.imwrite(save_dir + os.sep + img_name, dst)
    print('Dedistorted images have been saved to: %s successfully.' %save_dir)
    
if __name__ == '__main__':
    inter_corner_shape = (8,6)
    size_per_grid = 0.0245
    img_dir = "/home/zq/zq/git-space/drone_about/Calibration_ZhangZhengyou_Method/pic/IR"
    img_type = "png"
    # calibrate the camera
    mat_inter, coff_dis = calib(inter_corner_shape, size_per_grid, img_dir,img_type)
    # dedistort and save the dedistortion result. 
    save_dir = "/home/zq/zq/git-space/drone_about/Calibration_ZhangZhengyou_Method/pic/save_dedistortion"
    if(not os.path.exists(save_dir)):
        os.makedirs(save_dir)
    dedistortion(inter_corner_shape, img_dir, img_type, save_dir, mat_inter, coff_dis)
    
    """
    internal matrix:
 [[810.1423713    0.         316.45402718]
 [  0.         808.90153933 257.76302998]
 [  0.           0.           1.        ]]
distortion cofficients:
 [[ 5.45718267e-01 -1.43909388e+01  9.00797451e-03 -1.04702888e-02
   9.30467224e+01]]
   Average Error of Reproject:  0.15354787243113835
    """
    """
    internal matrix:
 [[810.1423713    0.         316.45402718]
 [  0.         808.90153933 257.76302998]
 [  0.           0.           1.        ]]
distortion cofficients:
 [[ 5.45718267e-01 -1.43909388e+01  9.00797451e-03 -1.04702888e-02
   9.30467224e+01]]

    """
"""
    internal matrix:
 [[829.21985322   0.         312.82667467]
 [  0.         829.57050777 258.88987187]
 [  0.           0.           1.        ]]
distortion cofficients:
 [[ 3.76355118e-01 -1.02710294e+01  4.10732374e-03 -9.58660867e-03
   7.23180347e+01]]
Average Error of Reproject:  0.13831088318368237
"""
"""
internal matrix:
 [[819.54289171   0.         314.53157417]
 [  0.         819.72768105 259.70322391]
 [  0.           0.           1.        ]]
distortion cofficients:
 [[ 2.28920007e-01  -5.11945261e+00 3.82815939e-03 -8.84696303e-03
   3.48622073e+01]]
Average Error of Reproject:  0.08573521656088101
"""

"""
internal matrix:
 [[732.77028682   0.         614.87813111]
 [  0.         733.39110606 349.18611676]
 [  0.           0.           1.        ]]
distortion cofficients:
 [[ 0.1119643  -0.22287516 -0.00228121 -0.00046944  0.14355533]]
0.058914151282405486
"""