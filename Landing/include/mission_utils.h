#ifndef MISSION_UTILS_H
#define MISSION_UTILS_H

#include <Eigen/Eigen>


using namespace std;




struct Detection_result
{
    string object_name;
    // 视觉检测原始信息，返回的结果为相机坐标系
    // 方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    prometheus_msgs::DetectionInfo Detection_info;      
    // 目标在机体系位置
    Eigen::Vector3f pos_body_frame;   
    // 目标在机体-惯性系位置 (原点位于质心，x轴指向前方，y轴指向左，z轴指向上的坐标系)
    Eigen::Vector3f pos_body_enu_frame;  
    // 目标在惯性系位置 (原点位于起飞点，x轴指向前方，y轴指向左，z轴指向上的坐标系)
    Eigen::Vector3f pos_enu_frame; 
    // 目标在机体系姿态
    Eigen::Vector3f att_body_frame;
    // 目标在惯性系姿态
    Eigen::Vector3f att_enu_frame;
    // 目标识别标志位,阈值:VISION_THRES
    bool is_detected = false; 
    int num_lost = 0;          //视觉丢失计数器
    int num_regain = 0;     
};





































#endif