// #include <mission_utils.h>
#include <iostream>
#include <fstream>

// #include <boost/thread/mutex.hpp>
// #include <boost/thread/shared_mutex.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>

// #include <prometheus.h>

using namespace std;
using namespace cv;

// prometheus_msgs::DetectionInfo pose_now;

struct DetectionInfo
{
    bool detected;
    int frame;
    Eigen::Vector3f position;
    Eigen::Vector3f sight_angle;
    float yaw_error;
} pose_now;

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs)
{

    cerr << filename << endl;
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }
    fs["cameraMatrix"] >> camMatrix;
    fs["distCoeffs"] >> distCoeffs;
    // cerr << camMatrix << endl;
    // cerr << distCoeffs << endl;
    return true;
}

// static bool readLandpadParameters(string filename, double &len_board){
//     cerr<<filename<<endl;
//     // 读取参数文档camera_param.yaml中的参数值；
//     YAML::Node camera_config = YAML::LoadFile(filename);
//     len_board=camera_config["landpad_det_len"].as<double>();
//     cerr<<len_board<<endl;
//     return true;
// }

int main(int argc, char **argv)
{

    Mat camera_matrix, distortion_coefficients;
    double landpad_det_len = 0.5;
    String camera_parameter = "/home/zq/zq/script/Landing/config/intrisic.xml";
    String landpad_parameter = "/home/zq/zq/script/Landing/config/landpad.yaml";
    bool readCam = readCameraParameters(camera_parameter, camera_matrix, distortion_coefficients);
    // bool readPad=readLandpadParameters(landpad_parameter,pad_length);

    if (!readCam)
    {
        cerr << "Invalid parameter files!" << endl;
    }
    cout << camera_matrix << endl;
    Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(10);
    vector<double> rv(3), tv(3);
    cv::Mat rvec(rv), tvec(tv);
    cv::VideoCapture capture(0);
    float last_x(0), last_y(0), last_z(0), last_yaw(0);
    // bool switch_state = is_suspanded;

    while (capture.grab())
    {
        Mat img;
        capture.retrieve(img);
        //------------------调用ArUco Marker库对图像进行识别--------------
        // markerids存储每个识别到二维码的编号  markerCorners每个二维码对应的四个角点的像素坐标
        std::vector<int> markerids;
        vector<vector<Point2f>> markerCorners, rejectedCandidate;
        Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(img, dictionary, markerCorners, markerids, parameters, rejectedCandidate);
        //-------------------多于一个目标被识别到，进入算法-----------------
        if (markerids.size() > 0)
        {
            // 未处理后的位置
            vector<cv::Point3f> vec_Position_OcInW;
            vector<double> vec_yaw;
            cv::Point3f A1_Sum_Position_OcInW(0, 0, 0);
            double A1_Sum_yaw = 0.0;
            double tx, ty, tz;
            int marker_count = 0;

            std::vector<int> markerids_sel;
            //first the center big then the center small , finally the four courners
            int t = -1;
            for (int tt = 0; tt < markerids.size(); tt++)
            {
                if (19 == markerids[tt])
                    t = tt;
            }
            if (-1 == t)
            {
                for (int tt = 0; tt < markerids.size(); tt++)
                {
                    if (43 == markerids[tt])
                        t = tt;
                }
            }
            if (-1 == t)
            {
                for (int tt = 0; tt < markerids.size(); tt++)
                {
                    if (1 == markerids[tt])
                        t = tt;
                }
            }
            if (-1 == t)
            {
                for (int tt = 0; tt < markerids.size(); tt++)
                {
                    if (2 == markerids[tt])
                        t = tt;
                }
            }
            if (-1 == t)
            {
                for (int tt = 0; tt < markerids.size(); tt++)
                {
                    if (3 == markerids[tt])
                        t = tt;
                }
            }
            if (-1 == t)
            {
                for (int tt = 0; tt < markerids.size(); tt++)
                {
                    if (4 == markerids[tt])
                        t = tt;
                }
            }

            if (-1 != t)
            {
                cv::Mat RoteM, TransM;
                // C2W代表 相机坐标系转换到世界坐标系  W2C代表 世界坐标系转换到相机坐标系 Theta为欧拉角
                cv::Point3f Theta_C2W;
                cv::Point3f Theta_W2C;
                cv::Point3f Position_OcInW;

                // 大二维码：19，小二维码：43
                //--------------对每一个Marker的相对位置进行解算----------------
                vector<vector<Point2f>> singMarkerCorner_19, singMarkerCorner_43;
                vector<vector<Point2f>> singMarkerCorner_1, singMarkerCorner_2, singMarkerCorner_3, singMarkerCorner_4;
                if (markerids[t] == 19)
                {
                    singMarkerCorner_19.push_back(markerCorners[t]);
                    cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_19, landpad_det_len * 0.666667, camera_matrix, distortion_coefficients, rvec, tvec);
                }
                else if (markerids[t] == 43)
                {
                    singMarkerCorner_43.push_back(markerCorners[t]);
                    cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_43, landpad_det_len * 0.066667, camera_matrix, distortion_coefficients, rvec, tvec);
                }
                else if (markerids[t] == 1)
                {
                    singMarkerCorner_1.push_back(markerCorners[t]);
                    cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_1, landpad_det_len * 0.133334, camera_matrix, distortion_coefficients, rvec, tvec);
                }
                else if (markerids[t] == 2)
                {
                    singMarkerCorner_2.push_back(markerCorners[t]);
                    cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_2, landpad_det_len * 0.133334, camera_matrix, distortion_coefficients, rvec, tvec);
                }
                else if (markerids[t] == 3)
                {
                    singMarkerCorner_3.push_back(markerCorners[t]);
                    cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_3, landpad_det_len * 0.133334, camera_matrix, distortion_coefficients, rvec, tvec);
                }
                else if (markerids[t] == 4)
                {
                    singMarkerCorner_4.push_back(markerCorners[t]);
                    cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_4, landpad_det_len * 0.133334, camera_matrix, distortion_coefficients, rvec, tvec);
                }
                else
                {
                    continue;
                }

                // 将解算的位置转化成旋转矩阵 并旋转计算无人机相对于目标的位置
                double rm[9];
                RoteM = cv::Mat(3, 3, CV_64FC1, rm);
                // 利用罗德里格斯公式将旋转向量转成旋转矩阵
                Rodrigues(rvec, RoteM);
                double r11 = RoteM.ptr<double>(0)[0];
                double r12 = RoteM.ptr<double>(0)[1];
                double r13 = RoteM.ptr<double>(0)[2];
                double r21 = RoteM.ptr<double>(1)[0];
                double r22 = RoteM.ptr<double>(1)[1];
                double r23 = RoteM.ptr<double>(1)[2];
                double r31 = RoteM.ptr<double>(2)[0];
                double r32 = RoteM.ptr<double>(2)[1];
                double r33 = RoteM.ptr<double>(2)[2];
                TransM = tvec;
                // 计算欧拉角
                double thetaz = atan2(r21, r11) / CV_PI * 180;
                double thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
                double thetax = atan2(r32, r33) / CV_PI * 180;

                Theta_C2W.z = thetaz;
                Theta_C2W.y = thetay;
                Theta_C2W.x = thetax;

                Theta_W2C.x = -1 * thetax;
                Theta_W2C.y = -1 * thetay;
                Theta_W2C.z = -1 * thetaz;
                // 偏移向量
                tx = tvec.ptr<double>(0)[0];
                ty = tvec.ptr<double>(0)[1];
                tz = tvec.ptr<double>(0)[2];

                Position_OcInW.x = tx;
                Position_OcInW.y = ty;
                Position_OcInW.z = tz;

                // 计算偏航角之差
                Eigen::Matrix3d rotateMatrix;
                rotateMatrix << r11, r12, r13, r21, r22, r23, r31, r32, r33;

                Eigen::Vector3d eulerVec;
                eulerVec(0) = (Theta_C2W.z) / 180 * CV_PI;
                vec_yaw.push_back(eulerVec(0));
                vec_Position_OcInW.push_back(Position_OcInW);

                A1_Sum_Position_OcInW += Position_OcInW;
                A1_Sum_yaw += eulerVec(0); // 待修改

                marker_count += 1;
            }
            if (-1 != t)
            {
                // 解算位置的平均值
                cout << marker_count << endl;
                cv::Point3f A1_Position_OcInW(0, 0, 0);
                double A1_yaw = 0.0;
                A1_Position_OcInW = A1_Sum_Position_OcInW / marker_count;
                A1_yaw = A1_Sum_yaw / marker_count;

                // 将解算后的位置发给控制端
                // pose_now.header.stamp = ros::Time::now();
                pose_now.detected = true;
                pose_now.frame = 0;
                pose_now.position[0] = tx;
                pose_now.position[1] = ty;
                pose_now.position[2] = tz;
                pose_now.sight_angle[0] = atan(tx / tz);
                pose_now.sight_angle[1] = atan(ty / tz);
                pose_now.yaw_error = A1_yaw;

                last_x = pose_now.position[0];
                last_y = pose_now.position[1];
                last_z = pose_now.position[2];
                last_yaw = pose_now.yaw_error;
            }
            else
            {
                // pose_now.header.stamp = ros::Time::now();
                pose_now.detected = false;
                pose_now.frame = 0;
                pose_now.position[0] = last_x;
                pose_now.position[1] = last_y;
                pose_now.position[2] = last_z;
                pose_now.sight_angle[0] = atan(last_x / last_z);
                pose_now.sight_angle[1] = atan(last_y / last_z);
                pose_now.yaw_error = last_yaw;
            }
        }
        else
        {
            // pose_now.header.stamp = ros::Time::now();
            pose_now.detected = false;
            pose_now.frame = 0;
            pose_now.position[0] = last_x;
            pose_now.position[1] = last_y;
            pose_now.position[2] = last_z;
            pose_now.sight_angle[0] = atan(last_x / last_z);
            pose_now.sight_angle[1] = atan(last_y / last_z);
            pose_now.yaw_error = last_yaw;
        }
        cout<<pose_now.position<<endl;
        cout<<pose_now.sight_angle<<endl;
        float axisLength = 0.5f *landpad_det_len;
        aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvec, tvec, axisLength);
        imshow("out",img);
        char key=(char)waitKey(100);
        if(key==27)break;


    }
}


// Detection_result landpad_det;

// void landpad_det_cb(){
//     landpad_det.object_name="landpad";
//     landpad_det.Detection_info=msg;

//     /****************************************************************************************
//      * the pose of marker in the coordinate of camera
//      * **********************************************************************************/
//     landpad_det.pos_body_frame[0] = - landpad_det.Detection_info.position[1] + DOWN_CAMERA_OFFSET_X;
//     landpad_det.pos_body_frame[1] = - landpad_det.Detection_info.position[0] + DOWN_CAMERA_OFFSET_Y;
//     landpad_det.pos_body_frame[2] = - landpad_det.Detection_info.position[2] + DOWN_CAMERA_OFFSET_Z;

//     landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;
//     //若已知降落板高度，则无需使用深度信息。
//     landpad_det.pos_body_enu_frame[2] = LANDPAD_HEIGHT - _DroneState.position[2];

//     landpad_det.pos_enu_frame[0] = _DroneState.position[0] + landpad_det.pos_body_enu_frame[0];
//     landpad_det.pos_enu_frame[1] = _DroneState.position[1] + landpad_det.pos_body_enu_frame[1];
//     landpad_det.pos_enu_frame[2] = _DroneState.position[2] + landpad_det.pos_body_enu_frame[2];

//     landpad_det.att_enu_frame[2] = 0.0;

//     if(landpad_det.Detection_info.detected)
//     {
//         landpad_det.num_regain++;
//         landpad_det.num_lost = 0;
//     }else
//     {
//         landpad_det.num_regain = 0;
//         landpad_det.num_lost++;
//     }

//     // 当连续一段时间无法检测到目标时，认定目标丢失
//     if(landpad_det.num_lost > VISION_THRES)
//     {
//         landpad_det.is_detected = false;
//     }

//     // 当连续一段时间检测到目标时，认定目标得到
//     if(landpad_det.num_regain > VISION_THRES)
//     {
//         landpad_det.is_detected = true;
//     }

// }
// }