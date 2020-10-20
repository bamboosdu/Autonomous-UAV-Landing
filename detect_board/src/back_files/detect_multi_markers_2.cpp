/*
 * @Author: Qiang Zhou 
 * @Date: 2020-10-14 18:46:28 
 * @Last Modified by: Qiang Zhou
 * @Last Modified time: 2020-10-15 19:16:09
 * This code is used for 2 marker detection(small center and big one)
 */
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <sstream>
#include <Utils.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace Eigen;

#define VISION_THRES 20

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);

//Kalman filter funtions headers
void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
void predictKalmanFilter(KalmanFilter &KF, Mat &translation_predicted, Mat &rotation_predicted);
void updateKalmanFilter(KalmanFilter &KF, Mat &measurements,
                        Mat &translation_estimated, Mat &rotation_estimated);
void fillMeasurements(Mat &measurements,
                      const Mat &translation_measured, const Mat &rotation_measured);
void rot_euler(const Mat &rotation_matrix, Mat &euler);

int main(int argc, char *argv[])
{

    String fname = "./param/intrisic.xml"; //choose the right intrisic of camera
    int video_l = 800;                     //choose the right resolution of camera
    int video_h = 600;
    bool saveVideo = false; //choose save video or not

    bool flag_v = false;              //read video from video or not
    String video = "./drone_1.mp4";   //the name of read video
    String saveName = "./result.mp4"; //the name of saved video
    int camId = 0;

    /***************************************************************************
     * 
     *                                Parameters
     * 
    ***************************************************************************/
    //aruco marker big parameters
    int dictionaryId_19 = 19;
    float markerLength_19 = 1.12;
    //aruco marker small parameters
    int dictionaryId_43 = 43;
    float markerLength_43 = 0.112;

    // read camera intrisic para
    Mat camMatrix, distCoeffs;

    bool readOk = readCameraParameters(fname, camMatrix, distCoeffs);
    if (!readOk)
    {
        cerr << "Invalid camera file" << endl;
        return 0;
    }

    //read detector parameter
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

    /******************************************************************************
     * 
     *                             Get predefined dictionary
     *                             Define the length of axis
     *                             Create board object
     *                              
     * ****************************************************************************/
    Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(10);

    //lenght of axis
    float axisLength = 0.5f * ((float)(markerLength_19));

    /******************************************************************************
     * 
     *                                       Capture video
     * 
     * ******************************************************************************/

    VideoCapture inputVideo;
    int waitTime;
    if (!video.empty() && flag_v)
    {

        inputVideo.open(video);
        waitTime = 10;
    }
    else
    {
        cout << "video is not empty" << endl;
        inputVideo.open(camId);

        waitTime = 10;
    }
    if (!inputVideo.isOpened())
    {
        CV_Assert("Cam open failed");
    }
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, video_l); //1280 720
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, video_h);

    double totalTime = 0;
    int totalIterations = 0;

    /************************************************************************************
    *                                   
    *                                    Kalman filter parameter
    * 
    **************************************************************************************/
    // Kalman Filter parameters
    int minInliersKalman = 30; // Kalman threshold updating
    KalmanFilter KF;           // instantiate Kalman Filter
    int nStates = 18;          // the number of states
    int nMeasurements = 6;     // the number of measured states
    int nInputs = 0;           // the number of control actions
    double dt = 0.125;         // time between measurements (1/FPS)

    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt); // init function
    Mat measurements(nMeasurements, 1, CV_64FC1);
    measurements.setTo(Scalar(0));
    bool good_measurement = false;
    /*************************************************************************************/

    int myFourCC = VideoWriter::fourcc('m', 'p', '4', 'v'); //mp4
    double rate = inputVideo.get(CAP_PROP_FPS);
    Size size = Size(video_l, video_h);
    VideoWriter writer(saveName, myFourCC, rate, size, true);

    while (inputVideo.grab())
    {
        Mat image, imageCopy;
        inputVideo.retrieve(image);
        Vec3d cam_rvec, cam_tvec;
        Vec3d eulerAngles;
        double yaw, roll, pitch;
        double tick = (double)getTickCount();
        int lost, got;

        //aruco board

        Vec3d rvec, tvec;

        std::vector<int> markerids;
        vector<vector<Point2f>> markerCorners,rejectedCandidate;
        Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(image, dictionary, markerCorners, markerids, parameters, rejectedCandidate);

        double roll_kf, yaw_kf, pitch_kf;
        Mat camera_R, camera_T;
        Mat translation_estimated(3, 1, CV_64F);
        // void poseEstimation(markerids,  markerCorners, rvec, tvec, distCoeffs, roll_kf, yaw_kf, pitch_kf, roll, yaw, pitch, markerLength_19, markerLength_43, camMatrix, camera_R, camera_T,measurements,KF, translation_estimated);
        

        image.copyTo(imageCopy);
        stringstream s1, s2, s3, s4, s5;
        s1 << "Drone Position: x=" << int(camera_T.at<double>(0, 0) * 100) << " y=" << int(camera_T.at<double>(1, 0) * 100) << " z=" << int(camera_T.at<double>(2, 0) * 100);
        s2 << "Drone Attitude: yaw=" << int(yaw) << " pitch=" << int(pitch) << " roll=" << int(roll);
        s3 << "After Kalman filter";
        s4 << "Drone Position: x=" << int(translation_estimated.at<double>(0, 0) * 100) << " y=" << int(translation_estimated.at<double>(1, 0) * 100) << " z=" << int(translation_estimated.at<double>(2, 0) * 100);
        s5 << "Drone Attitude: yaw=" << int(yaw_kf) << " pitch=" << int(pitch_kf) << " roll=" << int(roll_kf);

        cout << s4.str() << endl;
        cout << s5.str() << endl;
        String position = s1.str();
        String attitude = s2.str();
        String info = s3.str();
        String kf_position = s4.str();
        String kf_attitude = s5.str();

        int font_face = cv::FONT_HERSHEY_COMPLEX;
        int baseline;
        double font_scale = 0.5;
        int thinkness = 1.8;
        cv::Size text_size = cv::getTextSize(position, font_face, font_scale, thinkness, &baseline);
        cv::Point orgin_position, second_position, third_position, fourth_position, fifth_position;
        orgin_position.x = imageCopy.cols / 20;
        orgin_position.y = imageCopy.rows / 15;
        second_position.x = imageCopy.cols / 20;
        second_position.y = imageCopy.rows / 15 + 2 * text_size.height;
        third_position.x = imageCopy.cols / 20;
        third_position.y = imageCopy.rows / 15 + 4 * text_size.height;
        fourth_position.x = imageCopy.cols / 20;
        fourth_position.y = imageCopy.rows / 15 + 6 * text_size.height;
        fifth_position.x = imageCopy.cols / 20;
        fifth_position.y = imageCopy.rows / 15 + 8 * text_size.height;

        cv::putText(imageCopy, position, orgin_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, attitude, second_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, info, third_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, kf_position, fourth_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, kf_attitude, fifth_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);

        /********************************************************************************************/
        // void poseEstimation(&)
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerids);

        if (saveVideo)
            writer << imageCopy;

        imshow("out", imageCopy);

        char key = (char)waitKey(waitTime);
        if (key == 27)
            break;
    }
    inputVideo.release();

    writer.release();

    return 0;
}
void poseEstimation(vector<int> & markerids, vector<vector<Point2f>>& markerCorners, Vec3d & rvec, Vec3d & tvec, Mat & distCoeffs, double &roll_kf, double &yaw_kf, double &pitch_kf,  
double &roll, double &yaw, double &pitch, int & markerLength_19, int & markerLength_43, Mat &camMatrix, Mat &camera_R, Mat &camera_T, Mat &measurements, KalmanFilter &KF,Mat &translation_estimated)
        {
            if (markerids.size() > 0)
            {
                cout << "IDS:" << markerids.size() << endl;
                int t = -1;
                for (int tt = 0; tt < markerids.size(); tt++)
                {
                    if (19 == markerids[tt])
                        t = tt;
                    cout << "tt19:" << tt << endl;
                }
                if (-1 == t)
                {
                    for (int tt = 0; tt < markerids.size(); tt++)
                    {
                        if (43 == markerids[tt])
                            t = tt;
                        cout << "tt43:" << tt << endl;
                    }
                }
                if (-1 != t) //we detected the marker
                {
                    vector<vector<Point2f>> singMarkerCorner_19, singMarkerCorner_43;
                    if (markerids[t] == 19)
                    {
                        singMarkerCorner_19.push_back(markerCorners[t]);
                        cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_19, markerLength_19, camMatrix, distCoeffs, rvec, tvec);
                    }
                    else if (markerids[t] == 43)
                    {
                        singMarkerCorner_43.push_back(markerCorners[t]);
                        cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_43, markerLength_43, camMatrix, distCoeffs, rvec, tvec);
                    }
                    cout << "Rotation Vector:" << rvec << endl;
                    cout << "Translation Vector" << tvec << endl;

                    Mat R_cv, T_cv;
                    cv::Rodrigues(rvec, R_cv);   //calculate your markerboard pose----rotation matrix
                    camera_R = R_cv.t();         //calculate your camera pose---- rotation matrix
                    camera_T = -camera_R * tvec; //calculate your camera translation------translation vector

                    Mat kf_eulers(3, 1, CV_64F);
                    kf_eulers = rot2euler(camera_R); //convert camera matrix to euler angle
                    // cout << "Euler angles:" << kf_eulers.t() * 180 / CV_PI << endl;
                    roll = kf_eulers.at<double>(0) * 180 / CV_PI;  //roll
                    pitch = kf_eulers.at<double>(1) * 180 / CV_PI; //pitch
                    yaw = kf_eulers.at<double>(2) * 180 / CV_PI;   //yaw

                    fillMeasurements(measurements, camera_T, camera_R);
                    // good_measurement = true;

                    
                    Mat rotation_estimated(3, 3, CV_64F);
                    updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);

                    //get the updated attitude
                    Mat measured_eulers(3, 1, CV_64F);
                    measured_eulers = rot2euler(rotation_estimated); //convert camera matrix to euler angle
                    roll_kf = measured_eulers.at<double>(0) * 180 / CV_PI;
                    pitch_kf = measured_eulers.at<double>(1) * 180 / CV_PI;
                    yaw_kf = measured_eulers.at<double>(2) * 180 / CV_PI;
                }
            }
        }

    // if (markerids.size() > 0)
    // {
    //     cout << "IDS:" << markerids.size() << endl;
    //     int t = -1;
    //     for (int tt = 0; tt < markerids.size(); tt++)
    //     {
    //         if (19 == markerids[tt])
    //             t = tt;
    //         cout << "tt19:" << tt << endl;
    //     }
    //     if (-1 == t)
    //     {
    //         for (int tt = 0; tt < markerids.size(); tt++)
    //         {
    //             if (43 == markerids[tt])
    //                 t = tt;
    //             cout << "tt43:" << tt << endl;
    //         }
    //     }
    //     if (-1 != t)
    //     {
    //         vector<vector<Point2f>> singMarkerCorner_19, singMarkerCorner_43;
    //         if (markerids[t] == 19)
    //         {
    //             singMarkerCorner_19.push_back(markerCorners[t]);
    //             cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_19, markerLength_19, camMatrix, distCoeffs, rvec, tvec);
    //             cout << "rvec:" << rvec << endl;
    //             cout << "tvec:" << tvec << endl;
    //         }
    //         else if (markerids[t] == 43)
    //         {
    //             singMarkerCorner_43.push_back(markerCorners[t]);
    //             cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_43, markerLength_43, camMatrix, distCoeffs, rvec, tvec);
    //             cout << "rvec:" << rvec << endl;
    //             cout << "tvec:" << tvec << endl;
    //         }
    //         /**********************************************************************************************************
    //                                               To get Camera pose
    //        As we get the rotation vector and translation vector of the board wrt. camera , we need to transform the
    //        point into the coordinate system of the marker board.
    //        There are two ways to figure out.
    //        1. Just like below
    //        2. Using decomposeProjectionMatrix()
    //        **********************************************************************************************************/
    //         Mat R_cv, T_cv, camera_R, camera_T;
    //         cv::Rodrigues(rvec, R_cv);   //calculate your markerboard pose----rotation matrix
    //         camera_R = R_cv.t();         //calculate your camera pose---- rotation matrix
    //         camera_T = -camera_R * tvec; //calculate your camera translation------translation vector

    //         Mat kf_eulers(3, 1, CV_64F);
    //         kf_eulers = rot2euler(camera_R); //convert camera matrix to euler angle
    //         // cout << "Euler angles:" << kf_eulers.t() * 180 / CV_PI << endl;
    //         roll = kf_eulers.at<double>(0) * 180 / CV_PI;  //roll
    //         pitch = kf_eulers.at<double>(1) * 180 / CV_PI; //pitch
    //         yaw = kf_eulers.at<double>(2) * 180 / CV_PI;   //yaw

    //         /**********************************************************************************************************

    //                                                 Kalman Filter
    //                                         Get the updated attitude

    //         **********************************************************************************************************/
    //         fillMeasurements(measurements, camera_T, camera_R);
    //         good_measurement = true;

    //         Mat translation_estimated(3, 1, CV_64F);
    //         Mat rotation_estimated(3, 3, CV_64F);
    //         updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);

    //         Mat measured_eulers(3, 1, CV_64F);
    //         measured_eulers = rot2euler(rotation_estimated); //convert camera matrix to euler angle
    //         double roll_kf = measured_eulers.at<double>(0) * 180 / CV_PI;
    //         double pitch_kf = measured_eulers.at<double>(1) * 180 / CV_PI;
    //         double yaw_kf = measured_eulers.at<double>(2) * 180 / CV_PI;

    //         /**********************************************************************************************************

    //                          Draw the position and the attitude of the camera

    //         **********************************************************************************************************/
    //         image.copyTo(imageCopy);
    //         stringstream s1, s2, s3, s4, s5;
    //         s1 << "Drone Position: x=" << int(camera_T.at<double>(0, 0) * 100) << " y=" << int(camera_T.at<double>(1, 0) * 100) << " z=" << int(camera_T.at<double>(2, 0) * 100);
    //         s2 << "Drone Attitude: yaw=" << int(yaw) << " pitch=" << int(pitch) << " roll=" << int(roll);
    //         s3 << "After Kalman filter";
    //         s4 << "Drone Position: x=" << int(translation_estimated.at<double>(0, 0) * 100) << " y=" << int(translation_estimated.at<double>(1, 0) * 100) << " z=" << int(translation_estimated.at<double>(2, 0) * 100);
    //         s5 << "Drone Attitude: yaw=" << int(yaw_kf) << " pitch=" << int(pitch_kf) << " roll=" << int(roll_kf);

    //         cout << s4.str() << endl;
    //         cout << s5.str() << endl;
    //         String position = s1.str();
    //         String attitude = s2.str();
    //         String info = s3.str();
    //         String kf_position = s4.str();
    //         String kf_attitude = s5.str();

    //         int font_face = cv::FONT_HERSHEY_COMPLEX;
    //         int baseline;
    //         double font_scale = 0.5;
    //         int thinkness = 1.8;
    //         cv::Size text_size = cv::getTextSize(position, font_face, font_scale, thinkness, &baseline);
    //         cv::Point orgin_position, second_position, third_position, fourth_position, fifth_position;
    //         orgin_position.x = imageCopy.cols / 20;
    //         orgin_position.y = imageCopy.rows / 15;
    //         second_position.x = imageCopy.cols / 20;
    //         second_position.y = imageCopy.rows / 15 + 2 * text_size.height;
    //         third_position.x = imageCopy.cols / 20;
    //         third_position.y = imageCopy.rows / 15 + 4 * text_size.height;
    //         fourth_position.x = imageCopy.cols / 20;
    //         fourth_position.y = imageCopy.rows / 15 + 6 * text_size.height;
    //         fifth_position.x = imageCopy.cols / 20;
    //         fifth_position.y = imageCopy.rows / 15 + 8 * text_size.height;

    //         cv::putText(imageCopy, position, orgin_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
    //         cv::putText(imageCopy, attitude, second_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
    //         cv::putText(imageCopy, info, third_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
    //         cv::putText(imageCopy, kf_position, fourth_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
    //         cv::putText(imageCopy, kf_attitude, fifth_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);

    //         /**********************************************************************************************************
    //                                               To get Camera pose
    //        As we get the rotation vector and translation vector of the board wrt. camera , we need to transform the
    //        point into the coordinate system of the marker board.
    //        There are two ways to figure out.
    //        1. Just like below
    //        2. Using decomposeProjectionMatrix()
    //        **********************************************************************************************************/
    //         Mat R_cv, T_cv, camera_R, camera_T;
    //         cv::Rodrigues(rvec, R_cv);   //calculate your markerboard pose----rotation matrix
    //         camera_R = R_cv.t();         //calculate your camera pose---- rotation matrix
    //         camera_T = -camera_R * tvec; //calculate your camera translation------translation vector

    //         Mat kf_eulers(3, 1, CV_64F);
    //         kf_eulers = rot2euler(camera_R); //convert camera matrix to euler angle

    //         roll = kf_eulers.at<double>(0) * 180 / CV_PI;  //roll
    //         pitch = kf_eulers.at<double>(1) * 180 / CV_PI; //pitch
    //         yaw = kf_eulers.at<double>(2) * 180 / CV_PI;   //yaw

    //         /**********************************************************************************************************

    //                                                 Kalman Filter
    //                                         Get the updated attitude

    //         **********************************************************************************************************/
    //         fillMeasurements(measurements, camera_T, camera_R);
    //         good_measurement = true;

    //         Mat translation_estimated(3, 1, CV_64F);
    //         Mat rotation_estimated(3, 3, CV_64F);
    //         updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);

    //         Mat measured_eulers(3, 1, CV_64F);
    //         measured_eulers = rot2euler(rotation_estimated); //convert camera matrix to euler angle
    //         double roll_kf = measured_eulers.at<double>(0) * 180 / CV_PI;
    //         double pitch_kf = measured_eulers.at<double>(1) * 180 / CV_PI;
    //         double yaw_kf = measured_eulers.at<double>(2) * 180 / CV_PI;

    //         /**********************************************************************************************************

    //                          Draw the position and the attitude of the camera

    //         **********************************************************************************************************/
    //         image.copyTo(imageCopy);
    //         stringstream s1, s2, s3, s4, s5;
    //         s1 << "Drone Position: x=" << int(camera_T.at<double>(0, 0) * 100) << " y=" << int(camera_T.at<double>(1, 0) * 100) << " z=" << int(camera_T.at<double>(2, 0) * 100);
    //         s2 << "Drone Attitude: yaw=" << int(yaw) << " pitch=" << int(pitch) << " roll=" << int(roll);
    //         s3 << "After Kalman filter";
    //         s4 << "Drone Position: x=" << int(translation_estimated.at<double>(0, 0) * 100) << " y=" << int(translation_estimated.at<double>(1, 0) * 100) << " z=" << int(translation_estimated.at<double>(2, 0) * 100);
    //         s5 << "Drone Attitude: yaw=" << int(yaw_kf) << " pitch=" << int(pitch_kf) << " roll=" << int(roll_kf);

    //         cout << s4.str() << endl;
    //         cout << s5.str() << endl;
    //         String position = s1.str();
    //         String attitude = s2.str();
    //         String info = s3.str();
    //         String kf_position = s4.str();
    //         String kf_attitude = s5.str();

    //         int font_face = cv::FONT_HERSHEY_COMPLEX;
    //         int baseline;
    //         double font_scale = 0.5;
    //         int thinkness = 1.8;
    //         cv::Size text_size = cv::getTextSize(position, font_face, font_scale, thinkness, &baseline);
    //         cv::Point orgin_position, second_position, third_position, fourth_position, fifth_position;
    //         orgin_position.x = imageCopy.cols / 20;
    //         orgin_position.y = imageCopy.rows / 15;
    //         second_position.x = imageCopy.cols / 20;
    //         second_position.y = imageCopy.rows / 15 + 2 * text_size.height;
    //         third_position.x = imageCopy.cols / 20;
    //         third_position.y = imageCopy.rows / 15 + 4 * text_size.height;
    //         fourth_position.x = imageCopy.cols / 20;
    //         fourth_position.y = imageCopy.rows / 15 + 6 * text_size.height;
    //         fifth_position.x = imageCopy.cols / 20;
    //         fifth_position.y = imageCopy.rows / 15 + 8 * text_size.height;

    //         cv::putText(imageCopy, position, orgin_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
    //         cv::putText(imageCopy, attitude, second_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
    //         cv::putText(imageCopy, info, third_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
    //         cv::putText(imageCopy, kf_position, fourth_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
    //         cv::putText(imageCopy, kf_attitude, fifth_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);

    //         /********************************************************************************************/

    //         cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerids);

    //         if (saveVideo)
    //             writer << imageCopy;

    //         imshow("out", imageCopy);
    //     }
    //     char key = (char)waitKey(waitTime);
    //     if (key == 27)
    //         break;
    // }
    


void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
    KF.init(nStates, nMeasurements, nInputs, CV_64F); // init Kalman Filter

    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));     // set process noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2)); // set measurement noise
    setIdentity(KF.errorCovPost, Scalar::all(1));           // error covariance

    /** DYNAMIC MODEL **/

    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

    // position
    KF.transitionMatrix.at<double>(0, 3) = dt;
    KF.transitionMatrix.at<double>(1, 4) = dt;
    KF.transitionMatrix.at<double>(2, 5) = dt;
    KF.transitionMatrix.at<double>(3, 6) = dt;
    KF.transitionMatrix.at<double>(4, 7) = dt;
    KF.transitionMatrix.at<double>(5, 8) = dt;
    KF.transitionMatrix.at<double>(0, 6) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(1, 7) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(2, 8) = 0.5 * pow(dt, 2);

    // orientation
    KF.transitionMatrix.at<double>(9, 12) = dt;
    KF.transitionMatrix.at<double>(10, 13) = dt;
    KF.transitionMatrix.at<double>(11, 14) = dt;
    KF.transitionMatrix.at<double>(12, 15) = dt;
    KF.transitionMatrix.at<double>(13, 16) = dt;
    KF.transitionMatrix.at<double>(14, 17) = dt;
    KF.transitionMatrix.at<double>(9, 15) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(10, 16) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(11, 17) = 0.5 * pow(dt, 2);

    /** MEASUREMENT MODEL **/

    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

    KF.measurementMatrix.at<double>(0, 0) = 1;  // x
    KF.measurementMatrix.at<double>(1, 1) = 1;  // y
    KF.measurementMatrix.at<double>(2, 2) = 1;  // z
    KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
    KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
    KF.measurementMatrix.at<double>(5, 11) = 1; // yaw
}

/**********************************************************************************************************/
void updateKalmanFilter(KalmanFilter &KF, Mat &measurement,
                        Mat &translation_estimated, Mat &rotation_estimated)
{
    // First predict, to update the internal statePre variable
    Mat prediction = KF.predict();

    // The "correct" phase that is going to use the predicted value and our measurement
    Mat estimated = KF.correct(measurement);

    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);

    // Estimated euler angles
    Mat eulers_estimated(3, 1, CV_64F);
    eulers_estimated.at<double>(0) = estimated.at<double>(9);
    eulers_estimated.at<double>(1) = estimated.at<double>(10);
    eulers_estimated.at<double>(2) = estimated.at<double>(11);

    // Convert estimated quaternion to rotation matrix
    rotation_estimated = euler2rot(eulers_estimated);
}

/**********************************************************************************************************/
void fillMeasurements(Mat &measurements,
                      const Mat &translation_measured, const Mat &rotation_measured)
{
    // Convert rotation matrix to euler angles
    Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(rotation_measured);

    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
    measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}

void rot_euler(const Mat &rotation_matrix, Mat &euler)
{
    euler = rot2euler(rotation_matrix);
}

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs)
{
    cerr << filename << endl;
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["cameraMatrix"] >> camMatrix;
    fs["distCoeffs"] >> distCoeffs;
    cerr << camMatrix << endl;
    cerr << distCoeffs << endl;
    return true;
}

// void poseEstimation(vector<int> &markerids, Vec3d &rvec, Vec3d &tvec, Mat &distCoeffs, double &roll_kf, double &yaw_kf, double &pitch_kf, double &roll, double &yaw, double &pitch,int &markerLength_19, int &markerLength_43, Mat &camMatrix, Mat &camera_R, Mat &camera_T,KalmanFilter &KF)
// {
//     if (markerids.size() > 0)
//     {
//         cout << "IDS:" << markerids.size() << endl;
//         int t = -1;
//         for (int tt = 0; tt < markerids.size(); tt++)
//         {
//             if (19 == markerids[tt])
//                 t = tt;
//             cout << "tt19:" << tt << endl;
//         }
//         if (-1 == t)
//         {
//             for (int tt = 0; tt < markerids.size(); tt++)
//             {
//                 if (43 == markerids[tt])
//                     t = tt;
//                 cout << "tt43:" << tt << endl;
//             }
//         }
//         if (-1 != t) //we detected the marker
//         {
//             vector<vector<Point2f>> singMarkerCorner_19, singMarkerCorner_43;
//             if (markerids[t] == 19)
//             {
//                 singMarkerCorner_19.push_back(markerCorners[t]);
//                 cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_19, markerLength_19, camMatrix, distCoeffs, rvec, tvec);
//             }
//             else if (markerids[t] == 43)
//             {
//                 singMarkerCorner_43.push_back(markerCorners[t]);
//                 cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_43, markerLength_43, camMatrix, distCoeffs, rvec, tvec);
//             }
//             cout << "Rotation Vector:" << rvec << endl;
//             cout << "Translation Vector" << tvec << endl;

//             Mat R_cv, T_cv;
//             cv::Rodrigues(rvec, R_cv);   //calculate your markerboard pose----rotation matrix
//             camera_R = R_cv.t();         //calculate your camera pose---- rotation matrix
//             camera_T = -camera_R * tvec; //calculate your camera translation------translation vector

//             Mat kf_eulers(3, 1, CV_64F);
//             kf_eulers = rot2euler(camera_R); //convert camera matrix to euler angle
//             // cout << "Euler angles:" << kf_eulers.t() * 180 / CV_PI << endl;
//             roll = kf_eulers.at<double>(0) * 180 / CV_PI;  //roll
//             pitch = kf_eulers.at<double>(1) * 180 / CV_PI; //pitch
//             yaw = kf_eulers.at<double>(2) * 180 / CV_PI;   //yaw

//             fillMeasurements(measurements, camera_T, camera_R);
//             good_measurement = true;

//             Mat translation_estimated(3, 1, CV_64F);
//             Mat rotation_estimated(3, 3, CV_64F);
//             updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);

//             //get the updated attitude
//             Mat measured_eulers(3, 1, CV_64F);
//             measured_eulers = rot2euler(rotation_estimated); //convert camera matrix to euler angle
//             roll_kf = measured_eulers.at<double>(0) * 180 / CV_PI;
//             pitch_kf = measured_eulers.at<double>(1) * 180 / CV_PI;
//             yaw_kf = measured_eulers.at<double>(2) * 180 / CV_PI;
//         }
//     }
// }