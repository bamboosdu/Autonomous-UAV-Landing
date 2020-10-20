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
//#include <NX_C_Share.h>

using namespace std;
using namespace cv;
using namespace Eigen;

#define VISION_THRES 20

//read camera parameters
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);
//Kalman filter funtions headers
void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
void predictKalmanFilter(KalmanFilter &KF, Mat &translation_predicted, Mat &rotation_predicted);
void updateKalmanFilter(KalmanFilter &KF, Mat &measurements,Mat &translation_estimated, Mat &rotation_estimated);
void fillMeasurements(Mat &measurements,const Mat &translation_measured, const Mat &rotation_measured);
void rot_euler(const Mat &rotation_matrix, Mat &euler);

int main(int argc, char *argv[])
{
    /***************************************************************************
     * 
     *                           Program Parameters
     * 
    **************************************************************************/

    String fname = "./param/intrisic.xml"; //choose the right intrisic of camera
    int video_l = 848;//800                     //choose the right resolution of camera
    int video_h = 480;//600
    bool saveVideo = false; //choose save video or not

    bool flag_v = false;              //read video from video or not
    String video = "./drone_1.mp4";   //the name of read video
    String saveName = "./result.mp4"; //the name of saved video

    /***************************************************************************
     * 
     *                                Marker Parameters
     *                              Camera intrisic parameter
     * 
    ***************************************************************************/
    //aruco board parameters
    int markersX = 2;
    int markersY = 2;
    float markerLength = 0.606;
    float markerSeparation = 0.126;
    int dictionaryId = 2;
    bool showRejected = false;
    bool refindStrategy = true;
    int camId = 0;
    //aruco marker parameters
    int dictionaryId_center = 7;
    float markerLength_center = 0.12;

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
    //aruco board
    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    //center aruco marker
    Ptr<aruco::Dictionary> dictionary_center =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId_center));

    //lenght of axis
    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);
    // create board object
    Ptr<aruco::GridBoard> gridboard =
        aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

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

    /********************************************************
    * 
    *                   Record the result by video
    * 
    ********************************************************/

    int myFourCC = VideoWriter::fourcc('m', 'p', '4', 'v'); //mp4
    double rate = inputVideo.get(CAP_PROP_FPS);
    Size size = Size(video_l, video_h);
    VideoWriter writer(saveName, myFourCC, rate, size, true);


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

    
    while (inputVideo.grab())
    {
        Mat image, imageCopy;
        inputVideo.retrieve(image);
        Vec3d cam_rvec, cam_tvec;
        Vec3d eulerAngles;
        double yaw, roll, pitch;
        double tick = (double)getTickCount();
        int lost, got;
        /***********************************************************************************
         * 
         * The detected markers are stored in the markerCorners and markerIds structures:
         * !The output parameters rvecs and tvecs are the rotation and translation vectors of marker respectively, 
         * !for each of the markers in markerCorners
         * 
         * ****************************************************************************8**/
        //aruco board
        vector<int> ids;
        vector<vector<Point2f>> corners, rejected;
        Vec3d rvec, tvec;
        //marker
        vector<int> ids_center;
        vector<vector<Point2f>> corners_center, rejected_center;
        vector<Vec3d> rvecs_center, tvecs_center;

        /***********************************************************************************
         * 
         *                      Detect the board and the marker
         * 
         * ****************************************************************************8**/

        // detect aruco board
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        //detect aruco marker
        aruco::detectMarkers(image, dictionary_center, corners_center, ids_center,
                             detectorParams, rejected_center);
        // refind strategy to detect more markers
        if (refindStrategy)
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,
                                         distCoeffs);
        cout << "IDS_single_marker:" << ids_center.size() << endl;
        cout << "IDS_board:" << ids.size() << endl;
        
        int markersOfBoardDetected = 0;

        Mat R_cv, T_cv, camera_R, camera_T;
        if (ids.size() > 0 || ids_center.size() > 0)//The big one or the small one is deteceted
        {

            // if (ids_center.size() > 0)
            // {
            //     got++;
            //     lost = 0;
            // }
            // else
            // {
            //     got = 0;
            //     lost++;
            // }

            //if the center marker is detected
            if (ids_center.size() > 0)// && got > VISION_THRES
            {
                aruco::estimatePoseSingleMarkers(corners_center, markerLength_center, camMatrix, distCoeffs, rvecs_center,
                                                 tvecs_center);
                rvec = rvecs_center[0];
                tvec = tvecs_center[0];
                markersOfBoardDetected = 1;
            }
            else
            {
                markersOfBoardDetected =
                    aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);
            }
            /**********************************************************************************************************
                                                      To get Camera pose
            As we get the rotation vector and translation vector of the board wrt. camera , we need to transform the 
            point into the coordinate system of the marker board.
            There are two ways to figure out.
            1. Just like below
            2. Using decomposeProjectionMatrix()  
            **********************************************************************************************************/
            
            cv::Rodrigues(rvec, R_cv);   //calculate your markerboard pose----rotation matrix
            camera_R = R_cv.t();         //calculate your camera pose---- rotation matrix
            camera_T = -camera_R * tvec; //calculate your camera translation------translation vector

            Mat kf_eulers(3, 1, CV_64F);
            kf_eulers = rot2euler(camera_R); //convert camera matrix to euler angle
            // cout << "Euler angles:" << kf_eulers.t() * 180 / CV_PI << endl;
            roll = kf_eulers.at<double>(0) * 180 / CV_PI;  //roll
            pitch = kf_eulers.at<double>(1) * 180 / CV_PI; //pitch
            yaw = kf_eulers.at<double>(2) * 180 / CV_PI;   //yaw

            /**********************************************************************************************************
                                          imageCopy              
                                                        Kalman Filter
                                                Get the updated attitude
    
            **********************************************************************************************************/
            fillMeasurements(measurements, camera_T, camera_R);
            
        }//*TODO THE POSITION IS CONFUSING
        
        // update the Kalman filter with good measurements, otherwise with previous valid measurements
        Mat translation_estimated(3, 1, CV_64F);
        Mat rotation_estimated(3, 3, CV_64F);
        updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);

        //get the updated attitude
        Mat measured_eulers(3, 1, CV_64F);
        measured_eulers = rot2euler(rotation_estimated); //convert camera matrix to euler angle
        double roll_kf = measured_eulers.at<double>(0) * 180 / CV_PI;
        double pitch_kf = measured_eulers.at<double>(1) * 180 / CV_PI;
        double yaw_kf = measured_eulers.at<double>(2) * 180 / CV_PI;
        cout << "KF:" << ids.size() << endl;
        /**********************************************************************************************************
                                 
                                 Draw the position and the attitude of the camera
        
        **********************************************************************************************************/
        image.copyTo(imageCopy);
        cout << "KF2:" << ids.size() << endl;
        if (ids.size() > 0)
        {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }
        if(markersOfBoardDetected){
        
        stringstream s1, s2, s3, s4, s5;
        s1 << "Drone Position: x=" << int(camera_T.at<double>(0, 0) * 100) << " y=" << int(camera_T.at<double>(1, 0) * 100) << " z=" << int(camera_T.at<double>(2, 0) * 100);
        s2 << "Drone Attitude: yaw=" << int(yaw) << " pitch=" << int(pitch) << " roll=" << int(roll);
        s3 << "After Kalman filter";
        s4 << "Drone Position: x=" << int(translation_estimated.at<double>(0, 0) * 100) << " y=" << int(translation_estimated.at<double>(1, 0) * 100) << " z=" << int(translation_estimated.at<double>(2, 0) * 100);
        s5 << "Drone Attitude: yaw=" << int(yaw_kf) << " pitch=" << int(pitch_kf) << " roll=" << int(roll_kf);

        //set_vision_position_estimate(int(camera_T.at<double>(0, 0) * 100),int(camera_T.at<double>(1, 0) * 100),int(camera_T.at<double>(2, 0) * 100),int(roll),int(pitch),int(yaw),100);

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
        if (markersOfBoardDetected > 0)
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
        }

        imshow("out", imageCopy);
        if (saveVideo)
            writer << imageCopy;
        
        
        char key = (char)waitKey(waitTime);
        if (key == 27)
            break;
    }
    inputVideo.release();
    if (saveVideo)
        writer.release();

    return 0;
}

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
