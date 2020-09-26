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

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side lenght (in pixels) }"
        "{s        |       | Separation between two consecutive markers in the grid (in pixels)}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    cerr << filename << endl;
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["cameraMatrix"] >> camMatrix;
    fs["distCoeffs"] >> distCoeffs;
    cerr << camMatrix << endl;
    cerr << distCoeffs << endl;
    return true;
}
/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}
/*
get euler angles from rotation matrix
 */
void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles){

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}


//Kalman filter funtions headers
void initKalmanFilter( KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
void predictKalmanFilter( KalmanFilter &KF, Mat &translation_predicted, Mat &rotation_predicted );
void updateKalmanFilter( KalmanFilter &KF, Mat &measurements,
                         Mat &translation_estimated, Mat &rotation_estimated );
void fillMeasurements( Mat &measurements,
                       const Mat &translation_measured, const Mat &rotation_measured);
void rot_euler(const Mat &rotation_matrix, Mat &euler);

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 7) {
        parser.printMessage();
        return 0;
    }

    int markersX = parser.get<int>("w");
    int markersY = parser.get<int>("h");
    float markerLength = parser.get<float>("l");
    float markerSeparation = parser.get<float>("s");
    int dictionaryId = parser.get<int>("d");
    // bool showRejected = parser.has("r");
    bool showRejected = false;
    // bool refindStrategy = parser.has("rs");
    bool refindStrategy = true;
    int camId = parser.get<int>("ci");

     // Kalman Filter parameters
    int minInliersKalman = 30;    // Kalman threshold updating

    // read camera intrisic para
    Mat camMatrix, distCoeffs;
    if(parser.has("c")) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }
    //read detector parameter
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
    
    //capture video
    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    //get predefined dictionary
    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    if(!inputVideo.isOpened()){
        CV_Assert("Cam open failed");
    }
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH,1280);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT,720);


    //lenght of axis
    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);

    // create board object
    Ptr<aruco::GridBoard> gridboard =
        aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

    double totalTime = 0;
    int totalIterations = 0;

    /************************************************************************************
                                       Kalman filter parameter
    **************************************************************************************/
    KalmanFilter KF;             // instantiate Kalman Filter
    int nStates = 18;            // the number of states
    int nMeasurements = 6;       // the number of measured states
    int nInputs = 0;             // the number of control actions
    double dt = 0.125;           // time between measurements (1/FPS)

    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
    Mat measurements(nMeasurements, 1, CV_64FC1); measurements.setTo(Scalar(0));
    bool good_measurement = false;
    /*************************************************************************************/

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        Vec3d rvec, tvec;
        Vec3d cam_rvec, cam_tvec;
        Vec3d eulerAngles;
        double yaw, roll, pitch;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy)
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,
                                         distCoeffs);
        

        // estimate board pose

        int markersOfBoardDetected = 0;

        if(ids.size() > 0)
            /*rvec: Output vector (e.g. cv::Mat) corresponding to the rotation vector of the board*/
            /*tvec: Output vector (e.g. cv::Mat) corresponding to the translation vector of the board*/
            markersOfBoardDetected =
                aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);
            
            // tvec[0]=tvec[0]-0.12;
            // tvec[0]=tvec[0]-0.12;
            // tvec[2]=tvec[2]-0.12;
            /**********************************************************************************************************
                                                      To get Camera pose
            As we get the rotation vector and translation vector of the board wrt. camera , we need to transform the 
            point into the coordinate system of the marker board.
            There are two ways to figure out.
            1. Just like below
            2. Using decomposeProjectionMatrix()  
            **********************************************************************************************************/
            Mat R_cv, T_cv,camera_R,camera_T;
            cv::Rodrigues(rvec, R_cv);      //calculate your markerboard pose----rotation matrix
            camera_R=R_cv.t();              //calculate your camera pose---- rotation matrix
            camera_T=-camera_R*tvec;        //calculate your camera translation------translation vector
            
            Eigen::MatrixXd camera_r_eigen(3,3);
            cv2eigen(camera_R, camera_r_eigen);

            cout<<"matrix:\n"<<camera_R<<endl;
            cout<<"matrixxd:\n"<<camera_r_eigen<<endl;
            

            // Eigen::Quaterniond EigenQuat(camera_r_eigen);
            // getchar();
            // Eigen::Quaterniond q;
            // q=camera_r_eigen;
            // cout<<"quaternion = \n"<<q.coeffs()<<endl;


            
            
            
            // getEulerAngles(camera_R,eulerAngles);
            // cout<<"Euler angles-1:"<<eulerAngles<<endl;
            // pitch=eulerAngles[0];  //roll
            // yaw=eulerAngles[1];    //pitch
            // roll=eulerAngles[2];   //yaw
            Mat kf_eulers(3, 1, CV_64F);
            kf_eulers = rot2euler(camera_R);
            cout<<"Euler angles:"<<kf_eulers.t()*180/CV_PI<<endl;
            roll=kf_eulers.at<double>(0)*180/CV_PI;  //roll
            pitch=kf_eulers.at<double>(1)*180/CV_PI;    //pitch
            yaw=kf_eulers.at<double>(2)*180/CV_PI;   //yaw
    
            /**********************************************************************************************************
                                                        
                                                        Kalman Filter
    
            **********************************************************************************************************/
            fillMeasurements(measurements,camera_T,camera_R);
            good_measurement=true;
        
        Mat translation_estimated(3, 1, CV_64F);
        Mat rotation_estimated(3, 3, CV_64F);
        updateKalmanFilter(KF,measurements,translation_estimated,rotation_estimated);

        //get the updated attitude
        Mat measured_eulers(3, 1, CV_64F);
        measured_eulers = rot2euler(rotation_estimated);
        double roll_kf=measured_eulers.at<double>(0)*180/CV_PI;
        double pitch_kf=measured_eulers.at<double>(1)*180/CV_PI;
        double yaw_kf=measured_eulers.at<double>(2)*180/CV_PI;

        /**********************************************************************************************************/



        
        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        /**********************************************************************************************************
                                 
                                 Draw the position and the attitude of the camera
        
        **********************************************************************************************************/
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            
        }
        
        stringstream s1,s2,s3,s4,s5;
        s1<<"Drone Position: x="<<int(camera_T.at<double>(0,0)*100)<<" y="<<int(camera_T.at<double>(1,0)*100)<<" z="<<int(camera_T.at<double>(2,0)*100);
        s2<<"Drone Attitude: yaw="<<int(yaw)<<" pitch="<<int(pitch)<<" roll="<<int(roll);
        s3<<"After Kalman filter";
        s4<<"Drone Position: x="<<int(translation_estimated.at<double>(0,0)*100)<<" y="<<int(translation_estimated.at<double>(1,0)*100)<<" z="<<int(translation_estimated.at<double>(2,0)*100);
        s5<<"Drone Attitude: yaw="<<int(yaw_kf)<<" pitch="<<int(pitch_kf)<<" roll="<<int(roll_kf);

        String position=s1.str();
        String attitude=s2.str();
        String info=s3.str();
        String kf_position=s4.str();
        String kf_attitude=s5.str();
        int font_face=cv::FONT_HERSHEY_COMPLEX;
        int baseline;
        double font_scale=0.5;
        int thinkness=1.8;
        cv::Size text_size=cv::getTextSize(position,font_face, font_scale, thinkness, &baseline);
        cv::Point orgin_position,second_position,third_position,fourth_position,fifth_position;
        orgin_position.x=imageCopy.cols/20;
        orgin_position.y=imageCopy.rows/15;
        second_position.x=imageCopy.cols/20;
        second_position.y=imageCopy.rows/15+2*text_size.height;
        third_position.x=imageCopy.cols/20;
        third_position.y=imageCopy.rows/15+4*text_size.height;
        fourth_position.x=imageCopy.cols/20;
        fourth_position.y=imageCopy.rows/15+6*text_size.height;
        fifth_position.x=imageCopy.cols/20;
        fifth_position.y=imageCopy.rows/15+8*text_size.height;

        cv::putText(imageCopy,position,orgin_position,font_face,font_scale,cv::Scalar(0,255,0),thinkness,8,0);
        cv::putText(imageCopy,attitude,second_position,font_face,font_scale,cv::Scalar(0,255,0),thinkness,8,0);
        cv::putText(imageCopy,info,third_position,font_face,font_scale,cv::Scalar(0,255,0),thinkness,8,0);
        cv::putText(imageCopy,kf_position,fourth_position,font_face,font_scale,cv::Scalar(0,255,0),thinkness,8,0);
        cv::putText(imageCopy,kf_attitude,fifth_position,font_face,font_scale,cv::Scalar(0,255,0),thinkness,8,0);
        
        /********************************************************************************************/
        
        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
        // aruco::drawPlanarBoard(board, camMatrix, distCoeffs, rvec, tvec, axisLength)

        // tvec[0]=tvec[0]-0.12;
        // tvec[1]=tvec[1]-0.12;
        // tvec[2]=tvec[2]-0.12;

        if(markersOfBoardDetected > 0)
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

        imshow("out", imageCopy);
        cv::imwrite("i.jpg",imageCopy);
        // getChar();
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }

    return 0;
}

void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
    KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter

    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));       // set process noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));   // set measurement noise
    setIdentity(KF.errorCovPost, Scalar::all(1));             // error covariance

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
    KF.transitionMatrix.at<double>(0,3) = dt;
    KF.transitionMatrix.at<double>(1,4) = dt;
    KF.transitionMatrix.at<double>(2,5) = dt;
    KF.transitionMatrix.at<double>(3,6) = dt;
    KF.transitionMatrix.at<double>(4,7) = dt;
    KF.transitionMatrix.at<double>(5,8) = dt;
    KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);

    // orientation
    KF.transitionMatrix.at<double>(9,12) = dt;
    KF.transitionMatrix.at<double>(10,13) = dt;
    KF.transitionMatrix.at<double>(11,14) = dt;
    KF.transitionMatrix.at<double>(12,15) = dt;
    KF.transitionMatrix.at<double>(13,16) = dt;
    KF.transitionMatrix.at<double>(14,17) = dt;
    KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);


    /** MEASUREMENT MODEL **/

    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

    KF.measurementMatrix.at<double>(0,0) = 1;  // x
    KF.measurementMatrix.at<double>(1,1) = 1;  // y
    KF.measurementMatrix.at<double>(2,2) = 1;  // z
    KF.measurementMatrix.at<double>(3,9) = 1;  // roll
    KF.measurementMatrix.at<double>(4,10) = 1; // pitch
    KF.measurementMatrix.at<double>(5,11) = 1; // yaw
}

/**********************************************************************************************************/
void updateKalmanFilter( KalmanFilter &KF, Mat &measurement,
                         Mat &translation_estimated, Mat &rotation_estimated )
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
void fillMeasurements( Mat &measurements,
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

void rot_euler(const Mat &rotation_matrix, Mat &euler){
    euler=rot2euler(rotation_matrix);
}