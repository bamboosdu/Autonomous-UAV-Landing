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
#include <NX_C_Share.h>

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stropts.h>
#include <sys/select.h>
#include<fstream>  


#include <ctime>

using namespace std;
using namespace cv;
using namespace Eigen;

#define VISION_THRES 20

//read camera parameters
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);
//Kalman filter funtions headers
void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
void predictKalmanFilter(KalmanFilter &KF, Mat &translation_predicted, Mat &rotation_predicted);
void updateKalmanFilter(KalmanFilter &KF, Mat &measurements, Mat &translation_estimated, Mat &rotation_estimated);
void fillMeasurements(Mat &measurements, const Mat &translation_measured, const Mat &rotation_measured);
void rot_euler(const Mat &rotation_matrix, Mat &euler);
#include "kbhit.h"
#include <unistd.h> // read()
    
keyboard::keyboard(){
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    peek_character=-1;
}
    
keyboard::~keyboard(){
    tcsetattr(0, TCSANOW, &initial_settings);
}
    
int keyboard::kbhit(){
    unsigned char ch;
    int nread;
    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);

    if (nread == 1){
        peek_character = ch;
        return 1;
    }
    return 0;
}
    
int keyboard::getch(){
    char ch;

    if (peek_character != -1){
        ch = peek_character;
        peek_character = -1;
    }
    else read(0,&ch,1);
    return ch;
}
// extern "C"{
// void CLog::WriteLog(const wchar_t* filePath, float x_kf,float y_kf,float z_kf ,float roll_kf ,float pitch_kf ,float yaw_kf,int got)
// {
// 		//首先判断文件是否存在，如果不存在则创建，并在开头加入0xfeff;如果存在则直接写入
// 		if (_waccess(filePath, 0) == -1)
// 		{
// 			FILE* fp;
// 			_wfopen_s(&fp, filePath, L"wb");
// 			if (fp != NULL)
// 			{
// 				uint16_t wSignature = 0xFEFF;
// 				fwrite(&wSignature, 2, 1, fp);
// 				SYSTEMTIME st;
// 				GetLocalTime(&st);
// 				wchar_t buf[128] = { 0 };
// 				wchar_t buf_data[128] = { 0 };
// 				swprintf_s(buf, 128, L"%04d%02d%02d %02d:%02d:%02d	", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
// 				swprintf_s(buf_data, 128, L"%06f %06f %06f %06f %06f %06f %d	", x_kf, y_kf, z_kf , roll_kf , pitch_kf , yaw_kf, got);
// 				fwrite(buf, sizeof(wchar_t), wcslen(buf), fp);
// 				fwrite(buf_data, sizeof(wchar_t), wcslen(buf_data), fp);
// 				fwrite(L"\r\n", sizeof(wchar_t), 2, fp);
// 				fclose(fp);
// 			}
// 		}
// 		else 
// 		{
// 			FILE* fp;
// 			_wfopen_s(&fp, filePath, L"ab");
// 			if (fp != NULL)
// 			{
// 				SYSTEMTIME st;
// 				GetLocalTime(&st);
// 				wchar_t buf[128] = { 0 };
// 				wchar_t buf_data[128] = { 0 };
// 				swprintf_s(buf, 128, L"%04d%02d%02d %02d:%02d:%02d	", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
// 				swprintf_s(buf_data, 128, L"%06f %06f %06f %06f %06f %06f %d	", x_kf, y_kf, z_kf , roll_kf , pitch_kf , yaw_kf, got);
// 				fwrite(buf, sizeof(wchar_t), wcslen(buf), fp);
// 				fwrite(buf_data, sizeof(wchar_t), wcslen(buf_data), fp);
// 				fwrite(L"\r\n", sizeof(wchar_t), 2, fp);
// 				fclose(fp);
// 			}
// 		}
//     }
// 		}


int main(int argc, char *argv[])
{
    /***************************************************************************
     * 
     *                           Program Parameters
     * 
    **************************************************************************/

    String fname = "../param/intrisic.xml"; //choose the right intrisic of camera
    int video_l = 800;                      //800                     //choose the right resolution of camera
    int video_h = 600;                      //600
    bool saveVideo = true;                 //choose save video or not

    bool flag_v = false;              //read video from video or not
    String video = "./drone_1.mp4";   //the name of read video
    String saveName = "./result_001.mp4"; //the name of saved video
    ofstream fout("uttxt.txt");  
      
    Ptr<cv::aruco::Dictionary> dictionary_d = cv::aruco::getPredefinedDictionary(10);
    double landpad_det_len = 0.192;

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
     *                                    Capture video from device
     * 
     * ******************************************************************************/

    VideoCapture inputVideo;
    int waitTime=10;
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
    // cv.waitTime(100)
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, video_l); //1280 720
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, video_h);
    // int frameH   = inputVideo.get(cv::CAP_PROP_FRAME_HEIGHT);
	// int frameW    = inputVideo.get(cv::CAP_PROP_FRAME_WIDTH);
    // printf(frameH);
    // printf(frameW);
    cout<<"width "<<video_l<<endl;
    cout<<"height "<<video_h<<endl;
    // getchar();
    // video_h=1080;
    // video_l=1920;
    /********************************************************
    * 
    *                   Record the result by video
    * 
    ********************************************************/

    int myFourCC = VideoWriter::fourcc('M', 'P', '4', '2'); //mp4
    double rate = inputVideo.get(CAP_PROP_FPS);
    Size size = Size(video_l, video_h);
    
    VideoWriter writer(saveName, myFourCC, rate, size, true);
    if(!writer.isOpened())
    {
 	cout<< "Error : fail to open video writer\n"<<endl;
	return -1;
    }

    /************************************************************************************
         *                                   
        *                                    Kalman filter parameter
        * 
        **************************************************************************************/
    // Kalman Filter parameters
    // int minInliersKalman = 30; // Kalman threshold updating
    KalmanFilter KF;       // instantiate Kalman Filter
    int nStates = 18;      // the number of states
    int nMeasurements = 6; // the number of measured states
    int nInputs = 0;       // the number of control actions
    double dt = 0.125;     // time between measurements (1/FPS)

    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt); // init function
    Mat measurements(nMeasurements, 1, CV_64FC1);
    measurements.setTo(Scalar(0));
    bool good_measurement = false;
    /*************************************************************************************/
    int lost, got = 1;

    while (inputVideo.grab())
    {
        Mat image, imageCopy;
        inputVideo.retrieve(image);
        cout<<image.size()<<endl;
        // getchar();

        double yaw = 0, roll = 0, pitch = 0;

        double rm[9];
        double tm[3];
        Mat camera_R, camera_T;
        camera_R = Mat(3, 3, CV_64FC1, rm);
        camera_T = Mat(3, 1, CV_64FC2, tm);
        /***********************************************************************************
         * 
         * The detected markers are stored in the markerCorners and markerIds structures:
         * !The output parameters rvecs and tvecs are the rotation and translation vectors of marker respectively, 
         * !for each of the markers in markerCorners
         * 
         * ****************************************************************************8**/
        Vec3d rvec, tvec;
        vector<Vec3d> rvec_n, tvec_n;

        std::vector<int> markerids;
        vector<vector<Point2f>> markerCorners, rejectedCandidate;
        Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(image, dictionary_d, markerCorners, markerids, parameters, rejectedCandidate);

        if (markerids.size() > 0)
        {

            cv::Point3f A1_Sum_Position_OcInW(0, 0, 0);
            double tx, ty, tz;
            int t = -1;
            int key = 0;
            for (int tt = 0; tt < markerids.size(); tt++)
            {
                if (19 == markerids[tt])
                {
                    t = tt;
                    key = markerids[tt];
                }
            }
            if (-1 == t)
            {
                for (int tt = 0; tt < markerids.size(); tt++)
                {
                    if (43 == markerids[tt])
                    {
                        t = tt;
                        key = markerids[tt];
                    }
                }
            }

            if (-1 != t)
            {
                cv::Mat RoteM, TransM;
                cv::Point3f Theta_C2W;
                cv::Point3f Theta_W2C;
                cv::Point3f Position_OcInW;

                vector<vector<Point2f>> singMarkerCorner_19, singMarkerCorner_43;

                if (19 == markerids[t])
                {

                    singMarkerCorner_19.push_back(markerCorners[t]);
                    cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_19, landpad_det_len, camMatrix, distCoeffs, rvec_n, tvec_n);
                    tvec = tvec_n[0];
                    rvec = rvec_n[0];
                }
                else if (43 == markerids[t])
                {

                    singMarkerCorner_43.push_back(markerCorners[t]);
                    cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_43, landpad_det_len * 0.1, camMatrix, distCoeffs, rvec_n, tvec_n);
                    cout << "size of rvec is:" << rvec_n.size() << endl;
                    // getchar();
                    tvec = tvec_n[0];
                    rvec = rvec_n[0];
                }

                Mat R_cv;
                float x, y, z;
                // double roll, pitch, yaw;

                cv::Rodrigues(rvec, R_cv);   //calculate your markerboard pose----rotation matrix
                camera_R = R_cv.t();         //calculate your camera pose---- rotation matrix
                camera_T = -camera_R * tvec; //calculate your camera translation------translation vector

                Mat kf_eulers(3, 1, CV_64F);
                kf_eulers = rot2euler(camera_R); //convert camera matrix to euler angle
                cout << "Euler angles:" << kf_eulers.t() * 180 / CV_PI << endl;
                roll = kf_eulers.at<double>(0) * 180 / CV_PI;  //roll
                pitch = kf_eulers.at<double>(1) * 180 / CV_PI; //pitch
                yaw = kf_eulers.at<double>(2) * 180 / CV_PI;   //yaw
                x = camera_T.at<double>(0, 0) * 100;
                y = camera_T.at<double>(1, 0) * 100;
                z = camera_T.at<double>(2, 0) * 100;
                printf("Roll:%f, Pitch:%f, Yaw:%f\n", roll, pitch, yaw);
                printf("X:%f, Y:%f, Z:%f\n", x, y, z);
                if (got < 100)
                {
                    got++;
                }
                fillMeasurements(measurements, camera_T, camera_R);
                cv::aruco::drawDetectedMarkers(image, markerCorners, markerids);
                aruco::drawAxis(image, camMatrix, distCoeffs, rvec, tvec, landpad_det_len * 0.5);
            }
        }
        else
        {
            if (got > 1)
            {
                got--;
            }
        }

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
        double x_kf = translation_estimated.at<double>(0, 0);
        double y_kf = translation_estimated.at<double>(1, 0);
        double z_kf = translation_estimated.at<double>(2, 0);
  	
        image.copyTo(imageCopy);

        stringstream s1, s2, s3, s4, s5, s6;
        s1 << "Drone Position: x=" << int(camera_T.at<double>(0, 0) * 100) << " y=" << int(camera_T.at<double>(1, 0) * 100) << " z=" << int(camera_T.at<double>(2, 0) * 100);
        s2 << "Drone Attitude: yaw=" << int(yaw) << " pitch=" << int(pitch) << " roll=" << int(roll);
        s3 << "After Kalman filter";
        s4 << "Drone Position: x=" << int(x_kf * 100) << " y=" << int(y_kf * 100) << " z=" << int(z_kf * 100);
        s5 << "Drone Attitude: yaw=" << int(yaw_kf) << " pitch=" << int(pitch_kf) << " roll=" << int(roll_kf);
        s6 << "Confidence: conf=" << got;

        String position = s1.str();
        String attitude = s2.str();
        String info = s3.str();
        String kf_position = s4.str();
        String kf_attitude = s5.str();
        String confidence_s = s6.str();
        cout<<position<<endl;
	    cout<<attitude<<endl;

        int font_face = cv::FONT_HERSHEY_COMPLEX;
        int baseline;
        double font_scale = 0.5;
        int thinkness = 1.8;
        cv::Size text_size = cv::getTextSize(position, font_face, font_scale, thinkness, &baseline);
        cv::Point orgin_position, second_position, third_position, fourth_position, fifth_position, six_position;

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
        six_position.x = imageCopy.cols / 20;
        six_position.y = imageCopy.rows / 15 + 10 * text_size.height;

        cv::putText(imageCopy, position, orgin_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, attitude, second_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, info, third_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, kf_position, fourth_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, kf_attitude, fifth_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, confidence_s, six_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        set_vision_position_estimate(float(x_kf),float(y_kf),float(z_kf),float(roll_kf),float(pitch_kf),float(yaw_kf),got);
        //imshow("out", imageCopy);
        time_t now = time(0);
        struct tm  tstruct;
        char  buf[80];
        tstruct = *localtime(&now);
        strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);

        
        // WriteLog(log_file,float(x_kf),float(y_kf),float(z_kf),float(roll_kf),float(pitch_kf),float(yaw_kf),got);
        
        fout<<string(buf)<<" "<< float(x_kf)<<" "<< float(y_kf)<<" "<< float(z_kf)<<" "<< float(roll_kf)<<" "<< float(pitch_kf)<<" "<< float(yaw_kf)<<" "<< got<<endl;  
 

        
        
        if (saveVideo)
        {
            writer << imageCopy;
            cout<<"The frame has been saved"<<endl;
        }
            

        char key = (char)waitKey(waitTime);
        keyboard keyb;
        if(keyb.kbhit())
        {
            char key_q=keyb.getch();
            // break;
            if(key_q=='q')
                {break;}
        }
        
        if (key == 27)
            break;
    }
    inputVideo.release();
    fout<<flush;  
    fout.close(); 
    if (saveVideo)
   {    
       writer.release();
        cout<<"The video has been saved!"<<endl;
   }
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
