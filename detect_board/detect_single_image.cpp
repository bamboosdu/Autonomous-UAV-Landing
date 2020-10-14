#include <stdlib.h>
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
/**
 */
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
/**
 */

//Kalman filter funtions headers
void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
void predictKalmanFilter(KalmanFilter &KF, Mat &translation_predicted, Mat &rotation_predicted);
void updateKalmanFilter(KalmanFilter &KF, Mat &measurements,
                        Mat &translation_estimated, Mat &rotation_estimated);
void fillMeasurements(Mat &measurements,
                      const Mat &translation_measured, const Mat &rotation_measured);
void rot_euler(const Mat &rotation_matrix, Mat &euler);
void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  while(std::string::npos != pos2)
  {
    v.push_back(s.substr(pos1, pos2-pos1));
 
    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if(pos1 != s.length())
    v.push_back(s.substr(pos1));
}

int main(int argc, char *argv[])
{

    String fname = "./intrisic.xml";
    /***************************************************************************
     * 
     *                                Parameters
     * 
    ***************************************************************************/
    //aruco board parameters
    int markersX = 2;
    int markersY = 2;
    float markerLength = 0.09;
    float markerSeparation = 0.09;
    int dictionaryId = 2;
    bool showRejected = false;
    bool refindStrategy = true;
    int camId = 0;
    //aruco marker parameters
    int dictionaryId_center = 7;
    float markerLength_center = 0.03;

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


    string pattern_jpg = "/home/qiang/Pictures/Imgbar/*.jpg";
	std::vector<cv::String> image_files;
	cv::glob(pattern_jpg, image_files);

    for (unsigned int frame = 0; frame < image_files.size(); ++frame)
	{
		Mat image = cv::imread(image_files[frame]);
        Mat imageCopy;
        
        Vec3d cam_rvec, cam_tvec;
        Vec3d eulerAngles;
        double yaw, roll, pitch;
        double tick = (double)getTickCount();
        int lost, got;
        
        //aruco board
        vector<int> ids;
        vector<vector<Point2f>> corners, rejected;
        Vec3d rvec, tvec;
        //marker
        vector<int> ids_center;
        vector<vector<Point2f>> corners_center, rejected_center;
        vector<Vec3d> rvecs_center, tvecs_center;

        // detect aruco board
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        //detect aruco marker
        aruco::detectMarkers(image, dictionary_center, corners_center, ids_center,
                             detectorParams, rejected_center);

        // refind strategy to detect more markers
        if (refindStrategy)
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,
                                         distCoeffs);

        // estimate board pose

        int markersOfBoardDetected = 0;
        cout<<"IDS:"<<ids_center.size()<<endl;

        if (ids.size() > 0 || ids_center.size() > 0)

            if(ids_center.size()>0){
                got++;
                lost=0;
            }else{
                got=0;
                lost++;
            }

            //if the center marker is detected
            if (ids_center.size() > 0 && got>VISION_THRES)
            {
                aruco::estimatePoseSingleMarkers(corners_center, markerLength_center, camMatrix, distCoeffs, rvecs_center,
                                                 tvecs_center);
                rvec = rvecs_center[0];
                tvec = tvecs_center[0];
                markersOfBoardDetected=1;
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
        Mat R_cv, T_cv, camera_R, camera_T;
        cv::Rodrigues(rvec, R_cv);   //calculate your markerboard pose----rotation matrix
        camera_R = R_cv.t();         //calculate your camera pose---- rotation matrix
        camera_T = -camera_R * tvec; //calculate your camera translation------translation vector

        Mat kf_eulers(3, 1, CV_64F);
        kf_eulers = rot2euler(camera_R); //convert camera matrix to euler angle
        cout << "Euler angles:" << kf_eulers.t() * 180 / CV_PI << endl;
        roll = kf_eulers.at<double>(0) * 180 / CV_PI;  //roll
        pitch = kf_eulers.at<double>(1) * 180 / CV_PI; //pitch
        yaw = kf_eulers.at<double>(2) * 180 / CV_PI;   //yaw

        Eigen::MatrixXd camera_r_eigen(3, 3);
        cv2eigen(camera_R, camera_r_eigen); // cv::Mat-> Eigen::MatrixXd

        cout << "matrix:\n"
             << camera_R << endl;
        cout << "matrixxd:\n"
             << camera_r_eigen << endl;

        

        /**********************************************************************************************************
                                 
                                 Draw the position and the attitude of the camera
        
        **********************************************************************************************************/
        image.copyTo(imageCopy);
        if (ids.size() > 0)
        {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        stringstream s1, s2, s3, s4, s5;
        s1 << "Drone Position: x=" << int(camera_T.at<double>(0, 0) * 100) << " y=" << int(camera_T.at<double>(1, 0) * 100) << " z=" << int(camera_T.at<double>(2, 0) * 100);
        s2 << "Drone Attitude: yaw=" << int(yaw) << " pitch=" << int(pitch) << " roll=" << int(roll);
        // s3 <<image_files[frame];
        String position = s1.str();
        String attitude = s2.str();

        std::string s=image_files[frame];
        string pattern="/";
        vector <string>result;
        SplitString(s,result,pattern);
        cout<<result[result.size()-1]<<endl;


        
        String Drone_data=result[result.size()-1];
        
         
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

        cv::putText(imageCopy, position, orgin_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, attitude, second_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        cv::putText(imageCopy, Drone_data, third_position, font_face, font_scale, cv::Scalar(0, 255, 255), thinkness, 8, 0);
        
        /********************************************************************************************/

    
        if (markersOfBoardDetected > 0)
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
        
        imshow("out", imageCopy);
        // writer<<imageCopy;
        char filename[400];
        
        sprintf(filename,"/home/qiang/zq/git-space/drone_about/aruco_board/detect_board/build/saveImg/%d.jpg", frame);
        cout<<filename<<endl;
        cv::imwrite(filename,imageCopy);
        
        // getChar();
        char key = (char)waitKey();
       
        if (key == 27)
            break;
     
	}

   
    return 0;
}

