#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

    FileStorage fs("/home/zq/zq/git-space/drone_about/aruco_board/write_camera/intrisic.xml", FileStorage::WRITE);
   // if(!fs.isOpened())
     //   return false;
   // fs["camera_matrix"] >> camMatrix;
   //fs["distortion_coefficients"] >> distCoeffs;
    Mat cameraMatrix = (Mat_<double>(3,3) << 819.54289171, 0, 314.53157417, 0, 819.72768105, 259.70322391, 0, 0, 1); 
    Mat distCoeffs = (Mat_<double>(5,1) << 0.22, 0.01, -0.001, 0, 0);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs.release();

}
