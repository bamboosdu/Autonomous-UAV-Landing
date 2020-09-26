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
    Mat cameraMatrix = (Mat_<double>(3,3) << 732.77028682, 0, 614.87813111, 0, 733.39110606, 349.18611676, 0, 0, 1); 
    Mat distCoeffs = (Mat_<double>(5,1) << 0.1119643, -0.22287516, -0.00228121, -0.00046944, 0.14355533);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs.release();

}
