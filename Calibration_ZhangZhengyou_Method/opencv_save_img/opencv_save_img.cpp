#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
cv::Mat frame;

/****************************************************************************************************
 * 
 * This project is used for capture image to calibrate the camera
 * width & height: the parameters of the captured image
 * 
 * 
*****************************************************************************************************/


int main(int argc,char *argv[]){
    
    int count=0;
    int width=1280;
    int height=720;
    cv::Point origin_position;
    VideoCapture cap;
    cap.open(0);
    if(!cap.isOpened()){
        CV_Assert("Cam open failed");
        cout<<"dafgaf"<<endl;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,height);
    cout<<"dafgaf"<<endl;
    while(cap.grab()){
        cv::Mat frame;
        cap.retrieve(frame);
        char image_name[13];
        char key_board=waitKey(10);
        cv::imshow("current frame",frame);
        if(key_board=='s'){
            sprintf(image_name,"./imgs/%04d%s",count,".png");
            cv::imwrite(image_name,frame);
            origin_position.x=frame.cols/20;
            origin_position.y=frame.rows/15;
            // cv::putText(frame,"Image has been saved!",origin_position,cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0),1,8,0);
            count++;
        }
        
        if(key_board=='q'){
             CV_Assert("Cam open failed");
             return 0;
        }
    }

}