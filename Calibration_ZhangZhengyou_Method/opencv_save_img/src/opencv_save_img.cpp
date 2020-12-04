#include <opencv2/opencv.hpp>
#include "kbhit.h"
#include <unistd.h>
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


int main(int argc,char *argv[]){
    
    int count=0;
    int width=800;
    int height=600;
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
        char key_board=waitKey(0);
        //cv::imshow("current frame",frame);
        cout<<"READY TO CAPTURE "<<endl;
	keyboard keyb;
        if(keyb.kbhit())
        {
            char key_q=keyb.getch();
            // break;
            if(key_q=='q')
                {break;}
	    if(key_q=='s')
	    {
	    sprintf(image_name,"./imgs/%04d%s",count,".png");
            printf("%s has been saved",image_name);
            cv::imwrite(image_name,frame);
            //origin_position.x=frame.cols/20;
            //origin_position.y=frame.rows/15;
            // cv::putText(frame,"Image has been saved!",origin_position,cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0),1,8,0);
            count++;

	    }
        }
        //if(key_board=='s'){
          //  sprintf(image_name,"./imgs/%04d%s",count,".png");
            //printf("%s has been saved",image_name);
	   // cv::imwrite(image_name,frame);
            //origin_position.x=frame.cols/20;
            //origin_position.y=frame.rows/15;
            // cv::putText(frame,"Image has been saved!",origin_position,cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0),1,8,0);
           // count++;
       // }
        
        //if(key_board=='q'){
         //    CV_Assert("Cam open failed");
          //   return 0;
       // }
    }

}
