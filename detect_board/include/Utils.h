#ifndef UTILS_H_
#define UTILS_H_
#include <iostream>

#include <opencv2/features2d.hpp>


// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix);

// Converts a given Euler angles to Rotation Matrix
cv::Mat euler2rot(const cv::Mat & euler);






#endif /* UTILS_H_ */