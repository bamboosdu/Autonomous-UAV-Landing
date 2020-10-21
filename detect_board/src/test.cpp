Mat R_cv, T_cv, camera_R, camera_T;
        if (ids.size() > 0 || ids_center.size() > 0) //The big one or the small one is deteceted
        {

            if (ids_center.size() > 0) // && got > VISION_THRES
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

        } //*TODO THE POSITION IS CONFUSING

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
        // cout << "KF:" << ids.size() << endl;
        /**********************************************************************************************************
                                 
                                 Draw the position and the attitude of the camera
        
        **********************************************************************************************************/
        image.copyTo(imageCopy);