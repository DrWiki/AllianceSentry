//
// Created by nvidia on 19-5-8.
//

#ifndef ALLIANCE2019_SENTRY_KALMANFILTERALL_H
#define ALLIANCE2019_SENTRY_KALMANFILTERALL_H

#include <opencv2/opencv.hpp>
#include <iostream>
class KalmanFilterAll {
public:
    KalmanFilterAll(){}
    KalmanFilterAll(int flag = 0){
        Kf = new cv::KalmanFilter(6,6,0);
        switch(flag){
            case 0:

                Kf->measurementMatrix = cv::Mat::eye(6,6,CV_32FC1);
                Kf->transitionMatrix = cv::Mat::eye(6,6,CV_32FC1);
                Kf->transitionMatrix.at<float>(0,2) = 0.01f;
                Kf->transitionMatrix.at<float>(1,3) = 0.01f;
                Kf->transitionMatrix.at<float>(2,4) = 0.01f;
                Kf->transitionMatrix.at<float>(3,5) = 0.01f;

                Kf->transitionMatrix.at<float>(0,4) = 0.005f;
                Kf->transitionMatrix.at<float>(1,5) = 0.005f;


                Kf->processNoiseCov = cv::Mat::eye(6,6,CV_32FC1)*25.0f;

                Kf->measurementNoiseCov = cv::Mat::zeros(6,6,CV_32FC1);
                Kf->measurementNoiseCov.at<float>(0,0) = 2000.0f;
                Kf->measurementNoiseCov.at<float>(1,1) = 2000.0f;
                Kf->measurementNoiseCov.at<float>(2,2) = 500.0f;
                Kf->measurementNoiseCov.at<float>(3,3) = 500.0f;
                Kf->measurementNoiseCov.at<float>(4,4) = 100.0f;
                Kf->measurementNoiseCov.at<float>(5,5) = 100.0f;
                Kf->errorCovPost = cv::Mat::eye(6,6,CV_32FC1)*2.0f;
            break;
        }
    }
            cv::KalmanFilter *Kf;
            cv::Mat predict(cv::Mat mes);
        };


        #endif //ALLIANCE2019_SENTRY_KALMANFILTERALL_H
