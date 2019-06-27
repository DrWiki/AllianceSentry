//
// Created by nvidia on 3/18/19.
//

#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(int s) {
    if(s == 1){
        int c = 2;
        int m = 2;
        Kf = new cv::KalmanFilter(c,m,0);
        Kf->measurementMatrix = cv::Mat::eye(c,m,CV_32FC1);
        Kf->transitionMatrix = cv::Mat::eye(c,m,CV_32FC1);
        Kf->transitionMatrix.at<float>(0,1) = 0.01f;
        Kf->processNoiseCov = cv::Mat::eye(c,m,CV_32FC1)*25.0f;
        Kf->measurementNoiseCov = cv::Mat::zeros(c,m,CV_32FC1);
        Kf->measurementNoiseCov.at<float>(0,0) = 2100.0f;
        Kf->measurementNoiseCov.at<float>(1,1) = 500.0f;
        Kf->errorCovPost = cv::Mat::eye(c,m,CV_32FC1)*2.0f;
    }
    if(s == 2){
        int c = 2;
        int m = 2;
        Kf = new cv::KalmanFilter(c,m,0);
        Kf->measurementMatrix = cv::Mat::eye(c,m,CV_32FC1);
        Kf->transitionMatrix = cv::Mat::eye(c,m,CV_32FC1);
        Kf->transitionMatrix.at<float>(0,1) = 0.001f;
        Kf->processNoiseCov = cv::Mat::eye(c,m,CV_32FC1)*25.0f;
        Kf->measurementNoiseCov = cv::Mat::zeros(c,m,CV_32FC1);
        Kf->measurementNoiseCov.at<float>(0,0) = 2100.0f;
        Kf->measurementNoiseCov.at<float>(1,1) = 500.0f;
        Kf->errorCovPost = cv::Mat::eye(c,m,CV_32FC1)*2.0f;
    }
}
KalmanFilter::KalmanFilter(bool s){
    Kf = new cv::KalmanFilter(2,2,0);
    Kf->measurementMatrix = cv::Mat::eye(2,2,CV_32FC1);
    Kf->transitionMatrix = cv::Mat::eye(2,2,CV_32FC1);
    Kf->processNoiseCov = cv::Mat::eye(2,2,CV_32FC1)*500.0f;
    Kf->measurementNoiseCov = cv::Mat::zeros(2,2,CV_32FC1)*10000.0f;
}

cv::Point2f KalmanFilter::predict(cv::Point2f mes) {
    cv::Mat temp = cv::Mat::zeros(2,1,CV_32FC1);
    temp.at<float>(0,0) = mes.x;
    temp.at<float>(1,0) = mes.y;
    Kf->correct(temp);
    cv::Mat pre  = Kf->predict();
    p1 = p2;
    p2 = cv::Point2f(pre.at<float>(0,0),pre.at<float>(1,0));
    return p2;
}

cv::Mat KalmanFilter::predict(cv::Mat mes) {
    Kf->correct(mes);
    cv::Mat pre  = Kf->predict();
    return pre;
}
