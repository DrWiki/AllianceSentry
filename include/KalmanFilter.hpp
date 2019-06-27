//
// Created by nvidia on 3/18/19.
//

#ifndef ALLIANCE2019_SENTRY_KALMANFILTER_HPP
#define ALLIANCE2019_SENTRY_KALMANFILTER_HPP

#include <opencv2/opencv.hpp>

class KalmanFilter {
public:
    KalmanFilter(int);
    KalmanFilter(bool s);
    cv::Point2f p1;
    cv::Point2f p2;
    cv::KalmanFilter *Kf;

    cv::Point2f predict(cv::Point2f mes);
    cv::Mat predict(cv::Mat mes);
};


#endif //ALLIANCE2019_SENTRY_KALMANFILTER_HPP
