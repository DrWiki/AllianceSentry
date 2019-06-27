//
// Created by nvidia on 19-3-1.
//

#ifndef ALLIANCE2019_SENTRY_KALMANPREDICT_H
#define ALLIANCE2019_SENTRY_KALMANPREDICT_H

#include <opencv2/opencv.hpp>

class KalmanPredict_ {
    KalmanPredict_();
    cv::KalmanFilter *KF;
    void predict(cv::Point3f);
    void predict(cv::Point2f);
};


#endif //ALLIANCE2019_SENTRY_KALMANPREDICT_H
