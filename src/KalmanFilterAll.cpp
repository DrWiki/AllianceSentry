//
// Created by nvidia on 19-5-8.
//

#include "KalmanFilterAll.h"

cv::Mat KalmanFilterAll::predict(cv::Mat mes) {
    std::cout<<mes<<std::endl;
    Kf->correct(mes);
    cv::Mat pre  = Kf->predict();
    std::cout<<pre<<std::endl;
    return pre;
}
