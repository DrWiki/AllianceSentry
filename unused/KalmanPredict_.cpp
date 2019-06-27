//
// Created by nvidia on 19-3-1.
//

#include "KalmanPredict_.h"

KalmanPredict_::KalmanPredict_() {
    KF = new cv::KalmanFilter(2, 1, 0,CV_32F);
    KF->transitionMatrix = (cv::Mat_<float>(2, 2) << 1, 1, 0, 1);                 //转移矩阵A[1,1;0,1]
    //将下面几个矩阵设置为对角阵
    cv::setIdentity(KF->measurementMatrix);                                 //测量矩阵H
    cv::setIdentity(KF->processNoiseCov, cv::Scalar::all(1e-5));            //系统噪声方差矩阵Q
    cv::setIdentity(KF->measurementNoiseCov, cv::Scalar::all(1e-1));        //测量噪声方差矩阵R
    cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));                  //后验错误估计协方差矩阵P

}

void KalmanPredict_::predict(cv::Point3f) {
    /*
    measurement =measurement +KF.measurementMatrix*state;  //z = z + H*x;
    KF.correct(measurement);
    state = KF.transitionMatrix*state + processNoise;    //真实值运动
    cv::Mat prediction = KF.predict();                          //计算预测值，返回x'
     */
}

void KalmanPredict_::predict(cv::Point2f) {

}
