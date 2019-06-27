//
// Created by nvidia on 19-4-14.
//

#ifndef ALLIANCE2019_SENTRY_ANALYZESTATISTICS_H
#define ALLIANCE2019_SENTRY_ANALYZESTATISTICS_H

#include <iostream>
#include <opencv2/opencv.hpp>

class AnalyzeStatistics {
public:
    AnalyzeStatistics(int s,int h){
        size = s;
        boardf = cv::Mat ::zeros(s,h,CV_8UC3);
        boardi = cv::Mat ::zeros(s,h,CV_8UC3);
    }
    int size;
    std::vector<float> Dataf;
    std::vector<int> Datai;
    cv::Mat boardf;
    cv::Mat boardi;

    void Plotf(cv::Mat &M,const cv::Scalar c);
    void Ploti(cv::Mat &M,const cv::Scalar c);
    void catf(float ele);
    void cati(int ele);

    void inf(float ele);
    void ini(int ele);
    void clear(cv::Mat & M);
};


#endif //ALLIANCE2019_SENTRY_ANALYZESTATISTICS_H
