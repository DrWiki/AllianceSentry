//
// Created by nvidia on 19-2-18.
//

#ifndef ALLIANCE2019_SENTRY_ANALYZE_H
#define ALLIANCE2019_SENTRY_ANALYZE_H

#include <iostream>
#include <opencv2/opencv.hpp>

class Analyze {
public:
    Analyze();
    ~Analyze();
    cv::Mat OwnBoard;
    //Draw all the Contours After Contours Detection

    //Analyze the moments After the Contours Detection
    void AnalyzeTheMoment(std::vector<std::vector<cv::Point>> contours,cv::Size size = cv::Size(640,480));
    void AnalyzeArmor_Double(std::vector<cv::RotatedRect> Rect, cv::Mat Board);
    void AnalyzeArmor_Single(std::vector<std::vector<cv::Point>> contours,cv::Mat Board);

    //
};


#endif //ALLIANCE2019_SENTRY_ANALYZE_H
