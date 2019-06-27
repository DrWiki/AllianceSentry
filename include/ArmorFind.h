//
// Created by nvidia on 19-4-5.
//

#ifndef ALLIANCE2019_SENTRY_ARMORFIND_H
#define ALLIANCE2019_SENTRY_ARMORFIND_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "LedBar.h"
#include "Armor.h"
#include "Setter.h"

#define R cv::Scalar(0,0,255)
#define G cv::Scalar(0,255,0)
#define B cv::Scalar(255,0,0)
#define Y cv::Scalar(0,255,255)
#define C cv::Scalar(255,255,0)
#define P cv::Scalar(255,0,255)
#define W cv::Scalar(255,255,255)
#define D cv::Scalar(0,0,0)

class ArmorFind {
public:
    ArmorFind(cv::Mat *m1,cv::Mat *m2);
    cv::Mat *DrawingBoard_1;                          // 2 Draw
    cv::Mat *DrawingBoard_2;
    std::vector<std::vector<cv::Point>> CorrectLED;         // CorrectLED
    std::vector<cv::RotatedRect> CorrectLEDRotateRectangle;
    std::vector<LedBar> LedGroup;                           // all the LEDs


    void FindFromContours(cv::Mat &Frame,
                                     std::vector<std::vector<cv::Point>> AllContours,
                                     std::vector<Armor> &Armors,
                                     Setter setter);
    void FindFromContoursOptimize(cv::Mat &Frame,
                                             std::vector<std::vector<cv::Point>> AllContours,
                                             std::vector<Armor> &Armors,
                                             Setter setter);

private:
    void FindLed(cv::Mat &Frame,
                 std::vector<std::vector<cv::Point>> AllContours,
                 Setter setter);

};


#endif //ALLIANCE2019_SENTRY_ARMORFIND_H
