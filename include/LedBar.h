//
// Created by nvidia on 19-1-11.
//

#ifndef ALLIANCE2019_SENTRY_LEDBAR_H
#define ALLIANCE2019_SENTRY_LEDBAR_H


#include <opencv2/opencv.hpp>
#include "AllianceTool_SZW.h"
class LedBar {
public:
    cv::Point2f p1;
    cv::Point2f p2;
    cv::Point2f dirv;
    cv::Point2f dirv_E;
    cv::Point2f center;
    cv::RotatedRect rotateR;
    cv::Point2f Coners[4];
    float length;

    explicit LedBar(cv::RotatedRect rotrect);
    static void AdjustPointOrder(LedBar l1,LedBar l2,cv::Point2f points[]);
    float CheckArmor(LedBar);
    float checkParallel(LedBar L2);
    float checkEqualSize(LedBar L2);
    float checkRectangle(LedBar L2);
    float checkProportion(LedBar L2, float);
    float checkArea(LedBar L2);
    //float checkBadLine(LedBar L2);
    float DistanceWith(LedBar);
    void DrawLed(cv::Mat & src);
    static float cross(cv::Point2f A,cv::Point2f B);
    static float dot(cv::Point2f A,cv::Point2f B);
    static float dis(cv::Point2f A,cv::Point2f B);
};


#endif //ALLIANCE2019_SENTRY_LEDBAR_H
