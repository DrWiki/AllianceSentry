//
// Created by nvidia on 19-1-17.
//

#ifndef ALLIANCE2019_SENTRY_ARMOR_H
#define ALLIANCE2019_SENTRY_ARMOR_H

#include <opencv2/opencv.hpp>
#include "LedBar.h"
#include "AllianceTool_SZW.h"
/// /// /// /// /// ///
///
///
///
///  0  ///////////////////  1
///     //               //
///     //               //
///     //               //
///     //               //
///  3  ///////////////////  2
///
/// How to confirm the Point0:
/// :it has the minimum x+y
/// :if two points has the same value of x+y, the Point having lower x rank ahead *
///
///
/// *
class Armor {
public:
    cv::Point2f CornerPoints[4];
    cv::Point2f ArmorCenter;
    cv::RotatedRect ArmorRotatedRect;
    float height = 0;
    float width;
    float Angle = 0;
    float possibility = 0.0f;

    sint armorType = RM_NORMAL_ARMOR;   //describe which kind of armor this is.
                                         // 0: DEFAULT : normal armor
                                         // 1: Larger armor
                                         // others: ...*

    Armor(LedBar ,LedBar,float);
    Armor(cv::Point2f points[],float);
    void DrawArmorBGRLT(cv::Mat &);
    void DrawArmorBGR(cv::Mat & );
    void WriteLabelBGR(cv::Mat &);
    void WriteLabelBGRLT(cv::Mat &);

};


#endif //ALLIANCE2019_SENTRY_ARMOR_H
