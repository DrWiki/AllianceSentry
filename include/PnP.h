//
// Created by nvidia on 19-1-19.
//

#ifndef ALLIANCE2019_SENTRY_PNP_H
#define ALLIANCE2019_SENTRY_PNP_H

#include "AllianceTool_SZW.h"
#include "opencv2/opencv.hpp"
#include "Setter.h"
#include "Armor.h"

class PnP {
public:
    float NormalArmorWidth;
    float NormalArmorHeight;
    float LargerArmorWidth;
    float LargerArmorHeight;
    cv::Point3f GetXYZ(Armor);                 //Calculate the distance between shooter and armor

    cv::Point3f *ArmorCenter;
    cv::Point3f WorldArmorCenterPoint;
    cv::Mat CameraMatrix;              // Parameter List Matrix of Camera Objective
    cv::Mat DistCoeffs;                // Parameter List Matrix of tr(退化)

    std::vector<cv::Point3f> NormalArmorPoints;
    std::vector<cv::Point3f> LargerArmorPoints;

    PnP(Setter setter);

private:


};


#endif //ALLIANCE2019_SENTRY_PNP_H
