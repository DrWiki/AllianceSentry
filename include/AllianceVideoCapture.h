//
// Created by nvidia on 19-1-17.
//

#ifndef ALLIANCE2019_SENTRY_ALLIANCEVIDEOCAPTURE_H
#define ALLIANCE2019_SENTRY_ALLIANCEVIDEOCAPTURE_H

#include <opencv2/opencv.hpp>
#include "MindVisionCamera.h"
#include "AllianceTool_SZW.h"

union Camera{         //Using Point is necessary;Point needn't Default Constructor
    MindVisionCamera *IndustrialCamera;
    cv::VideoCapture *CommonCamera;
};

class AllianceVideoCapture {
public:
    explicit AllianceVideoCapture(sint cameraType);
    explicit AllianceVideoCapture(std::string);

    ~AllianceVideoCapture();
    bool read(cv::Mat &);
    Camera camera;
    bool isIndustrialCamera;
};


#endif //ALLIANCE2019_SENTRY_ALLIANCEVIDEOCAPTURE_H
