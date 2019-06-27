//
// Created by nvidia on 19-1-16.
//

#ifndef ALLIANCE2019_SENTRY_SETTER_H
#define ALLIANCE2019_SENTRY_SETTER_H

#include "AllianceTool_SZW.h"
class Setter {
public:
    Setter();
    void ShowParameters();

    sint key = 0;
    sint Iam = 2;
    sint Youare = 0;

    bool isShowOriginImage = true;
    bool isShowOtherImage = true;
    bool isShowFinallyImage = true;                    // A BGR image whit Drawn information.


    std::string SerialPath;
    bool isSerialOnline = false;
    bool isSerialDebugIndependently = true;
    bool isShowSerialWriteDataMainThread = false;
    bool isShowSerialWriteDataIndependentThread = false;
    bool isShowSerialReadDataMainThread = false;
    bool isShowSerialReadDataIndependentThread = false;
    //TODO:
    bool isSerialDebug = false;

    // Shooting
    bool isEnableShooting = false;
    bool isSaveOriginal = false;
    bool isSaveResultVideo = false;

    //CameraORvideo
    sint frameSource = -1;
    /// /// ///
    /// short int: 65535
    /// int: 2^32
    /// float: 4KB
    /// double: 8KB
    /// bool: 1KB
    /// 1. Take using sint when Calculating into Consideration
    int cvWaitKeyTime = 30;

    //Threshold
    int Threshold4Gray2Binary = 150;
    int Threshold4Gray2BinaryPowerTrigger = 100;

    std::string VideoPath;
    std::string CameraParameterPath;
    std::string Calibration;
    std::string VideoSavePath;

    cv::Size Rectangle;
    cv::Size CornersMatrix;
    cv::Size shootingImageSize;
    cv::Size resolution;

    float NormalArmorWidth;
    float NormalArmorHeight;
    float LargerArmorWidth;
    float LargerArmorHeight;

    cv::Mat CameraMatrix;
    cv::Mat DistCoeffs;

    cv::Size PowerTriggerTemplateSzie;

    float theMaximumdifference42Leds;
    float theMaximumDistance42Leds;
    float theMinimumDistance42Leds;

    int theMinmumSize4aContours = 20;
    float supLengthOverWidth = 15;
    float infLengthOverWidth = 2;
    float blueColorPercentage = 0.5;
    float redColorPercentage = 0.3;
    float MinmumPossibility = 70.0f;
    bool isRecgArmorNum = true;

    // NumberRecognition
    int LinearGain;
    float Dyaw;
};


#endif //ALLIANCE2019_SENTRY_SETTER_H
