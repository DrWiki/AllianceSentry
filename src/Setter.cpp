//
// Created by nvidia on 19-1-16.
//

#include <Setter.h>

#include "Setter.h"
Setter::Setter() {
    cv::FileStorage SettingFile;
    assert(SettingFile.open("../others/parameters_lists/para.yaml",cv::FileStorage::READ));

    SettingFile["key"]>>key;
    SettingFile["Iam"]>>Iam;
    SettingFile["Youare"]>>Youare;

    // Threshold
    SettingFile["Threshold4Gray2Binary"]>>Threshold4Gray2Binary;
    SettingFile["Threshold4Gray2BinaryPowerTrigger"]>>Threshold4Gray2BinaryPowerTrigger;
    //cv::imshow()
    SettingFile["isShowFinallyImage"]>>isShowFinallyImage;
    SettingFile["isShowOriginImage"]>>isShowOriginImage;
    SettingFile["isShowOtherImage"]>>isShowOtherImage;

    //Serial
    SettingFile["isSerialDebug"]>>isSerialDebug;
    SettingFile["isSerialOnline"]>>isSerialOnline;
    SettingFile["isShowSerialWriteDataMainThread"]>>isShowSerialWriteDataMainThread;
    SettingFile["isShowSerialWriteDataIndependentThread"]>>isShowSerialWriteDataIndependentThread;
    SettingFile["isShowSerialReadDataMainThread"]>>isShowSerialReadDataMainThread;
    SettingFile["isShowSerialReadDataIndependentThread"]>>isShowSerialReadDataIndependentThread;
    SettingFile["SerialPath"]>>SerialPath;
    SettingFile["isSerialDebugIndependently"]>>isSerialDebugIndependently;

    //Shoot1
    SettingFile["isEnableShooting"]>>isEnableShooting;
    SettingFile["isSaveOriginal"]>>isSaveOriginal;
    SettingFile["isSaveResultVideo"]>>isSaveResultVideo;

    //CameraORvideo
    SettingFile["VideoPath"]>>VideoPath;
    SettingFile["CameraParameterPath"]>>CameraParameterPath;
    SettingFile["frameSource"]>>frameSource;
    SettingFile["VideoSavePath"]>>VideoSavePath;                // unused

    //variable
    SettingFile["cvWaitKeyTime"]>>cvWaitKeyTime;
    SettingFile["shootingImageSize"]>>shootingImageSize;
    SettingFile["resolution"]>>resolution;

    //Armor
    SettingFile["NormalArmorWidth"]>>NormalArmorWidth;
    SettingFile["NormalArmorHeight"]>>NormalArmorHeight;
    SettingFile["LargerArmorWidth"]>>LargerArmorWidth;
    SettingFile["LargerArmorHeight"]>>LargerArmorHeight;

    //Calibration
    SettingFile["Calibration"]>>Calibration;
    SettingFile["Rectangle"]>>Rectangle;
    SettingFile["CornersMatrix"]>>CornersMatrix;
    CameraMatrix = cv::Mat(3,3,CV_32FC1);
    CameraMatrix = cv::Mat(1,14,CV_32FC1);
    SettingFile["CameraMatrix"]>>CameraMatrix;
    SettingFile["DistCoeffs"]>>DistCoeffs;

    //
    SettingFile["PowerTriggerTemplateSzie"]>>PowerTriggerTemplateSzie;

    SettingFile["theMaximumdifference42Leds"]>>theMaximumdifference42Leds;
    SettingFile["theMaximumDistance42Leds"]>>theMaximumDistance42Leds;
    SettingFile["theMinimumDistance42Leds"]>>theMinimumDistance42Leds;

    //
    SettingFile["theMinmumSize4aContours"]>>theMinmumSize4aContours;
    SettingFile["supLengthOverWidth"]>>supLengthOverWidth;
    SettingFile["infLengthOverWidth"]>>infLengthOverWidth;
    SettingFile["blueColorPercentage"]>>blueColorPercentage;
    SettingFile["redColorPercentage"]>>redColorPercentage;
    SettingFile["MinmumPossibility"]>>MinmumPossibility;
    SettingFile["isRecgArmorNum"]>>isRecgArmorNum;

    //
    SettingFile["LinearGain"]>>LinearGain;

    //
    SettingFile["Dyaw"]>>Dyaw;
}

void Setter::ShowParameters() {
    //TODO: Cout All the Parameters
}
