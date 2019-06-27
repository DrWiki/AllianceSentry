//
// Created by hero on 12/14/18.
//

#ifndef ARMOR_DETECTOR_ARMORDETECTOR_H
#define ARMOR_DETECTOR_ARMORDETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdbool.h>
#define BLUE 0
#define RED 1

static double threshold1=72;
static double threshold2=200;
using namespace std;

class ArmorDetector
{
public:
    void setEnemyColor(bool color);
    bool findArmor(cv::Mat &src);
    bool getArmorNumber(vector<vector<cv::Point2f>>);
    void writeArgs();
    void readArgs();
    vector<cv::Point2f*> final4Points;
    vector<cv::Mat> possibleArmor;
private:
    bool ENEMY_COLOR;
    int threshold1Now=72;
    int threshold2Now=100;
    int minContoursLength=25;
    int thresholdMax=255;
    float minHeightAndWidthRatio=2;
    float maxHeightAndWidthRatio=10;
    float minDipAngle=45;
    float maxDipAngle=135;
    float heightDiffRatio=0.2;
    float widthDiffRatio=0.35;
    float ellipseCenterDisRatioX=7;
    float ellipseCenterDisRatioY=0.2;
    float pixelSumDiffRatio=0.3;
    float lightbarAngleError=8;
    int lightbarImagenum=0;
    int notlightbarImagenum=0;
    float armorsHeight[20];
    float armorsWidth[20];
    cv::Mat inputImg;
    vector<vector<cv::Point>> contours;
    vector<cv::RotatedRect> rotatedEllipses;
    vector<vector<cv::Point2f>> allArmorPoints;
    vector<vector<cv::RotatedRect>> finalArmors;

    vector<cv::RotatedRect> minRectOfContours;
    vector<vector<cv::RotatedRect>> armors;
    vector<float> allRotatedRectHeight;
    vector<float> allRotatedRectWidth;

//    void createTrackbarHere();
//    void argsChangeByTrackbar();
    void cleanAllVectors();
    void findLightbar();
    int fromLightbarFindArmor();
//    int findTargetInEnemyColor();
    bool get4PointsFromTargets();
    void saveLightbar(cv::Point2f *pts,cv::Size dSize,char *filePath);
    bool perspectiveTransform();
};


#endif //ARMOR_DETECTOR_ARMORDETECTOR_H
