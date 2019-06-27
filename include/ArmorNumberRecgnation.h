//
// Created by nvidia on 19-2-22.
//

#ifndef ALLIANCE2019_SENTRY_ARMORNUMBERRECGNATION_H
#define ALLIANCE2019_SENTRY_ARMORNUMBERRECGNATION_H

#include <iostream>
#include <opencv2/opencv.hpp>

class ArmorNumberRecgnation {
public:
    ArmorNumberRecgnation();

    struct ContenAndLabel{
        std::vector<cv::Mat> Templates;
        std::vector<int> IntLabel;
        std::vector<std::string> stringLabel;
    } Template;


    int RecgnizeByImageSub(cv::Mat src);
    void RecgniseBySVM();
    void RecgnizeByKNN();
    void RecgnizeByCNN();


private:


};


#endif //ALLIANCE2019_SENTRY_ARMORNUMBERRECGNATION_H
