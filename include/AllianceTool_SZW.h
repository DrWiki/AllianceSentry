/// Created by SongZiwu from 20190101 to XXXXXXXX
/// Some enums
/// some variables and constant
/// Some process cases
/// if you want to change some parameters find them here below *

#ifndef ALLIANCE2019_SENTRY_ALLIANCETOOL_SZW_H
#define ALLIANCE2019_SENTRY_ALLIANCETOOL_SZW_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include "MicrosecondChronograph.h"
#define RUN                                  true // This is used in the super loop
#define ALLIANCE_PRINT(x)                    std::cout << "9160007100_Debug:" << x << std::endl;
#define ALLIANCE_PRINT2(s1,s2)               std::cout << "9160007100_Debug:"<< s1 << " : " << s2 << std::endl;
#define bug(x)                               std::cout << "9160007100_Debug:" << x << std::endl;
#define bug1(x)                              std::cout << "9160007100_Debug:"<< x << std::endl;
#define bug2(x,y)                            std::cout << "9160007100_Debug:" << x << "   " << y << std::endl;
#define bug3(x,y,z)                          std::cout << "9160007100_Debug:" << x << "   " << y << "   "<< z << std::endl;
typedef short sint;                          // short int is 4B, is half of a int.

#define RM_NORMAL_ARMOR (sint)0                    //
#define RM_LARGER_ARMOR (sint)1

struct InfoPack{                             //
    char currentAngle;
};
inline float norm(cv::Point2f A,cv::Point2f B){
    return sqrtf(((A.x-B.x)*(A.x-B.x))+((A.y-B.y)*(A.y-B.y)));
}
#endif //ALLIANCE2019_SENTRY_ALLIANCETOOL_SZW_H
