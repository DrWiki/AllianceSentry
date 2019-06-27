//
// Created by nvidia on 19-1-16.
//

#ifndef ALLIANCE2019_SENTRY_ROBOMASTER2019_H
#define ALLIANCE2019_SENTRY_ROBOMASTER2019_H

#include "AllianceTool_SZW.h"
#include "AllianceVideoCapture.h"
#include "Setter.h"
#include "LedBar.h"
#include "Armor.h"
#include "SerialPort.h"
#include "MicrosecondChronograph.h"
#include "TimerBase.h"
#include "TimerBaseClock.h"
#include "Analyze.h"
#include "ArmorNumberRecgnation.h"
#include "PnP.h"
#include "KalmanFilter.hpp"
#include <thread>         // std::thread, std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <thread>
#include "ArmorFind.h"
#include "AnalyzeStatistics.h"
#include "KalmanFilterAll.h"
#define  max(x,y)       x>y? y:x
#define  min(x,y)       x<y? y:x

#define SHOW_IMAGE1     if(setter.isShowOriginImage){cv::imshow("Frame",Frame);} \
if(setter.isShowFinallyImage){DrawingBoard_1 = Frame.clone();}\
if(setter.isShowOtherImage){cv::imshow("Gray",Gray);}
#define SHOW_IMAGE2     if(setter.isShowOriginImage && !Armors.empty()){\
for(Armor armori : Armors){\
armori.DrawArmorBGR(DrawingBoard_1);\
armori.WriteLabelBGR(DrawingBoard_1);\
}\
}\
if(setter.isShowFinallyImage && !Armors.empty()){\
Armors[maxPossibility_index].DrawArmorBGRLT(DrawingBoard_1);\
Armors[maxPossibility_index].WriteLabelBGRLT(DrawingBoard_1);\
}

#define R cv::Scalar(0,0,255)
#define G cv::Scalar(0,255,0)
#define B cv::Scalar(255,0,0)
#define Y cv::Scalar(0,255,255)
#define C cv::Scalar(255,255,0)
#define P cv::Scalar(255,0,255)
#define W cv::Scalar(255,255,255)
#define D cv::Scalar(0,0,0)

#define DRAW_IMAGE_PREDICT     if(setter.isShowFinallyImage){\
cv::circle(DrawingBoard_1,Predict,5,G,-1);\
cv::line(DrawingBoard_1,cv::Point(0,Predict.y),cv::Point(640,Predict.y),G,1);\
cv::line(DrawingBoard_1,cv::Point(Predict.x,0),cv::Point(Predict.x,640),G,1);\
cv::circle(DrawingBoard_1,FinallyTraget[TargetSize-1],5,Y,-1);\
cv::line(DrawingBoard_1,cv::Point(0,FinallyTraget[TargetSize-1].y),cv::Point(640,FinallyTraget[TargetSize-1].y),Y,1);\
cv::line(DrawingBoard_1,cv::Point(FinallyTraget[TargetSize-1].x,0),cv::Point(FinallyTraget[TargetSize-1].x,640),Y,1);\
cv::line(DrawingBoard_1,cv::Point(0,240),cv::Point(640,240),R,1);\
cv::line(DrawingBoard_1,cv::Point(320,0),cv::Point(320,640),R,1);\
cv::putText(DrawingBoard_1,\
std::to_string(FinallyTraget[TargetSize-1].x)+"$"+\
std::to_string(FinallyTraget[TargetSize-1].y)+"$"+\
std::to_string(PitchYawDistance.z),\
cv::Point(25,10),1,1,Y,1);\
}

inline std::string getTimeVideo() {
    time_t TimeValue;
    time (&TimeValue);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "../others/video/%Y-%m-%d_%H_%M_%S.avi",localtime(&TimeValue));
    return tmp;
}
inline std::string getTimePicture() {
    time_t TimeValue;
    time (&TimeValue);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "../others/picture/%Y-%m-%d_%H_%M_%S.png",localtime(&TimeValue));
    return tmp;
}

class RoboMaster2019 {
public:
     explicit RoboMaster2019(Setter & temp) {
         setter = temp;
        if(setter.isSerialDebug){
            serial = new SerialPort();
            if(serial != nullptr)
                serial->Open(setter.SerialPath.data(),115200);
        }
        key = setter.key;

        pnp = new PnP(setter);
        analyzeStatistics = new AnalyzeStatistics(400,400);
        error = new AnalyzeStatistics(400,400);
        dirS = new AnalyzeStatistics(400,400);
        Kf6 = new KalmanFilterAll(1);

        KFy = new KalmanFilter(1);
        KFp = new KalmanFilter(2);
        UltimateKFy = new KalmanFilter(true);
        UltimateKFp = new KalmanFilter(true);
        FindArmor = new ArmorFind(&DrawingBoard_1,&DrawingBoard_2);
        cap = setter.frameSource == -2? new AllianceVideoCapture(setter.VideoPath) :new AllianceVideoCapture(setter.frameSource);
        video = setter.isEnableShooting? new cv::VideoWriter(getTimeVideo(), CV_FOURCC('M', 'J', 'P', 'G'), 25, setter.shootingImageSize) : nullptr;

        for(int i = 0;i<TargetSize;i++){
            FinallyTraget[i] = cv::Point2f(setter.resolution.width/2,setter.resolution.height/2);
        }
        // init the Drawing board
        if(setter.isShowFinallyImage){
            DrawingBoard_1 = cv::Mat::zeros(setter.resolution,CV_8UC3);
            DrawingBoard_2 = cv::Mat::zeros(setter.resolution,CV_8UC3);
            DrawingBoard_map = cv::Mat::zeros(setter.resolution,CV_8UC3);
        }
        isSudden=0;
    }

    ~RoboMaster2019();

    static const int TargetSize = 40;
    int TargetSizeInv = 1024/40;
    int CalcGain(int n = TargetSize);
    float CalcGainf(int n = TargetSize);
    sint OpponentStatus = 0;
    sint Ignore = 0;
    // Area 4 Basic Fundamental
    sint key;                                        //key : turn from one mode to anther?
    int key_current_loop =0;                         //
    int key_previous = 0;                            //condition turnning.
    sint mode_Con_Vid_Pic = 0;                          //modeConVidPic : Control Continue,VideoShooting,Picture
    unsigned long int frameNum =0;
    float maxPossibility = 0;
    int maxPossibility_index = 0;
    unsigned char dataSend[10] = {0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};
    char dataReceive[10] = {0,0,0,0,0,0,0,0,0,0};
    bool isAllOK = false;
    sint RealAction;
    static const int PTZvSize = 40;
    int PTZvSizeInv = 1024/40;
    int Yaww[PTZvSize];
    int Paww[PTZvSize];
    float YouAngle[PTZvSize];
    float acceleration;
    int PTZva;
    sint PTZv[PTZvSize];
    int PTZvap;
    sint PTZvp[PTZvSize];
    sint naturalCoordinates = 0;
    sint NewnaturalCoordinates = 0;
    sint isSudden = 0;
    unsigned char bulletVelocity;
    sint isBlinking = 0;
    float abcur = 0;
    float abold = 0;
    int velocity4Bullet = 0;

    union { sint a; unsigned char subdata[2]; } Datas;
    cv::Mat Frame;                                   //bgr origin image
    cv::Mat Gray;                                    //
    cv::Mat DrawingBoard_1;                          // 2 Draw
    cv::Mat DrawingBoard_2;
    cv::Mat DrawingBoard_map;
    cv::Mat GrayEnhancement;                         // 4 Numbers Recognition
    cv::Point2f Predict;
    cv::Point2f PreDict_old;
    cv::Point3f PitchYawDistance;
    std::vector<Armor> Armors;
    cv::Point FinallyTraget[TargetSize];
    cv::Point PredictTraget[TargetSize];
    Setter setter;                                   //setter : all information of Setting file.
    Analyze analyze;
    ArmorNumberRecgnation Recg;
    MicrosecondChronograph timer;
    TimerBase timerBase;
    TimerBase timerBase4Collimation;
    TimerBaseClock timerBaseClock;
    std::chrono::steady_clock::time_point time_point= std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    KalmanFilter *KFy;
    KalmanFilter *KFp;
    KalmanFilter *UltimateKFy;
    KalmanFilter *UltimateKFp;
    KalmanFilterAll *Kf6;
    SerialPort *serial;
    cv::VideoWriter *video = nullptr;                          //video :
    AllianceVideoCapture *cap;                           //cap :
    ArmorFind *FindArmor;
    PnP *pnp;
    std::thread *ReadThread;
    std::thread *MainThread;
    AnalyzeStatistics * analyzeStatistics;
    AnalyzeStatistics * error;
    AnalyzeStatistics * dirS;


    void SentryMode();
    sint ScoutMode();                              // sub ~ Sentry Scout Mode
    void AutoCollimation2();
    void EndOneLoop();                                      // Call it When One loop is over
    void KeyBoardCallBack();
    void BuffExpert();
    void SentryBgr2GrayBuff();
    sint BuffMode();
    void SolveCloestPosition(float s,float dis,float yaw);

private:

    std::vector<std::vector<cv::Point>> AllContours;
    void SentryBgr2Gray();                                  //
    void ArmorExpert();
    cv::Mat PerspectiveTrans(cv::Mat src, cv::Point2f* scrPoints, cv::Point2f* dstPoints);
    void PredictTarget();
    void Makedata();
};
#endif //ALLIANCE2019_SENTRY_ROBOMASTER2019_H
