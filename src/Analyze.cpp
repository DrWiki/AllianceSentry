//
// Created by nvidia on 19-2-18.
//

#include "Analyze.h"

Analyze::Analyze() {
    OwnBoard = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
}

Analyze::~Analyze() {

}

void Analyze::AnalyzeTheMoment(std::vector<std::vector<cv::Point>> contours, cv::Size size){
    cv::Mat MainBoard = cv::Mat::zeros(size,CV_8UC(3));

    for(int i = 0; i<contours.size();++i){
        cv::Rect TempRect = cv::boundingRect(contours[i]);
        cv::RotatedRect TempRotated = cv::minAreaRect(contours[i]);
        cv::Moments TempMoment = cv::moments(contours[i]);

        //fitEllipse need at least 5 points, so to use if to avoid the abnormal situations
        if(contours[i].size()>=5){
            cv::RotatedRect TempEllipse = cv::fitEllipse(contours[i]);
            cv::ellipse(MainBoard,TempEllipse,cv::Scalar(0,255,255),1);
            cv::Point2f po[4];
            TempEllipse.points(po);
            cv::putText(MainBoard,"a",po[0],1,1,cv::Scalar(255,255,0),1);
            cv::putText(MainBoard,"b",po[1],1,1,cv::Scalar(255,255,0),1);
            cv::putText(MainBoard,"c",po[2],1,1,cv::Scalar(255,255,0),1);
            cv::putText(MainBoard,"d",po[3],1,1,cv::Scalar(255,255,0),1);

            cv::drawContours(MainBoard,contours,i,cv::Scalar(0,0,255),1);
        }else{
            cv::drawContours(MainBoard,contours,i,cv::Scalar(0,255,255),1);
        }
        cv::Point2f center = TempRotated.center;
        cv::Point2f mcenter(static_cast<float >(TempMoment.m10/TempMoment.m00), static_cast<float>(TempMoment.m01/TempMoment.m00));
        cv::arrowedLine(MainBoard,center,mcenter,cv::Scalar(255,0,0),1);
        cv::rectangle(MainBoard,TempRect.tl(),TempRect.br(),cv::Scalar(255,0,0),1);
        cv::putText(MainBoard,std::to_string(i),TempRect.tl(),1,1,cv::Scalar(0,255,0),1);
        std::stringstream content;
//        content<<"Hu M1: " << std::to_string(TempMoment.nu20+TempMoment.nu02)<<"/"<<std::to_string(TempMoment.m00)<<std::endl;
        content<<"Hu M1: " << std::to_string(TempMoment.nu20+TempMoment.nu02)<<"/"<<std::to_string((TempMoment.nu20-TempMoment.nu02)*(TempMoment.nu20-TempMoment.nu02)+4*TempMoment.nu11*TempMoment.nu11)<<std::endl;
        cv::putText(MainBoard,content.str(),TempRect.br(),1,1,cv::Scalar(255,255,255),1);

    }

    cv::imshow("Analyze Moment",MainBoard);
}

void Analyze::AnalyzeArmor_Double(std::vector<cv::RotatedRect> Rect, cv::Mat Board) {
    cv::Mat MainBoard = cv::Mat::zeros(Board.size(),CV_8UC(3));
    for (int i = 0;i<Rect.size();++i){
        for(int j = i+1;j<Rect.size();++j) {

            cv::circle(Board,cv::Point((Rect[i].center.x+Rect[j].center.x)/2,
                                           (Rect[i].center.y+Rect[j].center.y)/2),5,cv::Scalar::all(255),2);
        }
    }

    cv::imshow("Analyze_Double",Board);
}

void Analyze::AnalyzeArmor_Single(std::vector<std::vector<cv::Point>> contours, cv::Mat Board) {
    for (int i = 0;i<contours.size();++i){

    }
}
