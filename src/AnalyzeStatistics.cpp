//
// Created by nvidia on 19-4-14.
//

#include "AnalyzeStatistics.h"

void AnalyzeStatistics::Plotf(cv::Mat &M,const cv::Scalar c) {
    for(int i = 0;i<Dataf.size();i++){
        M.at<cv::Vec3b>(i,(int)Dataf[i])[0] += c[0]/2;
        M.at<cv::Vec3b>(i,(int)Dataf[i])[1] += c[1]/2;
        M.at<cv::Vec3b>(i,(int)Dataf[i])[2] += c[2]/2;
    }
}

void AnalyzeStatistics::inf(float ele) {
    if(Dataf.size()<size){
        Dataf.push_back(ele);
    }else{
        for(int i = 0;i<size-2;++i){
            Dataf[i-1] = Dataf[i];
        }
        Dataf[size-1] = ele;
    }
}

void AnalyzeStatistics::catf(float ele) {

}

void AnalyzeStatistics::cati(int ele) {

}

void AnalyzeStatistics::ini(int ele) {
    if(Datai.size()<size){
        Datai.push_back(ele);
    }else{
        for(int i = 1;i<size;++i){
            Datai[i-1] = Datai[i];
        }
        Datai[size-1] = ele;
    }
}

void AnalyzeStatistics::clear(cv::Mat &M) {
    M = cv::Mat::zeros(M.cols,M.rows,CV_8UC3);
    for(int i = 0;i<40;++i){
        cv::putText(M,std::to_string(-(i-5)*40),cv::Point(5,i*40),1,0.5,cv::Scalar::all(255));
        cv::line(M,cv::Point(0,i*40),cv::Point(size-1,i*40),cv::Scalar(120,120,120));
    }
}

void AnalyzeStatistics::Ploti(cv::Mat &M, const cv::Scalar c) {
    for(int i = 0;i<Datai.size();i++){
        M.at<cv::Vec3b>(Datai[i],i)[0] = c[0];
        M.at<cv::Vec3b>(Datai[i],i)[1] = c[1];
        M.at<cv::Vec3b>(Datai[i],i)[2] = c[2];
    }
}
