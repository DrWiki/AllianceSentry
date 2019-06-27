//
// Created by nvidia on 19-1-11.
//

#include <LedBar.h>

#include "LedBar.h"
#define K 10
LedBar::LedBar(cv::RotatedRect rotrect) {

    rotateR = rotrect;
    rotrect.points(Coners);

    float a = sqrtf((Coners[0].x-Coners[1].x)*(Coners[0].x-Coners[1].x)+(Coners[0].y-Coners[1].y)*(Coners[0].y-Coners[1].y));
    //length of p0 p3 is b
    float b = sqrtf((Coners[1].x-Coners[2].x)*(Coners[1].x-Coners[2].x)+(Coners[1].y-Coners[2].y)*(Coners[1].y-Coners[2].y));
    if(a>=b){
        p1.x = (Coners[1].x+Coners[2].x)/2;
        p1.y = (Coners[1].y+Coners[2].y)/2;
        p2.x = (Coners[3].x+Coners[0].x)/2;
        p2.y = (Coners[3].y+Coners[0].y)/2;
    }else{
        p1.x = (Coners[1].x+Coners[0].x)/2;
        p1.y = (Coners[1].y+Coners[0].y)/2;
        p2.x = (Coners[3].x+Coners[2].x)/2;
        p2.y = (Coners[3].y+Coners[2].y)/2;
    }



    center.x = (p1.x+p2.x)*0.5f;
    center.y = (p1.y+p2.y)*0.5f;

    length = sqrtf(((p1.x-p2.x)*(p1.x-p2.x))+((p1.y-p2.y)*(p1.y-p2.y)));

    dirv.x = p2.x-p1.x;
    dirv.y = p2.y-p1.y;

    dirv_E.x = K*dirv.x/(sqrtf(dirv.x*dirv.x+dirv.y*dirv.y));
    dirv_E.y = K*dirv.y/(sqrtf(dirv.x*dirv.x+dirv.y*dirv.y));
}

cv::Point2f E(cv::Point2f P){
    return cv::Point2f((K*P.x)/sqrtf(P.x*P.x+P.y*P.y),(K*P.y)/sqrtf(P.x*P.x+P.y*P.y));
}



void LedBar::AdjustPointOrder(LedBar l1,LedBar l2,cv::Point2f points[]) {
    if(l1.center.x<=l2.center.x){
        //l1 is on the left
        if(l1.p1.y<=l1.p2.y){
            points[0] = l1.p1;
            points[3] = l1.p2;
        }else{
            points[0] = l1.p2;
            points[3] = l1.p1;
        }

        if(l2.p1.y<=l2.p2.y){
            points[1] = l2.p1;
            points[2] = l2.p2;
        }else{
            points[1] = l2.p2;
            points[2] = l2.p1;
        }
    }else{
        //l1 is on the right
        if(l1.p1.y<=l1.p2.y){
            points[1] = l1.p1;
            points[2] = l1.p2;
        }else{
            points[1] = l1.p2;
            points[2] = l1.p1;
        }

        if(l2.p1.y<=l2.p2.y){
            points[0] = l2.p1;
            points[3] = l2.p2;
        }else{
            points[0] = l2.p2;
            points[3] = l2.p1;
        }
    }

}

void LedBar::DrawLed(cv::Mat & src) {
    cv::circle(src,p1,2,cv::Scalar(255,255,255),-1);
    //cv::circle(src,center,3,cv::Scalar(0,255,255),-1);
    cv::circle(src,p2,2,cv::Scalar(180,180,180),-1);
    cv::line(src,Coners[0],Coners[1],cv::Scalar(255,0,0),1);
    cv::line(src,Coners[1],Coners[2],cv::Scalar(0,255,0),1);
    cv::line(src,Coners[2],Coners[3],cv::Scalar(0,0,255),1);
    cv::line(src,Coners[3],Coners[0],cv::Scalar(255,255,255),1);
}

float LedBar::cross(cv::Point2f A, cv::Point2f B) {
    return A.x*B.y-A.y*B.x;
}

float LedBar::dot(cv::Point2f A, cv::Point2f B) {
    return A.x*B.x+A.y*B.y;
}

float LedBar::DistanceWith(LedBar other) {
    return sqrtf((center.x-other.center.x)*(center.x-other.center.x)+(center.y-other.center.y)*(center.y-other.center.y));
}

float LedBar::dis(cv::Point2f A, cv::Point2f B) {
    return sqrtf((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
}

float LedBar::checkParallel(LedBar L2) {
    return fabs(dot(dirv_E,L2.dirv_E));
}

float LedBar::checkEqualSize(LedBar L2) {
    return fabs(length-L2.length)/(0.5*(length+L2.length));
}
float LedBar::checkRectangle(LedBar L2) {
    if(dot(dirv_E,L2.dirv_E)>0){
        return fabs(dis(p1,L2.p2)-dis(p2,L2.p1));
    }else{
        return fabs(dis(p1,L2.p1)-dis(p2,L2.p2));
    }
}

float LedBar::checkProportion(LedBar L2,float ratio) {
    if(dot(dirv_E,L2.dirv_E)>0){
        return fabs((dis(p1,L2.p1)/((length+L2.length)/2))-ratio);
    }else{
        return fabs((dis(p1,L2.p2)/((length+L2.length)/2))-ratio);
    }
}

float LedBar::checkArea(LedBar L2) {
    if(dot(dirv_E,L2.dirv_E)>0){
        return (dis(p1,L2.p1)*length);
    }else{
        return (dis(p1,L2.p2)*length);
    }
}


