//
// Created by nvidia on 19-1-17.
//

#include "Armor.h"
cv::Point2f minus(cv::Point2f a,cv::Point b){
    return cv::Point2f(a.x-b.x,a.y-b.y);
}

Armor::Armor(cv::Point2f *points, float poss) {
    CornerPoints[0] = points[0];
    CornerPoints[1] = points[1];
    CornerPoints[2] = points[2];
    CornerPoints[3] = points[3];
    possibility = poss;
    ArmorCenter = (CornerPoints[0]+CornerPoints[1]+CornerPoints[2]+CornerPoints[3])/4;
    float  h1 = sqrtf((CornerPoints[0].x-CornerPoints[3].x)*(CornerPoints[0].x-CornerPoints[3].x)+
                      (CornerPoints[0].y-CornerPoints[3].y)*(CornerPoints[0].y-CornerPoints[3].y));
    float  h2 = sqrtf((CornerPoints[1].x-CornerPoints[2].x)*(CornerPoints[1].x-CornerPoints[2].x)+
                      (CornerPoints[1].y-CornerPoints[2].y)*(CornerPoints[1].y-CornerPoints[2].y));
    float  w1 = sqrtf((CornerPoints[0].x-CornerPoints[1].x)*(CornerPoints[0].x-CornerPoints[1].x)+
                      (CornerPoints[0].y-CornerPoints[1].y)*(CornerPoints[0].y-CornerPoints[1].y));
    float  w2 = sqrtf((CornerPoints[2].x-CornerPoints[3].x)*(CornerPoints[2].x-CornerPoints[3].x)+
                      (CornerPoints[2].y-CornerPoints[3].y)*(CornerPoints[2].y-CornerPoints[3].y));
    height = (h1+h2)*0.5f;
    width = (w1+w2)*0.5f;
    armorType = width/(height+1)>=3.1f? RM_LARGER_ARMOR:RM_NORMAL_ARMOR;
}


/// /// /// /// /// ///
/// *
/// \param l1
/// \param l2
/// description : CornerPoints is ordered Clock-Wise which is guaranteed by this Constructor *
Armor::Armor(LedBar l1, LedBar l2, float poss) {
    std::vector<cv::Point> PointVector;
    PointVector.push_back(l1.p1);
    PointVector.push_back(l1.p2);
    PointVector.push_back(l2.p1);
    PointVector.push_back(l2.p2);
    ArmorRotatedRect = cv::minAreaRect(PointVector);
    ArmorRotatedRect.points(CornerPoints);
    if(sqrtf((CornerPoints[0].x-CornerPoints[1].x)*(CornerPoints[0].x-CornerPoints[1].x)+
                     ((CornerPoints[0].y-CornerPoints[1].y))*((CornerPoints[0].y-CornerPoints[1].y)))>
                     sqrtf((CornerPoints[2].x-CornerPoints[1].x)*(CornerPoints[2].x-CornerPoints[1].x)+((CornerPoints[2].y-CornerPoints[1].y))*((CornerPoints[2].y-CornerPoints[1].y)))
                     ){
        cv::Point2f temp(CornerPoints[3]);
        CornerPoints[3] = CornerPoints[0];
        CornerPoints[0] = CornerPoints[1];
        CornerPoints[1] = CornerPoints[2];
        CornerPoints[2] = temp;

        Angle = ArmorRotatedRect.angle+90;
    }else{
        Angle = ArmorRotatedRect.angle;
    }

    possibility = poss;
    ArmorCenter.x = ArmorRotatedRect.center.x;
    ArmorCenter.y = ArmorRotatedRect.center.y;
    height = (l1.length+l2.length)/2;
}

/// /// /// /// /// ///
/// *
/// \param Board
/// description: Draw Armor on a color board*
void Armor::DrawArmorBGRLT(cv::Mat & Board) {
    assert(Board.channels() == 3);
    /*
    cv::line(Board, CornerPoints[0], CornerPoints[1], cv::Scalar(255, 0, 0), 2);
    cv::line(Board, CornerPoints[1], CornerPoints[2], cv::Scalar(0, 255, 0), 2);
    cv::line(Board, CornerPoints[2], CornerPoints[3], cv::Scalar(0, 0, 255), 2);
    cv::line(Board, CornerPoints[3], CornerPoints[0], cv::Scalar(0, 255, 255), 2);
     */

}
/// /// /// /// /// ///
/// *
/// \param Board
/// description: Draw Armor on a color board*
void Armor::DrawArmorBGR(cv::Mat & Board) {
    assert(Board.channels()==3);
    cv::line(Board,CornerPoints[0],CornerPoints[1],cv::Scalar(255,0,0),1);
    cv::line(Board,CornerPoints[1],CornerPoints[2],cv::Scalar(0,255,0),1);
    cv::line(Board,CornerPoints[2],CornerPoints[3],cv::Scalar(0,0,255),1);
    cv::line(Board,CornerPoints[3],CornerPoints[0],cv::Scalar(0,255,255),1);
}


/// /// /// /// /// ///
/// *
/// \param Board
/// description: put the Num text on a color board*
void Armor::WriteLabelBGR(cv::Mat & Board) {
//    assert(Board.channels()==3);
//    cv::putText(Board,std::to_string((possibility)),ArmorCenter,1,1,cv::Scalar(255,0,0));
//    cv::putText(Board,"0",CornerPoints[0],1,1,cv::Scalar(255,0,0));
//    cv::putText(Board,"1",CornerPoints[1],1,1,cv::Scalar(255,0,0));
//    cv::putText(Board,"2",CornerPoints[2],1,1,cv::Scalar(255,0,0));
//    cv::putText(Board,"3",CornerPoints[3],1,1,cv::Scalar(255,0,0));
}
void Armor::WriteLabelBGRLT(cv::Mat & Board) {
    assert(Board.channels()==3);
    cv::putText(Board,std::to_string((possibility)),ArmorCenter,1,2,cv::Scalar(0,0,255),2);
    cv::putText(Board,"0",CornerPoints[0],1,2,cv::Scalar(255,0,0),2);
    cv::putText(Board,"1",CornerPoints[1],1,2,cv::Scalar(255,0,0),2);
    cv::putText(Board,"2",CornerPoints[2],1,2,cv::Scalar(255,0,0),2);
    cv::putText(Board,"3",CornerPoints[3],1,2,cv::Scalar(255,0,0),2);
    if(armorType==RM_NORMAL_ARMOR)
        cv::circle(Board,ArmorCenter,norm(CornerPoints[0],CornerPoints[2])/2,cv::Scalar(255,255,0),1);
    else
        cv::circle(Board,ArmorCenter,norm(CornerPoints[0],CornerPoints[2])/2,cv::Scalar(255,255,0),2);
    cv::putText(Board,std::to_string(Angle),cv::Point(50,430),1,1,cv::Scalar(0,0,255),2);
}

