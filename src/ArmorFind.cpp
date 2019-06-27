//
// Created by nvidia on 19-4-5.
//

#include "ArmorFind.h"
ArmorFind::ArmorFind(cv::Mat *m1,cv::Mat *m2) {


    DrawingBoard_2 = m2;
    DrawingBoard_1 = m1;

}

void ArmorFind::FindFromContours(cv::Mat &Frame,
                                 std::vector<std::vector<cv::Point>> AllContours,
                                 std::vector<Armor> &Armors,
                                 Setter setter) {
    FindLed(Frame,AllContours,setter);
    //CorrectContoursAndLedbar to Armor
    //LedBar2Armor
    int ledsSzie = static_cast<int>(LedGroup.size());
    for(int i = 0;i<ledsSzie;++i){
        for(int j = i+1;j<ledsSzie;++j){
            if(LedGroup[i].DistanceWith(LedGroup[j])/(LedGroup[i].length+LedGroup[j].length) < 3.0f &&
               LedGroup[i].DistanceWith(LedGroup[j])/(LedGroup[i].length+LedGroup[j].length) >0.5f ) {
                cv::Point2f points[4];
                LedBar::AdjustPointOrder(LedGroup[i], LedGroup[j], points);

                cv::Point2f v01(0,0);
                cv::Point2f v03(0,0);
                v01 = points[1]-points[0];
                v03 = points[3]-points[0];
                float m01 = sqrtf(v01.x*v01.x+v01.y*v01.y);
                float m03 = sqrtf(v03.x*v03.x+v03.y*v03.y);
                v01 = v01/(m01+0.00001f);
                v03 = v03/(m03+0.00001f);
                float check = v01.x*v03.x+v01.y*v03.y;
                // ArmorLeds almost have the same length
                if(LedGroup[i].checkParallel(LedGroup[j])>98){                     // Degree of Parallel
                    if(LedGroup[i].checkEqualSize(LedGroup[j])<0.25) {                   // Have the same length
                        if (LedGroup[i].checkRectangle(LedGroup[j]) < 40) {                // Rectangle
                            if (LedGroup[i].checkProportion(LedGroup[j], 2.0) < 2.5) {    // width : height
                                if (LedGroup[i].checkArea(LedGroup[j]) > 100) {           // Area
                                    if(fabs(check)<0.2588){
                                        float possibilitytemp = -LedGroup[i].checkRectangle(LedGroup[j]);
                                        Armor tempArmor(points,possibilitytemp);
                                        tempArmor.possibility = -sqrtf((tempArmor.ArmorCenter.x-320)*(tempArmor.ArmorCenter.x-320)
                                                                       +(tempArmor.ArmorCenter.y-240)*(tempArmor.ArmorCenter.y-240));
                                        //tempArmor.possibility = -fabs(check);
                                        Armors.push_back(tempArmor);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void ArmorFind::FindFromContoursOptimize(cv::Mat &Frame,
                                  std::vector<std::vector<cv::Point>> AllContours,
                                  std::vector<Armor> &Armors,
                                  Setter setter) {
    FindLed(Frame,AllContours,setter);
    //CorrectContoursAndLedbar to Armor
    //LedBar2Armor
    int ledsSzie = static_cast<int>(LedGroup.size());
    for(int i = 0;i<ledsSzie;++i){
        for(int j = i+1;j<ledsSzie;++j){
            if(LedGroup[i].DistanceWith(LedGroup[j])/(LedGroup[i].length+LedGroup[j].length) < 3.0f &&
               LedGroup[i].DistanceWith(LedGroup[j])/(LedGroup[i].length+LedGroup[j].length) >0.5f ){
                cv::Point2f points[4];
                cv::Point2f pointss[4];
                LedBar::AdjustPointOrder(LedGroup[i],LedGroup[j],points);
                pointss[0] = cv::Point(0,0);
                pointss[1] = points[1]-points[0];
                pointss[2] = points[2]-points[0];
                pointss[3] = points[3]-points[0];

                float PImage[3][3] = {(pointss[2].x-pointss[3].x)/129, (pointss[2].x-pointss[1].x)/56,pointss[1].x-pointss[2].x+pointss[3].x,
                                      (pointss[2].y-pointss[3].y)/129, (pointss[2].y-pointss[1].y)/56,pointss[1].y-pointss[2].y+pointss[3].y,
                                      0,0,1};
                cv::Point2f stdLength = pointss[1]+pointss[3];
                float temp = sqrtf(stdLength.x*stdLength.x+stdLength.y*stdLength.y);

                cv::Mat res =  cv::Mat(3,3,CV_32FC1,PImage);
                float tx = res.at<float>(0,2);
                float ty = res.at<float>(1,2);
                float w = sqrtf((pointss[2]-pointss[3]).x*(pointss[2]-pointss[3]).x+(pointss[2]-pointss[3]).y*(pointss[2]-pointss[3]).y);
                float h = sqrtf((pointss[2]-pointss[3]).x*(pointss[2]-pointss[3]).x+(pointss[2]-pointss[3]).y*(pointss[2]-pointss[3]).y);
                if(w/temp>0.1 && w/temp<1 && h/temp>0.1 && h/temp<1){
                    if(sqrtf(tx*tx+ty*ty)/temp < 0.2){
                        Armor tempArmor(points,-sqrtf(tx*tx+ty*ty)/temp);
                        Armors.push_back(tempArmor);
                    }
                }
            }
        }
    }
}

void ArmorFind::FindLed(cv::Mat &Frame, std::vector<std::vector<cv::Point>> AllContours, Setter setter) {
//PickCorrectContoursAndLedBar
    int allContoursSize = static_cast<int>(AllContours.size());
    for(int i = 0;i<allContoursSize;++i){

        //cv::drawContours(*DrawingBoard_1,AllContours,i,W,1);
        int currentContourSize = static_cast<int>(AllContours[i].size());

        //find minAreaRect of each contours
        if(currentContourSize>=setter.theMinmumSize4aContours){
            cv::RotatedRect rect = cv::minAreaRect(AllContours[i]);
            float a,b,a_b;
            // Find2End4RotatedRect
            //four points of each Rect
            cv::Point2f RoRePoints[4];
            rect.points(RoRePoints);
            //calc Rect's a and b length.
            //length of p0 p1 is a
            a = sqrtf((RoRePoints[0].x-RoRePoints[1].x)*(RoRePoints[0].x-RoRePoints[1].x)+(RoRePoints[0].y-RoRePoints[1].y)*(RoRePoints[0].y-RoRePoints[1].y));
            //length of p0 p3 is b
            b = sqrtf((RoRePoints[1].x-RoRePoints[2].x)*(RoRePoints[1].x-RoRePoints[2].x)+(RoRePoints[1].y-RoRePoints[2].y)*(RoRePoints[1].y-RoRePoints[2].y));
            if(a>b){
                a_b = a/b;
            }else{
                a_b = b/a;
            }

            //range and area
            if(a_b>setter.infLengthOverWidth  && a_b <setter.supLengthOverWidth){
                sint blue = 0;
                sint red = 0;
                for(int j = 0;j<currentContourSize;++j){
                    const uchar* inData=Frame.ptr<uchar>(AllContours[i][j].y);
                    if(*(inData+AllContours[i][j].x*3+0)>*(inData+AllContours[i][j].x*3+2)+*(inData+AllContours[i][j].x*3+1)/4){
                        ++blue;
                    }
                    if(*(inData+AllContours[i][j].x*3+2)>(*(inData+AllContours[i][j].x*3+0)+*(inData+AllContours[i][j].x*3+1))){
                        ++red;
                    }
                }
                //Red or blue ?
                if((blue>currentContourSize*setter.blueColorPercentage && setter.Youare == 0) ||   // Opponent is Blue

                   (red>currentContourSize*setter.redColorPercentage && setter.Youare == 2) ||  // Opponent is red

                   ((blue>currentContourSize*setter.blueColorPercentage ||
                     red>currentContourSize*setter.redColorPercentage)
                    && setter.Iam == 1)){ // I can hurt My Teammates
                    LedBar temp(rect);
                    LedGroup.push_back(temp);
                    temp.DrawLed(*DrawingBoard_1);

                }else{
                    cv::Point2f Coners[4];
                    rect.points(Coners);
                    cv::putText(*DrawingBoard_1,"Color",Coners[2],1,1,cv::Scalar(0,0,255));
                }
            }else{
                cv::Point2f Coners[4];
                rect.points(Coners);
                cv::putText(*DrawingBoard_1,std::string("W_H")+std::to_string(a_b),Coners[2],1,1,cv::Scalar(0,0,255));
            }
        }
    }
}
