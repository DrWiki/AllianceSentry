//
// Created by Shen Qiujie on 2018/12/22.
// phone:18852084160
//

#include "../include/ArmorDetector.h"
#define debug 1
#define show 1
#define showMessage(x,y); cout<<x<<":"<<y<<endl;
void ArmorDetector::setEnemyColor(bool color) {
    ENEMY_COLOR=color;
}
void ArmorDetector::cleanAllVectors(){
    vector<vector<cv::Point>>().swap(contours);
    vector<cv::RotatedRect>().swap(rotatedEllipses);
    vector<vector<cv::Point2f>>().swap(allArmorPoints);
    vector<vector<cv::RotatedRect>>().swap(finalArmors);
    vector<cv::RotatedRect>().swap(minRectOfContours);
    vector<cv::Point2f*>().swap(final4Points);
    vector<vector<cv::RotatedRect>>().swap(armors);
    vector<float>().swap(allRotatedRectHeight);
    vector<float>().swap(allRotatedRectWidth);
    armorsHeight[0] = '\0';
    armorsWidth[0] = '\0';
    armorsHeight[0] = '\0';
    armorsWidth[0] = '\0';
}
void ArmorDetector::writeArgs(){
    cv::FileStorage fs;
    fs.open("../args.yaml",cv::FileStorage::WRITE);
    fs<<"minContoursLength"<<25;
    fs<<"thresholdMax"<<255;
    fs<<"minHeightAndWidthRatio"<<2;
    fs<<"maxHeightAndWidthRatio"<<14;
    fs<<"minDipAngle"<<45;
    fs<<"maxDipAngle"<<135;
    fs<<"heightDiffRatio"<<0.2;
    fs<<"widthDiffRatio"<<0.35;
    fs<<"ellipseCenterDisRatioX"<<7;
    fs<<"ellipseCenterDisRatioY"<<0.2;
    fs<<"pixelSumDiffRatio"<<0.3;
    fs<<"lightbarAngleError"<<8;
    fs.release();
}
void ArmorDetector::readArgs() {
    cv::FileStorage fs;
    fs.open("../args.yaml",cv::FileStorage::READ);
    fs["minContoursLength"]>>minContoursLength;
    fs["minContoursLength"]>>minContoursLength;
    fs["thresholdMax"]>>thresholdMax;
    fs["minHeightAndWidthRatio"]>>minHeightAndWidthRatio;
    fs["maxHeightAndWidthRatio"]>>maxHeightAndWidthRatio;
    fs["minDipAngle"]>>minDipAngle;
    fs["maxDipAngle"]>>maxDipAngle;
    fs["heightDiffRatio"]>>heightDiffRatio;
    fs["widthDiffRatio"]>>widthDiffRatio;
    fs["ellipseCenterDisRatioX"]>>ellipseCenterDisRatioX;
    fs["ellipseCenterDisRatioY"]>>ellipseCenterDisRatioY;
    fs["pixelSumDiffRatio"]>>pixelSumDiffRatio;
    fs["lightbarAngleError"]>>lightbarAngleError;
    cout<<"read args done."<<endl;
    bool showMessage=false;
    if(showMessage){
        showMessage("minContoursLength",minContoursLength);
        showMessage("thresholdMax",thresholdMax);
        showMessage("minHeightAndWidthRatio",minHeightAndWidthRatio);
        showMessage("maxHeightAndWidthRatio",maxHeightAndWidthRatio);
        showMessage("minDipAngle",minDipAngle);
        showMessage("maxDipAngle",maxDipAngle);
        showMessage("heightDiffRatio",heightDiffRatio);
        showMessage("widthDiffRatio",widthDiffRatio);
        showMessage("ellipseCenterDisRatioX",ellipseCenterDisRatioX);
        showMessage("ellipseCenterDisRatioY",ellipseCenterDisRatioY);
        showMessage("pixelSumDiffRatio",pixelSumDiffRatio);
        showMessage("lightbarAngleError",lightbarAngleError);
    }
    fs.release();
}
void ArmorDetector::findLightbar() {
    cv::RotatedRect rotatedEllipse;
    std::vector<cv::Mat> channels;
    cv::split(inputImg,channels);
    if(ENEMY_COLOR==BLUE){
        channels[0]=channels[0]-channels[2];
    }
    else{
        channels[0]=channels[2]-channels[0];
    }
    cv::threshold(channels[1], channels[1],130,255,cv:: THRESH_BINARY);
    cv::threshold(channels[0], channels[0],60,255,cv:: THRESH_BINARY);

    cv::Mat binaryImg=channels[0]&channels[1];
    cv::dilate(binaryImg,binaryImg,5);
    cv::erode(binaryImg,binaryImg,3);

    if(debug){
        cv::imshow("channels[0]",channels[0]);
        cv::imshow("channels[1]",channels[1]);
        cv::imshow("binary",binaryImg);
    }
    cv::findContours(binaryImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (auto &contour : contours) {
        if (contour.size() < minContoursLength) {
            continue;
        }
        cv::RotatedRect rect;
        rect=cv::minAreaRect(contour);
        rotatedEllipse = cv::fitEllipse(contour);
        rotatedEllipses.push_back(rotatedEllipse);
    }
    for (int i = 0; i < rotatedEllipses.size(); ++i) {
        //移除宽高比不在筛选范围内的轮廓,判定不是轮廓
        cout<<"高宽比："<<rotatedEllipses[i].size.height / rotatedEllipses[i].size.width<<endl;
        if ((rotatedEllipses[i].size.height / rotatedEllipses[i].size.width < minHeightAndWidthRatio)
                ||(rotatedEllipses[i].size.height / rotatedEllipses[i].size.width > maxHeightAndWidthRatio)){
            rotatedEllipses.erase(rotatedEllipses.begin()+i);
            --i;
            continue;
        }
    }
}
int ArmorDetector::fromLightbarFindArmor() {
    for (int i = 0; i < rotatedEllipses.size(); ++i) {
        for (int j = i + 1; j < rotatedEllipses.size(); ++j) {
            //控制灯条长度差距在一定范围内
            if (abs(rotatedEllipses[i].size.height - rotatedEllipses[j].size.height)
                    > heightDiffRatio * (rotatedEllipses[i].size.height + rotatedEllipses[j].size.height)
                //控制灯条宽度差距在一定范围内
                ||abs(rotatedEllipses[i].size.width - rotatedEllipses[j].size.width)
                    > widthDiffRatio * (rotatedEllipses[i].size.width + rotatedEllipses[j].size.width)){ continue; }
            float angle_i,angle_j;
            rotatedEllipses[i].angle<90?angle_i=-rotatedEllipses[i].angle:angle_i=180-rotatedEllipses[i].angle;
            rotatedEllipses[j].angle<90?angle_j=-rotatedEllipses[j].angle:angle_j=180-rotatedEllipses[j].angle;
            if(angle_i-angle_j>10)continue;
            vector<cv::RotatedRect> armor;
            armor.push_back(rotatedEllipses[i]);      //将成对的灯条压入一个armor
            armor.push_back(rotatedEllipses[j]);
            armors.push_back(armor);
        }
    }
    return rotatedEllipses.size();
}
//从找到的灯条对中计算出装甲的四个点
bool ArmorDetector::get4PointsFromTargets() {
    if(armors.empty()) return false;                              //armors为空时，返回false，则可不进行下一步
    for (auto &armor : armors) {
        cv::Point2f final4Point[4];                                //筛选到最后确定的装甲板的四个点
        cv::Point2f leftLightbar[4], rightLightbar[4],p0,p1,p2,p3; //左右灯条，以及左右灯条的上下中心点
        if (armor[0].center.x > armor[1].center.x) {               //判断左右灯条
            swap(armor[0], armor[1]);
        }
        armor[0].points(leftLightbar);
        armor[1].points(rightLightbar);
        if (armor[0].angle >= 90) {                        //这边是计算出左右灯条的上下窄边的中心点
             p0 = (leftLightbar[2] + leftLightbar[1]) / 2;
             p1 = (leftLightbar[3] + leftLightbar[0]) / 2;
        }
        else {
            p0 = (leftLightbar[0] + leftLightbar[3]) / 2;
            p1 = (leftLightbar[1] + leftLightbar[2]) / 2;
        }
        if (armor[1].angle >= 90) {
             p2 = (rightLightbar[3] + rightLightbar[0]) / 2;
             p3 = (rightLightbar[2] + rightLightbar[1]) / 2;
        }
        else {
             p2 = (rightLightbar[1] + rightLightbar[2]) / 2;
             p3 = (rightLightbar[0] + rightLightbar[3]) / 2;
        }
        final4Point[0]=((p0-p1)/2+p0);                    //将灯条窄边上的点合成为装甲板的四个点
        final4Point[1]=((p1-p0)/2+p1);
        final4Point[2]=((p2-p3)/2+p2);
        final4Point[3]=((p3-p2)/2+p3);
        if (show) {                                      //用于在图上标注四个点的顺序，调程序的时候用的
            for (int i = 0; i < 4; ++i) {
                cv::circle(inputImg, final4Point[i], 6, cv::Scalar(255, 0, 0));
                char c[2];
                sprintf(c, "%d", i);
                cv::putText(inputImg, c, final4Point[i], 2, 2, cv::Scalar(255, 0, 0));
            }
        }
        final4Points.push_back(final4Point);
        return true;
    }
}
bool ArmorDetector::perspectiveTransform() {
    char windowsName[20];
    cv::Point2f pts[4];
    for(int i=0;i<final4Points.size();++i){
        for(int j=0;j<4;++j){
            pts[j]=final4Points[i][j];
        }
        int dstImageSizeX=36,dstImageSizeY=36;
        cv::Point2f dstPoint[4]{cv::Point(0,dstImageSizeY),cv::Point(0,0),cv::Point(dstImageSizeX,0),cv::Point(dstImageSizeX,dstImageSizeY)};
        cv::Mat warp_mat=cv::getPerspectiveTransform(pts,dstPoint);
        cv::Mat dstImage;
        cv::warpPerspective(inputImg,dstImage,warp_mat,cv::Size(36,36));
        possibleArmor.emplace_back(dstImage);
        if(debug) {
            sprintf(windowsName, "armor%d", i);
            cv::namedWindow(windowsName, cv::WINDOW_NORMAL);
            cv::moveWindow(windowsName, 860, 300 * i);
            cv::imshow(windowsName, dstImage);
        }
    }
    return true;
}
bool ArmorDetector::findArmor(cv::Mat &src) {
    vector<vector<cv::Point2f>> armors4Points;
    inputImg = src;
    cleanAllVectors();
    findLightbar();
    fromLightbarFindArmor();
    get4PointsFromTargets();
    perspectiveTransform();
    if(debug) {
        cv::imshow("src", src);
        cv::moveWindow("dst", 100, 0);
    }
    inputImg.release();
    return true;
}