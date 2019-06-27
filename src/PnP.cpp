//
// Created by nvidia on 19-1-19.
//
#include "PnP.h"

PnP::PnP(Setter setter) {
    NormalArmorWidth = setter.NormalArmorWidth/10.0f;
    NormalArmorHeight = setter.NormalArmorHeight/10.0f;
    LargerArmorWidth = setter.LargerArmorWidth/10.0f;
    LargerArmorHeight = setter.LargerArmorHeight/10.0f;
    CameraMatrix = setter.CameraMatrix;
    DistCoeffs = setter.DistCoeffs;

    // why did you set the data type double
    // because PnpSolve returns a Matrix that has type of double
    // and ArmorCenter will be used to compute with Matrix;
    // YOU_CAN_IMPROVE_IT

    DistCoeffs.at<double>(0) = 0;
    DistCoeffs.at<double>(1) = 0;
    DistCoeffs.at<double>(2) = 0;
    DistCoeffs.at<double>(3) = 0;

    NormalArmorPoints.emplace_back(cv::Point3f(-NormalArmorWidth/2,NormalArmorHeight/2,0));//II
    NormalArmorPoints.emplace_back(cv::Point3f(-NormalArmorWidth/2,-NormalArmorHeight/2,0));//III
    NormalArmorPoints.emplace_back(cv::Point3f(NormalArmorWidth/2,-NormalArmorHeight/2,0));//VI
    NormalArmorPoints.emplace_back(cv::Point3f(NormalArmorWidth/2,NormalArmorHeight/2,0));//I

    LargerArmorPoints.emplace_back(cv::Point3f(0,0,0));//II
    LargerArmorPoints.emplace_back(cv::Point3f(LargerArmorWidth,0,0));//III
    LargerArmorPoints.emplace_back(cv::Point3f(LargerArmorWidth,LargerArmorHeight,0));//VI
    LargerArmorPoints.emplace_back(cv::Point3f(0,LargerArmorHeight,0));//I

}

cv::Point3f PnP::GetXYZ(Armor armor) {
//    cv::Mat rvec(3,1,cv::DataType<double>::type);
//    cv::Mat tvec(3,1,cv::DataType<double>::type);
    cv::Mat_<double> rvec(3,1);
    cv::Mat_<double> tvec(3,1);
    std::vector<cv::Point2f> armor_CornerPoints;
    armor_CornerPoints.push_back(armor.CornerPoints[0]);
    armor_CornerPoints.push_back(armor.CornerPoints[1]);
    armor_CornerPoints.push_back(armor.CornerPoints[2]);
    armor_CornerPoints.push_back(armor.CornerPoints[3]);
    float zzz = 0;
    if(armor.armorType == RM_NORMAL_ARMOR){
        cv::solvePnP(NormalArmorPoints, armor_CornerPoints, CameraMatrix, DistCoeffs, rvec, tvec);
        zzz = float(tvec.at<double>(2,0))*2.00f;
    }else{
        cv::solvePnP(LargerArmorPoints, armor_CornerPoints, CameraMatrix, DistCoeffs, rvec, tvec);
        zzz = float(tvec.at<double>(2,0))*0.81f;
    }
    cv::Mat_<double> rot;
    double val[3] = {0,0,0};
    cv::Mat_<double> temp(3,1,val);
    cv::Rodrigues(rvec,rot);

    for(int i = 0;i<3;++i){
        //temp[0][i] = temp[0][0]*rot[i][0]+temp[0][1]*rot[i][1]+temp[0][2]*rot[i][2];
        temp[0][i] = temp[0][0]*rot[0][i]+temp[0][1]*rot[1][i]+temp[0][2]*rot[2][i];
    }

    for(int i = 0;i<3;++i){
        temp[0][i] = temp[0][i]+tvec[0][i];
        //temp[0][i] = temp[0][0]*rot[0][i]+temp[0][1]*rot[1][i]+temp[0][2]*rot[2][i];
    }
    cv::Point3f result((float)temp[0][0],(float)temp[0][1],(float)temp[0][2]);
    cv::Point3f result2;
    //result2.z = sqrtf(result.x*result.x + result.y*result.y + result.z*result.z);
    /*
    result2.y = 90 - (float) (atan2(result2.z, result.y) * 57.27f);
    result2.x = 90 - (float) (atan2(result.z, result.x) * 57.27f);
     */
    result2.x = result.x;
    result2.y = result.y;
    result2.z = zzz;
    //bug3(result.x,result.y,result.z);
    //bug(zzz);
    return result2;
}
