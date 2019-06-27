//
// Created by Nicapoet on 18-11-20.
//

#ifndef MINDVISIONCAMERA_MINDVISIONCAMERA_H
#define MINDVISIONCAMERA_MINDVISIONCAMERA_H
#include"IndustrialCamera/CameraApi.h"
#include<opencv2/opencv.hpp>

class MindVisionCamera {
private:
    tSdkCameraDevInfo       tCameraEnumList;    //camera Info,ect: Product ID
    tSdkCameraCapbility     tCapability;        //camera setting,include tSdkImageResolution
    tSdkImageResolution    tImageResolution;    //camera output image setting

    unsigned char           * g_pRgbBuffer;     //get image buffer
    bool                    is_save_video;
    bool                    is_read_video;
    cv::VideoWriter writer;
    cv::VideoCapture local_video;

public:
    int h_camera;
    /*
     * @brife:  init Camera sdk and set params
     * @param1 isSaveVideo:  isSaveVideo=true,srcimage will be saved to Video.avi
     */
    explicit MindVisionCamera();
    /*
     * uInit Camera sdk
     */
    ~MindVisionCamera();
    /*
     * @brife:  set ExposureTime
     * @param1 fExposureTime: target exposure time,unit ms.
     */
    void SetExposureTime( double          fExposureTime);
    /*
     * @brife   :get a frame
     * @param1 frame:   a Mat variable;
     */
    bool read(cv::Mat &frame);
    void stop();
};


#endif //MINDVISIONCAMERA_MINDVISIONCAMERA_H
