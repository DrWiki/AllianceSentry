//
// Created by Nicapoet on 18-11-20.
//

#include <IndustrialCamera/CameraDefine.h>
#include <string>
#include "../include/MindVisionCamera.h"

MindVisionCamera::MindVisionCamera() : is_read_video(false), is_save_video(false) {
    int camera_counts = 1;
    CameraSdkInit(1);
    int status = CameraEnumerateDevice(&tCameraEnumList, &camera_counts);

    if (status) {
        std::cout << "camera enum fail,error code=" << status << std::endl;
        exit(-1);
    }

    if (!camera_counts) {
        std::cout << "camera_disconnect" << std::endl;
        exit(-2);
    }

    status = CameraInit(&tCameraEnumList, -1, -1, &h_camera);
    if (status) {
        std::cout << "camera init fail" << std::endl;
        exit(-3);
    }

    CameraGetCapability(h_camera, &tCapability);
    g_pRgbBuffer = (unsigned char *) malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
    //set exposure time by proto file
    SetExposureTime(1);//3
    //////////////////////////////////////////////////////
    //set Image Output Setting
    CameraGetImageResolution(h_camera, &tImageResolution);
    tImageResolution.iIndex = 0xff;
    //[1]set outputimage size
    tImageResolution.iWidth = 800;
    tImageResolution.iHeight = 600;
    tImageResolution.iWidthFOV = 800;
    tImageResolution.iHeightFOV = 600;
    //[2]set outputimage's offset in the full image
    tImageResolution.iHOffsetFOV = 240;
    tImageResolution.iVOffsetFOV = 212;
    CameraSetImageResolution(h_camera, &tImageResolution);
    CameraPlay(h_camera);

    CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);
    CameraSetAeState(h_camera, FALSE);
    CameraSetFrameSpeed(h_camera, 1);
}

MindVisionCamera::~MindVisionCamera() {
    std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXX";
    CameraStop(h_camera);
    if (is_read_video) {
        CameraUnInit(h_camera);
        free(g_pRgbBuffer);
    } else {
        local_video.release();
    }
    if (is_save_video) {
        writer.release();
    }
}

void MindVisionCamera::SetExposureTime(double ExposureTime_ms) {
    CameraSetExposureTime(h_camera, ExposureTime_ms * 1000);
}

bool MindVisionCamera::read(cv::Mat &frame) {
    IplImage *iplImage = NULL;
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    if (CameraGetImageBuffer(h_camera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
        CameraImageProcess(h_camera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
        if (iplImage) {
            cvReleaseImageHeader(&iplImage);
        }
        iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, 3);
        cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * 3);

        frame = cv::cvarrToMat(iplImage);
        if (is_save_video) { writer << frame; }
        //std::cout<<"cols="<<frame.cols<<" rows="<<frame.rows<<std::endl;
        CameraReleaseImageBuffer(h_camera, pbyBuffer);
        return true;
    }
    return false;
}
void MindVisionCamera::stop() {
    CameraStop(h_camera);
}

