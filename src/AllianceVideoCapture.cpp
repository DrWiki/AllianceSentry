//
// Created by nvidia on 19-1-17.
//

#include <AllianceVideoCapture.h>
#include "AllianceVideoCapture.h"

///
/// \param cameraType
/// Use Common USB Camera or Industrial Camera
AllianceVideoCapture::AllianceVideoCapture(sint cameraType) {
    if(cameraType<0){
        camera.IndustrialCamera = new MindVisionCamera();
        isIndustrialCamera = true;
    }else{
        camera.CommonCamera = new cv::VideoCapture;
        assert(camera.CommonCamera->open(static_cast<int>(cameraType)));           //abort if open failed
        isIndustrialCamera = false;
    }
}
///
/// \param VideoPath
AllianceVideoCapture::AllianceVideoCapture(std::string VideoPath) {
    camera.CommonCamera = new cv::VideoCapture;
    assert(camera.CommonCamera->open(VideoPath));           //abort if open failed
    isIndustrialCamera = false;
}
///
/// \param Frame
/// \return
bool AllianceVideoCapture::read(cv::Mat & Frame) {
    if(this->isIndustrialCamera){
        return this->camera.IndustrialCamera->read(Frame);
    }else{
        return this->camera.CommonCamera->read(Frame);
    }
    return 0;
}

AllianceVideoCapture::~AllianceVideoCapture() {
    if(camera.IndustrialCamera != nullptr){
        delete camera.IndustrialCamera;
    }
    std::cout<<"8888888888";
}

