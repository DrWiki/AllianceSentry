//
// Created by nicapoet on 19-1-1.
//

#ifndef SFMREBUILD_CALIBRATOR_H
#define SFMREBUILD_CALIBRATOR_H

#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <time.h>
#include <iostream>
class Calibrator {
    //the 3 dimensional Points in the world coordinates
    std::vector<std::vector<cv::Point3f>> object_poits;
    //the 3 dimensional Points in the Images
    std::vector<std::vector<cv::Point2f>> image_points;
    //output Matrix
    //CameraMatrix
    cv::Mat camera_Matrix;
    //Distortion Matrix
    cv::Mat dist_Coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    int flag;
public:
    Calibrator() = delete;

    Calibrator(
            const char *images_folder,
            cv::Size board_size,
            cv::Size square_size
    );

    ~Calibrator() = default;

    cv::Mat remap_correct(const cv::Mat srcImage);

    void save_calibrate_result();
    void read_calibrate_result(std::string int_file_name);
private:
    int addChessBoardPoins(
            std::vector<cv::String> file_list,
            cv::Size board_size,
            cv::Size square_size,
            cv::Size &image_size
    );

    //返回重投影误差（实在找不到对应的单词^_^;)
    double Calibrate(
            cv::Size image_size
    );

    void addPoints(
            std::vector<cv::Point2f> a_ImagePoints,
            std::vector<cv::Point3f> a_objectPoits
    );
};

#endif //SFMREBUILD_CALIBRATOR_H
