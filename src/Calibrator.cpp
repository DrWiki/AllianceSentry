//
// Created by nicapoet on 19-1-1.
//

#include "Calibrator.h"
std::string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "../others/parameters_lists/%Y-%m-%d_%H_%M_%S.yaml",localtime(&timep) );
    return tmp;
}

Calibrator::Calibrator(
        const char* images_folder,
        const cv::Size board_size,
        const cv::Size square_size)
{

    cv::Size image_size_temp;
    std::vector<cv::String>file_list;
    cv::glob(images_folder, file_list);
    addChessBoardPoins(file_list, board_size, square_size,image_size_temp);
    Calibrate(image_size_temp);
}

int Calibrator::addChessBoardPoins(
        const std::vector<cv::String>file_list,
        const cv::Size board_size,
        const cv::Size square_size,
        cv::Size &image_size)
{
    std::vector<cv::Point2f>image_corners;
    std::vector<cv::Point3f>object_corners;
    cv::Mat image;
    int successes = 0;
    //init Chess 3D coordinates;
    for (size_t i = 0; i < board_size.height; i++)
        for (size_t j = 0; j < board_size.width; j++)
            object_corners.emplace_back(cv::Point3f(float(i*square_size.width), float(j*square_size.height), 0.0f));
    for (auto file_name : file_list)
    {
        image = cv::imread(file_name,0);
        image_size = image.size();
        bool found = cv::findChessboardCorners(image, board_size, image_corners);
        if (found)
        {
            cv::cornerSubPix(
                    image,
                    image_corners,
                    cv::Size(5, 5),
                    cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 50, 0.001)
            );
            if (image_corners.size() == board_size.area())
            {
                addPoints(image_corners, object_corners);
                successes++;
            }
        }
        else
        {
            std::cout << "未找到角点\n" << std::endl;
        }
        for (auto cornersPt : image_corners)
        {
            cv::circle(image, cornersPt, 5, cv::Scalar(125), -1,1);
        }
        cv::imshow("image", image);
        cv::waitKey(50);
    }
    return successes;
}
void Calibrator::addPoints(
        std::vector<cv::Point2f>a_ImagePoints,
        std::vector<cv::Point3f>a_objectPoits
)
{
    image_points.push_back(a_ImagePoints);
    object_poits.push_back(a_objectPoits);
}
double Calibrator::Calibrate(const cv::Size image_size)
{
    std::vector<cv::Mat>rvecs, tvecs;
    double result = cv::calibrateCamera(
            object_poits,
            image_points,
            image_size,
            camera_Matrix,
            dist_Coeffs,
            rvecs, tvecs,
            flag
    );
    return result;
}
cv::Mat Calibrator::remap_correct(const cv::Mat srcImage)
{
    cv::Size image_size = srcImage.size();
    cv::Mat undistortImage;
    cv::Mat mapx(image_size, CV_32FC1);
    cv::Mat mapy(image_size, CV_32FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    initUndistortRectifyMap(camera_Matrix, dist_Coeffs, R, camera_Matrix, image_size, CV_32FC1, mapx, mapy);
    cv::Mat dstImage;
    cv::remap(srcImage, dstImage, mapx, mapy, cv::INTER_LINEAR);
    return dstImage;
}
void Calibrator::save_calibrate_result()
{
    std::map<std::string, cv::Mat>calibrate_Matrixes;
    calibrate_Matrixes["camera_Matrix"] = camera_Matrix;
    calibrate_Matrixes["dist_Coeffs"] = dist_Coeffs;
    cv::FileStorage fs(getTime(), cv::FileStorage::WRITE);
    for (auto camera_para : calibrate_Matrixes)
    {
        fs << camera_para.first << camera_para.second;
    }
    fs.release();
}
void Calibrator::read_calibrate_result(std::string input_file_name) {
    cv::FileStorage fs(input_file_name,cv::FileStorage::READ);
    fs["camera_Matrix"]>>camera_Matrix;
    fs["dist_Coeffs"]>>camera_Matrix;
    std::cout<<"camera_Matrix"<<camera_Matrix<<std::endl;
    std::cout<<"dist_Coeffs"<<dist_Coeffs<<std::endl;
    fs.release();
}