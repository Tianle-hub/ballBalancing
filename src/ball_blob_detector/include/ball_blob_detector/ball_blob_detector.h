#ifndef BALL_BLOB_DETECTOR_H
#define BALL_BLOB_DETECTOR_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Dense> 
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

std::vector<cv::KeyPoint> Blob;
class BallBlobDetector {
public:
    BallBlobDetector(int cameraIndex);
    ~BallBlobDetector();
    double getPosXPrev() const { return pos_x_prev; }
    double getPosYPrev() const { return pos_y_prev; }
    double getTimePrev() const { return time_prev; }
    double getMeasure() const { return measure; }
    void setPosXPrev(double x) { pos_x_prev = x; }
    void setPosYPrev(double x) { pos_y_prev = x; }
    void setTimePrev(double x) { time_prev = x; }
    void run();
    cv::VideoCapture capture;
    std::vector<cv::KeyPoint> processFrame();
    Eigen::Vector2d plateCenterDetection();
    bool calibration = true;
    void setupPlateCoordinate();
private:
    cv::Ptr<cv::SimpleBlobDetector> detector;
    void setupDetectorParams();
    
    double pos_x_prev = 0;
    double pos_y_prev = 0;
    double time_prev = 0;
    bool measure = false;
    double centerX = 0;
    double centerY = 0;
    std::vector<cv::KeyPoint> Blob;
    // aruco_marker
    // Placeholder values for camera calibration
    float markerSideLength = 0.048; // Replace with actual marker side length in meters
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1); // Replace with actual camera matrix
    cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << 0, 0, 0, 0, 0); // Replace with actual distortion coefficients

    cv::Mat frame, imageCopy;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // my brown plate
    // double PlateLengthMeter = 0.31;
    // double PlateWidthMeter = 0.22;

    // double plate_length_upper_bound = 0.3;
    // double plate_length_lower_bound = 0.25;
    // double plate_width_upper_bound = 0.2;
    // double plate_width_lower_bound = 0.16;

    // Big grey plate
    double PlateLengthMeter = 0.5;
    double PlateWidthMeter = 0.5;

    double plate_length_upper_bound = 0.5;
    double plate_length_lower_bound = 0.42;
    double plate_width_upper_bound = 0.5;
    double plate_width_lower_bound = 0.42;

    double CenterLength = 0;
    double CenterWidth = 0;
    Eigen::Matrix<double, 4, 2> MarkerPixelCenter;
    Eigen::Matrix<double, 2, 1> PlatePixelCenter;
    Eigen::Matrix<double, 2, 1> PlatePixelSize;

    double k_pixel_length = 0;
    double k_pixel_width = 0;

    Eigen::Matrix<double, 2, 1> PlatePixelCenter_fixed; 
    double k_pixel_length_fixed = 0;
    double k_pixel_width_fixed = 0;

};

#endif // BALL_BLOB_DETECTOR_H