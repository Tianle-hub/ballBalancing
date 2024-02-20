#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
int main() {
    cv::VideoCapture capture;
    capture.open(4); // Change the index for different cameras.

    // Placeholder values for camera calibration
    float markerSideLength = 0.045; // Replace with actual marker side length in meters
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1); // Replace with actual camera matrix
    cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << 0, 0, 0, 0, 0); // Replace with actual distortion coefficients

    cv::Mat frame, imageCopy;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    while (capture.grab()) {
        capture.retrieve(frame);
        frame.copyTo(imageCopy);
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        // If markers are detected, draw them on the image
        if (!markerIds.empty()) {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSideLength, cameraMatrix, distCoeffs, rvecs, tvecs);
            
            // Draw axis for each marker
            for (int i = 0; i < markerIds.size(); i++) {
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerSideLength * 0.5f);
                std::cout<<"markerIds:"<<i<<std::endl;
                std::cout << "Rotation Vector (rvecs): [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2] << "]" << std::endl;
                std::cout << "Translation Vector (tvecs): [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]" << std::endl;

            }
        }

        cv::imshow("Detected ArUco markers", imageCopy);
        char key = (char) cv::waitKey(10);
        if (key == 27) break; // Exit on ESC
    }
    
    return 0;
}