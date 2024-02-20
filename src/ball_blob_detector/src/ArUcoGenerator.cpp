#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
int main() {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);


    int numMarkers = 10; // Number of markers you want to generate
    int markerSize = 200; // Size of the marker image in pixels
    for (int i = 0; i < numMarkers; i++) {
        cv::Mat markerImage;
        cv::aruco::drawMarker(dictionary, i, markerSize, markerImage, 1);
        std::string fileName = "marker_" + std::to_string(i) + ".png";
        cv::imwrite(fileName, markerImage);
        std::cout << "Saved " << fileName << std::endl;
    }

    return 0;
}

