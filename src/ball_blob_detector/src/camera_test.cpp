#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    int cameraIndex = 4; // Adjust this based on your camera
    cv::VideoCapture capture(cameraIndex);
    if (!capture.isOpened()) {
        std::cerr << "Error: Cannot open camera" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        capture >> frame;
        if (frame.empty()) break; // End of video stream

        cv::imshow("Test Camera", frame);
        if (cv::waitKey(10) >= 0) break; // Wait for a keystroke in the window
    }
    return 0;
}
