#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    int cameraIndex = -1; // Adjust this based on your camera
    

    for (int i = 0;i<10;i++){
        std::cout<<"try camera "<< i << std::endl;
        cv::VideoCapture capture(cameraIndex);
        if(capture.isOpened()) {
            std::cout<<"Camera found at index:"<< i << std::endl;
            cameraIndex = i;
            capture.release();
            break;
        }

    }

    if (cameraIndex == -1) {
        std::cerr << "Error: Cannot open camera" << std::endl;
        return -1;
    }

    // cv::Mat frame;
    // while (true) {
    //     capture >> frame;
    //     if (frame.empty()) break; // End of video stream

    //     cv::imshow("Test Camera", frame);
    //     if (cv::waitKey(10) >= 0) break; // Wait for a keystroke in the window
    // }

    return 0;
}
