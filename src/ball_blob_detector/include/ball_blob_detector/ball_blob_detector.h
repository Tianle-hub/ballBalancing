#ifndef BALL_BLOB_DETECTOR_H
#define BALL_BLOB_DETECTOR_H


#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

class BallBlobDetector {
public:
    BallBlobDetector(int cameraIndex);
    ~BallBlobDetector();
    void run();

private:
    cv::VideoCapture capture;
    cv::Ptr<cv::SimpleBlobDetector> detector;
    bool processFrame();
    void setupDetectorParams();
};

#endif // BALL_BLOB_DETECTOR_H