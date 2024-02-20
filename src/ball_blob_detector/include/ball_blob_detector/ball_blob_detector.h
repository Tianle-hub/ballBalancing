#ifndef BALL_BLOB_DETECTOR_H
#define BALL_BLOB_DETECTOR_H


#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

class BallBlobDetector {
public:
    BallBlobDetector(int cameraIndex);
    ~BallBlobDetector();
    double getPosXPrev() const { return pos_x_prev; }
    double getPosYPrev() const { return pos_y_prev; }
    double getTimePrev() const { return time_prev; }

    void setPosXPrev(double x) { pos_x_prev = x; }
    void setPosYPrev(double x) { pos_y_prev = x; }
    void setTimePrev(double x) { time_prev = x; }
    void run();
    cv::VideoCapture capture;
    std::vector<cv::KeyPoint> processFrame();

private:
    cv::Ptr<cv::SimpleBlobDetector> detector;
    void setupDetectorParams();
    double pos_x_prev = 0;
    double pos_y_prev = 0;
    double time_prev = 0;
};

#endif // BALL_BLOB_DETECTOR_H