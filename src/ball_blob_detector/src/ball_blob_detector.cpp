#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char **argv)
{
    cv::VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cerr << "Error opening video stream" << std::endl;
        return -1;
    }

    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 10000;
    params.filterByCircularity = true;
    params.minCircularity = 0.1;
    // Uncomment if you want to consider the blobs' color in detection
    // params.filterByColor = true;
    // params.blobColor = 255; // Assuming white blobs on the red mask

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    cv::Mat frame;

    while (true) {
        capture >> frame;
        if (frame.empty())
            break;

        // Convert to HSV color space
        cv::Mat hsvImage;
        cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

        // Threshold the HSV image to get only red colors
        cv::Mat redMask1, redMask2, redMask;
        cv::inRange(hsvImage, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), redMask1);
        cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), redMask2);
        cv::bitwise_or(redMask1, redMask2, redMask);

        // Morphological operations
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernel);

        // Detect blobs on the red mask
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(redMask, keypoints);

        // Draw detected blobs as red circles on the original frame
        cv::Mat im_with_keypoints;
        drawKeypoints(frame, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Show the result
        cv::imshow("Red Ball Detection", im_with_keypoints);

        if (cv::waitKey(30) >= 0)
            break;
    }

    capture.release();
    cv::destroyAllWindows();

    return 0;
}
