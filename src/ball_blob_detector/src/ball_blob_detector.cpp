#include<ball_blob_detector/ball_blob_detector.h>

BallBlobDetector::BallBlobDetector(int cameraIndex) {
    capture.open(cameraIndex);
    if (!capture.isOpened()) {
        std::cerr << "Error opening video stream or file" << std::endl;
        exit(-1);
    }
    setupDetectorParams();
}

BallBlobDetector::~BallBlobDetector() {
    capture.release();
    cv::destroyAllWindows();
}

void BallBlobDetector::setupDetectorParams() {
    cv::SimpleBlobDetector::Params params;
    // Initialize parameters...
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 10000;
    params.filterByCircularity = true;
    params.filterByColor = true;
    params.blobColor = 255;
    params.minCircularity = 0.1;

    detector = cv::SimpleBlobDetector::create(params);
}

std::vector<cv::KeyPoint> BallBlobDetector::processFrame() {
    cv::Mat frame;
    // Process frame...
    capture >> frame;
    if (frame.empty())
        return std::vector<cv::KeyPoint>();
    cv::Mat hsvImage, ycrcbImage;

    // Convert the image from BGR to HSV color space
    cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

    // // Convert the same image from BGR to YCrCb color space
    // cv::cvtColor(frame, ycrcbImage, cv::COLOR_BGR2YCrCb);

    // Threshold the HSV image to get only red colors
    cv::Mat redMask1, redMask2, redMask_hsv;
    cv::inRange(hsvImage, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), redMask1);
    cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), redMask2);

    cv::Mat maskYellowOrange;
    // Adjust the hue values to target yellow to orange. 
    // These values might need to be fine-tuned based on your specific lighting conditions and the exact color of the ball.
    cv::inRange(hsvImage, cv::Scalar(15, 70, 50), cv::Scalar(30, 255, 255), maskYellowOrange);


    cv::bitwise_or(redMask1, redMask2, redMask_hsv);

    // // Threshold for red color in YCrCb space
    // cv::Mat redMask_YCrCb;
    // cv::inRange(ycrcbImage, cv::Scalar(0, 133, 77), cv::Scalar(255, 173, 127), redMask_YCrCb);
    
    // cv::Mat red_combinedMask;
    // cv::bitwise_and(redMask_hsv, redMask_YCrCb, red_combinedMask);

    // Morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(redMask_hsv, redMask_hsv, cv::MORPH_CLOSE, kernel);  // maskYellowOrange, redMask_hsv
    cv::morphologyEx(redMask_hsv, redMask_hsv, cv::MORPH_OPEN, kernel);   // maskYellowOrange, redMask_hsv

    // Detect blobs on the red mask
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(redMask_hsv, keypoints);   // maskYellowOrange, redMask_hsv
    std::vector<cv::KeyPoint> largestBlob;
    if (!keypoints.empty()) {
        // Sort keypoints by size in descending order
        std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
            return a.size > b.size;
        });

        // Consider only the largest blob
        largestBlob.push_back(keypoints[0]);

        // Output the coordinates of the largest blob
        // std::cout << "Blob coordinates: " << largestBlob[0].pt.x << ", " << largestBlob[0].pt.y << std::endl;
        
        // Draw detected blob as a red circle on the original frame
        cv::Mat im_with_keypoints;
        drawKeypoints(frame, largestBlob, im_with_keypoints, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Show the result
        cv::imshow("Red Ball Detection", im_with_keypoints);
        return largestBlob;
    }
    cv::KeyPoint kp1(0, 0, 1, 45);
    largestBlob.push_back(kp1);
    return largestBlob;
}

void BallBlobDetector::run() {
    while (true) {
        std::vector<cv::KeyPoint> Blob = processFrame();
        if (Blob.empty()) break;
        if (cv::waitKey(30) >= 0) break;
    }
    // capture.release();
    // cv::destroyAllWindows();
}

int main(int argc, char **argv) {
    // Use the appropriate camera index 
    // 0 for internal, 4 for usb camera in Tianle Laptop left
    // 0 for PC in ics, top usb
    int cameraIndex = 4;  

    BallBlobDetector blob_detector(cameraIndex); 
    blob_detector.run();
    return 0;
}