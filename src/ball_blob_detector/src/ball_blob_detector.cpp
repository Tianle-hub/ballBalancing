#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream> // For outputting coordinates to the console

int main(int argc, char **argv)
{

    int cameraIndex = 4;  // 0 for internal, 4 for usb camera in Tianle Laptop left

    cv::VideoCapture capture(cameraIndex); // Open the camera with the found index
    std::cout<<"1111";
    if (!capture.isOpened()) {
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    }

    // cv::VideoCapture capture(1);  // 1 for usb, 0 for internal camera
    // if (!capture.isOpened()) {
    //     std::cerr << "Error opening video stream" << std::endl;
    //     return -1;
    // }

    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 10000;
    params.filterByCircularity = true;
    params.minCircularity = 0.1;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    cv::Mat frame;

    int frameSkip = 10; // Only process every 10th frame
    int frameCount = 0;
    while (true) {
        

        capture >> frame;
        std::cout<<frame.channels();
        if (frame.empty())
            break;

        // // Resize frame
        // cv::Mat resizedFrame;
        // float scale = 1; // Example scale factor to reduce resolution
        // cv::resize(frame, resizedFrame, cv::Size(), scale, scale);

        if (frameCount % frameSkip == 0) {
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

            if (!keypoints.empty()) {
                // Sort keypoints by size in descending order
                std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                    return a.size > b.size;
                });

                // Consider only the largest blob
                std::vector<cv::KeyPoint> largestBlob;
                largestBlob.push_back(keypoints[0]);

                // Output the coordinates of the largest blob
                // std::cout << "Blob coordinates: " << largestBlob[0].pt.x << ", " << largestBlob[0].pt.y << std::endl;

                // Draw detected blob as a red circle on the original frame
                cv::Mat im_with_keypoints;
                drawKeypoints(frame, largestBlob, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                // Show the result
                cv::imshow("Red Ball Detection", im_with_keypoints);
            }
        }

        frameCount++;
        if (cv::waitKey(30) >= 0)
            break;
    }

    capture.release();
    cv::destroyAllWindows();

    return 0;
}
