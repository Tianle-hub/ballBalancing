#include <ball_blob_detector/ball_blob_detector.h>

BallBlobDetector::BallBlobDetector(int cameraIndex)
{
    capture.open(cameraIndex);
    if (!capture.isOpened())
    {
        std::cerr << "Error opening video stream or file" << std::endl;
        exit(-1);
    }
    setupDetectorParams();
}

BallBlobDetector::~BallBlobDetector()
{
    capture.release();
    cv::destroyAllWindows();
}

void BallBlobDetector::setupDetectorParams()
{
    cv::SimpleBlobDetector::Params params;
    // Initialize parameters...
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 50000;
    params.filterByCircularity = true;
    params.filterByColor = true;
    params.blobColor = 255;
    params.minCircularity = 0.1;

    detector = cv::SimpleBlobDetector::create(params);
}

Eigen::Vector2d BallBlobDetector::plateCenterDetection()
{

    cv::Mat frame;
    // Capture frame-by-frame
    capture >> frame;

    // If the frame is empty, break immediately
    if (frame.empty())
        return Eigen::Vector2d::Zero();

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    // Detect edges using Canny
    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 150, 3);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (const auto &contour : contours)
    {
        // Approximate the contour to a polygon
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(contour, polygon, cv::arcLength(contour, true) * 0.02, true);

        // Check if the polygon can be considered a rectangle
        if (polygon.size() == 4)
        {
            // Calculate the center of the rectangle
            cv::Moments m = cv::moments(contour);
            centerX = (int)(m.m10 / m.m00);
            centerY = (int)(m.m01 / m.m00);

            // Optionally, draw the contour and a circle at the center
            cv::drawContours(frame, std::vector<std::vector<cv::Point>>{polygon}, 0, cv::Scalar(0, 255, 0), 3);
            cv::circle(frame, cv::Point(centerX, centerY), 5, cv::Scalar(255, 0, 0), -1);
        }
    }
    // Display the resulting frame
    cv::imshow("Frame", frame);
    return Eigen::Vector2d(centerX, centerY);
}

void BallBlobDetector::setupPlateCoordinate()
{
    // capture.retrieve(frame);
    capture >> frame;
    frame.copyTo(imageCopy);
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    // If markers are detected, draw them on the image
    if (!markerIds.empty())
    {
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSideLength, cameraMatrix, distCoeffs, rvecs, tvecs);
        if (markerIds.size() == 4)
        {
            ROS_INFO("Four Marker detected");

            // Be careful!! Use markerIds[i] as index
            // The sequence will not follow 0123 order
            // Marker sequence
            // 0 ------ 1
            // |        |
            // |        |
            // 2 ------ 3

            // Corner sequence
            // 0 ------ 1
            // |        |
            // |        |
            // 3 ------ 2

            CenterLength = abs(tvecs[1][0] - tvecs[0][0]);
            CenterWidth = abs(tvecs[0][1] - tvecs[2][1]);

            // Get marker center in pixel

            for (size_t i = 0; i < markerCorners.size(); i++)
            {
                float centerX = 0.0, centerY = 0.0;
                for (size_t j = 0; j < markerCorners[i].size(); j++)
                {
                    centerX += markerCorners[i][j].x;
                    centerY += markerCorners[i][j].y;
                }
                centerX /= markerCorners[i].size();
                centerY /= markerCorners[i].size();
                MarkerPixelCenter(markerIds[i], 0) = centerX;
                MarkerPixelCenter(markerIds[i], 1) = centerY;

                // std::cout << "Marker ID: " << markerIds[i] << " Center: (" << centerX << ", " << centerY << ")" << std::endl;
            }
            // std::cout << "center distance estimate:" << CenterLength << ", " << CenterWidth << std::endl;
            // std::cout << "corner center coordinate in pixel:\n" << MarkerPixelCenter << std::endl;
            // Marker sequence
            // 0 ------ 1
            // |        |
            // |        |
            // 2 ------ 3

            PlatePixelCenter<<(MarkerPixelCenter(0,0)+MarkerPixelCenter(1,0))/2, (MarkerPixelCenter(2,1)+MarkerPixelCenter(0,1))/2;
            PlatePixelSize<< MarkerPixelCenter(1,0)-MarkerPixelCenter(0,0), MarkerPixelCenter(2,1)-MarkerPixelCenter(0,1);
            // std::cout << "plate center coordinate in pixel:\n" << PlatePixelCenter << std::endl;
            k_pixel_length = CenterLength/ PlatePixelSize(0,0);
            k_pixel_width = CenterWidth/ PlatePixelSize(1,0);
            std::cout << "k_pixel: " << k_pixel_length << ", " << k_pixel_width << std::endl;

            // Draw axis for each marker
            for (int i = 0; i < markerIds.size(); i++)
            {
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerSideLength * 0.5f);
                // std::cout<<"markerIds:"<<markerIds[i]<<std::endl;
                // std::cout << "Rotation Vector (rvecs): [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2] << "]" << std::endl;
                // std::cout << "Translation Vector (tvecs): [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]" << std::endl;
                // std::cout << "Corner0" << ": (" << markerCorners[i][0].x << ", " << markerCorners[i][0].y << ")" << std::endl;
            }

            // judge whether plate length is reasonable
            // !! Change bounds when plate is changed
            // inside four marker logic
            if (CenterLength >= plate_length_lower_bound 
            && CenterLength <= plate_length_upper_bound 
            && CenterWidth >= plate_width_lower_bound 
            && CenterWidth <= plate_width_upper_bound 
            && calibration)
            {
                ROS_INFO("Calibration done");
                calibration = false;   // once turned false, never true, blobDetect activate
                PlatePixelCenter_fixed<<PlatePixelCenter(0,0), PlatePixelCenter(1,0);
                k_pixel_length_fixed = k_pixel_length;
                k_pixel_width_fixed = k_pixel_width;
            }

            if (abs(PlatePixelCenter_fixed(0,0) - PlatePixelCenter(0,0)) >= 10 ||
                abs(PlatePixelCenter_fixed(1,0) - PlatePixelCenter(1,0)) >= 10 ){
                
                ROS_INFO("Calibration updated ");
                // update pixel center 
                PlatePixelCenter_fixed<<PlatePixelCenter(0,0), PlatePixelCenter(1,0);
                k_pixel_length_fixed = k_pixel_length;
                k_pixel_width_fixed = k_pixel_width;
            }

        }// marker 4
        
    } // !markerId.empty
    if (calibration) cv::imshow("Detected ArUco markers", imageCopy);
}

std::vector<cv::KeyPoint> BallBlobDetector::processFrame()
{
    // cv::Mat frame;
    // Process frame...
    capture >> frame;
    if (frame.empty()){
        measure = false;
        return std::vector<cv::KeyPoint>();
    }


    int width = frame.cols;  // 640
    int height = frame.rows; // 480
    // std::cout << "Image size: " <<width<<","<<height<<std::endl;

    cv::Mat hsvImage, ycrcbImage;

    // Convert the image from BGR to HSV color space
    cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

    // // Convert the same image from BGR to YCrCb color space
    // cv::cvtColor(frame, ycrcbImage, cv::COLOR_BGR2YCrCb);

    // Threshold the HSV image to get only red colors
    cv::Mat redMask1, redMask2, redMask_hsv;
    // tight values : hue, saturation ,luminance value
    cv::inRange(hsvImage, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), redMask1);
    cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), redMask2);

    // loose values: 
    // cv::inRange(hsvImage, cv::Scalar(0, 120, 60), cv::Scalar(10, 255, 255), redMask1);
    // cv::inRange(hsvImage, cv::Scalar(170, 120, 60), cv::Scalar(180, 255, 255), redMask2);


    cv::Mat maskYellowOrange;
    // Adjust the hue values to target yellow to orange.
    // These values might need to be fine-tuned based on your specific lighting conditions and the exact color of the ball.
    cv::inRange(hsvImage, cv::Scalar(20, 80, 100), cv::Scalar(60, 255, 255), maskYellowOrange);

    cv::Mat maskOrange;
    cv::inRange(hsvImage, cv::Scalar(10, 100, 20), cv::Scalar(22, 255, 255), maskOrange);

    // cv::imshow("maskOrange", maskOrange);
    cv::bitwise_or(redMask1, redMask2, redMask_hsv);

    // cv::Mat maskedFrame;
    // // Threshold for red color in YCrCb space
    // cv::Mat redMask_YCrCb;
    // cv::inRange(ycrcbImage, cv::Scalar(0, 133, 77), cv::Scalar(255, 173, 127), redMask_YCrCb);

    // cv::Mat red_combinedMask;
    // cv::bitwise_and(redMask_hsv, redMask_YCrCb, red_combinedMask);

    // Morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(redMask_hsv, redMask_hsv, cv::MORPH_CLOSE, kernel); // maskYellowOrange, redMask_hsv
    cv::morphologyEx(redMask_hsv, redMask_hsv, cv::MORPH_OPEN, kernel);  // maskYellowOrange, redMask_hsv

    // Detect blobs on the red mask
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(redMask_hsv, keypoints); // maskYellowOrange, redMask_hsv
    std::vector<cv::KeyPoint> largestBlob;

    cv::imshow("mask_yellow", redMask_hsv);
    if (!keypoints.empty())
    {
        // Sort keypoints by size in descending order
        std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b)
                  { return a.size > b.size; });

        // Consider only the largest blob
        largestBlob.push_back(keypoints[0]);
        float largestBlobRadius = largestBlob[0].size;
        

        // Draw detected blob as a red circle on the original frame
        cv::Mat im_with_keypoints;
        drawKeypoints(frame, largestBlob, im_with_keypoints, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Show the result
        cv::imshow("Red Ball Detection", im_with_keypoints);
        // Output the coordinates of the largest blob

        // Ball 
            // ------>x (0, size_x)
            // | 
            // |        
            // |
            // v
            // y (0, size_y)

        // std::cout << "Blob coordinates(pixel): " << largestBlob[0].pt.x << ", " << largestBlob[0].pt.y << std::endl;
        largestBlob[0].pt.x = double(largestBlob[0].pt.x - PlatePixelCenter_fixed(0,0))*k_pixel_length_fixed;
        largestBlob[0].pt.y = double(largestBlob[0].pt.y - PlatePixelCenter_fixed(1,0))*k_pixel_width_fixed;
        std::cout << "Blob coordinates(meter): " << largestBlob[0].pt.x<< ", " << largestBlob[0].pt.y << std::endl;
        // std::cout << "largestBlobRadius:" << largestBlobRadius << std::endl;
        // std::cout << "plate center coordinate in pixel:\n" << PlatePixelCenter << std::endl;
        // std::cout << "plate size estimate:" << CenterLength << ", " << CenterWidth << std::endl;

        measure = true;
        return largestBlob;
    }
    measure = false;
    cv::KeyPoint kp1(0, 0, 1, 45);
    largestBlob.push_back(kp1);
    return largestBlob;
}

void BallBlobDetector::run()
{
    
    while (true)
    {   
        setupPlateCoordinate();
        if(!calibration){
            Blob = processFrame();
            if (Blob.empty()) break;
        }
                    
        // Eigen::Vector2d center = plateCenterDetection();

        if (cv::waitKey(30) >= 0)
            break;
        

    }
    // capture.release();
    // cv::destroyAllWindows();
}

int main(int argc, char **argv)
{
    // Use the appropriate camera index
    // 0 for internal, 4 for usb camera in Tianle Laptop left
    // 0 for PC in ics, top usb
    int cameraIndex = 4;

    BallBlobDetector blob_detector(cameraIndex);
    blob_detector.run();
    return 0;
}