sudo apt-get update
sudo apt-get install libopencv-dev

## TO FIND camera index
sudo apt-get update
sudo apt-get install v4l-utils
v4l2-ctl --list-devices

## Test code
rosrun ball_blob_detector ball_blob_detector_node

## FILES
ball_blob_detector_test   non-object for testing
ball_blob_detector  object way