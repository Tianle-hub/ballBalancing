#include "ros/ros.h"
#include "std_msgs/String.h"
#include<ball_blob_detector/ball_blob_detector.h>
#include "ball_blob_detector/PosVel2D.h"
#include <sstream>


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ballBlobDetector");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  // Use the appropriate camera index 
  // 0 for internal, 4 for usb camera in Tianle Laptop left
  // 0 for PC in ics, top usb
  int cameraIndex = 4;  
  BallBlobDetector blob_detector(cameraIndex); 
  
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher ball_pub = nh.advertise<ball_blob_detector::PosVel2D>("ball_pos_vel", 1000);
  ball_blob_detector::PosVel2D pv_msg;

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    // std_msgs::String msg;
    std::vector<cv::KeyPoint> Blob = blob_detector.processFrame();
    if (Blob.empty() || cv::waitKey(30) >= 0){
        blob_detector.capture.release();
        cv::destroyAllWindows();
    }
    pv_msg.position.x = Blob[0].pt.x;
    pv_msg.position.y = Blob[0].pt.y;
    pv_msg.velocity.linear.x = 0.5;
    pv_msg.velocity.linear.y = -0.5;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    ball_pub.publish(pv_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}