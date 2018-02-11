#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include <bits/stdc++.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("video/image", 1);
  
  cv::VideoCapture cap(*(argv+1));
  if(!cap.isOpened()){
    std::cout<<"Error opening file!";
    return -1;
  }
  cv::Mat frame;
  ros::Rate loop_rate(40);
  while(nh.ok()){
    cap>>frame;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
