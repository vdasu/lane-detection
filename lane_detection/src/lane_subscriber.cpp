#include <bits/stdc++.h>
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "lane_detection/lane_detection.h"
#include "lane_detection/Coefficients.h"


class Callback{

public:

  Callback(ros::NodeHandle nh){

    image_transport::ImageTransport it(nh);
    sub = it.subscribe("video/image", 1, &Callback::imageCallback, this);
    pub = nh.advertise<lane_detection::Coefficients>("/coefficients", 1000);
  }

  void initMsg(lane_detection::Coefficients &msg, Line l){
    msg.flag = l.flag;
    msg.ml = l.ml;
    msg.cl = l.cl;
    msg.mr = l.mr;
    msg.cr=  l.cr;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& img){

    try{

      Line l;
      lane_detection::Coefficients msg;

      cv::Mat frame = cv_bridge::toCvShare(img, "bgr8")->image;
      lane_detect(frame, l);

      initMsg(msg, l);
      pub.publish(msg);

      cv::imshow("Image", frame);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e){

      ROS_ERROR("Could not convert from '%s' to 'bgr8'", img->encoding.c_str());
    }
    catch (cv::Exception& e){

      std::cout<<"Empty frame received. Video over.\n";
      exit(0);
    }
  }

private:

  image_transport::Subscriber sub;
  ros::Publisher pub;

};

int main(int argc, char **argv){

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  cv::namedWindow("Image");
  cv::startWindowThread();

  Callback obj(nh);

  ros::spin();

  return 0;
}