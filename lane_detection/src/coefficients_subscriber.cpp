#include <bits/stdc++.h>
#include "ros/ros.h"
#include "lane_detection/Coefficients.h"


void msgCallback(const lane_detection::Coefficients& msg){

	if(!msg.flag){

		std::cout<<"LANE NOT FOUND\n";
	}else{
		
		std::cout<<"LEFT LANE: y = "<<msg.ml<<"x + "<<msg.cl<<"\n";
		std::cout<<"RIGHT LANE: y = "<<msg.mr<<"x + "<<msg.cr<<"\n";
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "coeff_subscriber");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/coefficients", 1000, msgCallback);

	ros::spin();

	return 0;
}