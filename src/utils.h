#ifndef utils_h
#define utils_h

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <string>
#include <cv_bridge/cv_bridge.h>

using namespace std;

#define DOUBLE_EPS 1e-6

int double_equ_check(double x,double y,double eps=DOUBLE_EPS);
sensor_msgs::Image img2msg(cv::Mat& img, ros::Time& ros_stamp, string encoding);

#endif
