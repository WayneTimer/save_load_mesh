#include "utils.h"

// 0:equ, -1:left<right, 1:left>right
int double_equ_check(double x,double y,double eps)
{
    double t;
    t = x-y;
    if (t<-eps) return -1;
    if (t>eps) return 1;
    return 0;
}

sensor_msgs::Image img2msg(cv::Mat& img, ros::Time& ros_stamp, string encoding)
{
    cv_bridge::CvImage cvimg;
    cvimg.header.stamp = ros_stamp;
    cvimg.encoding = encoding;
    cvimg.image = img;
    return *cvimg.toImageMsg();
}
