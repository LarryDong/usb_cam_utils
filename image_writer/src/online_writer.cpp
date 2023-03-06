
#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

// TODO: save images.

void imageCallback(const sensor_msgs::ImageConstPtr &msg){
    // ROS_INFO("Image callback");
    cv_bridge::CvImagePtr cv_ptr;
    try{
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
            cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
        else
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    }
    catch (cv_bridge::Exception &e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat img = cv_ptr->image;
    long int ts = cv_ptr->header.stamp.toNSec();
    
    imshow("src", img);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "image_writer");
	ros::NodeHandle nh;

    string output_folder;

    ros::param::get("~output_folder", output_folder);     // get videoX, X is defined in roslaunch file.

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImage = it.subscribe("/image", 10, imageCallback);

    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

