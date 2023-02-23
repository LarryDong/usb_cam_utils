
#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// #include <gflags/gflags.h>

using namespace std;
using namespace cv;


int main(int argc, char **argv){
    // google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "rgb_driver");
	ros::NodeHandle nh;
    int video_port = 0;
    ros::param::get("~video_port", video_port);     // get videoX, X is defined in roslaunch file.
    ROS_INFO_STREAM("Open camera from: /dev/video" << video_port);

    bool show_image = false;
    ros::param::get("~show_image", show_image);     // show image.
    
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pubImage = it.advertise("/image", 10);

    VideoCapture cap(video_port);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1280);
    while(!cap.isOpened()){
        ROS_WARN("Cannot open camera...");
        sleep(1);
    }
    ROS_INFO("Begin to publish rgb image...");

    ROS_INFO_STREAM("Image size: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << ", " << cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    ros::Rate r(100);
    while (ros::ok()) {
        Mat src;
        cap >> src;
        if(show_image){
            cv::imshow("video"+to_string(video_port), src);
            int key = waitKey(1);
            if (key == 'q')
                return 0;
        }
        std_msgs::Header hd;
        hd.stamp = ros::Time::now();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(hd, "bgr8", src).toImageMsg();
        pubImage.publish(msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
