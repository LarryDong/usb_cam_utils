
#include <iostream>
#include <fstream>
// #include <boost/foreach.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;



int main(int argc, char **argv){

    ros::init(argc, argv, "offline_writer");
	ros::NodeHandle nh;

    string input_bag("test.bag");
    string output_folder("/home/larrydong/imu_ws/output");
    string image_topic("/image");

    ros::param::get("~input_bag", input_bag);               //
    ros::param::get("~image_topic", image_topic);               //
    ros::param::get("~output_folder", output_folder);       // get videoX, X is defined in roslaunch file.
    
    rosbag::Bag bag;
    bag.open(input_bag, rosbag::bagmode::Read);
    if(!bag.isOpen()){
        ROS_ERROR_STREAM("Cannot open bag: " << input_bag);
    }

    vector<double> timestamps;
    vector<Mat> images;

    std::vector<std::string> topics;
    topics.push_back(image_topic);
    ROS_INFO_STREAM("Begin to extract: " << image_topic << " from: " << input_bag);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (rosbag::MessageInstance const m : view){
        if(m.getTopic() == image_topic){
            cv_bridge::CvImagePtr cv_ptr;
            sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
            try{
                if (sensor_msgs::image_encodings::isColor(msg->encoding))
                    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                else
                    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
            }
            catch (cv_bridge::Exception &e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
            Mat img = cv_ptr->image;
            long ts = cv_ptr->header.stamp.toNSec();
            images.push_back(img);
            timestamps.push_back(ts);
        }
        else{
            cerr << "unpected topics" << endl;
        }
    }

    string ts_filename = output_folder + "/ts.csv";
    string image_folder = output_folder + "/image";
    ROS_INFO_STREAM("Saving images and ts to: " << image_folder);
    for (int i = 0; i < images.size(); ++i){
        Mat img = images[i];
        long ts = timestamps[i];
        string image_filename = image_folder + "/" + to_string(ts) + ".bmp";
        imwrite(image_filename, img);
    }
    ROS_INFO_STREAM("Saved " << images.size() << " images.");

    return 0;
}