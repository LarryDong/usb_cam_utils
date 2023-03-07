
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;



int main(int argc, char **argv){

    ros::init(argc, argv, "offline_imu_writer");
	ros::NodeHandle nh;
    
    ROS_WARN("IMU");

    string input_bag("test.bag");
    string output_folder("/home/larrydong/imu_ws/output");
    string image_topic("/image");
    string imu_topic("/imu/data");

    ros::param::get("~input_bag", input_bag);               //
    ros::param::get("~image_topic", image_topic);               //
    ros::param::get("~imu_topic", imu_topic);               //
    ros::param::get("~output_folder", output_folder);       // get videoX, X is defined in roslaunch file.
    
    rosbag::Bag bag;
    bag.open(input_bag, rosbag::bagmode::Read);
    if(!bag.isOpen()){
        ROS_ERROR_STREAM("Cannot open bag: " << input_bag);
    }

    vector<double> image_timestamps, imu_timestamps;
    vector<Mat> images;
    vector<geometry_msgs::Vector3> accs, gyros;

    std::vector<std::string> topics;
    topics.push_back(image_topic);
    topics.push_back(imu_topic);
    ROS_INFO_STREAM("Begin to extract image topic: " << image_topic);
    ROS_INFO_STREAM("Begin to extract imu topic: " << imu_topic);

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
            image_timestamps.push_back(ts);
        }
        else if(m.getTopic() == imu_topic){
            sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();
            auto gyro = msg->angular_velocity;
            auto acc = msg->linear_acceleration;
            accs.push_back(acc);
            gyros.push_back(gyro);
            imu_timestamps.push_back(msg->header.stamp.toNSec());
            // cout << "gyro: " << gyro.x << ", " << gyro.y << ", " << gyro.z << endl;
            // cout << "acc : " << acc.x << ", " << acc.y << ", " << acc.z << endl;
        }
        else{
            cerr << "unpected topics" << endl;
            std::abort();
        }
    }

    string image_ts_filename = output_folder + "/ts.csv";
    string image_folder = output_folder + "/image";
    string imu_filename = output_folder + "/imu.csv";
    ROS_INFO_STREAM("Saving images, image_ts, and imu...");
    
    // save image and image_ts
    ofstream of(image_ts_filename);
    for (int i = 0; i < images.size(); ++i){
        Mat img = images[i];
        long ts = image_timestamps[i];
        string image_filename = image_folder + "/" + to_string(ts) + ".bmp";
        imwrite(image_filename, img);
        of << ts << endl;
    }
    of.close();
    ROS_INFO_STREAM("Saved " << images.size() << " images to: " << image_folder);
    ROS_INFO_STREAM("Saved " << image_timestamps.size() << " image_ts to: " << image_ts_filename);

    // save IMU.
    of.open(imu_filename);
    of << "# ts [ns],wx [rad s^-1],wy [rad s^-1],wz [rad s^-1],ax [m s^-2],ay [m s^-2],az [m s^-2]" << endl;
    for (int i = 0; i < accs.size(); ++i){
        auto acc = accs[i];
        auto gyro = gyros[i];
        long ts = imu_timestamps[i];
        of << ts << "," << gyro.x << "," << gyro.y << "," << gyro.z << "," << acc.x << "," << acc.y << "," << acc.z << endl;
    }
    of.close();
    ROS_INFO_STREAM("Saved "<< imu_timestamps.size()<< " imu to: " << imu_filename);

    ROS_WARN("Done without error.");
    return 0;
}

