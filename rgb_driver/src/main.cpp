
#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <thread>       // thread for reading to clean the buffer.
#include <mutex>

using namespace std;
using namespace cv;

std::mutex g_mutex;
Mat g_newest_frame;

void read_video_buffer(VideoCapture& cap){
    cout << "--> New thread begin." << endl;
    if(!cap.isOpened()){
        cout << "[Error]. No cap found." << endl;
        return ;
    }
    while(1){                   // always retriving image using "grab" in a new thread.
        bool res = cap.grab();
        g_mutex.lock();
        if(res){
            cap.retrieve(g_newest_frame);
        }
        g_mutex.unlock();
    }
}

int main(int argc, char **argv){
    // google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "rgb_driver");
	ros::NodeHandle nh;
    int video_port = 0;
    int fps = 10;
    double exposure = 0;
    double gain = 0;                                // not implemented
    bool show_image = false;

    ros::param::get("~video_port", video_port);     // get videoX, X is defined in roslaunch file.
    ros::param::get("~fps", fps);                   // set FPS.
    ros::param::get("~exposure", exposure);         // set exposure time. unit: s.
    int output_rate = 100;
    ros::param::get("~output_rate", output_rate);         // set exposure time. unit: s.
    ros::param::get("~show_image", show_image);         // show image or not.

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pubImage = it.advertise("/image", 1);

    ROS_INFO_STREAM("Open camera from: /dev/video" << video_port);
    VideoCapture cap(video_port);
    while(!cap.isOpened()){
        ROS_WARN("Cannot open camera...");
        sleep(1);
    }
    ROS_INFO("Camera opened. Setting camera properties...");
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1280);
    cap.set(cv::CAP_PROP_FPS, fps);
    if(exposure == 0){
        ROS_INFO("Using auto-exposure");
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, -1);
    }
    else{
        // ROS_INFO_STREAM("Set exposure time: " << exposure << "s.");
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
        cap.set(cv::CAP_PROP_EXPOSURE, exposure);
    }
    int actual_fps = cap.get(cv::CAP_PROP_FPS);
    if(actual_fps != fps){
        ROS_WARN_STREAM("FPS not setted correctly. Set: " << fps << ", actual: " << actual_fps);
    }
    if(actual_fps < output_rate){
        ROS_WARN_STREAM("image publish rate (" <<output_rate<<") is larger than actual fps ("<<actual_fps<<"), which may contain duplicated frames");
    }

    ROS_INFO("Begin to publish rgb image...");

    cout << "------------------------------ Settings ------------------------------" << endl;
    cout << "Image size: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << ", " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "Exposure time: " << cap.get(cv::CAP_PROP_EXPOSURE) << "s. (Setting: " << to_string(exposure) << ")." << endl;
    cout << "FPS: " << cap.get(cv::CAP_PROP_FPS) << ". (Settings " << to_string(fps) << "). " << endl;
    cout << "Gain: " << cap.get(cv::CAP_PROP_GAIN) << endl;
    cout << "------------------------------ Settings ------------------------------" << endl;

    thread t(read_video_buffer,std::ref(cap));   // start a new treading for reading.
    
    ROS_INFO_STREAM("Publish image topic rate (MAX): " << output_rate);
    ros::Rate r(output_rate);          // publish the topic by output-rate
    while (ros::ok()) {
        g_mutex.lock();
        Mat src = g_newest_frame.clone();
        g_mutex.unlock();
        if(!src.empty()){
            std_msgs::Header hd;
            hd.stamp = ros::Time::now();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(hd, "bgr8", src).toImageMsg();
            pubImage.publish(msg);

            if(show_image){
                cv::imshow("video"+to_string(video_port), src);
                int key = waitKey(1);
                if (key == 'q'){
                    ROS_INFO("Stop viewing image.");
                    show_image = false;
                    cv::destroyWindow("video"+to_string(video_port));
                }
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

