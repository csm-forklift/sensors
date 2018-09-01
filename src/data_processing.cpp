/*
 * This Node grabs the odometry data from /odom/filtered and stores the (x,y)
 * position in a .csv file which can then be used for plotting the path and
 * error in Matlab or Python.
 *
 * It is intended to be run before playing a ".bag" file with the desired data.
 * It waits for the topic to begin publishing and then starts storing the data.
 * After the topic has stopped publishing for ~5 seconds it closes the file.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

// Global variables for saving data to file
std::string folder_path = "/home/turtlebot/Desktop/odometry_testing/";
std::string filename = "fb_22ft_enc_odomdiff_f_prelim2.csv";
std::ofstream datafile;
bool collecting_data = false;
double max_time = 3; // seconds between last message before triggering shutdown
time_t last_msg; // time of last message

// FIXME: For handling shutdown stuff: https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
// // Signal-safe flag for whether shutdown is requested
// sig_atomic_t volatile g_request_shutdown = 0;
//
// // Replacement SIGINT handler
// void mySigIntHandler(int sig)
// {
//   g_request_shutdown = 1;
// }

void odomCallback(nav_msgs::OdometryConstPtr msg)
{
    if (collecting_data == false) {
        ROS_INFO("First message received, beginning data conversion");
        collecting_data = true;
    }
    datafile << msg->pose.pose.position.x << ",";
    datafile << msg->pose.pose.position.y << "\n";
    last_msg = time(0);
}

bool checkTime() {
    double elapsed = difftime(time(0), last_msg);
    if (elapsed >= max_time) {
        return true;
    }
    else {
        return false;
    }
}

int main(int argc, char** argv)
{
    // Start ROS node
    ros::init(argc, argv, "data_processing", ros::init_options::NoSigintHandler);

    ROS_INFO("data_processing node started, waiting for .bag file\n");

    ros::NodeHandle nh;

    // Set up .csv file for data
    filename = folder_path + filename;
    std::cout << "file path: " << filename << std::endl;
    datafile.open(filename.c_str());

    // Start subscriber after setting up .csv file
    ros::Subscriber odom_sub;
    odom_sub = nh.subscribe("odom/filtered", 10, odomCallback);

    ros::Rate rate(10);

    while (ros::ok()) {

        //std::cout << "Time since last messsage: " << difftime(time(0), last_msg) << "\n";

        // Check how long it has been since /odom/filtered stopped publishing
        // If it's past time, close the file and shutdown this node
        if (collecting_data && checkTime()) {
            ROS_INFO("Timeout, closing data file.");
            datafile.close();
            ros::shutdown();
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
