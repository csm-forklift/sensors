/**
 *  This node allows you to receive raw IMU data and publishes a transform to
 *  allow you to view it as a frame in RVIZ.
 */

 #include <ros/ros.h>
 #include <tf/transform_broadcaster.h>
 #include <sensor_msgs/Imu.h>
 #include <string>

class ImuViewer
{
private:
    // ROS Objects
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub;
    tf::TransformBroadcaster imu_link_broadcaster;
    std::string imu_topic;
    std::string origin_frame;
    std::string imu_frame;

public:
    ImuViewer() : nh_("imu_viewer")
    {
        // Grab parameters
        nh_.param<std::string>("imu_topic", imu_topic, "/arduino/imu");
        nh_.param<std::string>("origin_frame", origin_frame, "origin");
        nh_.param<std::string>("imu_frame", imu_frame, "base_link");

        // Set up subcriber
        imu_sub = nh_.subscribe<sensor_msgs::Imu>(imu_topic.c_str(), 10, &ImuViewer::imuCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu msg)
    {
        // Broadcast transform for odom to encoder's predicted base_link
        imu_link_broadcaster.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                          tf::Vector3(0, 0, 0)),
            ros::Time::now(),
            origin_frame.c_str(),
            imu_frame.c_str()
        ));
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_viewer");
    ImuViewer imu_viewer;
    ros::spin();

    return 0;
}
