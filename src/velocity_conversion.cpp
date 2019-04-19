/**
 *  This node converts the linear velocity and steering angle into a Twist
 *  message by calculating the angular velocity.
 */


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <cmath>


class VelocityConversion
{
private:
    //----- ROS Objects
    ros::NodeHandle nh_;
    ros::Subscriber velocity_sub; // grabs velocity in m/s
    ros::Subscriber steering_angle_sub; // grabs steering angle in rad
    ros::Publisher twist_pub; // publishes combined linear and angular velocities

    //----- Parameters
    // Forklift Dimensions
    double forklift_length; // length of wheelbase {m}
    double length_to_base; // length from front axle to base_link frame {m}

    // Motion variables
    double velocity; // m/s
    double angle; // rad
    geometry_msgs::Twist twist;

public:
    VelocityConversion() : nh_("~")
    {
        // Set up ROS connections
        velocity_sub = nh_.subscribe("/velocity_node/velocity", 10, &VelocityConversion::velocityCallback, this);
        steering_angle_sub = nh_.subscribe("/steering_node/filtered_angle", 10, &VelocityConversion::angleCallback, this);
        twist_pub = nh_.advertise<geometry_msgs::Twist>("~/twist", 10);

        // Set parameters
        nh_.param<double>("forklift_length", forklift_length, 1);
        nh_.param<double>("length_to_base", length_to_base, 0);

        // Set initial state
        velocity = 0;
        angle = 0;
    }

    void velocityCallback(const std_msgs::Float32 msg)
    {

    }

    void angleCallback(const std_msgs::Float32 msg)
    {

    }

    geometry_msgs::Twist calculateTwist(const double velocity, const double angle)
    {
        // Calculate the linear velocity at the base_link
        double v_b = velocity*sqrt(1 + pow((length_to_base/forklift_length)*tan(angle), 2));
        double theta_dot = velocity*(tan(angle)/forklift_length);

        geometry_msgs::Twist forklift_twist;
        forklift_twist.linear.x = v_b;
        forklift_twist.angular.z = theta_dot;

        return forklift_twist;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_conversion");
    VelocityConversion velocity_conversion();
    ros::spin();

    return 0;
}
