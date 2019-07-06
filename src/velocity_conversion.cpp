/**
 *  This node converts the linear velocity and steering angle into a Twist
 *  message by calculating the angular velocity.
 */


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <string>


class VelocityConversion
{
private:
    //----- ROS Objects
    ros::NodeHandle nh_;
    ros::Subscriber velocity_sub; // grabs velocity in m/s
    ros::Subscriber steering_angle_sub; // grabs steering angle in rad
    ros::Publisher twist_with_cov_pub; // publishes combined linear and angular velocities with covariance
    ros::Publisher twist_pub; // publishes combined linear and angular velocities
    ros::Rate rate;
    double publish_frequency; // desired publishing rate
    std::string twist_frame; // frame id of the twist
    bool use_covariance; // if true, uses the TwistWithCovarianceStamped message, else uses Twist

    //----- Parameters
    // Forklift Dimensions
    double wheelbase; // length of wheelbase {m}
    double length_to_base; // length from front axle to base_link frame {m}

    // Motion variables
    double velocity; // m/s
    double angle; // rad
    geometry_msgs::TwistWithCovarianceStamped twist_with_cov;
    geometry_msgs::Twist twist;

public:
    VelocityConversion() : nh_("~"), rate(30)
    {
        // Set parameters
        nh_.param<double>("forklift/wheels/front_axle_to_back_axle", wheelbase, 1.7249);
        nh_.param<double>("length_to_base", length_to_base, 0); // length from the middle of the front axle to the base_link origin
        nh_.param<std::string>("base_link_frame", twist_frame, "base_link");
        nh_.param<double>("publish_frequency", publish_frequency, 30);
        nh_.param<bool>("use_covariance", use_covariance, true);

        // Set up ROS connections
        velocity_sub = nh_.subscribe("/velocity_node/velocity", 10, &VelocityConversion::velocityCallback, this);
        steering_angle_sub = nh_.subscribe("/steering_node/filtered_angle", 10, &VelocityConversion::angleCallback, this);

        if (use_covariance) {
            twist_with_cov_pub = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 10);
        }
        else {
            twist_pub = nh_.advertise<geometry_msgs::Twist>("twist", 10);
        }

        rate = ros::Rate(publish_frequency);

        // Set initial state
        velocity = 0;
        angle = 0;

        // Wait to receive one of each message type before continuing
        boost::shared_ptr<std_msgs::Float64 const> msg_ptr;
        msg_ptr = ros::topic::waitForMessage<std_msgs::Float64>("/steering_node/filtered_angle");
        angle = msg_ptr->data;
        msg_ptr = ros::topic::waitForMessage<std_msgs::Float64>("/velocity_node/velocity");
        velocity = msg_ptr->data;
    }

    void velocityCallback(const std_msgs::Float64 msg)
    {
        velocity = msg.data;

        if (use_covariance) {
            calculateTwistWithCov(twist_with_cov);
            twist_with_cov_pub.publish(twist_with_cov);
        }
        else {
            calculateTwist(twist);
            twist_pub.publish(twist);
        }
    }

    void angleCallback(const std_msgs::Float64 msg)
    {
        angle = msg.data;

        // // It is best to only send a twist when the velocity is updated because
        // // it is the slowest signal at 4Hz
        // if (use_covariance) {
        //     calculateTwistWithCov(twist_with_cov);
        //     twist_with_cov_pub.publish(twist_with_cov);
        // }
        // else {
        //     calculateTwist(twist);
        //     twist_pub.publish(twist);
        // }
    }

    void calculateTwistWithCov(geometry_msgs::TwistWithCovarianceStamped &forklift_twist)
    {
        // FIXME: try increasing the velocity value when turning
        //velocity *= 1 + (angle/M_PI);

        // Calculate the linear velocity at the base_link
        double v_b = velocity*sqrt(1 + pow((length_to_base/wheelbase)*tan(angle), 2));
        double theta_dot = (velocity/wheelbase)*(-tan(angle));

        forklift_twist.header.stamp = ros::Time::now();
        forklift_twist.header.frame_id = twist_frame.c_str();
        forklift_twist.twist.twist.linear.x = v_b;
        forklift_twist.twist.twist.angular.z = theta_dot;
    }

    void calculateTwist(geometry_msgs::Twist &forklift_twist)
    {
        // FIXME: try increasing the velocity value when turning
        //velocity *= 1 + (angle/M_PI);

        // Calculate the linear velocity at the base_link
        double v_b = velocity*sqrt(1 + pow((length_to_base/wheelbase)*tan(angle), 2));
        double theta_dot = (velocity/wheelbase)*(-tan(angle));

        forklift_twist.linear.x = v_b;
        forklift_twist.angular.z = theta_dot;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_conversion");
    VelocityConversion velocity_conversion;
    ros::spin();

    return 0;
}
