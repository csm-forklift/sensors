/**
 *  This node converts the linear velocity and steering angle into a Twist
 *  message by calculating the angular velocity.
 */


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <cmath>
#include <string>


class VelocityConversion
{
private:
    //----- ROS Objects
    ros::NodeHandle nh_;
    ros::Subscriber velocity_sub; // grabs velocity in m/s
    ros::Subscriber steering_angle_sub; // grabs steering angle in rad
    ros::Publisher twist_pub; // publishes combined linear and angular velocities
    ros::Rate rate;
    double publish_frequency; // desired publishing rate
    std::string twist_frame; // frame id of the twist

    //----- Parameters
    // Forklift Dimensions
    double forklift_length; // length of wheelbase {m}
    double length_to_base; // length from front axle to base_link frame {m}

    // Motion variables
    double velocity; // m/s
    double angle; // rad
    geometry_msgs::TwistWithCovarianceStamped twist;

public:
    VelocityConversion() : nh_("~"), rate(30)
    {
        // Set up ROS connections
        velocity_sub = nh_.subscribe("/velocity_node/velocity", 10, &VelocityConversion::velocityCallback, this);
        steering_angle_sub = nh_.subscribe("/steering_node/filtered_angle", 10, &VelocityConversion::angleCallback, this);
        twist_pub = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 10);
        nh_.param<double>("publish_frequency", publish_frequency, 30);
        rate = ros::Rate(publish_frequency);
        nh_.param<std::string>("base_link_frame", twist_frame, "base_link");

        // Set parameters
        nh_.param<double>("forklift/body/length", forklift_length, 2.5601);
        nh_.param<double>("length_to_base", length_to_base, 0); // length from the middle of the front axle to the base_link origin

        // Set initial state
        velocity = 0;
        angle = 0;

        // Wait to receive one of each message type before continuing
        boost::shared_ptr<std_msgs::Float32 const> msg_ptr;
        msg_ptr = ros::topic::waitForMessage<std_msgs::Float32>("/steering_node/filtered_angle");
        angle = msg_ptr->data;
        msg_ptr = ros::topic::waitForMessage<std_msgs::Float32>("/velocity_node/velocity");
        velocity = msg_ptr->data;
    }

    void spin()
    {
        while (ros::ok()) {
            // Uupdate twist message and publish
            calculateTwist(twist);
            twist_pub.publish(twist);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void velocityCallback(const std_msgs::Float32 msg)
    {
        velocity = msg.data;
    }

    void angleCallback(const std_msgs::Float32 msg)
    {
        angle = msg.data;
    }

    void calculateTwist(geometry_msgs::TwistWithCovarianceStamped &forklift_twist)
    {
        // Calculate the linear velocity at the base_link
        double v_b = velocity*sqrt(1 + pow((length_to_base/forklift_length)*tan(angle), 2));
        double theta_dot = velocity*(tan(angle)/forklift_length);

        forklift_twist.header.stamp = ros::Time::now();
        forklift_twist.header.frame_id = twist_frame.c_str();
        forklift_twist.twist.twist.linear.x = v_b;
        forklift_twist.twist.twist.angular.z = theta_dot;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_conversion");
    VelocityConversion velocity_conversion;
    velocity_conversion.spin();

    return 0;
}
