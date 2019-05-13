/**
 *  This node reads the raw data from the axle potentiometer passed on by the
 *  Arduino, filters the data to reduce noise, then converts it using
 *  pre-determined regression parameters to an angle.
 */


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <cmath>

class SteeringNode
{
private:
    // ROS Objects
    ros::NodeHandle nh_;
    ros::Subscriber raw_pot_sub; // receives raw potentiometer value
    ros::Publisher steering_angle_deg_pub; // publishes value in degrees
    ros::Publisher steering_angle_pub; // publishes value in radians

    // Data processing parameters
    double filter_gain; // adjusts time-delay/smoothness of the data filtering
    int prev_value; // previous value of the filtered data
    int value; // current value fo the filtered data
    double m_pos; // slope for positive region regression
    double b_pos; // y-intercept for positive region regression
    double m_neg; // slope for negative retion regression
    double b_neg; // y-intercept for negative region regression
    int region_divide; // x-value where the to regions cross

public:
    SteeringNode() : nh_("~")
    {
        // Set up ROS connections
        raw_pot_sub = nh_.subscribe("/steering_node/potentiometer/raw_data", 10, &SteeringNode::rawPotCallback, this);
        steering_angle_deg_pub = nh_.advertise<std_msgs::Float32>("filtered_angle_deg", 10);
        steering_angle_pub = nh_.advertise<std_msgs::Float32>("filtered_angle", 10);

        // Set up data parameters
        nh_.param<double>("filter_gain", filter_gain, 0.5);
        prev_value = -1; // initially set to an invalid value to check if it has been defined before once data is received.
        m_pos = 2.91186044;
        b_pos = -1535.39415892;
        m_neg = 3.88249541;
        b_neg = -2047.64178971;
        // For calculating the x-value where two lines cross
        // y = m_pos*x + b_pos
        // y = m_neg*x + b_neg
        // (m_neg - m_pos)*x = (b_pos - b_neg)
        // x = (b_pos - b_neg) / (m_neg - m_pos)
        region_divide = (b_pos - b_neg) / (m_neg - m_pos);
    }

    void rawPotCallback(const std_msgs::Int16 msg)
    {
        // Check if data has been stored before
        if (prev_value != -1) {
            // Filter data with exponentially weighted average
            value = prev_value + filter_gain*(msg.data - prev_value);
        }
        else {
            // If no data has been received yet, store the first value
            value = msg.data;
        }

        // Convert raw potentiometer analog read value to steering angle (the
        // regression was split into two regions, one for positive angles and
        // one for negative angles)
        // See the file "sensors/config/steering_regression_parameters.txt" for
        // the regression values.
        std_msgs::Float32 angle_deg;
        std_msgs::Float32 angle;
        if (value >= region_divide) {
            angle_deg.data = (m_pos*value + b_pos)/10.0;
        }
        else {
            angle_deg.data = (m_neg*value + b_neg)/10.0;
        }

        // Convert to radians
        angle.data = angle_deg.data * (M_PI/180.0);

        // Publish
        steering_angle_deg_pub.publish(angle_deg);
        steering_angle_pub.publish(angle);

        // Update previous value
        prev_value = value;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "steering_node");
    SteeringNode steering_node;

    ros::spin();

    return 0;
}
