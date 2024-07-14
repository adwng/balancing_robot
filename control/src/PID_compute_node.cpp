#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <functional>
#include <algorithm>
#include <cmath>

float KP_pitch = 4.543;
float KD_pitch = 0.4221;
float KI_pitch = 0.9243;
float KP_yaw = 0.5;

double Convert(double rad)
{
    return rad;
}

class RobotState : public rclcpp::Node
{
public:
    RobotState()
    : Node("robot_state")
    {

        // Use ROS time
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        // Subscribers
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&RobotState::imu_callback, this, std::placeholders::_1));
        
        teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_out", 10, std::bind(&RobotState::teleop_callback, this, std::placeholders::_1));

        // Publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&RobotState::pid_compute, this));

        last_time_ = clock_->now();
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract pitch and pitch rate from IMU data
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);


        // pitch_degrees = Convert(pitch);
        // yaw_degrees = Convert(yaw);

        pitch_error = pitch_ref_  - pitch;
        pitch_rate = msg->angular_velocity.y;
        pitch_error_sum += pitch_error;

        yaw_error = yaw_ref_ - yaw;

        control_pitch = (KP_pitch * pitch_error) + (KD_pitch * pitch_rate) + (KI_pitch * pitch_error_sum);
        control_yaw = KP_yaw * yaw_error;


        // RCLCPP_INFO(this->get_logger(), "Control Pitch: %lf, Control Yaw: %lf", control_pitch, control_yaw);
    }

    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear_x = msg->linear.x;
        angular_z = msg->angular.z;
    }

    void pid_compute()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();

        double throttle = 0.5*(control_pitch) + linear_x;
        double steering = 0.5*(control_yaw) + angular_z;

        if (throttle > 5.0)
        {
            throttle = 5.0;
        }
        else if (throttle < -5)
        {
            throttle = -5.0;
        }

        if (steering > 5.0)
        {
            steering = 5.0;
        }else if (steering < -5.0)
        {
            steering =  -5.0;
        }

        cmd_msg.linear.x = throttle;
        cmd_msg.angular.z = steering;

        RCLCPP_INFO(this->get_logger(), "throttle: %lf,  Steering: %lf", throttle, steering);

        velocity_publisher_->publish(cmd_msg);
    }

    double pitch_;
    double yaw_; 

    double pitch_ref_ = 0.0;
    double pitch_rate;
    double pitch_error;
    double pitch_error_sum;
    
    double yaw_ref_ = 0.0;
    double yaw_rate = 0;
    double yaw_error = 0;
    double prev_yaw_error = 0;
    double yaw_error_difference = 0;

    double pitch_degrees;
    double yaw_degrees;

    double linear_x;
    double angular_z;

    double control_pitch = 0.0;
    double control_yaw = 0.0;

    double dt_; // delta time

    rclcpp::Time last_time_;
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotState>());
    rclcpp::shutdown();
    return 0;
}
