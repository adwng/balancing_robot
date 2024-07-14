#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <functional>
#include <algorithm>

class RobotState : public rclcpp::Node
{
public:
    RobotState()
    : Node("robot_state")
    {
        // Initialize state variables
        pitch_ = 0.0;
        pitch_rate_ = 0.0;
        position_ = 0.0;    
        velocity_ = 0.0;

        // Use ROS time
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        // Subscribers
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&RobotState::imu_callback, this, std::placeholders::_1));
        
        // joints_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "/dynamic_joint_states", 10, std::bind(&RobotState::joints_callback, this, std::placeholders::_1));

        teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_out", 10, std::bind(&RobotState::teleop_callback, this, std::placeholders::_1));

        // Publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&RobotState::lqr_compute, this));

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

        double alpha = 0.98; // Complementary filter constant
        pitch_ = alpha * (pitch_ + pitch_rate_ * dt_) + (1 - alpha) * pitch;
        pitch_rate_ = msg->angular_velocity.y;
    }

    // void joints_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    // {
    //     double velocity_left = 0.0;
    //     double velocity_right = 0.0;

    //     for (size_t i = 0; i < msg->name.size(); ++i)
    //     {
    //         std::string joint_name = msg->name[i];
    //         double velocity = msg->velocity[i];

    //         if (joint_name == "left_wheel_joint")
    //             velocity_left = velocity;
    //         else if (joint_name == "right_wheel_joint")
    //             velocity_right = velocity;
    //     }

    //     double average_linear_velocity = (velocity_left + velocity_right) / 2.0;
    //     position_ += average_linear_velocity * dt_;
    //     velocity_ = average_linear_velocity;
    // }

    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        velocity_ref_ = msg->linear.x;
        angular_velocity_ref_ = msg->angular.z;
    }

    void lqr_compute()
    {
        auto current_time = clock_->now();
        dt_ = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double pitch_error = pitch_ref_ - pitch_;
        double pitch_rate_error = pitch_rate_ref_ - pitch_rate_;
        // double velocity_error = velocity_ref_ - velocity_; 
        // double position_error = position_ref_ - position_;

        // std::vector<double> state = {pitch_error, pitch_rate_error, position_error, velocity_error};
        std::vector<double> state = {pitch_error, pitch_rate_error};

        float K[4] = {-1.1180, 2.0436};

        double control_linear = 0.0;
        for (size_t i = 0; i < 3; ++i) {
            control_linear += K[i] * state[i];
        }

        control_angular_ = angular_velocity_ref_; 

        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = control_linear;
        cmd_msg.angular.z = control_angular_;
        velocity_publisher_->publish(cmd_msg);
    }

    double pitch_;
    double pitch_rate_;
    double position_;
    double velocity_;

    double velocity_ref_;
    double angular_velocity_ref_;
    double pitch_ref_ = 0.0;
    double pitch_rate_ref_ = 0.0;
    double position_ref_ = 0.0;

    double control_angular_;

    double dt_; // delta time

    rclcpp::Time last_time_;
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_subscriber_;
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
