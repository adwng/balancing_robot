#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <functional>

class RobotState : public rclcpp::Node
{
public:
    RobotState()
    : Node("robot_state")
    {
        // Initialize state variables
        pitch_ = 0.0;
        pitch_rate_ = 0.0;
        position = 0.0;    
        cmd_vel_linear_ = 0.0;
        cmd_vel_angular_ = 0.0;

        // Subscribers
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&RobotState::imu_callback, this, std::placeholders::_1));
        
        joints_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/dynamic_joint_states", 10, std::bind(&RobotState::joints_callback, this, std::placeholders::_1));

        teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_out", 10, std::bind(&RobotState::teleop_callback, this, std::placeholders::_1)
        );

        // Publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&RobotState::lqr_compute, this));

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

        pitch_ = pitch;
        pitch_rate_ = msg->angular_velocity.y;
    }

    void joints_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Constants for wheel parameters (replace with actual values)

        double dt = (this->now() - last_time_).seconds();  // Time step in seconds

        // Compute average linear velocity of the wheels
        double velocity_left = 0.0;
        double velocity_right = 0.0;

        // Find left and right wheel velocities based on joint names
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            std::string joint_name = msg->name[i];
            double velocity = msg->velocity[i];

            // Assuming joint names indicate left and right wheels, adjust velocities accordingly
            if (joint_name == "left_wheel_joint")
                velocity_left = velocity;
            else if (joint_name == "right_wheel_joint")
                velocity_right = velocity;
        }

        // Average linear velocity of the wheels
        double average_linear_velocity = (velocity_left + velocity_right) / 2.0;

        // Update linear position (x) using integration
        position += average_linear_velocity * dt;

        // Update linear velocity (xdot)
        velocity= average_linear_velocity;

        // Optionally, update other state variables or perform additional computations

        // Update last time
        last_time_ = this->now();
    }

    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract linear and angular velocities from teleop message
        cmd_vel_linear_ = msg->linear.x;
        cmd_vel_angular_ = msg->angular.z;
    }

    void lqr_compute()
    {
        // State vector [pitch, pitch_rate, position, velocity]
        std::vector<double> state = {pitch_, pitch_rate_, position, velocity};

        // LQR gain matrix (example values)
        float K[4] = {-1.1180, 2.0436, -24.4672, -3.7660};

        // Compute control commands using LQR
        double control_linear = 0.0;
        double control_angular = 0.0;

        // Calculate control based on LQR
        for (size_t i = 0; i < 4; ++i) {
            control_linear += K[i] * state[i];
        }

        // Optionally, incorporate teleop commands
        control_linear += cmd_vel_linear_;
        control_angular += cmd_vel_angular_; 

        // Publish control commands
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = control_linear;
        cmd_msg.angular.z = control_angular;
        velocity_publisher_->publish(cmd_msg);
    }

    double pitch_;
    double pitch_rate_;
    double position;
    double velocity;
    double cmd_vel_linear_;
    double cmd_vel_angular_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_subscriber_;
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
