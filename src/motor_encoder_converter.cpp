#include <iostream>
#include <vector>
#include <fstream> // Add for file operations
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MotorEncoderConverter : public rclcpp::Node {
public:
    MotorEncoderConverter() : Node("motor_encoder_converter") {
        // Initialize the radians vector
        radians_.resize(2, 0.0);

        // Radius in centimeters
        double radius = 3.5; // cm

        // Create a file to store the positions
        position_file_.open("./position_file.txt");

        // Declare the joint_state_msg variable outside the lambda
        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

        // Subscribe to the /repair_robot/joint_states topic
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/repair_robot/joint_states", 10,
            [joint_state_msg, this, &radius](const sensor_msgs::msg::JointState::SharedPtr msg) {
                 RCLCPP_INFO(this->get_logger(), "Received joint state message");
                // Check if the message contains position data
                if (msg->position.size() == 2) {
                    // Update the radians vector with received positions
                    radians_[0] = msg->position[0];
                    radians_[1] = msg->position[1];
                    // Print radius
                    std::ostringstream radius_str;
                    radius_str << "radius: " << radius;
                    RCLCPP_INFO(this->get_logger(), radius_str.str());
                    // Convert radians to centimeters
                    std::vector<double> centimeters = convertToCentimeters(radians_, 3.5);

                    // Publish the converted distances as a JointState message
                    joint_state_msg->name.push_back("right_motor");
                    joint_state_msg->name.push_back("left_motor");
                    joint_state_msg->position = centimeters;
                    joint_state_msg->header.stamp = msg->header.stamp;

                    // Publish the JointState message
                    joint_state_publisher_->publish(*joint_state_msg);

                    // Save the positions to the file
                    savePositionsToFile(centimeters);
                }
            });

        // Publish the JointState message
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/repair_robot/joint_states_converted", 10);
        auto timer_callback = [this, joint_state_msg]() {
            RCLCPP_INFO(this->get_logger(), "Publishing converted motor positions");
            joint_state_publisher_->publish(*joint_state_msg);
        };
        timer_ = this->create_wall_timer(std::chrono::seconds(1), timer_callback);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> radians_;
    std::ofstream position_file_; // File to store positions

    std::vector<double> convertToCentimeters(const std::vector<double>& radians, double radius) {
        std::vector<double> centimeters;
        // Print the centimeters vector using RCLCPP_INFO
        std::ostringstream centimeters_str;
        
        for (const double& radian : radians) {
            double cm = radian * radius;
            centimeters.push_back(cm);
        }
        
        for (const double& cm : centimeters) {
            centimeters_str << "centimeters: ";
            centimeters_str << cm << " ";
        }
        // Print the radians_ vector using RCLCPP_INFO
        std::ostringstream radians_str;
        radians_str << "radians_: ";
        for (const double& radian : radians) {
            radians_str << radian << " ";
        }
        RCLCPP_INFO(this->get_logger(), radians_str.str());
        RCLCPP_INFO(this->get_logger(), centimeters_str.str());
        return centimeters;
    }

    void publishConvertedPositions() {
        // Create a JointState message with the converted positions
        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state_msg->name.push_back("right_motor");
        joint_state_msg->name.push_back("left_motor");
        joint_state_msg->position = convertToCentimeters(radians_, 3.5); // Convert and publish latest positions
        joint_state_msg->header.stamp = this->now();

        // Publish the JointState message
        joint_state_publisher_->publish(*joint_state_msg);
    }

    void savePositionsToFile(const std::vector<double>& positions) {
        // Open the file in append mode and write positions
        if (position_file_.is_open()) {
            for (const double& position : positions) {
                position_file_ << position << " ";
            }
            position_file_ << "\n"; // Add a newline to separate entries
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorEncoderConverter>());
    rclcpp::shutdown();
    return 0;
}
