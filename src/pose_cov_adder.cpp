#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PoseCovAdderNode : public rclcpp::Node {
public:
    PoseCovAdderNode() : Node("pose_cov_adder_node"), current_scale_(1.0f) {
        
        scale_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/slam_scale", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                current_scale_ = msg->data;
            });

        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vo_pose", 10, std::bind(&PoseCovAdderNode::pose_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vo_pose_cov", 10);
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header = msg->header;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        
        odom_msg.pose.pose.position.x = msg->pose.position.z * current_scale_;  
        odom_msg.pose.pose.position.y = -msg->pose.position.x * current_scale_; 
        odom_msg.pose.pose.position.z = -msg->pose.position.y * current_scale_; 

        
        tf2::Quaternion q_orig(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        
        tf2::Quaternion q_rot;
        q_rot.setRPY(-M_PI/2.0, 0, -M_PI/2.0); 
        
        tf2::Quaternion q_final = q_rot * q_orig;
        
        odom_msg.pose.pose.orientation.x = q_final.x();
        odom_msg.pose.pose.orientation.y = q_final.y();
        odom_msg.pose.pose.orientation.z = q_final.z();
        odom_msg.pose.pose.orientation.w = q_final.w();

        
        for (int i = 0; i < 36; i++) odom_msg.pose.covariance[i] = 0.0;
        odom_msg.pose.covariance[0] = 0.01;  // X
        odom_msg.pose.covariance[7] = 0.01;  // Y
        odom_msg.pose.covariance[35] = 0.01; // Z

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scale_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    float current_scale_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseCovAdderNode>());
    rclcpp::shutdown();
    return 0;
}
