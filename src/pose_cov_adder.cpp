#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class PoseCovAdder : public rclcpp::Node {
public:
    PoseCovAdder() : Node("pose_cov_adder") {
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/vo_pose_cov", 10);
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vo_pose", 10, std::bind(&PoseCovAdder::pose_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto cov_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        cov_msg.header = msg->header;
        cov_msg.pose.pose = msg->pose;

        // Ustawienie diagonalnej kowariancji
        cov_msg.pose.covariance[0]  = 0.01; // X
        cov_msg.pose.covariance[7]  = 0.01; // Y
        cov_msg.pose.covariance[14] = 0.01; // Z
        cov_msg.pose.covariance[21] = 0.01; // Roll
        cov_msg.pose.covariance[28] = 0.01; // Pitch
        cov_msg.pose.covariance[35] = 0.01; // Yaw

        pub_->publish(cov_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseCovAdder>());
    rclcpp::shutdown();
    return 0;
}
