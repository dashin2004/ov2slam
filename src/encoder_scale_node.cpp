#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstring>

class EncoderScaleNode : public rclcpp::Node {
public:
    EncoderScaleNode() : Node("encoder_scale_node"), state_(State::WAITING), scale_(1.0), i2c_file_(-1) {
        // I2C Setup
        if ((i2c_file_ = open("/dev/i2c-1", O_RDWR)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus /dev/i2c-1");
        } else {
            
            int esp32_address = 0x08; 
            if (ioctl(i2c_file_, I2C_SLAVE, esp32_address) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to ESP32 at address 0x%02X", esp32_address);
            }
        }

        scale_pub_ = this->create_publisher<std_msgs::msg::Float32>("/slam_scale", 10);
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vo_pose", 10, std::bind(&EncoderScaleNode::pose_callback, this, std::placeholders::_1));

        // 50Hz timer for I2C and scale publishing
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&EncoderScaleNode::timer_callback, this));
    }

private:
    enum class State {
        WAITING,
        CALIBRATING,
        CALIBRATED
    };

    State state_;
    double scale_;
    int i2c_file_;
    
    // Calibration variables
    rclcpp::Time calibration_start_time_;
    double accumulated_encoder_dist_ = 0.0;
    double accumulated_vo_dist_ = 0.0;
    
    geometry_msgs::msg::PoseStamped::SharedPtr last_pose_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr scale_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float read_encoder_speed() {
        if (i2c_file_ < 0) return 0.0f;
        
        // TODO: Update this according to ESP32 data format.
        // For now, assuming a 4-byte float is sent.
        float speed = 0.0f;
        char data[4];
        if (read(i2c_file_, data, 4) == 4) {
            std::memcpy(&speed, data, 4);
        } else {
            // Handle read error if needed
        }
        return speed;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!last_pose_) {
            last_pose_ = msg;
            return;
        }
        
        if (state_ == State::CALIBRATING) {
            double dx = msg->pose.position.x - last_pose_->pose.position.x;
            double dy = msg->pose.position.y - last_pose_->pose.position.y;
            double dz = msg->pose.position.z - last_pose_->pose.position.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            accumulated_vo_dist_ += dist;
        }
        
        last_pose_ = msg;
    }

    void timer_callback() {
        float enc_speed = read_encoder_speed();
        double dt = 0.02; // 50 Hz = 20 ms
        
        if (state_ == State::WAITING) {
            // Speed threshold to start calibrating (e.g., > 0.05 m/s)
            if (std::abs(enc_speed) > 0.05) {
                state_ = State::CALIBRATING;
                calibration_start_time_ = this->get_clock()->now();
                accumulated_encoder_dist_ = 0.0;
                accumulated_vo_dist_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "Starting calibration based on encoder movement...");
            }
        } 
        else if (state_ == State::CALIBRATING) {
            accumulated_encoder_dist_ += std::abs(enc_speed) * dt;
            
            auto now = this->get_clock()->now();
            double elapsed = (now - calibration_start_time_).seconds();
            
            if (elapsed >= 1.0) { // 1 second calibration
                if (accumulated_vo_dist_ > 0.001) {
                    scale_ = accumulated_encoder_dist_ / accumulated_vo_dist_;
                    RCLCPP_INFO(this->get_logger(), "Calibration finished! Scale set to: %f", scale_);
                    state_ = State::CALIBRATED;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Calibration failed (VO distance too small). Retrying.");
                    state_ = State::WAITING;
                }
            }
        }

        // Publish scale continuously
        auto msg = std_msgs::msg::Float32();
        msg.data = scale_;
        scale_pub_->publish(msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderScaleNode>());
    rclcpp::shutdown();
    return 0;
}
