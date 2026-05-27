#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <chrono>
#include <memory>

class SkalaNode : public rclcpp::Node {
public:
    SkalaNode() : Node("skala_node"), current_stan_(WAITING), dist_z_enkoderow_(0.0), current_scale_(1.0f) {
        
        if ((i2c_file_ = open("/dev/i2c-1", O_RDWR)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Nie mozna otworzyc I2C (/dev/i2c-1)");
        } else {
            if (ioctl(i2c_file_, I2C_SLAVE, 0x67) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Nie mozna znalezc ESP32 (0x67)");
            }
        }

        
        scale_pub_ = this->create_publisher<std_msgs::msg::Float32>("/slam_scale", 10);

        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vo_pose_cov", 10, 
            std::bind(&SkalaNode::odom_callback, this, std::placeholders::_1)
        );

        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), 
            std::bind(&SkalaNode::timer_callback, this)
        );
    }

private:
    enum Stan { WAITING, PROCESSING, FINISHED };
    Stan current_stan_;

    int i2c_file_ = -1;
    float dist_z_enkoderow_;
    float current_scale_;
    rclcpp::Time czas_startu_;
    
    
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    nav_msgs::msg::Odometry::SharedPtr start_odom_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr scale_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = msg; 
    }

    
    float get_encoder_speed() {
        if (i2c_file_ < 0) return 0.0f;

        char reg[1] = {0x01};
        if (write(i2c_file_, reg, 1) == 1) {   
            char data[4];
            if (read(i2c_file_, data, 4) == 4) { 
                int32_t encoder_speed = (uint8_t)data[0] | 
                                       ((uint8_t)data[1] << 8) | 
                                       ((uint8_t)data[2] << 16) | 
                                       ((uint8_t)data[3] << 24);
                
                return static_cast<float>(encoder_speed) / 1000000.0f;
            }
        }
        return 0.0f;
    }

    
    void timer_callback() {
        float fencoder_speed = get_encoder_speed();

        if (current_stan_ == WAITING) {
            
            auto msg = std_msgs::msg::Float32();
            msg.data = 1.0f;
            scale_pub_->publish(msg);

            
            if (std::abs(fencoder_speed) > 0.05 && last_odom_) {
                czas_startu_ = this->get_clock()->now();
                start_odom_ = last_odom_; 
                dist_z_enkoderow_ = 0.0;
                current_stan_ = PROCESSING;
                RCLCPP_INFO(this->get_logger(), "Rozpoczeto kalibracje skali na bazie enkoderow...");
            }
        } 
        else if (current_stan_ == PROCESSING) {
            
            dist_z_enkoderow_ += std::abs(fencoder_speed) * 0.02;

            
            auto teraz = this->get_clock()->now();
            if ((teraz - czas_startu_).seconds() >= 3.0) {
                
                if (last_odom_ && start_odom_) {
                    
                    
                    double dx = last_odom_->pose.pose.position.x - start_odom_->pose.pose.position.x;
                    double dy = last_odom_->pose.pose.position.y - start_odom_->pose.pose.position.y;
                    double dz = last_odom_->pose.pose.position.z - start_odom_->pose.pose.position.z;
                    double dist_ze_slama = std::sqrt(dx*dx + dy*dy + dz*dz);

                    
                    if (dist_ze_slama > 0.01) { 
                        current_scale_ = dist_z_enkoderow_ / dist_ze_slama;
                        RCLCPP_INFO(this->get_logger(), "KALIBRACJA ZAKONCZONA SUCESEM. Wyliczona skala: %f", current_scale_);
                        current_stan_ = FINISHED;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "SLAM nie zarejestrowal wystarczajacego ruchu Ponawiam od poczatku.");
                        current_stan_ = WAITING; 
                    }
                }
            }
        }
        else if (current_stan_ == FINISHED) {
            
            auto msg = std_msgs::msg::Float32();
            msg.data = current_scale_;
            scale_pub_->publish(msg);
        }
    }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SkalaNode>());
    rclcpp::shutdown();
    return 0;
}
