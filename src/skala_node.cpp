#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <chrono>
#include <memory>
#include <cstdio>

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
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vo_pose", 10, 
            std::bind(&SkalaNode::pose_callback, this, std::placeholders::_1)
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
    
    geometry_msgs::msg::PoseStamped::SharedPtr last_pose_;
    geometry_msgs::msg::PoseStamped::SharedPtr start_pose_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr scale_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        last_pose_ = msg; 
    }

    float get_encoder_speed() {
        if (i2c_file_ < 0) return 0.0f;

        char reg[1] = {0x01};
        if (write(i2c_file_, reg, 1) == 1) {   
            char data[4];
            if (read(i2c_file_, data, 4) == 4) { 
                
                int32_t encoder_speed = ((uint8_t)data[0] << 24) | 
                                        ((uint8_t)data[1] << 16) | 
                                        ((uint8_t)data[2] << 8)  | 
                                         (uint8_t)data[3];
                
                float f_speed = static_cast<float>(encoder_speed) / 1000000.0f;
                
                static int debug_counter = 0;
                if (debug_counter++ % 50 == 0) {
                    RCLCPP_INFO(this->get_logger(), "ESP I2C DEBUG | RAW: %02X %02X %02X %02X | INT32: %d | PREDKOSC: %f m/s", 
                                (uint8_t)data[0], (uint8_t)data[1], (uint8_t)data[2], (uint8_t)data[3], 
                                encoder_speed, f_speed);
                }
                
                return f_speed;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Blad odczytu (read) z ESP32 (0x67)");
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Blad zapisu (write) do ESP32 (0x67)");
        }
        return 0.0f;
    }

    void timer_callback() {
        float fencoder_speed = get_encoder_speed();

        if (current_stan_ == WAITING) {
            auto msg = std_msgs::msg::Float32();
            msg.data = 1.0f;
            scale_pub_->publish(msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Stan WAITING. Predkosc z ESP32: %f m/s, VO Pose: %s", 
                fencoder_speed, last_pose_ ? "DOSTEPNE" : "BRAK");

            if (std::abs(fencoder_speed) > 0.05 && last_pose_) {
                czas_startu_ = this->get_clock()->now();
                start_pose_ = last_pose_;
                dist_z_enkoderow_ = 0.0;
                current_stan_ = PROCESSING;
                RCLCPP_INFO(this->get_logger(), "Rozpoczeto kalibracje skali na bazie enkoderow...");
            }
        } 
        else if (current_stan_ == PROCESSING) {
            auto msg = std_msgs::msg::Float32();
            msg.data = 1.0f;
            scale_pub_->publish(msg);

            dist_z_enkoderow_ += std::abs(fencoder_speed) * 0.02;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Stan PROCESSING. Dystans enkoderow: %f m", dist_z_enkoderow_);

            auto teraz = this->get_clock()->now();
            if ((teraz - czas_startu_).seconds() >= 3.0) {
                if (last_pose_ && start_pose_) {
                    
                    double dx = last_pose_->pose.position.x - start_pose_->pose.position.x;
                    double dy = last_pose_->pose.position.y - start_pose_->pose.position.y;
                    double dz = last_pose_->pose.position.z - start_pose_->pose.position.z;
                    double dist_ze_slama = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (dist_ze_slama > 0.01) { 
                        current_scale_ = dist_z_enkoderow_ / dist_ze_slama;
                        RCLCPP_INFO(this->get_logger(), "KALIBRACJA ZAKONCZONA SUCESEM. Wyliczona skala: %f", current_scale_);
                        current_stan_ = FINISHED;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "SLAM nie zarejestrowal wystarczajacego ruchu (ponizej 1cm)! Ponawiam od poczatku.");
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
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SkalaNode>());
    rclcpp::shutdown();
    return 0;
}
