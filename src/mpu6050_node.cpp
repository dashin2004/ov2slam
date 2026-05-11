#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

class Mpu6050Node : public rclcpp::Node {
public:
    Mpu6050Node() : Node("mpu6050_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        
        if ((i2c_file_ = open("/dev/i2c-1", O_RDWR)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Nie mozna otworzyc I2C (/dev/i2c-1)");
            return;
        }
        if (ioctl(i2c_file_, I2C_SLAVE, 0x68) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Nie mozna znalezc MPU6050 (0x68)");
            return;
        }

        // Wybudzenie sensora
        char buf[2] = {0x6B, 0x00};
        write(i2c_file_, buf, 2);

        // Timer 100Hz (10ms)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Mpu6050Node::timer_callback, this));
    }

private:
    int i2c_file_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

    int16_t read_word_2c(int addr) {
        char reg[1] = {(char)addr};
        write(i2c_file_, reg, 1);
        char data[2];
        read(i2c_file_, data, 2);
        return (data[0] << 8) | data[1];
    }

    void timer_callback() {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_link";

        double acc_scale = 16384.0;
        double g = 9.80665;
        msg.linear_acceleration.x = (read_word_2c(0x3B) / acc_scale) * g;
        msg.linear_acceleration.y = (read_word_2c(0x3D) / acc_scale) * g;
        msg.linear_acceleration.z = (read_word_2c(0x3F) / acc_scale) * g;

        double gyro_scale = 131.0;
        msg.angular_velocity.x = (read_word_2c(0x43) / gyro_scale) * (M_PI / 180.0);
        msg.angular_velocity.y = (read_word_2c(0x45) / gyro_scale) * (M_PI / 180.0);
        msg.angular_velocity.z = (read_word_2c(0x47) / gyro_scale) * (M_PI / 180.0);

        msg.orientation_covariance[0] = -1.0;
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mpu6050Node>());
    rclcpp::shutdown();
    return 0;
}
