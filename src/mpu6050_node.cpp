#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
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

        // Przeprowadzenie autokalibracji przed startem publikacji
        calibrate_sensor();

        // Timer 100Hz (10ms)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Mpu6050Node::timer_callback, this));
    }

private:
    int i2c_file_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;


    // Zmienne przechowujące wyliczone błędy (offsety)
    double accel_offset_x_ = 0.0;
    double accel_offset_y_ = 0.0;
    double accel_offset_z_ = 0.0;
    double gyro_offset_x_ = 0.0;
    double gyro_offset_y_ = 0.0;
    double gyro_offset_z_ = 0.0;



    int16_t read_word_2c(int addr) {
        char reg[1] = {(char)addr};
        write(i2c_file_, reg, 1);
        char data[2];
        read(i2c_file_, data, 2);
        return (data[0] << 8) | data[1];
    }

    void calibrate_sensor() {
        RCLCPP_INFO(this->get_logger(), "ROZPOCZYNAM KALIBRACJE - NIE RUSZAJ CZUJNIKIEM!");
        
        int num_samples = 500;
        long sum_ax = 0, sum_ay = 0, sum_az = 0;
        long sum_gx = 0, sum_gy = 0, sum_gz = 0;

        for (int i = 0; i < 100; i++) {
            read_word_2c(0x3B);
            usleep(2000); 
        }

        for (int i = 0; i < num_samples; i++) {
            sum_ax += read_word_2c(0x3B);
            sum_ay += read_word_2c(0x3D);
            sum_az += read_word_2c(0x3F);
            sum_gx += read_word_2c(0x43);
            sum_gy += read_word_2c(0x45);
            sum_gz += read_word_2c(0x47);
            usleep(2000); 
        }

        accel_offset_x_ = (double)sum_ax / num_samples;
        accel_offset_y_ = (double)sum_ay / num_samples;
        accel_offset_z_ = ((double)sum_az / num_samples) - 16384.0; 

        gyro_offset_x_ = (double)sum_gx / num_samples;
        gyro_offset_y_ = (double)sum_gy / num_samples;
        gyro_offset_z_ = (double)sum_gz / num_samples;

        RCLCPP_INFO(this->get_logger(), "KALIBRACJA ZAKONCZONA.");
    }

    void timer_callback() {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_link";

        // Odczyt raw wartości tylko raz
        int16_t raw_ax = read_word_2c(0x3B);
        int16_t raw_ay = read_word_2c(0x3D);
        int16_t raw_az = read_word_2c(0x3F);
        int16_t raw_gx = read_word_2c(0x43);
        int16_t raw_gy = read_word_2c(0x45);
        int16_t raw_gz = read_word_2c(0x47);

        double acc_scale = 16384.0;
        double g = 9.80665;
        // Zastosowanie offsetów z kalibracji
        msg.linear_acceleration.x = ((raw_ax - accel_offset_x_) / acc_scale) * g;
        msg.linear_acceleration.y = ((raw_ay - accel_offset_y_) / acc_scale) * g;
        msg.linear_acceleration.z = ((raw_az - accel_offset_z_) / acc_scale) * g;

        double gyro_scale = 131.0;
        msg.angular_velocity.x = ((raw_gx - gyro_offset_x_) / gyro_scale) * (M_PI / 180.0);
        msg.angular_velocity.y = ((raw_gy - gyro_offset_y_) / gyro_scale) * (M_PI / 180.0);
        msg.angular_velocity.z = ((raw_gz - gyro_offset_z_) / gyro_scale) * (M_PI / 180.0);

        msg.orientation_covariance[0] = -1.0; 

        msg.angular_velocity_covariance[0] = 0.002; // X
        msg.angular_velocity_covariance[4] = 0.002; // Y
        msg.angular_velocity_covariance[8] = 0.002; // Z

        msg.linear_acceleration_covariance[0] = 0.04; // X
        msg.linear_acceleration_covariance[4] = 0.04; // Y
        msg.linear_acceleration_covariance[8] = 0.04; // Z

        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mpu6050Node>());
    rclcpp::shutdown();
    return 0;
}
