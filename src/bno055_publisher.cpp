#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "bno055.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher(std::string_view devDirectory, uint8_t devAddress)
        : Node("imu_node"), imu_(devDirectory, devAddress) {

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_msg",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)));
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MinimalPublisher::timer_callback, this));

        constexpr std::array accelCovariance = { 0.017, 0.017, 0.017 };
        constexpr std::array gyroCovariance = { 0.04, 0.04, 0.04 };
        constexpr std::array quatCovariance = { 0.0159, 0.0159, 0.0159 };

        for (size_t i = 0; i < imuMsg_.angular_velocity_covariance.size(); i++) {
            if (i % 3 == 0) {
                imuMsg_.angular_velocity_covariance[i] = gyroCovariance[i / 3];
                imuMsg_.linear_acceleration_covariance[i] = accelCovariance[i / 3];
                imuMsg_.orientation_covariance[i] = quatCovariance[i / 3];
            } else {
                imuMsg_.angular_velocity_covariance[i] = 0;
                imuMsg_.linear_acceleration_covariance[i] = 0;
                imuMsg_.orientation_covariance[i] = 0;
            }
        }
    }

private:
    void timer_callback() {
        auto accel = imu_.getLinearAccelMsq();
        auto gyro = imu_.getGyroRps();
        auto quats = imu_.getQuaternion();

        imuMsg_.linear_acceleration.x = accel.x;
        imuMsg_.linear_acceleration.y = accel.y;
        imuMsg_.linear_acceleration.z = accel.z;

        imuMsg_.angular_velocity.x = gyro.x;
        imuMsg_.angular_velocity.y = gyro.y;
        imuMsg_.angular_velocity.z = gyro.z;

        imuMsg_.orientation.w = quats.w;
        imuMsg_.orientation.x = quats.x;
        imuMsg_.orientation.y = quats.y;
        imuMsg_.orientation.z = quats.z;

        RCLCPP_INFO(this->get_logger(), "LinearAccel: x: %3.2f,  y: %3.2f,  z: %3.2f\n", accel.x, accel.y, accel.z);
        RCLCPP_INFO(this->get_logger(), "AngularVel : x: %3.2f,  y: %3.2f,  z: %3.2f\n", gyro.x, gyro.y, gyro.z);
        RCLCPP_INFO(this->get_logger(), "Quaternion : w: %3.2f,  x: %3.2f,  y: %3.2f,  z: %3.2f\n\n", quats.w, quats.x, quats.y, quats.z);
        publisher_->publish(imuMsg_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    sensor_msgs::msg::Imu imuMsg_ = sensor_msgs::msg::Imu();
    imu::BNO055 imu_ {};
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error! Invalid command line arguments!\n");
        return -1;
    }

    std::string_view devDirectory { argv[1] };

    uint8_t devAddress {};
    try {
        auto devAdressInput = std::stoul(argv[2], nullptr, 16);
        if (devAdressInput > std::numeric_limits<uint8_t>().max()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error! Device I2C adress is not valid!\n");
            return -1;
        } else {
            devAddress = static_cast<uint8_t>(devAdressInput);
        }

    } catch (std::invalid_argument& err) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error! Argument \"%5s\" is not a proper I2C adress!\n", argv[2]);
        return -1;
    }

    try {
        rclcpp::spin(std::make_shared<MinimalPublisher>(devDirectory, devAddress));
    } catch (std::runtime_error& err) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), err.what());
    }

    rclcpp::shutdown();
    return 0;
}