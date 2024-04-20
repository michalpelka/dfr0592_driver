#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"


#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
using std::placeholders::_1;


#include <iostream>

class PIController {
private:
  float kp; // Proportional gain
  float ki; // Integral gain
  float setpoint; // Target setpoint
  float integral; // Integral term

public:
  // Constructor
  PIController(float proportional_gain, float integral_gain) : kp(proportional_gain), ki(integral_gain), setpoint(0), integral(0) {}

  // Set the target setpoint
  void setSetpoint(float sp) {
    setpoint = sp;
  }

  // Calculate the control output
  float compute(float process_variable) {
    if (setpoint == 0)
    {
      integral = 0;
      return 0;
    }
    float error = setpoint - process_variable;
    integral += error;
    return kp * error + ki * integral;
  }
};

namespace dfrobot {

constexpr char REG_MOTOR1_PWM = 0x0e;
constexpr char REG_MOTOR1_ORIENTATION = 0x0F;
constexpr char REG_MOTOR1_SPEED = 0x10;
constexpr char REG_MOTOR2_ORIENTATION = 0x12;
constexpr char REG_MOTOR2_SPEED = 0x13;

void setPWMFreq(int fd, float freq) {
  char buffer[2] = {0};
  buffer[0] = REG_MOTOR1_PWM;
  buffer[1] = static_cast<int>(freq / 50);
  if (write(fd, buffer, 2) != 2) {
    std::cerr << "Error writing to I2C bus." << std::endl;
  }
  usleep(1000);
}

void setMotorSpeed(int fd, int motorId, int duty, int direction) {

  char buffer[3] = {0};
  if (motorId == 0) {
    buffer[0] = REG_MOTOR1_ORIENTATION;
  } else if (motorId == 1) {
    buffer[0] = REG_MOTOR2_ORIENTATION;
  }
  buffer[1] = direction;
  if (write(fd, buffer, 2) != 2) {
    std::cerr << "Error writing to I2C bus." << std::endl;
  }
  usleep(1000);

  if (motorId == 0) {
    buffer[0] = REG_MOTOR1_SPEED;
  } else if (motorId == 1) {
    buffer[0] = REG_MOTOR2_SPEED;
  }
  buffer[1] = duty;
  if (write(fd, buffer, 3) != 3) {
    std::cerr << "Error writing to I2C bus." << std::endl;
  }

  usleep(1000);
}

void setMotor(int i2c_fd, int motorId, float duty) {
  duty = std::max(-1.0f, std::min(duty, 1.0f));
  char dutyByte = 100 * std::abs(duty);
  char direction = duty > 0 ? 1 : 2;
  setMotorSpeed(i2c_fd, motorId, dutyByte, direction);
}
} // namespace dfrobot

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("dfr_driver") {
    // PCF8574 I2C address
    int i2cadress = 0x10;

    // Open the I2C bus
    const char *i2c_device = "/dev/i2c-1"; // Use /dev/i2c-1 for Raspberry Pi
    i2c_fd = open(i2c_device, O_RDWR);

    if (i2c_fd == -1) {
      std::cerr << "Error opening I2C bus." << std::endl;
      std::abort();
    }

    // Set the I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, i2cadress) < 0) {
      std::cerr << "Error setting I2C slave address." << std::endl;
      close(i2c_fd);
      std::abort();
    }

    dfrobot::setPWMFreq(i2c_fd, 1000);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_calibrated", 10, std::bind(&MinimalSubscriber::imu_callback, this, _1));
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu& msg) {
    gyroZ = msg.angular_velocity.z;
  }
  void topic_callback(const geometry_msgs::msg::Twist &twist){

    controller.setSetpoint(twist.angular.z);
    float control = controller.compute(gyroZ);


    dfrobot::setMotor(i2c_fd, 0, twist.linear.x - control);
    dfrobot::setMotor(i2c_fd, 1, twist.linear.x + control);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  int i2c_fd = 0;
  float gyroZ = 0;
  PIController controller{0.2, 0.05};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
