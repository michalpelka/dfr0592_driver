#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <iostream>
using std::placeholders::_1;

namespace dfrobot {

constexpr char REG_MOTOR1_PWM = 0x0e;
constexpr char REG_MOTOR1_ORIENTATION = 0x0F;
constexpr char REG_MOTOR1_SPEED = 0x10;
constexpr char REG_MOTOR2_ORIENTATION = 0x12;
constexpr char REG_MOTOR2_SPEED = 0x13;



void setPWMFreq(int fd, float freq)
{
  char buffer[2] = {0};
  buffer[0] = REG_MOTOR1_PWM;
  buffer[1] = static_cast<int>(freq/50);
  if (write(fd, buffer, 2) != 2) {
    std::cerr << "Error writing to I2C bus." << std::endl;
  }
  usleep(1000);
}

void setMotorSpeed(int fd, int motorId, int duty, int direction)
{

  char buffer[3] = {0};
  if (motorId == 0)
  {
    buffer[0] = REG_MOTOR1_ORIENTATION;
  }else if (motorId == 1)
  {
    buffer[0] = REG_MOTOR2_ORIENTATION;
  }
  buffer[1] = direction;
  if (write(fd, buffer, 2) != 2) {
    std::cerr << "Error writing to I2C bus." << std::endl;
  }
  usleep(1000);

  if (motorId == 0)
  {
    buffer[0] = REG_MOTOR1_SPEED;
  }else if (motorId == 1)
  {
    buffer[0] = REG_MOTOR2_SPEED;
  }
  buffer[1] = duty;
  if (write(fd, buffer, 3) != 3) {
    std::cerr << "Error writing to I2C bus." << std::endl;
  }

  usleep(1000);
}

void setMotor(int i2c_fd, int motorId, float duty)
{
  duty = std::max(-1.0f, std::min(duty,1.0f));
  char dutyByte = 100*std::abs(duty);
  char direction = duty > 0 ? 1:2;
  setMotorSpeed(i2c_fd,motorId,dutyByte,direction);
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
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist &twist) const {
  
      std::cout << "I heard: " << twist.linear.x << " " << twist.angular.z << std::endl;
      dfrobot::setMotor(i2c_fd, 0, twist.linear.x - twist.angular.z);
      dfrobot::setMotor(i2c_fd, 1, twist.linear.x + twist.angular.z);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  int i2c_fd =0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
