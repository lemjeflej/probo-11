#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ControleurRail : public rclcpp::Node {
public:
  ControleurRail() : Node("controleur_rail") {
    amp_    = this->declare_parameter<double>("amp", 0.20);      // m
    center_ = this->declare_parameter<double>("center", 0.50);   // m
    freq_   = this->declare_parameter<double>("freq", 0.10);     // Hz
    rate_   = this->declare_parameter<double>("rate", 50.0);     // Hz

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    names_ = {"rail_joint","joint1","joint2","joint3","joint4","joint5","joint6","joint7"};
    fixed_q_.assign(7, 0.0);

    t0_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&ControleurRail::tick, this)
    );

    RCLCPP_INFO(this->get_logger(), "controleur_rail: publishing /joint_states (rail sine, arm fixed).");
  }

private:
  void tick() {
    const double t = (this->now() - t0_).seconds();
    const double y = center_ + amp_ * std::sin(2.0 * M_PI * freq_ * t);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = names_;
    msg.position.reserve(8);
    msg.position.push_back(y);
    msg.position.insert(msg.position.end(), fixed_q_.begin(), fixed_q_.end());

    pub_->publish(msg);
  }

  double amp_{0.2}, center_{0.5}, freq_{0.1}, rate_{50.0};
  rclcpp::Time t0_;

  std::vector<std::string> names_;
  std::vector<double> fixed_q_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControleurRail>());
  rclcpp::shutdown();
  return 0;
}
