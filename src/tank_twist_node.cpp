#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TankTwistNode : public rclcpp::Node
{
public:
  TankTwistNode()
  : Node("tank_twist_node")
  {
    // Declare parameters (same defaults as Python)
    wheel_separation_ =
      this->declare_parameter<double>("wheel_separation", 0.230);
    wheel_radius_ =
      this->declare_parameter<double>("wheel_radius", 0.11);
    wheel_separation_multiplier_ =
      this->declare_parameter<double>("wheel_separation_multiplier", 10.0);
    left_wheel_radius_multiplier_ =
      this->declare_parameter<double>("left_wheel_radius_multiplier", -0.0325);
    right_wheel_radius_multiplier_ =
      this->declare_parameter<double>("right_wheel_radius_multiplier", 0.0325);

    joy_left_track_axis_ =
      this->declare_parameter<int>("joy_left_track_axis", 1);
    joy_right_track_axis_ =
      this->declare_parameter<int>("joy_right_track_axis", 3);

    // Publisher
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1);

    // Subscription
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      1,
      std::bind(&TankTwistNode::onJoy, this, std::placeholders::_1)
    );
  }

private:
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Safety check
    if (msg->axes.size() <=
        static_cast<size_t>(std::max(joy_left_track_axis_, joy_right_track_axis_)))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Joy axes array too small"
      );
      return;
    }

    double left_track  = msg->axes[joy_left_track_axis_];
    double right_track = msg->axes[joy_right_track_axis_];

    // Apply multipliers (same as diff_drive_controller)
    double ws  = wheel_separation_multiplier_ * wheel_separation_;
    double lwr = left_wheel_radius_multiplier_ * wheel_radius_;
    double rwr = right_wheel_radius_multiplier_ * wheel_radius_;

    // Interpret joystick axis directly as wheel velocities
    double vel_left  = left_track;
    double vel_right = right_track;

    // Invert diff_drive equations to get Twist
    double lin = (vel_left * lwr + vel_right * rwr) / 2.0;
    double ang = (vel_right * rwr - vel_left * lwr) / ws;

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = lin;
    twist.angular.z = ang;

    twist_pub_->publish(twist);
  }

  // Parameters
  double wheel_separation_;
  double wheel_radius_;
  double wheel_separation_multiplier_;
  double left_wheel_radius_multiplier_;
  double right_wheel_radius_multiplier_;
  int joy_left_track_axis_;
  int joy_right_track_axis_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TankTwistNode>());
  rclcpp::shutdown();
  return 0;
}
