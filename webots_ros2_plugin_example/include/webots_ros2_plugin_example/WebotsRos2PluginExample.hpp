#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace webots_ros2_plugin_example
{
  class WebotsRos2PluginExample : public webots_ros2_driver::PluginInterface
  {
  public:
    // Your plugin has to override step() and init() methods
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    bool preStep();
    void publishData();
    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;
    geometry_msgs::msg::TwistStamped cmd_vel_msg;

    // Motors
    WbDeviceTag right_motor;
    WbDeviceTag left_motor;

    double publish_timestep;
    bool mAlwaysOn;
    int mPublishTimestepSyncedMs;
    double last_update;

  };
}
#endif