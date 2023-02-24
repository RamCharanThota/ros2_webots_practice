#include "webots_ros2_plugin_example/WebotsRos2PluginExample.hpp"

#include "rclcpp/rclcpp.hpp"
#include <webots/motor.h>
#include <webots/robot.h>
#include <functional>
#include <cstdio>

using std::placeholders::_1;

#define HALF_DISTANCE_BETWEEN_WHEELS 0.08
#define WHEEL_RADIUS 0.033

namespace webots_ros2_plugin_example
{
  void WebotsRos2PluginExample::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // This method is executed once the plugin is loaded by the `webots_ros2_driver` package.
    // The `webots_ros2_driver::WebotsNode` inherits the `rclcpp::Node`, so you have all methods available from there.
    // The C API of the Webots controller library must be used.
    
    // Print a simple message to see if your plugin has been loaded correctly:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello from WebotsRos2PluginExample!");
    right_motor = wb_robot_get_device("right wheel motor");
    left_motor = wb_robot_get_device("left wheel motor");
  
    // This parameter is read when loading the URDF file
    publish_timestep = parameters.count("updateRate") ? 1.0 / atof(parameters["updateRate"].c_str()) : 0;

    // Initialize motors
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0.0);
    wb_motor_set_velocity(left_motor, 0.0);
    
    cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel_plugin",
                                                                                       rclcpp::SensorDataQoS().reliable(),
                                                                                       std::bind(&MyRobotDriverPlugin::cmd_vel_callback, 
                                                                                       this, _1));

  }
  void WebotsRos2PluginExample::step()
  {
    // This method is executed on each Webots step
    auto forward_speed = cmd_vel_msg.twist.linear.x;
    auto angular_speed = cmd_vel_msg.twist.angular.z;

    auto command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS;
    auto command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS;

    wb_motor_set_velocity(left_motor, command_motor_left);
    wb_motor_set_velocity(right_motor, command_motor_right);
  }

  bool WebotsRos2PluginExample::preStep() {
    // Update only if needed
    if (wb_robot_get_time() - last_update < publish_timestep)
      return false;
    last_update = wb_robot_get_time();
    return true;
  }

  void WebotsRos2PluginExample::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    cmd_vel_msg.header = msg->header;
    cmd_vel_msg.twist = msg->twist;
  }

}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_ros2_plugin_example::WebotsRos2PluginExample, webots_ros2_driver::PluginInterface)