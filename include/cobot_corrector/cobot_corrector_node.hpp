#ifndef COBOT_CORRECTOR__COBOT_CORRECTOR_NODE_HPP_
#define COBOT_CORRECTOR__COBOT_CORRECTOR_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

#include <cobot_corrector_params.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class CobotCorrectorNode
{
public:
  rclcpp::Node::SharedPtr node;  // The ROS 2 node for the cobot corrector.

  /**
   * Construct a new Cobot Corrector Node object.
   *
   * @param[in] node The ROS 2 node for the cobot corrector.
   */
  CobotCorrectorNode(rclcpp::Node::SharedPtr node);

private:
  /**
   * Callback for the robot description subscriber.
   *
   * @param[in] msg The message from the robot description subscriber.
   */
  void robot_description_callback_(const std_msgs::msg::String::SharedPtr msg);

  std::unique_ptr<cobot_corrector_params::ParamListener> param_listener_;
  std::unique_ptr<cobot_corrector_params::Params> params_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // The listener for transforms.
  tf2_ros::Buffer::SharedPtr tf_buffer_;                     // The buffer for storing transforms.

  KDL::Chain kdl_chain_;               // The kinematic chain of the cobot.
  KDL::JntArray kdl_joint_positions_;  // The joint positions of the cobot.
};

#endif