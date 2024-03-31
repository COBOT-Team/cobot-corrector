#ifndef COBOT_CORRECTOR__COBOT_CORRECTOR_NODE_HPP_
#define COBOT_CORRECTOR__COBOT_CORRECTOR_NODE_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

#include <cobot_corrector_params.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
   * @param[in] msg
   */
  void robot_description_callback_(const std_msgs::msg::String::SharedPtr msg);

  /**
   * Callback for the joint states subscriber.
   *
   * @param[in] msg
   */
  void joint_states_callback_(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * Callback for the end effector pose subscriber.
   *
   * @param[in] msg
   */
  void eef_pose_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::unique_ptr<cobot_corrector_params::ParamListener> param_listener_;
  std::unique_ptr<cobot_corrector_params::Params> params_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commands_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> eef_pose_sub_;

  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>> eef_pose_filter_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // The listener for transforms.
  tf2_ros::Buffer::SharedPtr tf_buffer_;                     // The buffer for storing transforms.

  KDL::Chain kdl_chain_;               // The kinematic chain of the cobot.
  KDL::JntArray kdl_joint_positions_;  // The joint positions of the cobot.
};

#endif