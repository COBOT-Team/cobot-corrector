#include "cobot_corrector/cobot_corrector_node.hpp"

#include <kdl_parser/kdl_parser.hpp>

using namespace cobot_corrector_params;
using namespace std;

CobotCorrectorNode::CobotCorrectorNode(rclcpp::Node::SharedPtr node) : node(node)
{
  param_listener_ = std::make_unique<ParamListener>(node);
  params_ = std::make_unique<Params>(param_listener_->get_params());

  // Init tf buffer and listener.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Init publishers.
  commands_pub_ =
      node->create_publisher<std_msgs::msg::Float64MultiArray>(params_->publishers.commands, 10);

  // Init subscribers.
  robot_description_sub_ = node->create_subscription<std_msgs::msg::String>(
      params_->subscriptions.robot_description, 10,
      std::bind(&CobotCorrectorNode::robot_description_callback_, this, std::placeholders::_1));
  joint_states_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      params_->subscriptions.joint_states, 10,
      std::bind(&CobotCorrectorNode::joint_states_callback_, this, std::placeholders::_1));

  // Init message filters.
  eef_pose_sub_.subscribe(node, params_->subscriptions.eef_pose);
  eef_pose_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>(
      eef_pose_sub_, *tf_buffer_, params_->transforms.base_frame, 100,
      node->get_node_logging_interface(), node->get_node_clock_interface());
}

void CobotCorrectorNode::robot_description_callback_(const std_msgs::msg::String::SharedPtr msg)
{
  // Parse the URDF model.
  urdf::Model model;
  if (!model.initString(msg->data)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF model.");
    return;
  }

  // Get the kinematic tree.
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL tree from URDF model.");
    return;
  }

  // Get the kinematic chain from the tree.
  if (!tree.getChain(params_->links.base, params_->links.last_link, kdl_chain_)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL chain from KDL tree.");
    return;
  }

  // Set the joint array size.
  const auto num_joints = kdl_chain_.getNrOfJoints();
  if (kdl_joint_positions_.rows() != num_joints) {
    RCLCPP_INFO(node->get_logger(), "Resizing joint positions array to %d.", num_joints);
    kdl_joint_positions_.resize(num_joints);
  }
}