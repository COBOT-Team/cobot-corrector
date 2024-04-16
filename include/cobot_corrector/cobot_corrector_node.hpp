#ifndef COBOT_CORRECTOR__COBOT_CORRECTOR_NODE_HPP_
#define COBOT_CORRECTOR__COBOT_CORRECTOR_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

#include <chess_msgs/srv/correct_cobot.hpp>
#include <cobot_corrector_params.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
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
   * Applies the cobot's prefix to a string.
   *
   * @param[in] val The string to apply the prefix to.
   * @return The string with the prefix applied.
   */
  std::string apply_prefix_(const std::string val, const std::string join = "_");

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
   * Callback for camera subscriber.
   *
   * @param[in] image
   * @param[in] cinfo
   */
  void camera_callback_(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo);

  /**
   * Callback that corrects the cobot joints.
   *
   * @param[in] request
   * @param[out] response
   */
  void execute_srv_callback_(const std::shared_ptr<chess_msgs::srv::CorrectCobot::Request> request,
                             std::shared_ptr<chess_msgs::srv::CorrectCobot::Response> response);

  std::unique_ptr<cobot_corrector_params::ParamListener> param_listener_;
  std::unique_ptr<cobot_corrector_params::Params> params_;

  std::shared_ptr<image_transport::ImageTransport> it_;

  rclcpp::Service<chess_msgs::srv::CorrectCobot>::SharedPtr execute_srv_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commands_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr stamped_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::shared_ptr<image_transport::CameraSubscriber> camera_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // The listener for transforms.
  tf2_ros::Buffer::SharedPtr tf_buffer_;                     // The buffer for storing transforms.

  std::string camera_frame_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat image_;
  cv::Ptr<cv::aruco::GridBoard> aruco_board_;

  bool found_image_;
  bool found_robot_desc_;
  bool found_joint_states_;

  KDL::Chain kdl_chain_;               // The kinematic chain of the cobot.
  KDL::JntArray kdl_joint_positions_;  // The joint positions of the cobot.
  KDL::JntArray mutation_factors_;     // The mutation factors for each joint.
};

#endif