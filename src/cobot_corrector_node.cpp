#include "cobot_corrector/cobot_corrector_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cstdlib>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <opencv2/aruco.hpp>

using namespace cobot_corrector_params;
using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

using CorrectCobotMsg = chess_msgs::srv::CorrectCobot;

//                                                                                                //
// ========================================== General =========================================== //
//                                                                                                //

CobotCorrectorNode::CobotCorrectorNode(rclcpp::Node::SharedPtr node) : node(node)
{
  srand(time(nullptr));

  // Grab params.
  param_listener_ = std::make_unique<ParamListener>(node);
  params_ = std::make_unique<Params>(param_listener_->get_params());

  // Init tf buffer and listener.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Init image transport.
  it_ = make_shared<image_transport::ImageTransport>(node);

  // Init publishers.
  const auto commands_pub_name = apply_prefix_(params_->publishers.commands, "/");
  commands_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(commands_pub_name, 10);

  // Init subscribers.
  robot_description_sub_ = node->create_subscription<std_msgs::msg::String>(
      params_->subscriptions.robot_description, 10,
      bind(&CobotCorrectorNode::robot_description_callback_, this, _1));
  joint_states_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      params_->subscriptions.joint_states, 10,
      bind(&CobotCorrectorNode::joint_states_callback_, this, std::placeholders::_1));
  camera_sub_ = make_shared<image_transport::CameraSubscriber>(
      it_->subscribeCamera(params_->subscriptions.camera_base_topic, 1,
                           bind(&CobotCorrectorNode::camera_callback_, this, _1, _2)));

  // Init service.
  const auto service_name = apply_prefix_(params_->services.execute, "/");
  execute_srv_ = node->create_service<CorrectCobotMsg>(
      service_name, bind(&CobotCorrectorNode::execute_srv_callback_, this, _1, _2));
}

string CobotCorrectorNode::apply_prefix_(const string val, const string join)
{
  const string& prefix = params_->cobot_prefix;
  return prefix.empty() ? val : prefix + join + val;
}

//                                                                                                //
// ========================================= Callbacks ========================================== //
//                                                                                                //

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
  const auto base_link = apply_prefix_(params_->links.base, "_");
  const auto eef_aruco_link = apply_prefix_(params_->links.eef_aruco, "_");
  if (!tree.getChain(base_link, eef_aruco_link, kdl_chain_)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL chain from KDL tree.");
    return;
  }

  // Set the joint array size.
  const auto num_joints = kdl_chain_.getNrOfJoints();
  if (kdl_joint_positions_.rows() != num_joints) {
    RCLCPP_INFO(node->get_logger(), "Resizing joint positions array to %d.", num_joints);
    kdl_joint_positions_.resize(num_joints);
  }
  if (mutation_factors_.rows() != num_joints) {
    RCLCPP_INFO(node->get_logger(), "Resizing mutation factors array to %d.", num_joints);
    mutation_factors_.resize(num_joints);
  }
}

void CobotCorrectorNode::joint_states_callback_(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const auto num_segments = kdl_chain_.getNrOfSegments();

  // Loop through all joints in our KDL chain and match them to a joint state.
  size_t joint_idx = 0;
  for (size_t i = 0; i < num_segments; ++i) {
    const auto seg = kdl_chain_.getSegment(i);
    const auto joint = seg.getJoint();
    if (joint.getType() == KDL::Joint::None) continue;
    const auto joint_name = joint.getName();

    // Search our JointState message for a joint position that matches our joint's name.
    const auto joint_pos = [&] {
      const auto it = find(msg->name.begin(), msg->name.end(), joint_name);
      if (it == msg->name.end()) {
        RCLCPP_WARN(node->get_logger(), "Joint %s not found in joint_states", joint_name);
        return 0.0;
      }
      const auto idx = distance(msg->name.begin(), it);
      return msg->position[idx];
    }();

    // Search our joint mutation factors for a match.
    const auto mutation_factor = [&] {
      const auto& names = params_->genetic_alg.joints.names;
      const auto& factors = params_->genetic_alg.joints.mutation_factor;
      const auto it = find(names.begin(), names.end(), joint_name);
      if (it == names.end()) {
        RCLCPP_WARN(node->get_logger(), "Joint %s not found in mutation factors", joint_name);
        return 0.0;
      }
      const auto idx = distance(names.begin(), it);
      return factors[idx];
    }();

    // Update the joint position and mutation factor.
    kdl_joint_positions_(joint_idx) = joint_pos;
    mutation_factors_(joint_idx) = mutation_factor;
    ++joint_idx;
  }
}

void CobotCorrectorNode::camera_callback_(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo)
{
  // Get the camera frame.
  camera_frame_ = cinfo->header.frame_id;

  // Get the camera matrix and distortion coefficients.
  cv::Mat(3, 3, CV_64F, (void*)cinfo->k.data()).copyTo(camera_matrix_);
  cv::Mat(1, 5, CV_64F, (void*)cinfo->d.data()).copyTo(dist_coeffs_);

  // Get the image.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image);
    image_ = cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

//                                                                                                //
// ======================================== Main service ======================================== //
//                                                                                                //

void CobotCorrectorNode::execute_srv_callback_(const shared_ptr<CorrectCobotMsg::Request> request,
                                               shared_ptr<CorrectCobotMsg::Response> response)
{
  response->success = false;
  const auto now = node->get_clock()->now();

  // Setup static aruco detector.
  static const auto aruco_detector_params = [] {
    auto params = cv::aruco::DetectorParameters();
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    return params;
  }();
  static const auto aruco_dictionary =
  cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50); static const
  cv::aruco::ArucoDetector aruco_detector(aruco_dictionary, aruco_detector_params);

  // Find aruco marker in the most recent image.
  const vector<cv::Point2f> img_points = [&] {
    vector<int> ids;
    vector<vector<cv::Point2f>> corners;
    aruco_detector.detectMarkers(image_, corners, ids);
    const auto it = find(ids.begin(), ids.end(), params_->aruco.id);
    if (it == ids.end()) return vector<cv::Point2f>();
    const auto idx = distance(ids.begin(), it);
    return corners[idx];
  }();
  if (img_points.size() != 4) {
    RCLCPP_WARN(node->get_logger(), "Couldn't find aruco marker");
    return;
  }

  // Solve PnP problem to estimate the marker's pose.
  const auto obj_points = [&] {
    const auto aruco_size = params_->aruco.size;
    return vector<cv::Point3d>{ cv::Point3d(-aruco_size, aruco_size, 0),
                                cv::Point3d(aruco_size, aruco_size, 0),
                                cv::Point3d(aruco_size, -aruco_size, 0),
                                cv::Point3d(-aruco_size, -aruco_size, 0) };
  }();
  const auto [pnp_ok, tvec, rmat] = [&] {
    cv::Vec3d rvec, tvec;
    cv::Mat rmat(3, 3, CV_64F);
    if (!solvePnP(obj_points, img_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
                  cv::SOLVEPNP_IPPE_SQUARE)) {
      RCLCPP_WARN(node->get_logger(), "Failed to solve the PnP problem");
      return make_tuple(false, tvec, rmat);
    }
    cv::solvePnPRefineVVS(obj_points, img_points, camera_matrix_, dist_coeffs_, rvec, tvec);
    cv::Rodrigues(rvec, rmat);
    return make_tuple(true, tvec, rmat);
  }();
  if (!pnp_ok) return;

  // Convert to ROS2 pose.
  const auto marker_pose_camera_frame = [&] {
    const tf2::Vector3 t(tvec(0), tvec(1), tvec(2));
    const tf2::Matrix3x3 r(rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
                           rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
                           rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2));
    tf2::Transform transform(r, t);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = camera_frame_;
    tf2::toMsg(transform, pose.pose);
    return pose;
  }();

  // Transform pose into base frame.
  geometry_msgs::msg::PoseStamped marker_pose_base_frame;
  try {
    const auto base_frame = apply_prefix_(params_->transforms.base_frame, "_");
    const auto transform =
        tf_buffer_->lookupTransform(base_frame, camera_frame_, tf2::TimePointZero);
    tf2::doTransform(marker_pose_camera_frame, marker_pose_base_frame, transform);
  } catch (tf2::TransformException& e) {
    RCLCPP_WARN(node->get_logger(), "Failed to transform pose into base frame: %s", e.what());
    return;
  }

  // Convert to KDL pose.
  KDL::Frame marker_pose_kdl;
  tf2::fromMsg(marker_pose_base_frame.pose, marker_pose_kdl);

  // Run genetic algorithm to find the actual joint angles.
  const int max_iterations = params_->genetic_alg.max_iterations;    // Max iterations
  const int generation_size = params_->genetic_alg.generation_size;  // Generation size
  const double exit_error = params_->genetic_alg.exit_error;         // Exit error
  const double top_cut = params_->genetic_alg.top_cut;               // Top cut
  const auto& axes_weights = params_->genetic_alg.axes_weights;      // Axes weights
  KDL::JntArray base_mutation_factors(mutation_factors_);            // Base mutation factors
  vector<KDL::JntArray> bases{ kdl_joint_positions_ };               // Base joint positions
  KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);              // Kinematics solver
  vector<pair<KDL::JntArray, double>> results;                       // Vector of results
  results.reserve(generation_size);
  for (size_t iteration = 0; iteration < max_iterations; ++iteration) {
    results.clear();

    // Run a single generation.
    for (size_t genome = 0; genome < generation_size; ++genome) {
      // Calculate random mutations.
      const auto mutations = [&] {
        KDL::JntArray mutations(base_mutation_factors.rows());
        for (size_t i = 0; i < mutations.rows(); ++i) {
          const double rand_num = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
          mutations(i) = base_mutation_factors(i) * (2 * rand_num - 1);
        }
        return mutations;
      }();

      // Apply mutations to a random base.
      const auto mutated = [&] {
        const size_t idx = rand() % bases.size();
        KDL::JntArray mutated(bases[idx]);
        mutated.data += mutations.data;
        return mutated;
      }();

      // Calculate forward kinematics.
      KDL::Frame cartpos;
      const auto fk_pose = fksolver.JntToCart(mutated, cartpos);

      // Calculate error between the FK frame and the pose measured by the camera.
      const auto error = [&] {
        const auto delta = marker_pose_kdl.p - cartpos.p;
        const auto rot = cartpos.M.Inverse() * marker_pose_kdl.M;
        const auto rot_axis = rot.GetRot();
        return delta.x() * axes_weights.x + delta.y() * axes_weights.y +
               delta.z() * axes_weights.z + rot_axis.x() * axes_weights.roll +
               rot_axis.y() * axes_weights.pitch + rot_axis.z() * axes_weights.yaw;
      }();

      // Add to results.
      results.emplace_back(make_pair(mutated, error));
    }

    // Sort the results.
    sort(results.begin(), results.end(),
         [](const auto& a, const auto& b) { return a.second < b.second; });

    // If the lowest error is below the exit error, exit early.
    if (results.front().second < exit_error) {
      RCLCPP_INFO(node->get_logger(), "An acceptable solution was found early");
      break;
    }

    // Take the top cut and place them in `bases` for the next generation to build off of.
    const auto top_cut_idx = static_cast<size_t>(generation_size * top_cut);
    bases.clear();
    for (size_t i = 0; i < top_cut_idx; ++i) bases.push_back(results[i].first);

    // If we're not at the last iteration, adjust the mutation factors.
    if (iteration < max_iterations - 1) {
      const auto cooldown =
          1.0 - static_cast<double>(iteration) / static_cast<double>(max_iterations);
      base_mutation_factors.data *= cooldown;
    }
  }
  RCLCPP_INFO(node->get_logger(), "Found solution with error %f", results.front().second);

  // Publish the results.
  // TODO: is this guaranteed to be in order?
  const auto& best = results.front().first;
  auto msg = std_msgs::msg::Float64MultiArray();
  for (size_t i = 0; i < best.rows(); ++i) {
    msg.data.emplace_back(best(i));
  }
  commands_pub_->publish(msg);

  response->success = true;
}
