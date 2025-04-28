#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

std::string loadKinematicsYamlAsString(const std::string& file_path)
{
    std::ifstream fin(file_path);
    if (!fin.is_open())
    {
        throw std::runtime_error("Failed to open kinematics.yaml");
    }

    std::stringstream buffer;
    buffer << fin.rdbuf();
    return buffer.str();
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Load the kinematics.yaml file
  std::string package_share_directory;
  try {
      package_share_directory = ament_index_cpp::get_package_share_directory("ur_moveit_config");
  } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "Could not find package ur_moveit_config: %s", e.what());
      return 1;
  }

  std::string yaml_path = package_share_directory;
  yaml_path += "/config/kinematics.yaml";
  std::cout << "YAML path: " << yaml_path << std::endl;

  try
  {
      // Load the kinematics.yaml file
      std::string yaml_string = loadKinematicsYamlAsString(yaml_path);
      // Declare the parameter with the loaded YAML string
      node->declare_parameter("robot_description_kinematics", yaml_string);
  }
  catch (const std::exception& e)
  {
      RCLCPP_ERROR(logger, "Error loading kinematics.yaml: %s", e.what());
      return 1;
  }

  printf("1\n");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  printf("1.5\n");
  
  
  move_group_interface.setStartStateToCurrentState();

  printf("2\n");


  // // まず、現在の状態を取得する
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(1.0);  // 10秒待つ
  //moveit::core::RobotState current_state = move_group_interface.getStartState();  // 10秒待つ

  // // 現在のエンドエフェクタの姿勢を取得する
  const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("tool0");  // "tool0" はエンドエフェクタのリンク名

  // Eigen型なので、必要に応じて分解して出力できる
  std::cout << "Translation: \n" << end_effector_state.translation() << std::endl;
  std::cout << "Rotation: \n" << end_effector_state.rotation() << std::endl;

  // この関数では、現在の位置が取得できていません。
  auto target_pose_stamped = move_group_interface.getCurrentPose("tool0");
  std::cout << "TEST" << std::endl;
  std::cout << "Target Pose:" << std::endl;
  std::cout << "  Position - x: " << target_pose_stamped.pose.position.x
            << ", y: " << target_pose_stamped.pose.position.y
            << ", z: " << target_pose_stamped.pose.position.z << std::endl;
  std::cout << "  Orientation - x: " << target_pose_stamped.pose.orientation.x
            << ", y: " << target_pose_stamped.pose.orientation.y
            << ", z: " << target_pose_stamped.pose.orientation.z
            << ", w: " << target_pose_stamped.pose.orientation.w << std::endl;
  
  // 何故か固定値が返ってくる
  // Position - x: 0.8172, y: 0.2329, z: 0.0628
  // Orientation - x: 2.00307e-16, y: 0.707107, z: 0.707107, w: -4.32978e-17
  #if 0

  auto target_pose = target_pose_stamped.pose;
  target_pose.position.z += 0.1;

  #else
  //Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = -1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = -0.5;
    return msg;
  }();
  #endif

  move_group_interface.setPoseTarget(target_pose);

  printf("3\n");

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  printf("4\n");

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  printf("5\n");

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}