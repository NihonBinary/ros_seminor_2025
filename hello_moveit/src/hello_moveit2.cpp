#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[])
{
  // ROSを初期化してノードを作成
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // ロガーを生成
  auto const logger = rclcpp::get_logger("hello_moveit2");

  // Executorを使って非同期でスピン（これによりMoveIt!の状態が準備できる）
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);  // ノードをExecutorに追加
  std::thread spin_thread([&executor]() {
      executor.spin();  // Executorを非同期でスピン
  });
  
  // MoveIt MoveGroup Interfaceを作成
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // MoveItVisualToolsを構築し初期化
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  
  // 可視化のためのクロージャを作成

  // ロボットの直上にタイトルを表示
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  // プログラムをブロックして、ユーザーに次のステップを促す
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  // ロボットの軌道をRvizに描画する
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")]
      (auto const trajectory) {
        //moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);  // -> [ERROR] [*] [moveit_visual_tools]: Unable to get end effector tips from jmg
        const moveit::core::LinkModel* end_effector_link = jmg->getLinkModel("tool0");
        moveit_visual_tools.publishTrajectoryLine(trajectory, end_effector_link, jmg);
      };

  // ターゲットポーズ
  auto const start_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 1.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.5;
    msg.position.y = 0.5;
    msg.position.z = 0.5;
    return msg;
  }();

  // 目標位置を作成
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose = start_pose;
  target_pose.position.z -= 0.15;
  waypoints.push_back(target_pose);
  target_pose.position.x -= 0.15;
  waypoints.push_back(target_pose);
  target_pose.position.z += 0.15;
  waypoints.push_back(target_pose);
  target_pose.position.x += 0.15;
  waypoints.push_back(target_pose);

  // 速度と加速度のスケーリングを設定
  double velocity_scaling_factor = 0.1;  // 速度スケーリング (0.0〜1.0)
  double acceleration_scaling_factor = 0.1;  // 加速度スケーリング (0.0〜1.0)
  move_group_interface.setMaxVelocityScalingFactor(velocity_scaling_factor);
  move_group_interface.setMaxAccelerationScalingFactor(acceleration_scaling_factor);

  // ユーザに実行を促す
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  
  // Cartesian経路を計算
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;  // 1cm刻み
  move_group_interface.setStartStateToCurrentState();
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;

  if (plan.trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(logger, "Trajectory is EMPTY!");
  } else {
      RCLCPP_INFO(logger, "Trajectory has %zu points", plan.trajectory.joint_trajectory.points.size());
  }

  // 軌跡を可視化
  draw_trajectory_tool_path(plan.trajectory);
  moveit_visual_tools.trigger();
  // ユーザに実行を促す
  prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
  draw_title("Executing");
  moveit_visual_tools.trigger();

  // 実際にプランを実行
  move_group_interface.execute(plan);
  
  // ROSをシャットダウン
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
