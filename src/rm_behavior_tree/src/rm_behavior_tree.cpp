#include "rm_behavior_tree/rm_behavior_tree.h"
#include "rm_behavior_tree/bt_conversions.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS 2 系统
  rclcpp::init(argc, argv);

  // 创建一个 BehaviorTreeFactory 对象，用于构建行为树
  BT::BehaviorTreeFactory factory;

  // 定义变量存储行为树的 XML 文件路径
  std::string bt_xml_path;

  // 创建一个 ROS 2 节点，用于行为树的运行
  auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree");

  // 声明和获取行为树 XML 文件路径的参数
  node->declare_parameter<std::string>(
    "style", "./rm_decision_ws/rm_behavior_tree/rm_behavior_tree.xml");
  node->get_parameter_or<std::string>(
    "style", bt_xml_path, "./rm_decision_ws/rm_behavior_tree/config/attack_left.xml");

  // 输出日志信息，显示正在加载的行为树 XML 文件路径
  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  // 创建 ROS 节点参数对象，分别用于消息更新、机器人控制和发送目标
  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

  BT::RosNodeParams params_robot_control;
  params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
  params_robot_control.default_port_value = "robot_control";

  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
  params_send_goal.default_port_value = "goal_pose";

  // 定义消息更新插件库的名称
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_all_robot_hp",
    "sub_robot_status",
    "sub_game_status",
    "sub_armors",
    "sub_decision_num",
  };

  // 定义行为树插件库的名称
  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "decision_switch",
    "is_game_time",
    "is_status_ok",
    "is_detect_enemy",
    "is_attacked",
    "is_friend_ok",
    "is_outpost_ok",
    "get_current_location",
    "move_around",
    "print_message",
  };

  // 注册所有消息更新插件库到 BehaviorTreeFactory
  for (const auto & p : msg_update_plugin_libs) {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
  }

  // 注册所有行为树插件库到 BehaviorTreeFactory
  for (const auto & p : bt_plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  // 注册发送目标和机器人控制的 ROS 节点
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);

  // 从指定的 XML 文件创建行为树
  auto tree = factory.createTreeFromFile(bt_xml_path);

  // 连接 Groot2Publisher 以允许 Groot2 获取行为树并轮询状态更新
  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  // 在 ROS 2 系统正常运行时，不断执行行为树
  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  // 关闭 ROS 2 系统
  rclcpp::shutdown();
  return 0;
}
