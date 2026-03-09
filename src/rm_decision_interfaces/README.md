# 虚拟裁判系统和视觉部分，发布假消息

- robot_status

    ```zsh
    source install/setup.zsh
    ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus "{
        robot_id: 7,
        current_hp: 200,
        shooter_heat: 19,
    }"
    ```

- game_status

  ```zsh
  source install/setup.zsh
    ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus "{
        game_progress: 3,
        stage_remain_time: 5,
    }"
  ```

- robot_hp

    ```zsh
    source install/setup.zsh
    ros2 topic pub -r 3 /robot_hp rm_decision_interfaces/msg/AllRobotHP "{
        red_1_robot_hp: 200,
        red_2_robot_hp: 90,
        red_3_robot_hp: 100,
        red_4_robot_hp: 40,
        red_5_robot_hp: 200,
        red_7_robot_hp: 200,
        red_outpost_hp: 0,
        red_base_hp: 30,
        blue_1_robot_hp: 95,
        blue_2_robot_hp: 85,
        blue_3_robot_hp: 75,
        blue_4_robot_hp: 200,
        blue_5_robot_hp: 55,
        blue_7_robot_hp: 45,
        blue_outpost_hp: 35,
        blue_base_hp: 25
    }"
    ```

- friend_location

    ```zsh
    source install/setup.zsh
    ros2 topic pub -r 1 /friend_location rm_decision_interfaces/msg/FriendLocation "{
        hero_x: 2.75, hero_y: 0.0, 
        engineer_x: 10.36, engineer_y: -3.79, 
        standard_3_x: 7.91, standard_3_y: 3.74, 
        standard_4_x: 12.00, standard_4_y: 4.15, 
        standard_5_x: 18.75, standard_5_y: 0.0
    }" 
    ```

    `hero`: 己方哨兵出生点  
    `engineer`: 能量机关（靠己方）  
    `standard_3`: 己方环形高地  
    `standard_4`: 敌方前哨站  
    `standard_5`: 敌方基地  

- RFID
#include "rm_behavior_tree/plugins/action/sub_all_robot_hp.hpp"

namespace rm_behavior_tree
{

SubAllRobotHPAction::SubAllRobotHPAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::AllRobotHP>(name, conf, params)
{
}

BT::NodeStatus SubAllRobotHPAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::AllRobotHP> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, red_1_robot_hp: %s", name().c_str(),
      std::to_string(last_msg->red_1_robot_hp).c_str());
    setOutput("robot_hp", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubAllRobotHPAction, "SubAllRobotHP");
    ```zsh
    source install/setup.zsh
    ros2 topic pub -r 3 /rfid rm_decision_interfaces/msg/RFID "{
        rfid_patrol_status: 0
    }"
