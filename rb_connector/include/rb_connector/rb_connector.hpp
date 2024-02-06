
#ifndef RB_CONNECTOR_H
#define RB_CONNECTOR_H

#include <thread>
#include <memory>
#include <chrono>
#include <vector>
#include <iostream>
#include <cerrno>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>

#include <actionlib_msgs/msg/goal_status_array.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_controller_state.hpp>

#include <rb_connector/msg/rb_command.hpp>
#include <rb_connector/msg/rb_data.hpp>

#include "common_header.hpp"

// for socket client
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#include <signal.h>
#include <sstream>

#include <fstream>
#include <iostream>

#define RB_CMD_PORT 5000
#define RB_DATA_PORT 5001
#define RX_DATA_SIZE 1000
#define NO_OF_JNT 6

class RbRobot : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  RbRobot();
  void initialize();
  void rb_send_command(std::string cmd);
  void updateJoint();

  int sock_data;
  int sock_command;

private:
  std::string default_ip;
  std::string ip;

  struct sockaddr_in server_data;
  struct sockaddr_in server_command;

  std::thread THREAD_T_DATA;
  std::thread THREAD_T_COMMAND;

  int threadWorking_Data;
  int threadWorking_Command;
  int connectionStatus_Data;
  int connectionStatus_Command;
  int command_seq;

  systemSTAT systemStat;
  systemCONFIG systemConfig;
  systemPOPUP systemPopup;

  const float D2Rf = 0.0174533;
  const float R2Df = 57.2957802;

  std::array<std::string, NO_OF_JNT> JNTNameList;

  rb_connector::msg::RbData::SharedPtr RB_DATA;
  rb_connector::msg::RbCommand::SharedPtr RB_COMMAND;

  std::array<float, NO_OF_JNT> moveit_excute_final_pos;
  int update_flag;
  int pause_flag;

  sensor_msgs::msg::JointState joint_state;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<rb_connector::msg::RbData>::SharedPtr rb_data_pub_;
  rclcpp::Publisher<rb_connector::msg::RbCommand>::SharedPtr rb_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr rviz_update_state_pub_;
  rclcpp::Publisher<FollowJointTrajectory::Feedback>::SharedPtr action_feed_pub_;

  rclcpp::Subscription<rb_connector::msg::RbCommand>::SharedPtr rb_command_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr moveit_plan_sub_;

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> planned_trajectory_points_;

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr as_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  int connect_nonb(int sockfd, const struct sockaddr* saptr, socklen_t salen, int nsec);
  std::optional<int> CreateSocket_Data(const char* addr, int port);
  std::optional<int> CreateSocket_Command(const char* addr, int port);
  int Connect2Server_Data();
  int Connect2Server_Command();

  void Thread_Data();
  void Thread_Command();

  void createSocket();
  void UpdateRBData();

  void rb_command_callback(const rb_connector::msg::RbCommand::SharedPtr msg);
  void rb_command_publish(std::string str);
  void moveit_plan_trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);
};

#endif