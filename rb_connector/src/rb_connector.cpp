
#include "rb_connector.hpp"

RbRobot::RbRobot()
  : Node("rb_connector")
  , JNTNameList({ "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3" })
  , moveit_excute_final_pos({ 999, 999, 999, 999, 999, 999 })
{
  initialize();

  this->as_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, "follow_joint_trajectory",
      std::bind(&RbRobot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RbRobot::handle_cancel, this, std::placeholders::_1),
      std::bind(&RbRobot::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Hello ROS 2 from RB_Driver!");
}

void RbRobot::initialize()
{
  default_ip = "10.0.2.7";

  this->declare_parameter<std::string>("ip", default_ip);
  this->set_parameter(rclcpp::Parameter("ip", default_ip));
  this->get_parameter("ip", ip);

  RCLCPP_INFO(this->get_logger(), "RB IP : %s", ip.c_str());

  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 100);
  rb_command_pub_ = this->create_publisher<rb_connector::msg::RbCommand>("/rb_command", 10);
  rb_data_pub_ = this->create_publisher<rb_connector::msg::RbData>("/rb_data", 10);
  rviz_update_state_pub_ = this->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/update_start_state", 1);
  action_feed_pub_ = this->create_publisher<FollowJointTrajectory::Feedback>("/follow_joint_trajectory/feedback", 10);

  rb_command_sub_ = this->create_subscription<rb_connector::msg::RbCommand>(
      "/rb_command", 10, std::bind(&RbRobot::rb_command_callback, this, std::placeholders::_1));
  moveit_plan_sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "/move_group/display_planned_path", 10,
      std::bind(&RbRobot::moveit_plan_trajectory_callback, this, std::placeholders::_1));

  joint_state.name.resize(NO_OF_JNT);
  joint_state.position.resize(NO_OF_JNT);

  for (int i = 0; i < NO_OF_JNT; i++)
  {
    joint_state.name[i] = JNTNameList[i];
  }

  RB_DATA = std::make_shared<rb_connector::msg::RbData>();

  sock_data = 0;
  sock_command = 0;

  threadWorking_Data = false;
  threadWorking_Command = false;
  connectionStatus_Data = false;
  connectionStatus_Command = false;
  command_seq = 0;
  update_flag = -1;
  pause_flag = -1;

  createSocket();
}

void RbRobot::createSocket()
{
  if (!CreateSocket_Data(ip.data(), RB_DATA_PORT))
  {
    RCLCPP_ERROR(this->get_logger(), "Create Socket Error..(Data)");
    return;
  }

  try
  {
    THREAD_T_DATA = std::thread([this] { this->Thread_Data(); });
    RCLCPP_INFO(this->get_logger(), "Create Thread Succeed. (Data)");
  }
  catch (const std::system_error& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Create Thread Error..(Data): %s", e.what());
    return;
  }

  if (!CreateSocket_Command(ip.data(), RB_CMD_PORT))
  {
    RCLCPP_ERROR(this->get_logger(), "Create Socket Error..(Command)");
    return;
  }

  try
  {
    THREAD_T_COMMAND = std::thread([this] { this->Thread_Command(); });
    RCLCPP_INFO(this->get_logger(), "Create Thread Succeed. (Command)");
  }
  catch (const std::system_error& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Create Thread Error..(Command): %s", e.what());
    return;
  }
}

rclcpp_action::GoalResponse RbRobot::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RbRobot::handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;

  if (pause_flag == 0)
  {
    std::string text;
    text = "task pause";
    rb_send_command(text);

    text = "task stop";
    rb_send_command(text);

    RCLCPP_INFO(this->get_logger(), "Preempted");

    pause_flag = 1;
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RbRobot::handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&RbRobot::execute, this, _1), goal_handle }.detach();
}

void RbRobot::rb_send_command(std::string cmd)
{
  if (connectionStatus_Command)
  {
    if (command_seq == 0)
    {
      RB_DATA->command_on_passing = true;
      write(sock_command, cmd.data(), cmd.size());
      command_seq = 1;
    }

    int success = false;

    while (1)
    {
      if (command_seq == 4)
      {
        success = true;

        break;
      }
    }

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "command send done");
    }

    command_seq = 0;
    RB_DATA->command_on_passing = false;
  }
}

void RbRobot::rb_command_callback(const rb_connector::msg::RbCommand::SharedPtr msg)
{
  std::string cmd = msg->cmd;
  if (connectionStatus_Command)
  {
    if (command_seq == 0)
    {
      RB_DATA->command_on_passing = true;
      write(sock_command, cmd.data(), cmd.size());
      command_seq = 1;
    }

    int success = false;
    for (int i = 0; i < 2000; i++)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if (command_seq == 4)
      {
        success = true;
        break;
      }
    }

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "command execute done");
    }
    command_seq = 0;
    RB_DATA->command_on_passing = false;
  }
}

void RbRobot::rb_command_publish(std::string str)
{
  RB_COMMAND->header.stamp = this->now();
  RB_COMMAND->cmd = str;

  rb_command_pub_->publish(*RB_COMMAND);
}

void RbRobot::moveit_plan_trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
{
  planned_trajectory_points_ = msg->trajectory[0].joint_trajectory.points;
}

void RbRobot::execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  int vectorSize = planned_trajectory_points_.size();

  if (vectorSize > 0)
  {
    std::string text;
    text = "move_ros_j_clear()";
    rb_send_command(text);

    for (int i = 0; i < vectorSize; i++)
    {
      int time_sec = planned_trajectory_points_[i].time_from_start.sec;
      int time_nsec = planned_trajectory_points_[i].time_from_start.nanosec;

      float real_time = ((float)time_sec) + ((float)time_nsec / 1000000000.);

      float pos[6];
      float vel[6];
      float acc[6];

      for (int j = 0; j < 6; j++)
      {
        pos[j] = planned_trajectory_points_[i].positions[j] * R2Df;
        vel[j] = planned_trajectory_points_[i].velocities[j] * R2Df;
        acc[j] = planned_trajectory_points_[i].accelerations[j] * R2Df;
      }

      text = "move_ros_j_add(" + std::to_string(real_time) + ", jnt[" + std::to_string(pos[0]) + ", " +
             std::to_string(pos[1]) + ", " + std::to_string(pos[2]) + ", " + std::to_string(pos[3]) + ", " +
             std::to_string(pos[4]) + ", " + std::to_string(pos[5]) + "], jnt[" + std::to_string(vel[0]) + ", " +
             std::to_string(vel[1]) + ", " + std::to_string(vel[2]) + ", " + std::to_string(vel[3]) + ", " +
             std::to_string(vel[4]) + ", " + std::to_string(vel[5]) + "], jnt[" + std::to_string(acc[0]) + ", " +
             std::to_string(acc[1]) + ", " + std::to_string(acc[2]) + ", " + std::to_string(acc[3]) + ", " +
             std::to_string(acc[4]) + ", " + std::to_string(acc[5]) + "])";

      rb_send_command(text);
    }

    for (int i = 0; i < 6; i++)
    {
      moveit_excute_final_pos[i] = planned_trajectory_points_[vectorSize - 1].positions[i] * R2Df;
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory size : %d", vectorSize);
    text = "move_ros_j_run(0," + std::to_string(vectorSize) + ")";
    rb_send_command(text);

    RCLCPP_INFO(this->get_logger(), "========== Trajectory transferd ==========  ");
  }
}

std::optional<int> RbRobot::CreateSocket_Data(const char* addr, int port)
{
  // 소켓 생성
  int sock_data = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_data == -1)
  {
    return std::nullopt;  // 실패 시 std::nullopt 반환
  }

  server_data.sin_addr.s_addr = inet_addr(addr);
  server_data.sin_family = AF_INET;
  server_data.sin_port = htons(port);

  int optval = 1;
  if (setsockopt(sock_data, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1 ||
      setsockopt(sock_data, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) == -1)
  {
    close(sock_data);  // 소켓 설정 실패 시 리소스 정리
    return std::nullopt;
  }

  // 소켓 설정이 성공적으로 완료되면 소켓 파일 디스크립터 반환
  return sock_data;
}

std::optional<int> RbRobot::CreateSocket_Command(const char* addr, int port)
{
  // 소켓 생성
  int sock_command = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_command == -1)
  {
    return std::nullopt;  // 실패 시 std::nullopt 반환
  }

  server_command.sin_addr.s_addr = inet_addr(addr);
  server_command.sin_family = AF_INET;
  server_command.sin_port = htons(port);

  int optval = 1;
  if (setsockopt(sock_command, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1 ||
      setsockopt(sock_command, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) == -1)
  {
    close(sock_command);  // 소켓 설정 실패 시 리소스 정리
    return std::nullopt;
  }

  // 소켓 설정이 성공적으로 완료되면 소켓 파일 디스크립터 반환
  return sock_command;
}

int RbRobot::Connect2Server_Data()
{
  if (sock_data == 0)
  {
    auto sock_opt = CreateSocket_Data(ip.data(), RB_DATA_PORT);
    if (!sock_opt.has_value())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create data socket.");
      return false;
    }
    sock_data = sock_opt.value();
  }

  int result = connect_nonb(sock_data, (struct sockaddr*)&server_data, sizeof(server_data), 5);
  if (result < 0)
  {
    if (result == -10)
    {
      RCLCPP_ERROR(this->get_logger(), "Non-blocking connect failed immediately with errno: %s", strerror(errno));
    }
    else if (result == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Connect timed out or failed with error: %s", strerror(errno));
    }
    close(sock_data);
    sock_data = 0;
    return false;
  }

  int flags = fcntl(sock_data, F_GETFL, 0);
  fcntl(sock_data, F_SETFL, flags | O_NONBLOCK);
  RCLCPP_INFO(this->get_logger(), "Successfully connected to server (Data).");
  return true;
}

int RbRobot::Connect2Server_Command()
{
  if (sock_command == 0)
  {
    auto sock_opt = CreateSocket_Data(ip.data(), RB_CMD_PORT);
    if (!sock_opt.has_value())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create data socket.");
      return false;
    }
    sock_command = sock_opt.value();
  }

  int result = connect_nonb(sock_command, (struct sockaddr*)&server_command, sizeof(server_command), 5);
  if (result < 0)
  {
    if (result == -10)
    {
      RCLCPP_ERROR(this->get_logger(), "Non-blocking connect failed immediately with errno: %s", strerror(errno));
    }
    else if (result == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Connect timed out or failed with error: %s", strerror(errno));
    }
    close(sock_command);
    sock_command = 0;
    return false;
  }

  int flags = fcntl(sock_command, F_GETFL, 0);
  fcntl(sock_command, F_SETFL, flags | O_NONBLOCK);
  RCLCPP_INFO(this->get_logger(), "Successfully connected to server (Command).");
  return true;
}

void RbRobot::updateJoint()
{
  joint_state.header.stamp = this->now();

  for (int i = 0; i < NO_OF_JNT; i++)
  {
    joint_state.position[i] = systemStat.sdata.jnt_ref[i] * D2Rf;
  }

  joint_pub_->publish(joint_state);

  // GoalID goal_id
  // uint8 status
  // uint8 PENDING         = 0   # The goal has yet to be processed by the action server
  // uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
  // uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
  //                             #   and has since completed its execution (Terminal State)
  // uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
  // uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due

  // RB_DATA->robot_state
  // 0 = None
  // 1 = Idle
  // 2 = Pause
  // 3 = Moving

  FollowJointTrajectory::Feedback actionFeedback;

  actionFeedback.header.stamp = this->now();

  if (RB_DATA->robot_state == 1)
  {
    // actionFeedback.status.goal_id.id = "";
    // actionFeedback.status.status = 3;
    // actionFeedback.status.text = "IDLE";
    pause_flag = 0;
  }
  else if (RB_DATA->robot_state == 2)
  {
    // actionFeedback.status.goal_id.id = "";
    // actionFeedback.status.status = 2;
    // actionFeedback.status.text = "PAUSE";
  }
  else if (RB_DATA->robot_state == 3)
  {
    // actionFeedback.status.goal_id.id = "";
    // actionFeedback.status.status = 1;
    // actionFeedback.status.text = "MOVING";
  }

  actionFeedback.joint_names = { "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3" };

  sensor_msgs::msg::JointState error_state;
  error_state.name.resize(NO_OF_JNT);
  error_state.position.resize(NO_OF_JNT);

  for (int i = 0; i < NO_OF_JNT; i++)
  {
    error_state.position[i] = 0.0f;
  }

  actionFeedback.actual.positions = joint_state.position;
  actionFeedback.desired.positions = joint_state.position;
  actionFeedback.error.positions = error_state.position;

  action_feed_pub_->publish(actionFeedback);
}

void RbRobot::UpdateRBData()
{
  RB_DATA->header.stamp = this->now();
  RB_DATA->time = systemStat.sdata.time;
  for (int i = 0; i < 6; i++)
  {
    RB_DATA->joint_reference[i] = systemStat.sdata.jnt_ref[i];
    RB_DATA->joint_encoder[i] = systemStat.sdata.jnt_ang[i];
    RB_DATA->joint_current[i] = systemStat.sdata.cur[i];
    RB_DATA->tcp_reference[i] = systemStat.sdata.tcp_ref[i];
    RB_DATA->temperature[i] = systemStat.sdata.temperature_mc[i];
    RB_DATA->joint_information[i] = systemStat.sdata.jnt_info[i];
  }

  RB_DATA->task_state = systemStat.sdata.task_state;
  RB_DATA->robot_state = systemStat.sdata.robot_state;
  RB_DATA->power_state = systemStat.sdata.power_state;
  RB_DATA->collision_detect = systemStat.sdata.collision_detect_onoff;
  RB_DATA->freedrive_mode = systemStat.sdata.is_freedrive_mode;
  RB_DATA->program_mode = systemStat.sdata.program_mode;

  RB_DATA->op_stat_collision_occur = systemStat.sdata.op_stat_collision_occur;
  RB_DATA->op_stat_sos_flag = systemStat.sdata.op_stat_sos_flag;
  RB_DATA->op_stat_self_collision = systemStat.sdata.op_stat_self_collision;
  RB_DATA->op_stat_soft_estop_occur = systemStat.sdata.op_stat_soft_estop_occur;
  RB_DATA->op_stat_ems_flag = systemStat.sdata.op_stat_ems_flag;

  for (int i = 0; i < 4; i++)
  {
    RB_DATA->analog_in[i] = systemStat.sdata.analog_in[i];
    RB_DATA->analog_out[i] = systemStat.sdata.analog_out[i];
  }
  for (int i = 0; i < 16; i++)
  {
    RB_DATA->digital_in[i] = systemStat.sdata.digital_in[i];
    RB_DATA->digital_out[i] = systemStat.sdata.digital_out[i];
  }
  for (int i = 0; i < 2; i++)
  {
    RB_DATA->tfb_analog_in[i] = systemStat.sdata.tfb_analog_in[i];
    RB_DATA->tfb_digital_in[i] = systemStat.sdata.tfb_digital_in[i];
    RB_DATA->tfb_digital_out[i] = systemStat.sdata.tfb_digital_out[i];
  }
  RB_DATA->tfb_voltage_out = systemStat.sdata.tfb_voltage_out;

  RB_DATA->default_speed = systemStat.sdata.default_speed;

  rb_data_pub_->publish(*RB_DATA);

  // update moveit start state
  float err[6];

  for (int i = 0; i < 6; i++)
  {
    err[i] = moveit_excute_final_pos[i] - RB_DATA->joint_reference[i];
  }

  // check norm
  if (sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2] + err[3] * err[3] + err[4] * err[4] + err[5] * err[5]) <
      0.01)
  {
    update_flag = 1;
  }

  if (update_flag == 1)
  {
    std_msgs::msg::Empty myMsg;
    rviz_update_state_pub_->publish(myMsg);

    RCLCPP_INFO(this->get_logger(), "Updated Start Pose");

    update_flag = 0;

    for (int i = 0; i < 6; i++)
    {
      moveit_excute_final_pos[i] = 999.0f;
    }

    FollowJointTrajectory::Result result;
    result.error_code = 0;

    // as_->setSucceeded(result, "Goal Reached");
  }
}

void RbRobot::Thread_Data()
{
  threadWorking_Data = true;

  unsigned int tcp_status = 0x00;
  int tcp_size = 0;
  int connectCnt = 0;
  int data_req_cnt = 0;

  std::vector<unsigned char> totalData;
  unsigned char recv_data[RX_DATA_SIZE];

  while (threadWorking_Data)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(400));
    if (tcp_status == 0x00)
    {
      // If client was not connected
      if (sock_data == 0)
      {
        auto sock_opt = CreateSocket_Data(ip.data(), RB_DATA_PORT);
        if (sock_opt.has_value())
        {
          sock_data = sock_opt.value();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Socket creation failed.");
          continue;
        }
      }
      if (Connect2Server_Data())
      {
        tcp_status = 0x01;
        connectionStatus_Data = true;
        connectCnt = 0;
        totalData.clear();
      }
      else
      {
        if (connectCnt % 10 == 0)
        {
          RCLCPP_INFO(this->get_logger(), "Connect to Server Failed..(Data)");
        }
        connectCnt++;
      }
      std::this_thread::sleep_for(std::chrono::nanoseconds(1000 * 1000));
    }
    if (tcp_status == 0x01)
    {
      // If client was connected
      tcp_size = recv(sock_data, recv_data, RX_DATA_SIZE, 0);

      if (tcp_size == -1)
      {
        if (errno != EWOULDBLOCK && errno != EAGAIN)
        {
          RCLCPP_ERROR(this->get_logger(), "SOCKET ERROR: %s", strerror(errno));
          // Consider reconnecting or resetting connection state
          tcp_status = 0x00;
          connectionStatus_Data = false;
          close(sock_data);
          sock_data = 0;
          continue;  // Skip processing and try reconnecting
        }
      }
      else if (tcp_size == 0)
      {
        // Peer has closed the connection
        RCLCPP_INFO(this->get_logger(), "Socket Disconnected..(Data)");
        tcp_status = 0x00;
        connectionStatus_Data = false;
        close(sock_data);
        sock_data = 0;
      }
      else if (tcp_size > 0)
      {
        // Process received data
        totalData.insert(totalData.end(), &recv_data[0], &recv_data[tcp_size]);
        while (totalData.size() > 4)
        {
          if (totalData[0] == '$')
          {
            unsigned int size =
                ((unsigned int)((unsigned char)totalData[2] << 8) | (unsigned int)((unsigned char)totalData[1]));

            if (totalData.size() >= size)
            {
              if (totalData[3] == 3)
              {
                memcpy(&systemStat, totalData.data(), sizeof(systemSTAT));
                totalData.erase(totalData.begin(), totalData.begin() + sizeof(systemSTAT));
                UpdateRBData();
                if (command_seq == 3)
                {
                  command_seq = 4;
                }
              }
              else if (totalData[3] == 4)
              {
                memcpy(&systemConfig, totalData.data(), sizeof(systemCONFIG));
                totalData.erase(totalData.begin(), totalData.begin() + sizeof(systemCONFIG));
              }
              else if (totalData[4] == 10)
              {
                memcpy(&systemPopup, totalData.data(), sizeof(systemPOPUP));
                totalData.erase(totalData.begin(), totalData.begin() + sizeof(systemPOPUP));
              }
              else
              {
                totalData.erase(totalData.begin(), totalData.begin() + 1);
              }
            }
          }
          else
          {
            totalData.erase(totalData.begin(), totalData.begin() + 1);
          }
        }
      }
      // request data -----
      data_req_cnt++;
      if (data_req_cnt % 500 == 0)
      {
        if (command_seq == 2)
        {
          write(sock_data, "reqdata\0", 8);
          command_seq = 3;
        }
        else
        {
          write(sock_data, "reqdata\0", 8);
        }
      }
    }
  }
  return;
}

void RbRobot::Thread_Command()
{
  threadWorking_Command = true;

  unsigned int tcp_status = 0x00;
  int tcp_size = 0;
  int connectCnt = 0;

  static std::vector<unsigned char> totalData;
  static char recv_data[RX_DATA_SIZE];

  while (threadWorking_Command)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    if (tcp_status == 0x00)
    {
      // If client was not connected
      if (sock_command == 0)
      {
        CreateSocket_Command(ip.data(), RB_CMD_PORT);
      }
      if (Connect2Server_Command())
      {
        tcp_status = 0x01;
        connectionStatus_Command = true;
        connectCnt = 0;
      }
      else
      {
        if (connectCnt % 10 == 0)
        {
          RCLCPP_INFO(this->get_logger(), "Connect to Server Failed..(Command)");
        }
        connectCnt++;
      }
      std::this_thread::sleep_for(std::chrono::nanoseconds(1000 * 1000));
    }

    if (tcp_status == 0x01)
    {
      // If client was connected
      tcp_size = recv(sock_command, recv_data, RX_DATA_SIZE, 0);
      if (tcp_size > 0)
      {
        std::string temp_str = recv_data;

        if (temp_str.compare("The command was executed\n") == 0)
        {
          RCLCPP_INFO(this->get_logger(), "[o] %s", temp_str.data());

          command_seq = 4;
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "[x] %s", temp_str.data());
        }
      }
    }
  }
  return;
}

int RbRobot::connect_nonb(int sockfd, const struct sockaddr* saptr, socklen_t salen, int nsec)
{
  int flags, n, error;
  socklen_t len;
  fd_set rset, wset;
  struct timeval tval;

  flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

  error = 0;
  if ((n = connect(sockfd, saptr, salen)) < 0)
  {
    if (errno != EINPROGRESS)
    {
      return (-10);
    }
  }

  if (n == 0)
  {
    goto done;  // connect completed immediately
  }

  FD_ZERO(&rset);
  FD_SET(sockfd, &rset);
  wset = rset;
  tval.tv_sec = nsec;
  tval.tv_usec = 0;

  if ((n = select(sockfd + 1, &rset, &wset, NULL, nsec ? &tval : NULL)) == 0)
  {
    errno = ETIMEDOUT;
    return (-1);  // timeout
  }

  if (FD_ISSET(sockfd, &rset) || FD_ISSET(sockfd, &wset))
  {
    len = sizeof(error);
    if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len) < 0)
    {
      return (-1);  // getsockopt failed
    }
  }
  else
  {
    return -1;  // select error: sockfd not set
  }

done:
  fcntl(sockfd, F_SETFL, flags);  // restore file status flags

  if (error)
  {
    errno = error;
    return (-1);  // connect error
  }
  return (0);  // success
}
