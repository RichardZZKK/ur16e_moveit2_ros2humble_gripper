#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> // 使用 TwistStamped
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include "ur16e_move_server/action/move_to_pose.hpp"

// TODO: Replace with your actual perception service message types
// #include "open_set_object_detection_msgs/srv/get_object_locations.hpp"
// #include "ur_msgs/srv/set_io.hpp"

using namespace std::chrono_literals;

class SuctionGripperMTP : public rclcpp::Node
{
public:
  using MoveToPose = ur16e_move_server::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;

  SuctionGripperMTP() : Node("suction_gripper_mtp_server")
  {
    // Parameters
    planning_group_ = declare_parameter<std::string>("planning_group", "left_arm");
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "left_suction_gripper_tcp");
    robot_prefix_ = declare_parameter<std::string>("robot_prefix", "left");
    
    // Motion parameters
    pick_place_height_ = declare_parameter<double>("pick_place_height", 0.3);
    look_height_ = declare_parameter<double>("look_height", 0.23);
    object_clearance_ = declare_parameter<double>("object_clearance", 0.05);
    eef_step_ = declare_parameter<double>("eef_step", 0.005);
    cartesian_min_fraction_ = declare_parameter<double>("cartesian_min_fraction", 0.90);
    
    // Force/Torque parameters
    ft_setpoint_ = declare_parameter<double>("ft_setpoint", 64.0);
    ft_error_allowance_ = declare_parameter<double>("ft_error_allowance", 1.0);
    velocity_z_ = declare_parameter<double>("velocity_z", -0.015);
    ft_control_rate_ = declare_parameter<double>("ft_control_rate", 30.0);
    
    // Gripper IO parameters
    gripper_io_fun_ = declare_parameter<int>("gripper_io_fun", 1);
    gripper_io_pin_ = declare_parameter<int>("gripper_io_pin", 12);
    gripper_activate_state_ = declare_parameter<int>("gripper_activate_state", 1);
    gripper_deactivate_state_ = declare_parameter<int>("gripper_deactivate_state", 0);
    gripper_activation_delay_ = declare_parameter<double>("gripper_activation_delay", 1.0);
    
    // Controller names
    scaled_joint_trajectory_controller_ = declare_parameter<std::string>("scaled_joint_trajectory_controller", 
                                                            "scaled_pos_joint_traj_controller");
    forward_position_controller_ = declare_parameter<std::string>("forward_position_controller", "forward_position_controller");
    
    // Pre-action pose (optional)
    use_pre_action_pose_ = declare_parameter<bool>("use_pre_action_pose", false);
    
    // Perception offsets (to correct detection errors)
    perception_offset_x_ = declare_parameter<double>("perception_offset_x", -0.01);
    perception_offset_y_ = declare_parameter<double>("perception_offset_y", -0.005);
    
    // Initialize atomic variables
    force_z_.store(0.0);
    have_joint_state_.store(false);
    
    // Subscribers
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr) {
        have_joint_state_.store(true);
      });
    
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      robot_prefix_ + "/wrench", 10,
      std::bind(&SuctionGripperMTP::wrench_callback, this, std::placeholders::_1));
    
    // Publisher
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
    
    // Service clients
    zero_ft_client_ = create_client<std_srvs::srv::Trigger>(
      robot_prefix_ + "/ur_hardware_interface/zero_ftsensor");
    
    switch_controller_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      robot_prefix_ + "/controller_manager/switch_controller");
      
    start_servo_client_ = create_client<std_srvs::srv::Trigger>(
      "/servo_node/start_servo");
    stop_servo_client_ = create_client<std_srvs::srv::Trigger>(
      "/servo_node/stop_servo");
    
    // TODO: Uncomment when you have the actual service types
    // perception_client_ = create_client<GetObjectLocations>(
    //   robot_prefix_ + "_get_object_locations");
    
    // set_io_client_ = create_client<SetIO>(
    //   robot_prefix_ + "/ur_hardware_interface/set_io");
    
    // Action server
    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "suction_pick_place",
      std::bind(&SuctionGripperMTP::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SuctionGripperMTP::on_cancel, this, std::placeholders::_1),
      std::bind(&SuctionGripperMTP::on_accept, this, std::placeholders::_1));
    
    init_timer_ = create_wall_timer(0ms, std::bind(&SuctionGripperMTP::delayed_init, this));
    
    RCLCPP_INFO(get_logger(), "Suction Gripper MTP Server initialized");
  }

private:
  // ROS components
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  
  // Service clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zero_ft_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
  
  // rclcpp::Client<GetObjectLocations>::SharedPtr perception_client_;
  // rclcpp::Client<SetIO>::SharedPtr set_io_client_;
  
  // State variables
  std::atomic<double> force_z_;
  std::atomic<bool> have_joint_state_;
  bool mgi_ready_{false};
  std::mutex exec_mtx_;
  
  // Parameters
  std::string planning_group_, end_effector_link_, robot_prefix_;
  std::string scaled_joint_trajectory_controller_, forward_position_controller_;
  double pick_place_height_, look_height_, object_clearance_;
  double eef_step_, cartesian_min_fraction_;
  double ft_setpoint_, ft_error_allowance_, velocity_z_, ft_control_rate_;
  int gripper_io_fun_, gripper_io_pin_, gripper_activate_state_, gripper_deactivate_state_;
  double gripper_activation_delay_;
  bool use_pre_action_pose_;
  double perception_offset_x_, perception_offset_y_;

  // ========== Utility Functions ==========
  
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    force_z_.store(msg->wrench.force.z);
  }
  
  geometry_msgs::msg::Pose shift_pose(const geometry_msgs::msg::Pose& p, 
                                       double dx, double dy, double dz)
  {
    auto out = p;
    out.position.x += dx;
    out.position.y += dy;
    out.position.z += dz;
    return out;
  }
  
  geometry_msgs::msg::Pose make_pose(double x, double y, double z,
                                     const geometry_msgs::msg::Quaternion& orientation)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation = orientation;
    return p;
  }
  
  bool wait_for_joint_states(double timeout)
  {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate r(100);
    while (rclcpp::ok()) {
      if (have_joint_state_.load()) return true;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > timeout)
        return false;
      r.sleep();
    }
    return false;
  }
  
  // ========== Controller Management ==========
  
  bool switch_controller(const std::string& start_controller, 
                        const std::string& stop_controller)
  {
    if (!switch_controller_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "Switch controller service not available");
      return false;
    }
    
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = {start_controller};
    request->stop_controllers = {stop_controller};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->start_asap = true;
    request->timeout = rclcpp::Duration(0, 0);
    
    auto future = switch_controller_client_->async_send_request(request);
    
    using namespace std::chrono_literals;
    auto status = future.wait_for(5s);
    
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Failed to call switch controller service - timeout");
      return false;
    }
    
    auto result = future.get();
    if (!result->ok) {
      RCLCPP_ERROR(get_logger(), "Controller switch failed");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "Switched from %s to %s", 
                stop_controller.c_str(), start_controller.c_str());
    return true;
  }
  
  bool call_trigger_service(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client, 
                            const std::string& name, double timeout_s = 2.0)
  {
    if (!client) {
      RCLCPP_ERROR(get_logger(), "Trigger service client '%s' is null.", name.c_str());
      return false;
    }
    if (!client->wait_for_service(std::chrono::duration<double>(timeout_s))) {
      RCLCPP_WARN(get_logger(), "Trigger service '%s' not available", name.c_str());
      return false;
    }
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    
    using namespace std::chrono_literals;
    auto status = future.wait_for(std::chrono::duration<double>(timeout_s));
    
    if (status != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Failed to call trigger service '%s' - timeout", name.c_str());
      return false;
    }
    
    auto result = future.get();
    if (!result->success) {
      RCLCPP_WARN(get_logger(), "Trigger service '%s' unsuccessful: %s", name.c_str(), result->message.c_str());
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "Trigger service '%s' successful: %s", name.c_str(), result->message.c_str());
    return true;
  }
  
  // ========== Force/Torque Control ==========
  
  bool zero_ft_sensor()
  {
    if (!zero_ft_client_->wait_for_service(2s)) {
      RCLCPP_WARN(get_logger(), "Zero FT sensor service not available");
      return false;
    }
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = zero_ft_client_->async_send_request(request);
    
    using namespace std::chrono_literals;
    auto status = future.wait_for(2s);
    
    if (status != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Failed to zero FT sensor - timeout");
      return false;
    }
    
    auto result = future.get();
    if (!result->success) {
      RCLCPP_WARN(get_logger(), "FT sensor zeroing unsuccessful: %s", result->message.c_str());
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "FT sensor zeroed successfully");
    return true;
  }
  
  bool touch_with_ft_feedback()
  {
    RCLCPP_INFO(get_logger(), "Starting FT feedback touch control");
    
    if (!switch_controller_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Controller manager not available, skipping FT touch");
      return true;
    }
    
    if (!switch_controller(forward_position_controller_, scaled_joint_trajectory_controller_)) {
      RCLCPP_WARN(get_logger(), "Failed to switch to twist controller, skipping FT touch");
      return true;
    }
    
    // Call /servo_node/start_servo
    if (!call_trigger_service(start_servo_client_, "/servo_node/start_servo")) {
      RCLCPP_ERROR(get_logger(), "Failed to start servo node.");
      switch_controller(scaled_joint_trajectory_controller_, forward_position_controller_);
      return false;
    }

    zero_ft_sensor();
    
    rclcpp::Rate rate(ft_control_rate_);
    int stable_count = 0;
    const int stable_threshold = 10;
    int max_iterations = static_cast<int>(10.0 * ft_control_rate_);
    int iterations = 0;
    
    geometry_msgs::msg::TwistStamped twist_cmd;
    twist_cmd.header.frame_id = end_effector_link_;
    
    while (rclcpp::ok() && iterations++ < max_iterations) {
      double error = ft_setpoint_ - force_z_.load();
      
      if (error > 0) {
        twist_cmd.twist.linear.z = velocity_z_;
        stable_count = 0;
      } else {
        twist_cmd.twist.linear.z = 0.0;
        stable_count++;
        
        if (stable_count > stable_threshold) {
          RCLCPP_INFO(get_logger(), "Contact achieved, force: %.2f N", force_z_.load());
          break;
        }
      }
      
      twist_cmd.header.stamp = now();
      twist_pub_->publish(twist_cmd);
      rate.sleep();
    }
    
    if (iterations >= max_iterations) {
      RCLCPP_WARN(get_logger(), "FT touch control timeout");
    }
    
    // Stop motion
    twist_cmd.twist.linear.z = 0.0;
    twist_cmd.header.stamp = now();
    twist_pub_->publish(twist_cmd);
    
    rclcpp::sleep_for(200ms);
    
    // Call /servo_node/stop_servo
    call_trigger_service(stop_servo_client_, "/servo_node/stop_servo");

    if (!switch_controller(scaled_joint_trajectory_controller_, forward_position_controller_)) {
      return false;
    }
    
    return true;
  }
  
  // ========== Gripper Control ==========
  bool control_gripper(bool activate)
  {
    // TODO: Implement actual service call for UR SetIO
    RCLCPP_WARN(get_logger(), "Gripper control service not implemented. Using dummy delay.");
    if (activate) {
      RCLCPP_INFO(get_logger(), "Activating gripper...");
    } else {
      RCLCPP_INFO(get_logger(), "Deactivating gripper...");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(gripper_activation_delay_ * 1000)));
    return true;
  }
  
  // ========== Perception ==========
  bool get_object_location(const std::string& prompt, geometry_msgs::msg::Pose& object_pose)
  {
    // TODO: Implement actual perception service call
    
    // Dummy implementation for now
    RCLCPP_WARN(get_logger(), "Perception service not implemented yet, using dummy pose");
    
    object_pose.position.x = 0.5;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.1;
    
    // Assuming vertical tool orientation
    object_pose.orientation.w = 1.0;
    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    
    RCLCPP_INFO(get_logger(), "Detected object at: [%.3f, %.3f, %.3f]",
                object_pose.position.x, object_pose.position.y, object_pose.position.z);
    
    return true;
  }
  
  // ========== Motion Execution ==========
  
  bool execute_cartesian_path(const std::vector<geometry_msgs::msg::Pose>& waypoints, 
                              const std::string& name, int max_attempts = 3)
  {
    if (!mgi_ready_) return false;
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction;
    
    for (int i = 0; i < max_attempts; ++i) {
      fraction = move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, trajectory);
      
      if (fraction >= cartesian_min_fraction_) {
        RCLCPP_INFO(get_logger(), "Cartesian path (%s) planned successfully (Fraction: %.2f)", 
                    name.c_str(), fraction);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        
        if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(get_logger(), "Cartesian path (%s) executed successfully", name.c_str());
          return true;
        } else {
          RCLCPP_WARN(get_logger(), "Cartesian path (%s) execution failed on attempt %d", name.c_str(), i + 1);
        }
      } else {
        RCLCPP_WARN(get_logger(), "Cartesian path (%s) planning failed (Fraction: %.2f) on attempt %d", 
                    name.c_str(), fraction, i + 1);
      }
      rclcpp::sleep_for(100ms);
    }
    
    RCLCPP_ERROR(get_logger(), "Cartesian path (%s) failed after %d attempts", name.c_str(), max_attempts);
    return false;
  }
  
  bool execute_joint_space_plan(const geometry_msgs::msg::Pose& target_pose, const std::string& name)
  {
    if (!mgi_ready_) return false;
    
    move_group_->setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(get_logger(), "Joint space move (%s) %s", name.c_str(), 
                  success ? "executed successfully" : "execution failed");
    } else {
      RCLCPP_WARN(get_logger(), "Joint space plan (%s) failed", name.c_str());
    }
    
    move_group_->clearPoseTarget();
    return success;
  }
  
  // ========== Initialization ==========
  
  void delayed_init()
  {
    if (mgi_ready_) return;
    
    if (!wait_for_joint_states(2.0)) {
      RCLCPP_WARN(get_logger(), "Joint states not received yet.");
      return;
    }
    
    try {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
      move_group_->setEndEffectorLink(end_effector_link_);
      move_group_->setPlannerId("RRTConnectkConfigDefault");
      move_group_->setPlanningTime(5.0);
      
      psi_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
      
      RCLCPP_INFO(get_logger(), "MoveGroupInterface and PlanningSceneInterface initialized successfully.");
      mgi_ready_ = true;
      init_timer_->cancel();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "MoveIt initialization failed: %s", e.what());
    }
  }
  
  // ========== Action Server Callbacks ==========
  
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID& uuid,
                                    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    std::lock_guard<std::mutex> lock(exec_mtx_);
    if (!mgi_ready_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not ready, rejecting goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Received goal request for X:%.3f, Y:%.3f, Z:%.3f",
                goal->target_pose.pose.position.x, goal->target_pose.pose.position.y, goal->target_pose.pose.position.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    move_group_->stop();
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void on_accept(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&SuctionGripperMTP::execute_pick_place, this, std::placeholders::_1), goal_handle}.detach();
  }
  
  // ========== Main Pick and Place Logic ==========
  
  void execute_pick_place(const std::shared_ptr<GoalHandle> gh)
  {
    auto result = std::make_shared<MoveToPose::Result>();
    std::lock_guard<std::mutex> lock(exec_mtx_);
    
    if (!rclcpp::ok()) {
      result->result = false;
      gh->abort(result);
      return;
    }
    
    const auto start_pose = gh->get_goal()->target_pose.pose;
    
    try {
      
      // ========== Step 1: Pre-Action Pose (optional) ==========
      if (use_pre_action_pose_) {
        // Implementation for moving to a safe pre-action joint state (omitted for brevity)
        RCLCPP_WARN(get_logger(), "Step 1: Pre-action pose is enabled but not implemented yet.");
      }
      
      // ========== Step 2: Move to Look/Pre-Pick Position (Joint Space) ==========
      RCLCPP_INFO(get_logger(), "Step 2: Moving to Look Position (Joint Space)");
      
      auto prepick = make_pose(start_pose.position.x, start_pose.position.y, pick_place_height_, start_pose.orientation);
      auto look_pose = prepick;
      look_pose.position.z = look_height_;
      
      if (!execute_joint_space_plan(look_pose, "Look/Pre-Pick")) {
        throw std::runtime_error("Failed to move to look position.");
      }
      
      // ========== Step 3: Call Perception Service (Simulated) ==========
      RCLCPP_INFO(get_logger(), "Step 3: Calling Perception Service");
      
      geometry_msgs::msg::Pose detected_object_pose;
      if (!get_object_location("target_object", detected_object_pose)) {
        throw std::runtime_error("Perception failed to find the object.");
      }
      
      // ========== Step 4: Corrected Approach Position (Cartesian) ==========
      RCLCPP_INFO(get_logger(), "Step 4: Moving to Corrected Approach Position (Cartesian)");
      
      auto pick_pose = detected_object_pose;
      pick_pose.position.x += perception_offset_x_;
      pick_pose.position.y += perception_offset_y_;
      pick_pose.position.z += object_clearance_;
      
      auto current_pose = move_group_->getCurrentPose().pose;
      // Small horizontal correction at the look height
      auto correction_pose = make_pose(pick_pose.position.x, pick_pose.position.y, look_pose.position.z, start_pose.orientation);
      
      std::vector<geometry_msgs::msg::Pose> waypoints = {current_pose, correction_pose, pick_pose};
      
      if (!execute_cartesian_path(waypoints, "Approach")) {
        throw std::runtime_error("Failed to execute approach cartesian path.");
      }
      
      // ========== Step 5: Force Feedback Touch Down (Twist Control) ==========
      RCLCPP_INFO(get_logger(), "Step 5: Executing Force Feedback Touch Down");
      
      if (!touch_with_ft_feedback()) {
        throw std::runtime_error("Force feedback touch failed or timed out.");
      }
      
      // ========== Step 6: Activate Gripper ==========
      RCLCPP_INFO(get_logger(), "Step 6: Activating gripper");
      
      if (!control_gripper(true)) {
        throw std::runtime_error("Failed to activate gripper.");
      }
      
      // ========== Step 7: Retract (Lift) ==========
      RCLCPP_INFO(get_logger(), "Step 7: Retracting with object");
      
      current_pose = move_group_->getCurrentPose().pose;
      auto lift_pose = shift_pose(current_pose, 0.0, 0.0, object_clearance_);
      
      waypoints = {current_pose, lift_pose};
      
      if (!execute_cartesian_path(waypoints, "Lift")) {
        RCLCPP_WARN(get_logger(), "Failed to execute lift path.");
      }
      
      // ========== Step 8: Move to Pre-Place Position (Joint Space) ==========
      RCLCPP_INFO(get_logger(), "Step 8: Moving to Pre-Place Position");
      
      // TODO: Get actual place location from goal
      auto place_pose = make_pose(0.3, -0.3, 0.01, start_pose.orientation);
      auto preplace = make_pose(place_pose.position.x, place_pose.position.y, pick_place_height_, start_pose.orientation);
      
      if (!execute_joint_space_plan(preplace, "Pre-Place")) {
        throw std::runtime_error("Failed to move to pre-place position.");
      }
      
      // ========== Step 9: Lowering to Place Position (Cartesian) ==========
      RCLCPP_INFO(get_logger(), "Step 9: Lowering to place position");
      
      waypoints = {preplace, place_pose};
      
      if (!execute_cartesian_path(waypoints, "Place")) {
        throw std::runtime_error("Failed to reach place position");
      }
      rclcpp::sleep_for(200ms);
      
      // ========== Step 10: Deactivate Gripper ==========
      RCLCPP_INFO(get_logger(), "Step 10: Deactivating gripper");
      
      if (!control_gripper(false)) {
        RCLCPP_WARN(get_logger(), "Failed to deactivate gripper");
      }
      
      // ========== Step 11: Retract ==========
      RCLCPP_INFO(get_logger(), "Step 11: Retracting");
      
      current_pose = move_group_->getCurrentPose().pose;
      auto retract_pose = shift_pose(current_pose, 0.0, 0.0, object_clearance_);
      waypoints = {current_pose, retract_pose, preplace};
      
      if (!execute_cartesian_path(waypoints, "Retract")) {
        RCLCPP_WARN(get_logger(), "Failed to retract properly");
      }
      
      // Success!
      RCLCPP_INFO(get_logger(), "=== Pick and Place Completed Successfully ===");
      result->result = true;
      gh->succeed(result);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Pick and place failed: %s", e.what());
      result->result = false;
      gh->abort(result);
      
      if (move_group_) {
        move_group_->stop();
        move_group_->clearPoseTargets();
      }
      // Attempt to switch back to safe trajectory controller
      switch_controller(scaled_joint_trajectory_controller_, forward_position_controller_);
      call_trigger_service(stop_servo_client_, "/servo_node/stop_servo");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SuctionGripperMTP>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}