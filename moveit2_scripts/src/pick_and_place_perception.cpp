#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <custom_perception_messages/msg/detected_objects.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulation";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

// object info custom messages
using DetectedObjects = custom_perception_messages::msg::DetectedObjects;

class PickAndPlace {
public:
  PickAndPlace(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Pick And Place Class initialization..");

    // configure node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node in a separate thread
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    // They are the key to interact with the move_group_node via the
    // API (getCurrentState(),...) and get info about the robot
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get gripper and robot groups metadata
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get gripper and robot positions and copy them into their respective
    // joint_group_positions_... vectors
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state of robot and gripper to current state
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place");

    // PERCEPTION PART
    // Create subscriber to perception node responsible for finding xyz pose of
    // the object
    auto object_pose_sub = std::bind(&PickAndPlace::object_pose_callback, this,
                                     std::placeholders::_1);
    object_pose_subscriber_ = base_node_->create_subscription<DetectedObjects>(
        "/object_detected", 1, object_pose_sub);
  }

  ~PickAndPlace() { RCLCPP_INFO(LOGGER, "Class terminated: Pick And Place"); }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Pick And Place...");

    // Just wait for the flag - the spin thread handles callbacks
    while (!object_detected_ && rclcpp::ok()) {
      RCLCPP_INFO(LOGGER, "Waiting for object detection...");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    if (!object_detected_) {
      RCLCPP_ERROR(LOGGER, "No object detected!");
      return;
    }

    RCLCPP_INFO(LOGGER, "Object detected! Starting pick and place...");

    // move to home position
    RCLCPP_INFO(LOGGER, "Going to Home Position...");
    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Setup the Joint Value target...");
    setup_joint_value_target(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708,
                             +0.0000);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // move above blue object
    RCLCPP_INFO(
        LOGGER,
        "Going above blue object at positions x: %f | y: %f + 0.15 | z: %f",
        object_coordinates_[0], object_coordinates_[1], object_coordinates_[2]);
    // setup robot pose target
    RCLCPP_INFO(LOGGER, "Setup the goal pose");
    // setup_goal_pose_target(+0.340, -0.0195, +0.30, -1.000, +0.000, +0.000,
    //                        +0.000);
    setup_goal_pose_target(object_coordinates_[0], object_coordinates_[1],
                           object_coordinates_[2] + 0.28, -1.000, +0.000,
                           +0.000, +0.000);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Goal Pose Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Goal Pose Trajectory...");
    execute_trajectory_kinematics();

    // Open Gripper
    RCLCPP_INFO(LOGGER, "about to open gripper");
    // setup gripper closed angle value
    setup_joint_value_gripper(0.05);
    // plan and execute trajecty
    RCLCPP_INFO(LOGGER, "Planning Gripper Trajectory...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Trajectory...");
    execute_trajectory_gripper();

    // Get down the Z axis until at grasp position
    RCLCPP_INFO(LOGGER, "Coming down to grasp position");
    // setup cartesian target of grasp position
    setup_waypoints_target(+0.0, +0.0, -delta_);
    // plan and execute trajectory
    RCLCPP_INFO(LOGGER, "Planning cartesian Trajectory...");
    plan_trajectory_cartesian();
    RCLCPP_INFO(LOGGER, "Executing cartesian Trajectory...");
    execute_trajectory_cartesian();

    std::this_thread::sleep_for(std::chrono::milliseconds(15000));

    // Almost close Gripper
    RCLCPP_INFO(LOGGER, "about to close gripper");
    // setup gripper closed angle value
    setup_joint_value_gripper(0.63);
    // plan and execute trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Trajectory...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Trajectory...");
    execute_trajectory_gripper();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Close Gripper
    RCLCPP_INFO(LOGGER, "about to close gripper");
    // setup gripper closed angle value
    setup_joint_value_gripper(0.645);
    // plan and execute trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Trajectory...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Trajectory...");
    execute_trajectory_gripper();

    // wait a bit before getting up
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Get up the Z axis back to pre-grasp position
    RCLCPP_INFO(LOGGER, "Coming down to grasp position");
    // setup cartesian target of pre-grasp position
    setup_waypoints_target(+0.0, +0.0, +delta_);
    // plan and execute trajectory
    RCLCPP_INFO(LOGGER, "Planning cartesian Trajectory...");
    plan_trajectory_cartesian();
    RCLCPP_INFO(LOGGER, "Executing cartesian Trajectory...");
    execute_trajectory_cartesian();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // move 180° with shoulder_pan_joint
    RCLCPP_INFO(LOGGER, "Setup joint position");
    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    setup_joint_value_target(
        +3.05, joint_group_positions_robot_[1], joint_group_positions_robot_[2],
        joint_group_positions_robot_[3], joint_group_positions_robot_[4],
        joint_group_positions_robot_[5]);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // // move 180° with shoulder_pan_joint
    // RCLCPP_INFO(LOGGER, "Setup target position");
    // // setup target position of the robot
    // setup_goal_pose_target(-0.340, 0.017, 0.300, -0.004, 1.000, -0.000,
    // -0.000);
    // // plan and execute the trajectory
    // RCLCPP_INFO(LOGGER, "Planning Goal Pose Trajectory...");
    // plan_trajectory_kinematics();
    // RCLCPP_INFO(LOGGER, "Executing Goal Pose Trajectory...");
    // execute_trajectory_kinematics();

    // Open Gripper
    RCLCPP_INFO(LOGGER, "about to close gripper");
    // setup gripper closed angle value
    setup_joint_value_gripper(0.05);
    // plan and execute trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Trajectory...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Trajectory...");
    execute_trajectory_gripper();

    // move back to home position
    RCLCPP_INFO(LOGGER, "Coming back to Home Position...");
    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Setup the Joint Value target...");
    setup_joint_value_target(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708,
                             +0.0000);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  // declare subscriber to object pose
  rclcpp::Subscription<DetectedObjects>::SharedPtr object_pose_subscriber_;
  std::vector<float> object_coordinates_;
  bool object_detected_ = false;

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node_;

  // declare move_group node
  rclcpp::Node::SharedPtr move_group_node_;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor_;

  // declare move_group_interface variables for robot and gripper
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // declare joint_model_group for robot and gripper
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // declare trajectory planning variables for robot
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;

  // declare trajectory planning variables for gripper
  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;
  float delta_ = 0.125; // meters

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    // execute the planned trajectory to target using kinematics
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    // add the desired pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    // plan the trajectory to target using cartesian path
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }
  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }
  void setup_joint_value_gripper(float angle) {
    // set the joint values for each joint of gripper
    // based on values provided
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_named_pose_gripper(std::string pose_name) {
    // set the joint values for each joint of gripper
    // based on predefined pose names
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    // plan the gripper action
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    // execute the planned gripper action
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

  void object_pose_callback(const DetectedObjects::SharedPtr msg) {
    object_coordinates_.clear();
    object_coordinates_.push_back(msg->position.x);
    object_coordinates_.push_back(msg->position.y);
    object_coordinates_.push_back(msg->position.z);

    object_detected_ = true;

    RCLCPP_INFO(LOGGER, "Object detected at: x=%.3f, y=%.3f, z=%.3f",
                msg->position.x, msg->position.y, msg->position.z);
  }

}; // Class Pick And Place

int main(int argc, char **argv) {
  // initialize program node
  rclcpp::init(argc, argv);

  // initilize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");

  // Start spinning in a separate thread to receive messages
  std::thread spin_thread([&base_node]() { rclcpp::spin(base_node); });

  // instantiate class
  PickAndPlace pick_and_place_node(base_node);

  // execute trajectory plan
  pick_and_place_node.execute_trajectory_plan();

  rclcpp::shutdown();
  spin_thread.join();

  return 0;
}