#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class GripperControl {
public:
  GripperControl(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Gripper Control...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of gripper
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // get current state of gripper
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state of gripper to current state
    move_group_gripper_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Gripper Control");
  }

  ~GripperControl() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Gripper Control");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Gripper Control...");

    // close the gripper
    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_joint_value_gripper(+0.780);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // open the gripper
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    // setup the gripper target by pose name
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_named_pose_gripper("gripper_opened");
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");

    RCLCPP_INFO(LOGGER, "Gripper Control Execution Complete");
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node_;

  // declare move_group node
  rclcpp::Node::SharedPtr move_group_node_;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor_;

  // declare move_group_interface variables for gripper
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // declare joint_model_group for gripper
  const JointModelGroup *joint_model_group_gripper_;

  // declare trajectory planning variables for gripper
  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

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

}; // class GripperControl

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("gripper_control");

  // instantiate class
  GripperControl gripper_control_node(base_node);

  // execute trajectory plan
  gripper_control_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code