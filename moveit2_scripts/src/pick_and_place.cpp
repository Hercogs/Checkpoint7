#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "rclcpp/rclcpp.hpp"
#include <string>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlace : public rclcpp::Node {
public:
  PickAndPlace(std::shared_ptr<rclcpp::Node> move_group_node)
      : Node("pick_and_place"),
        move_group_arm(move_group_node, PLANNING_GROUP_ARM),
        joint_model_group_arm(
            move_group_arm.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP_ARM)),
        move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER),
        joint_model_group_gripper(
            move_group_gripper.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP_GRIPPER)) {

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&PickAndPlace::timer_callback, this));

  } // end of constructor

  // Getting Basic Information
  void get_info() {

    RCLCPP_INFO(LOGGER, "Planning frame: %s",
                move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s",
                move_group_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_arm.getJointModelGroupNames().begin(),
              move_group_arm.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
  }

  void current_state() {
    RCLCPP_INFO(LOGGER, "Get Robot Current State");

    current_state_arm = move_group_arm.getCurrentState(10);
    current_state_gripper = move_group_gripper.getCurrentState(10);

    current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
                                               this->joint_group_positions_arm);
    current_state_gripper->copyJointGroupPositions(
        this->joint_model_group_gripper, this->joint_group_positions_gripper);
  }

  // Plan to End-Effector Pose
  bool pregrasp() {

    RCLCPP_INFO(LOGGER, "Planning to End-Effector Pose");

    target_pose1.orientation.x = -1.0;
    target_pose1.orientation.y = 0.00;
    target_pose1.orientation.z = 0.00;
    target_pose1.orientation.w = 0.00;
    target_pose1.position.x = 0.34;
    target_pose1.position.y = 0.28;
    target_pose1.position.z = 0.30;

    // joint_group_positions_arm[0] = -0.917; // Shoulder Pan
    // joint_group_positions_arm[1] = -2.09;  // Shoulder Lift
    // joint_group_positions_arm[2] = 4.13;   // Elbow
    // joint_group_positions_arm[3] = 4.58;   // Wrist 1
    // joint_group_positions_arm[4] = 1.57;   // Wrist 2
    // joint_group_positions_arm[5] = 5.57;   // Wrist 3

    move_group_arm.setPoseTarget(target_pose1);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    (void)success_arm;

    RCLCPP_INFO(LOGGER, "Planning to End-Effector Pose finished: %d",
                success_arm);

    if (success_arm) {
      move_group_arm.execute(my_plan_arm);
      return true;

    } else {
      return false;
    }
  }

  void open_gripper() {
    RCLCPP_INFO(LOGGER, "Open Gripper!");

    joint_group_positions_gripper[0] = 0.4;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

    bool success_gripper =
        (move_group_gripper.plan(my_plan_gripper) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    (void)success_gripper;

    // Execute
    move_group_gripper.execute(my_plan_gripper);
  }

  void approach() {

    RCLCPP_INFO(LOGGER, "Approach to object");

    float delta = 0.04;

    target_pose1.position.z = target_pose1.position.z - delta;
    move_group_arm.setPoseTarget(target_pose1);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    (void)success_arm;

    // Execute
    move_group_arm.execute(my_plan_arm);
  }

  void close_gripper() {
    RCLCPP_INFO(LOGGER, "Close Gripper!");

    joint_group_positions_gripper[0] = -0.65;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

    bool success_gripper =
        (move_group_gripper.plan(my_plan_gripper) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    (void)success_gripper;
    if (success_gripper) {
      // Execute
      move_group_gripper.execute(my_plan_gripper);
    }
    RCLCPP_INFO(LOGGER, "Close Gripper finished %d!", success_gripper);
  }

  // Timer Callback function
  void timer_callback() {

    bool status;

    this->timer_->cancel();
    get_info();
    current_state();
    status = pregrasp();

    if (status) {
      open_gripper();
    }

    approach();

    current_state();

    close_gripper();
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;

  geometry_msgs::msg::Pose target_pose1;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

  rclcpp::TimerBase::SharedPtr timer_;

  moveit::planning_interface::MoveGroupInterface move_group_arm;
  moveit::planning_interface::MoveGroupInterface move_group_gripper;

  const moveit::core::JointModelGroup *joint_model_group_arm;
  const moveit::core::JointModelGroup *joint_model_group_gripper;

  moveit::core::RobotStatePtr current_state_arm;
  moveit::core::RobotStatePtr current_state_gripper;

}; // End of Class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_demo", node_options);

  rclcpp::executors::SingleThreadedExecutor planner_executor;
  std::shared_ptr<PickAndPlace> planner_node =
      std::make_shared<PickAndPlace>(move_group_node);
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}