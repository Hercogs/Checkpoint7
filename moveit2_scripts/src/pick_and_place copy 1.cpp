#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "rclcpp/rclcpp.hpp"
#include <string>

#include <array>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_test");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

std::mutex mtx;

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

    auto my_callback_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = my_callback_group;

    this->sub_joint_states =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&PickAndPlace::joint_state_clb, this,
                      std::placeholders::_1),
            options);

  } // end of constructor

  void joint_state_clb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    mtx.lock();

    this->joint_group_positions_arm_tmp[0] = msg->position[0];
    this->joint_group_positions_arm_tmp[1] = msg->position[2];
    this->joint_group_positions_arm_tmp[2] = msg->position[3];
    this->joint_group_positions_arm_tmp[3] = msg->position[4];
    this->joint_group_positions_arm_tmp[4] = msg->position[5];
    this->joint_group_positions_arm_tmp[5] = msg->position[6];

    // this->joint_group_positions_arm_tmp.insert(
    //     this->joint_group_positions_arm_tmp.begin() + 0, msg->position[0]);
    // this->joint_group_positions_arm_tmp.insert(
    //     this->joint_group_positions_arm_tmp.begin() + 1, msg->position[2]);
    // this->joint_group_positions_arm_tmp.insert(
    //     this->joint_group_positions_arm_tmp.begin() + 2, msg->position[3]);
    // this->joint_group_positions_arm_tmp.insert(
    //     this->joint_group_positions_arm_tmp.begin() + 3, msg->position[4]);
    // this->joint_group_positions_arm_tmp.insert(
    //     this->joint_group_positions_arm_tmp.begin() + 4, msg->position[5]);
    // this->joint_group_positions_arm_tmp.insert(
    //     this->joint_group_positions_arm_tmp.begin() + 5, msg->position[6]);

    // RCLCPP_INFO(LOGGER, "Vector size: %ld",
    //             this->joint_group_positions_arm_tmp.size());

    mtx.unlock();
  }

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

    // current_state_arm = move_group_arm.getCurrentState(10);

    current_state_arm = move_group_arm.getCurrentState(10);

    current_state_gripper = move_group_gripper.getCurrentState(10);

    current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
                                               this->joint_group_positions_arm);

    current_state_gripper->copyJointGroupPositions(
        this->joint_model_group_gripper, this->joint_group_positions_gripper);

    // for (auto x : this->joint_group_positions_gripper)
    //   RCLCPP_INFO(LOGGER, "joint_group_positions_gripper: %.2f", x);

    // for (auto x : this->joint_group_positions_arm)
    //   RCLCPP_INFO(LOGGER, "joint_group_positions_arm: %.2f", x);
  }

  // Plan to End-Effector Pose
  bool pregrasp() {

    RCLCPP_INFO(LOGGER, "Planning to End-Effector Pose");

    target_pose1.orientation.x = -1.0;
    target_pose1.orientation.y = 0.00;
    target_pose1.orientation.z = 0.00;
    target_pose1.orientation.w = 0.00;
    target_pose1.position.x = 0.34;
    target_pose1.position.y = 0.27;
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

  bool open_gripper() {
    RCLCPP_INFO(LOGGER, "Open Gripper!");

    joint_group_positions_gripper[0] = 0.35;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

    bool success_gripper =
        (move_group_gripper.plan(my_plan_gripper) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    (void)success_gripper;

    if (success_gripper) {
      // Execute
      move_group_gripper.execute(my_plan_gripper);
      return true;

    } else {
      return false;
    }

    RCLCPP_INFO(LOGGER, "Open Gripper finished %d!", success_gripper);
  }

  bool approach() {

    RCLCPP_INFO(LOGGER, "Approach to object");

    float delta = 0.04;

    target_pose1.position.z = target_pose1.position.z - delta;
    move_group_arm.setPoseTarget(target_pose1);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    (void)success_arm;

    if (success_arm) {
      move_group_arm.execute(my_plan_arm);
      return true;

    } else {
      return false;
    }
  }

  bool close_gripper() {
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
      return true;

    } else {
      return false;
    }

    RCLCPP_INFO(LOGGER, "Close Gripper finished %d!", success_gripper);
  }

  void deapproach() {

    RCLCPP_INFO(LOGGER, "Deapproach to object");

    float delta = 0.04;

    target_pose1.position.z = target_pose1.position.z + delta;
    move_group_arm.setPoseTarget(target_pose1);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    (void)success_arm;

    // Execute
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Deapproach finished: %.2f",
                joint_group_positions_arm[0]);
  }

  bool move_to_drop() {

    RCLCPP_INFO(LOGGER, "Planning  to drop");

    // Copy join values
    mtx.lock();
    this->joint_group_positions_arm = this->joint_group_positions_arm_tmp;
    mtx.unlock();

    for (auto x : this->joint_group_positions_arm) {
      RCLCPP_INFO(LOGGER, "joint_group_positions_arm: %.2f", x);
    }

    // For some reason I cannot get current  joint positions, so I need to use
    // basolute positions

    // joint_group_positions_arm[0] += 3.14; // Elbow
    // joint_group_positions_arm[1] = -2.09; // Shoulder Lift
    joint_group_positions_arm[2] += 3.14; // Shoulder Pan
    // joint_group_positions_arm[3] = 4.58;  // Wrist 1
    // joint_group_positions_arm[4] = 1.57;  // Wrist 2
    // joint_group_positions_arm[5] = 5.7;   // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    // // move_group_arm.setPoseTarget(target_pose1);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    (void)success_arm;

    RCLCPP_INFO(LOGGER, "Planning  to drop finished: %d", success_arm);

    if (success_arm) {
      move_group_arm.execute(my_plan_arm);
      return true;

    } else {
      return false;
    }
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

    status = approach();

    current_state();

    close_gripper();

    current_state();

    deapproach();

    current_state();

    move_to_drop();

    current_state();

    open_gripper();
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::vector<double> joint_group_positions_arm_tmp{0, 0, 0, 0, 0, 0};

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

  // Joint state subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      sub_joint_states;

}; // End of Class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_demo", node_options);

  rclcpp::executors::MultiThreadedExecutor planner_executor;
  std::shared_ptr<PickAndPlace> planner_node =
      std::make_shared<PickAndPlace>(move_group_node);
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}