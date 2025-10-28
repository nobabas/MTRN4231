// -----------------------------------------------
// moveit_server.cpp
// -----------------------------------------------
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <interfaces/srv/move_request.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// -----------------------------------------------
// Moveit Server
// -----------------------------------------------
class MoveitServer {
public:
  explicit MoveitServer(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Server...");

    // Initialise MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_,
      "ur_manipulator",
      std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(10.0)
    );

    // Initialise collision objects in the scene
    setupCollisionObjects();

    // -------------------------------------------
    // Configure MoveIt planner parameters
    // -------------------------------------------

    // Planning time
    node_->declare_parameter("planning_time", 20.0);
    move_group_->setPlanningTime(
      node_->get_parameter("planning_time").as_double()
    );

    // Goal tolerances
    node_->declare_parameter("goal_position_tolerance", 0.001);
    node_->declare_parameter("goal_orientation_tolerance", 0.001);
    node_->declare_parameter("goal_joint_tolerance", 0.001);

    move_group_->setGoalPositionTolerance(
      node_->get_parameter("goal_position_tolerance").as_double()
    );
    move_group_->setGoalOrientationTolerance(
      node_->get_parameter("goal_orientation_tolerance").as_double()
    );
    move_group_->setGoalJointTolerance(
      node_->get_parameter("goal_joint_tolerance").as_double()
    );

    // Planner ID
    node_->declare_parameter("planner_id", "RRTConnectkConfigDefault");
    move_group_->setPlannerId(
      node_->get_parameter("planner_id").as_string()
    );

    // -------------------------------------------
    // Joint state subscriber
    // -------------------------------------------
    joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(&MoveitServer::jointStateCallback, this, _1)
    );

    // -------------------------------------------
    // Service
    // -------------------------------------------
    service_ = node_->create_service<interfaces::srv::MoveRequest>(
      "/moveit_path_plan",
      std::bind(&MoveitServer::handleRequest, this, _1, _2)
    );
  }

  // -----------------------------------------------
  // Handle Requests
  // -----------------------------------------------
  void handleRequest(
    const std::shared_ptr<interfaces::srv::MoveRequest::Request> request,
    std::shared_ptr<interfaces::srv::MoveRequest::Response> response)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Received MoveIt Request: command=%s, positions.size()=%zu",
      request->command.c_str(),
      request->positions.size()
    );

    // Positions must always be length 6
    if (request->positions.size() < 6) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Expected 6 elements in positions, got %zu",
        request->positions.size()
      );
      response->success = false;
      return;
    }

    // Clear previous targets and constraints
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    // Handle command types
    bool target_set = false;
    if (request->command == "joint") {
      target_set = setJointTargets(request->positions);
    } else if (request->command == "cartesian") {
      target_set = setCartesianTarget(request->positions);
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Unknown command: %s (expected 'joint' or 'cartesian')",
        request->command.c_str()
      );
      response->success = false;
      return;
    }

    if (!target_set) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to set target for command: %s",
        request->command.c_str()
      );
      response->success = false;
      return;
    }

    // Plan and execute
    response->success = planNExecute();
  }

private:
  // -----------------------------------------------
  // Joint State Callback
  // -----------------------------------------------
  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_joint_state_ = *msg;
  }

  // -----------------------------------------------
  // Plan and Execute
  // -----------------------------------------------
  bool planNExecute() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int attempts = 0;
    const int max_attempts = 100;

    while (!success && attempts < max_attempts) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Planning attempt: %d",
        attempts + 1
      );

      auto code = move_group_->plan(plan);
      success = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      attempts++;

      if (!success) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Planning attempt %d failed. Retrying...",
          attempts
        );

        // Progressively relax planning time
        move_group_->setPlanningTime(
          move_group_->getPlanningTime() + 2.0
        );
      }
    }

    if (!success) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Planning failed after %d attempts.",
        max_attempts
      );
      return false;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Planning successful in %d attempts. Executing plan...",
      attempts
    );

    auto exec_code = move_group_->execute(plan);
    bool exec_ok =
      (exec_code == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!exec_ok) {
      RCLCPP_ERROR(node_->get_logger(), "Execution failed.");
      return false;
    }

    return true;
  }

  // -----------------------------------------------
  // Collision Obstacles
  // -----------------------------------------------
  void setupCollisionObjects() {
    const std::string frame_id = "world";

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(generateCollisionObject(
      2.4, 0.04, 3.0,
      0.70, -0.60, 0.5,
      frame_id, "back_wall"
    ));
    objects.push_back(generateCollisionObject(
      0.04, 2.4, 3.0,
      -0.55, 0.25, 0.8,
      frame_id, "side_wall"
    ));
    objects.push_back(generateCollisionObject(
      3.0, 3.0, 0.01,
      0.85, 0.25, 0.0,
      frame_id, "table"
    ));
    objects.push_back(generateCollisionObject(
      2.4, 2.4, 0.04,
      0.85, 0.25, 1.5,
      frame_id, "ceiling"
    ));

    planning_scene_interface_.applyCollisionObjects(objects);
  }

  moveit_msgs::msg::CollisionObject generateCollisionObject(
    float sizeX, float sizeY, float sizeZ,
    float x, float y, float z,
    const std::string &frame_id,
    const std::string &id)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = sizeX;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = sizeY;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = sizeZ;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }

  // -----------------------------------------------
  // Target-setting helpers
  // -----------------------------------------------

  // Joint mode: interpret positions[] as joint angles
  bool setJointTargets(const std::vector<double>& positions) {
    try {
      auto joints = createJointPose(positions);
      move_group_->setJointValueTarget(joints);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Error setting joint targets: %s",
        e.what()
      );
      return false;
    }
  }

  std::map<std::string, double> createJointPose(
    const std::vector<double>& positions)
  {
    if (positions.size() < 6) {
      throw std::runtime_error("joints requires 6 joint values");
    }

    return {
      {"shoulder_pan_joint",   positions[5]},
      {"shoulder_lift_joint",  positions[0]},
      {"elbow_joint",          positions[1]},
      {"wrist_1_joint",        positions[2]},
      {"wrist_2_joint",        positions[3]},
      {"wrist_3_joint",        positions[4]}
    };
  }

  // Cartesian mode: interpret positions[] as [x,y,z,roll,pitch,yaw]
  bool setCartesianTarget(const std::vector<double>& positions) {
    try {
      geometry_msgs::msg::Pose target_pose = createCartesianPose(positions);
      move_group_->setPoseTarget(target_pose);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Error setting Cartesian target: %s",
        e.what()
      );
      return false;
    }
  }

  geometry_msgs::msg::Pose createCartesianPose(
    const std::vector<double>& positions)
  {
    if (positions.size() < 6) {
      throw std::runtime_error(
        "cartesian requires [x,y,z,roll,pitch,yaw]"
      );
    }

    tf2::Quaternion q;
    q.setRPY(positions[3], positions[4], positions[5]);
    q.normalize();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = positions[0];
    target_pose.position.y = positions[1];
    target_pose.position.z = positions[2];
    target_pose.orientation = tf2::toMsg(q);

    return target_pose;
  }

  // -----------------------------------------------
  // Members
  // -----------------------------------------------
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Service<interfaces::srv::MoveRequest>::SharedPtr service_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  sensor_msgs::msg::JointState current_joint_state_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

// -----------------------------------------------
// Main
// -----------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("moveit_server");
  auto server = std::make_shared<MoveitServer>(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

// ros2 service call /moveit_path_plan interfaces/srv/MoveRequest "{command: 'cartesian', positions: [0.0, 0.0, 0.0, 0.0, 3.14, 0.0]}"

// Joint Home Position
// ros2 service call /moveit_path_plan interfaces/srv/MoveRequest "{command: 'joint', positions: [-1.3, 1.57, -1.83, -1.57, 0, 0]}"