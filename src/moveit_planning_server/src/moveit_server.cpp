// -----------------------------------------------
// moveit_server.cpp 
// -----------------------------------------------
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <unordered_map>
#include <atomic>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <interfaces/srv/move_request.hpp>
#include <interfaces/msg/marker2_d_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <rmw/qos_profiles.h>   // for rmw_qos_profile_services_default

// ----------------------
// Settings
// ----------------------
constexpr double kPlanningTime     = 10.0;   // seconds
constexpr int    kPlanningAttempts = 100;
constexpr double kGoalPosTol       = 0.001;  // meters
constexpr double kGoalOriTol       = 0.01;   // radians
constexpr double kGoalJointTol     = 0.01;   // radians

static const std::string kPlanningGroup = "ur_manipulator";
static const std::string kBaseFrame     = "base_link";
static const std::string kWorldFrame    = "world";
static const std::string kEEFrame       = "tool0";

inline geometry_msgs::msg::Quaternion toolDownRPY()
{
  tf2::Quaternion q; q.setRPY(M_PI, 0.0, 0.0); q.normalize();
  return tf2::toMsg(q);
}

class MoveitServer {
public:
  explicit MoveitServer(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    // Params for vision → EE
    z_above_work_ = node_->declare_parameter<double>("marker_target_z", 0.40);  // meters
    vel_scale_    = node_->declare_parameter<double>("max_vel_scale",    0.20);  // 0..1
    acc_scale_    = node_->declare_parameter<double>("max_acc_scale",    0.10);  // 0..1

    RCLCPP_INFO(node_->get_logger(), "Starting bare-bones MoveIt Server");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, kPlanningGroup, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(10.0));

    // Planner / tolerances
    move_group_->setPlanningTime(kPlanningTime);
    move_group_->setNumPlanningAttempts(kPlanningAttempts);
    move_group_->setGoalPositionTolerance(kGoalPosTol);
    move_group_->setGoalOrientationTolerance(kGoalOriTol);
    move_group_->setGoalJointTolerance(kGoalJointTol);
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setMaxVelocityScalingFactor(vel_scale_);
    move_group_->setMaxAccelerationScalingFactor(acc_scale_);

    // Scene objects
    setupCollisionObjects();

    // Joint state sub
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&MoveitServer::jointStateCb, this, _1));

    // Marker target visual
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // Callback groups for services (allow concurrent callbacks)
    move_service_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    stop_service_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Service for manual commands
    service_ = node_->create_service<interfaces::srv::MoveRequest>(
      "/moveit_path_plan",
      std::bind(&MoveitServer::handleRequest, this, _1, _2),
      rmw_qos_profile_services_default,
      move_service_group_);

    // Stop service
    stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/moveit_stop",
      std::bind(&MoveitServer::handleStop, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      stop_service_group_);

    // Subscribe to vision results (array of markers)
    vision_sub_ = node_->create_subscription<interfaces::msg::Marker2DArray>(
      "/vision/markers",
      rclcpp::SensorDataQoS(),
      std::bind(&MoveitServer::visionCb, this, _1));

    // Obstacle sub
    obstacle_sub_ = node_->create_subscription<moveit_msgs::msg::CollisionObject>(
      "/vision/obstacles", rclcpp::SystemDefaultsQoS(),
      [this](const moveit_msgs::msg::CollisionObject::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Received dynamic obstacle: %s", msg->id.c_str());
        // Add object to this planning scene
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(*msg);
        planning_scene_.applyCollisionObjects(collision_objects);
    });


    // Let MoveIt sync with /joint_states then set start state
    move_group_->startStateMonitor(5.0); // up to 5s to discover joint states
    rclcpp::sleep_for(std::chrono::seconds(1));
    move_group_->setStartStateToCurrentState();

    // Timer to drain the vision queue sequentially (non-blocking callback)
    process_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MoveitServer::processQueue, this));
  }

private:
  // ---------------------------
  // Service: joint / pose / cartesian / line
  // ---------------------------
  void handleRequest(
      const std::shared_ptr<interfaces::srv::MoveRequest::Request> req,
      std::shared_ptr<interfaces::srv::MoveRequest::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(),
                "Received MoveIt Request: command=%s, positions.size()=%zu",
                req->command.c_str(), req->positions.size());

    if (req->positions.size() < 6) {
      RCLCPP_ERROR(node_->get_logger(), "Expected 6 values.");
      res->success = false;
      return;
    }

    // Clear any previous goals and constraints
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setStartStateToCurrentState();

    bool ok = false;

    // =================================================================
    // JOINT CONSTRAINT LOGIC
    // =================================================================

    moveit_msgs::msg::Constraints path_constraints;
{
    // 3. Create the constraint for the shoulder_lift_joint
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "shoulder_lift_joint";
    
    // Example: Limit lift joint to be between -2.57 and -0.57 radians
    // (This is centered at -1.57, or -90 degrees)
    jc.position = -1.57;
    jc.tolerance_below = 1.2; // 1.0 rad tolerance
    jc.tolerance_above = 1.2; // 1.0 rad tolerance
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);
}

{ 
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "shoulder_pan_joint";
    
    // Example: Limit lift joint to be between -2.57 and -0.57 radians
    // (This is centered at -1.57, or -90 degrees)
    jc.position = 0;
    jc.tolerance_below = 2; // 1.0 rad tolerance
    jc.tolerance_above = 2; // 1.0 rad tolerance
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);
}
    if (req->command == "joint") {
      move_group_->setPathConstraints(path_constraints);
      ok = setJointTargets(req->positions);

    } else if (req->command == "pose") {
      move_group_->setPathConstraints(path_constraints);

      geometry_msgs::msg::Pose pose = createPose(req->positions);
      move_group_->setPoseTarget(pose, kEEFrame);
      publishTargetMarker(pose);
      ok = planAndExecute();

    } else if (req->command == "line") {
      // Add Z-line constraint + joint constraint
      auto current = computeEEFfromJointState();
      geometry_msgs::msg::Pose target = current;
      target.position.z = req->positions[2];

      moveit_msgs::msg::Constraints c = path_constraints; 
      addZLineConstraint(c, current, target);
      move_group_->setPathConstraints(c);

      move_group_->setPoseTarget(target, kEEFrame);
      publishTargetMarker(target);
      ok = planAndExecute();

    } else if (req->command == "cartesian") {
      // Cartesian move using joint-state-based EEF pose as start
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      move_group_->setStartStateToCurrentState();

      geometry_msgs::msg::Pose pose = createPose(req->positions);
      publishTargetMarker(pose);
      ok = computeCartesianPose(pose);

    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "Unknown command: %s (use 'joint', 'pose', 'line', or 'cartesian')",
                   req->command.c_str());
      res->success = false;
      return;
    }

    res->success = ok;
  }

  // ---------------------------
  // Stop service
  // ---------------------------
  void handleStop(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_WARN(node_->get_logger(), "Stop requested! Stopping current motion.");
    cancel_requested_.store(true);
    move_group_->stop();

    res->success = true;
    res->message = "Motion stop requested.";
  }

  // ---------------------------
  // Z-line constraint
  // ---------------------------
  // Build a thin box centered between current.z and target.z to force Z-line motion
  void addZLineConstraint(moveit_msgs::msg::Constraints& out,
                          const geometry_msgs::msg::Pose& current,
                          const geometry_msgs::msg::Pose& /*target*/)
  {
    moveit_msgs::msg::PositionConstraint pc;
    pc.header.frame_id = "base_link";
    pc.link_name = move_group_->getEndEffectorLink();
    pc.weight = 1.0;

    RCLCPP_INFO(node_->get_logger(), "Header frame: %s", pc.header.frame_id.c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", pc.link_name.c_str());

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    // skinny in X/Y, long in Z
    box.dimensions = {0.1, 0.1, 1.0};

    geometry_msgs::msg::Pose region_pose;
    region_pose.orientation = current.orientation;
    region_pose.position.x = current.position.x;
    region_pose.position.y = current.position.y;
    region_pose.position.z = current.position.z;

    pc.constraint_region.primitives.emplace_back(box);
    pc.constraint_region.primitive_poses.emplace_back(region_pose);

    out.position_constraints.emplace_back(pc);
    out.name = "use_equality_constraints";
  }

  // ---------------------------
  // Joint-state-based FK for EEF pose
  // ---------------------------
  geometry_msgs::msg::Pose computeEEFfromJointState()
  {
    geometry_msgs::msg::Pose pose;
    // default identity pose if no joint states yet
    if (last_js_.name.empty() || last_js_.position.empty()) {
      RCLCPP_WARN(node_->get_logger(), "computeEEFfromJointState: no joint_state received yet");
      pose.position.x = pose.position.y = pose.position.z = 0.0;
      pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      return pose;
    }

    // RobotModel
    moveit::core::RobotModelConstPtr model = move_group_->getRobotModel();
    if (!model) {
      RCLCPP_ERROR(node_->get_logger(), "computeEEFfromJointState: robot model is null");
      pose.orientation.w = 1.0;
      return pose;
    }

    moveit::core::RobotState state(model);

    // Get the JointModelGroup for the planning group
    const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(kPlanningGroup);
    if (!jmg) {
      RCLCPP_ERROR(node_->get_logger(),
                   "computeEEFfromJointState: failed to get JointModelGroup '%s'",
                   kPlanningGroup.c_str());
      pose.orientation.w = 1.0;
      return pose;
    }

    // Build a name->position map from last_js_
    std::unordered_map<std::string, double> js_map;
    const size_t n = std::min(last_js_.name.size(), last_js_.position.size());
    for (size_t i = 0; i < n; ++i) {
      js_map[last_js_.name[i]] = last_js_.position[i];
    }

    // Set group variables only when present in the joint_state message
    for (const auto &var : jmg->getVariableNames()) {
      auto it = js_map.find(var);
      if (it != js_map.end()) {
        state.setVariablePosition(var, it->second);
      }
      // else leave state value as default
    }

    state.update();

    // Get transform for end effector link
    const std::string ee_link = move_group_->getEndEffectorLink();
    const Eigen::Isometry3d tf = state.getGlobalLinkTransform(ee_link);

    // Convert Eigen -> geometry_msgs::msg::Pose
    const Eigen::Vector3d t = tf.translation();
    const Eigen::Quaterniond q(tf.rotation());

    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
  }

  // ---------------------------
  // Compute Cartesian Path
  // ---------------------------
  bool computeCartesianPose(const geometry_msgs::msg::Pose& target)
  {
    // Use joint-state-based FK instead of move_group_->getCurrentPose()
    geometry_msgs::msg::Pose start = computeEEFfromJointState();

    if (last_js_.name.empty() || last_js_.position.empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "computeCartesianPose: no joint_state received yet, aborting.");
      return false;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start);
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step       = 0.01;  // path resolution (m)
    const double jump_threshold = 0.0;   // disable jump detection

    cancel_requested_.store(false);

    double fraction = move_group_->computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory, true);

    RCLCPP_INFO(node_->get_logger(),
                "computeCartesianPose: Cartesian path fraction = %.3f", fraction);

    if (fraction < 0.99) {
      RCLCPP_ERROR(node_->get_logger(),
                   "computeCartesianPose: Failed to compute full Cartesian path.");
      return false;
    }

    if (cancel_requested_.load()) {
      RCLCPP_WARN(node_->get_logger(),
                  "computeCartesianPose: cancelled before execution.");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    auto exec_code = move_group_->execute(plan);
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(),
                   "computeCartesianPose: Execution failed or was stopped.");
      return false;
    }

    return true;
  }

  // ---------------------------
  // Vision subscriber + queue
  // ---------------------------
  void visionCb(const interfaces::msg::Marker2DArray::SharedPtr msg)
  {
    // Enqueue all markers in the message
    for (const auto& m : msg->markers) {
      Target t;
      t.id = m.id;
      t.x  = m.x;
      t.y  = m.y;
      t.z  = z_above_work_;            // use configured Z height
      t.q  = toolDownRPY();            // keep tool “down”
      pending_.push(std::move(t));
    }
  }

  void processQueue()
  {
    if (busy_ || pending_.empty()) return;
    busy_ = true;

    const auto tgt = pending_.front(); pending_.pop();

    // Build pose from vision target
    geometry_msgs::msg::Pose p;
    p.position.x = tgt.x;
    p.position.y = tgt.y;
    p.position.z = tgt.z;
    p.orientation = tgt.q;

    RCLCPP_INFO(node_->get_logger(), "Moving to marker id=%.0f at (%.3f, %.3f, %.3f)",
                tgt.id, tgt.x, tgt.y, tgt.z);

    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(p, kEEFrame);
    publishTargetMarker(p);

    const bool ok = planAndExecute();
    if (!ok) {
      RCLCPP_WARN(node_->get_logger(), "Failed to reach marker id=%.0f", tgt.id);
    }
    busy_ = false;
  }

  struct Target {
    float id{};
    double x{}, y{}, z{};
    geometry_msgs::msg::Quaternion q;
  };

  // ---------------------------
  // Scene / planning helpers
  // ---------------------------
  void setupCollisionObjects()
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(makeBox("back_wall",  2.4, 0.04, 1.0,  0.85, -0.30, 0.50, kWorldFrame));
    objects.push_back(makeBox("side_wall",  0.04,1.20, 1.0, -0.30,  0.25, 0.50, kWorldFrame));
    objects.push_back(makeBox("table",      2.4, 2.40, 0.04, 0.85,  0.25, 0.013, kWorldFrame));
    objects.push_back(makeBox("ceiling",    2.4, 2.40, 0.04, 0.85,  0.25, 1.20,  kWorldFrame));
    planning_scene_.applyCollisionObjects(objects);
  }

  moveit_msgs::msg::CollisionObject makeBox(const std::string& id,
                                            double sx, double sy, double sz,
                                            double x, double y, double z,
                                            const std::string& frame)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame;
    obj.id = id;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {sx, sy, sz};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = x; pose.position.y = y; pose.position.z = z;

    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
    return obj;
  }

  bool setJointTargets(const std::vector<double>& v)
  {
    if (v.size() < 6) {
      RCLCPP_ERROR(node_->get_logger(), "Joint mode needs 6 values.");
      return false;
    }
    std::map<std::string, double> joints = {
      {"shoulder_pan_joint",   v[5]},
      {"shoulder_lift_joint",  v[0]},
      {"elbow_joint",          v[1]},
      {"wrist_1_joint",        v[2]},
      {"wrist_2_joint",        v[3]},
      {"wrist_3_joint",        v[4]}
    };
    move_group_->setJointValueTarget(joints);
    return planAndExecute();
  }

  geometry_msgs::msg::Pose createPose(const std::vector<double>& v)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = v[0];
    p.position.y = v[1];
    p.position.z = v[2];
    tf2::Quaternion q; q.setRPY(v[3], v[4], v[5]); q.normalize();
    p.orientation = tf2::toMsg(q);
    return p;
  }

  void publishTargetMarker(const geometry_msgs::msg::Pose& pose)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = kBaseFrame;
    m.header.stamp = node_->now();
    m.ns = "target_marker";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = pose;
    m.scale.x = m.scale.y = m.scale.z = 0.05;
    m.color.a = 1.0; m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0;
    marker_pub_->publish(m);
  }

  bool planAndExecute()
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    cancel_requested_.store(false);

    for (int i = 1; i <= kPlanningAttempts && !success; ++i) {
      if (cancel_requested_.load()) {
        RCLCPP_WARN(node_->get_logger(), "Plan cancelled before completion.");
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "Planning attempt: %d", i);
      moveit::core::MoveItErrorCode code = move_group_->plan(plan);
      success = (code == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success && i % 3 == 0) {
        move_group_->setPlanningTime(move_group_->getPlanningTime() + 2.0);
      }
    }

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
      return false;
    }

    if (cancel_requested_.load()) {
      RCLCPP_WARN(node_->get_logger(), "Execution cancelled before start.");
      return false;
    }

    auto exec_code = move_group_->execute(plan);
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Execution failed or was stopped.");
      return false;
    }
    return true;
  }

  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    last_js_ = *msg;
  }

  // Members
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Service<interfaces::srv::MoveRequest>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  rclcpp::CallbackGroup::SharedPtr move_service_group_;
  rclcpp::CallbackGroup::SharedPtr stop_service_group_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState last_js_;

  rclcpp::Subscription<interfaces::msg::Marker2DArray>::SharedPtr vision_sub_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr obstacle_sub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  std::queue<Target> pending_;
  bool busy_{false};

  std::atomic<bool> cancel_requested_{false};

  // params
  double z_above_work_{0.20};
  double vel_scale_{0.2}, acc_scale_{0.1};
};

// -----------------------------------------------
// Main
// -----------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node   = std::make_shared<rclcpp::Node>("moveit_server");
  auto server = std::make_shared<MoveitServer>(node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

// -----------------------------------------------
// Usage Examples
// -----------------------------------------------

// HOME - NEED TO RUN THIS FIRST OR MOVEIT WILL FAIL
// ros2 service call /moveit_path_plan interfaces/srv/MoveRequest "{command: 'joint', positions: [-1.3, 1.57, -1.83, -1.57, 0.0, 0.0]}"

// World: X=0.461m, Y=-0.001m, Z=0.400m

// POSE TEST
// ros2 service call /moveit_path_plan interfaces/srv/MoveRequest "{command: 'pose', positions: [0.461, 0.01, 0.35, -3.1415, 0.0, 1.57]}"
// ros2 service call /moveit_path_plan interfaces/srv/MoveRequest "{command: 'pose', positions: [0.60, 0.35, 0.35, -3.1415, 0.0, 1.57]}"

// Cartesian TEST
// ros2 service call /moveit_path_plan interfaces/srv/MoveRequest "{command: 'cartesian', positions: [0.50, 0.35, 0.65, -3.1415, 0.0, 1.57]}"
// ros2 service call /moveit_path_plan interfaces/srv/MoveRequest "{command: 'cartesian', positions: [0.50, 0.35, 0.30, -3.1415, 0.0, 1.57]}"  # line down in Z

// STOP TEST (while arm is moving)
// ros2 service call /moveit_stop std_srvs/srv/Trigger "{}"

// Marker Movement Test
/*
ros2 topic pub -1 /vision/markers interfaces/msg/Marker2DArray "
markers:
 - {id: 1.0, x: 0.50, y: 0.30}
 - {id: 2.0, x: 0.55, y: 0.35}
 - {id: 3.0, x: 0.60, y: 0.55}
"
*/
