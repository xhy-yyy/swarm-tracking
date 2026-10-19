#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
bool allow_ego_control = false;
std::string drone_ns;
double fixed_height = 0.0;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  allow_ego_control = (current_state.mode == "OFFBOARD" && current_state.armed);
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
}

void position_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  if (!allow_ego_control)
    return;

  static ros::Publisher pub;
  static bool pub_inited = false;
  if (!pub_inited)
  {
    ros::NodeHandle nh;
    pub = nh.advertise<mavros_msgs::PositionTarget>(
        "/" + drone_ns + "/mavros/setpoint_raw/local", 10);
    pub_inited = true;
    ROS_INFO("[ego_to_mavros] Publishing to /%s/mavros/setpoint_raw/local", drone_ns.c_str());
  }

  mavros_msgs::PositionTarget target;
  target.header.stamp = ros::Time::now();
  target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

  // PX4 使用 EGO 给的 position + velocity + acceleration + yaw
  // 不设 IGNORE_PX/PY/PZ → PX4 会跟踪位置
  uint16_t mask = 0;
  mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  target.type_mask = mask;

  target.position.x = cmd->position.x;
  target.position.y = cmd->position.y;
  target.position.z = cmd->position.z;

  target.velocity.x = cmd->velocity.x;
  target.velocity.y = cmd->velocity.y;
  target.velocity.z = cmd->velocity.z;

  target.acceleration_or_force.x = cmd->acceleration.x;
  target.acceleration_or_force.y = cmd->acceleration.y;
  target.acceleration_or_force.z = cmd->acceleration.z;

  target.yaw = cmd->yaw;

  // 固定高度模式：覆盖 Z 方向指令
  if (fixed_height > 0.0)
  {
    target.position.z = fixed_height;
    target.velocity.z = 0.0;
    target.acceleration_or_force.z = 0.0;
  }

  pub.publish(target);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ego_to_mavros");
  ros::NodeHandle nh("~");

  nh.param<std::string>("drone_ns", drone_ns, "typhoon_h480_depth_0");
  nh.param<double>("fixed_height", fixed_height, 0.0);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
      "/" + drone_ns + "/mavros/state", 10, state_cb);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/" + drone_ns + "/mavros/local_position/pose", 10, pose_cb);

  ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
      "/position_cmd", 50, position_cmd_cb);

  ROS_INFO("[ego_to_mavros] Started. Listening to /position_cmd, publishing to /%s/mavros/setpoint_raw/local",
           drone_ns.c_str());
  ROS_INFO("[ego_to_mavros] Will only publish when OFFBOARD + armed.");

  ros::spin();
  return 0;
}
