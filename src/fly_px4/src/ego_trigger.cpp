#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/PositionTarget.h>
#include <thread>
#include <mutex>
#include <clocale>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
std::string drone_ns;

std::mutex cmd_mutex;
float fly_vx = 0, fly_vy = 0, fly_vz = 0;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

void set_fly_vel(float vx, float vy, float vz)
{
    std::lock_guard<std::mutex> lock(cmd_mutex);
    fly_vx = vx;
    fly_vy = vy;
    fly_vz = vz;
}

struct Timer
{
    ros::Time start;
    bool running = false;
    bool elapsed(double sec)
    {
        auto now = ros::Time::now();
        if (!running) { start = now; running = true; }
        if ((now - start).toSec() >= sec) { running = false; return true; }
        return false;
    }
    void reset() { running = false; }
};

// 发送目标点到 EGO
void sendGoal(ros::Publisher &goal_pub, double x, double y, double z)
{
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.w = 1.0;
    goal_pub.publish(goal);
    ROS_INFO("[ego_trigger] 目标点已发送: (%.1f, %.1f, %.1f)", x, y, z);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "ego_trigger");
    ros::NodeHandle nh("~");

    nh.param<std::string>("drone_ns", drone_ns, "typhoon_h480_depth_0");

    std::string prefix = "/" + drone_ns + "/mavros";

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        prefix + "/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        prefix + "/local_position/pose", 10, pose_cb);

    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>(
        prefix + "/setpoint_raw/local", 10);

    ros::ServiceClient param_client = nh.serviceClient<mavros_msgs::ParamSet>(
        prefix + "/param/set");
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        prefix + "/set_mode");
    ros::ServiceClient arm_client = nh.serviceClient<mavros_msgs::CommandBool>(
        prefix + "/cmd/arming");

    // ── EGO 动态目标话题（flight_type=1 时 EGO 订阅此话题） ──
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, true);

    // ── setpoint 发布线程 ──
    std::thread cmd_thread([&](){
        ros::Rate rate(30);
        while (ros::ok())
        {
            mavros_msgs::PositionTarget target;
            target.header.stamp = ros::Time::now();
            target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                               mavros_msgs::PositionTarget::IGNORE_PY |
                               mavros_msgs::PositionTarget::IGNORE_PZ |
                               mavros_msgs::PositionTarget::IGNORE_AFX |
                               mavros_msgs::PositionTarget::IGNORE_AFY |
                               mavros_msgs::PositionTarget::IGNORE_AFZ |
                               mavros_msgs::PositionTarget::IGNORE_YAW |
                               mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex);
                target.velocity.x = fly_vx;
                target.velocity.y = fly_vy;
                target.velocity.z = fly_vz;
            }
            setpoint_pub.publish(target);
            rate.sleep();
        }
    });
    cmd_thread.detach();

    // ── 1. 等飞控连接 ──
    ROS_INFO("[ego_trigger] 等待飞控连接...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("[ego_trigger] 飞控已连接");

    // ── 2. 设参数 ──
    {
        mavros_msgs::ParamSet srv;
        mavros_msgs::ParamValue v;
        v.integer = 4; v.real = 0.0;
        srv.request.value = v;

        srv.request.param_id = "NAV_RCL_ACT";
        param_client.call(srv);

        srv.request.param_id = "COM_RCL_EXCEPT";
        param_client.call(srv);
    }

    // ── 3. 切 OFFBOARD ──
    ROS_INFO("[ego_trigger] 切换 OFFBOARD...");
    Timer timer;
    while (ros::ok() && current_state.mode != "OFFBOARD")
    {
        if (timer.elapsed(3.0))
        {
            mavros_msgs::SetMode cmd;
            cmd.request.custom_mode = "OFFBOARD";
            mode_client.call(cmd);
        }
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    ROS_INFO("[ego_trigger] OFFBOARD 切换成功");

    // ── 4. 解锁 ──
    ROS_INFO("[ego_trigger] 解锁...");
    timer.reset();
    while (ros::ok() && !current_state.armed)
    {
        if (timer.elapsed(3.0))
        {
            mavros_msgs::CommandBool cmd;
            cmd.request.value = true;
            arm_client.call(cmd);
        }
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    ROS_INFO("[ego_trigger] 解锁成功");

    // ── 5. 起飞到 3m ──
    ROS_INFO("[ego_trigger] 起飞到 3m...");
    while (ros::ok() && current_pose.pose.position.z < 3.0)
    {
        set_fly_vel(0, 0, 1.5);
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    set_fly_vel(0, 0, 0);
    ROS_INFO("[ego_trigger] 已到达 3m，悬停建图中...");

    // ── 6. 悬停等 EGO 建图 ──
    ros::Duration(8.0).sleep();
    ROS_INFO("[ego_trigger] 建图等待完成");

    // ── 7. 发送目标点给 EGO ──
    sendGoal(goal_pub, 20, 6, 3.0);

    ROS_INFO("[ego_trigger] 无人机已由 EGO 接管！");

    ros::Rate main_rate(1);
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(10, "[ego_trigger] 运行中... 位置: (%.1f, %.1f, %.1f)m, 模式: %s",
                          current_pose.pose.position.x,
                          current_pose.pose.position.y,
                          current_pose.pose.position.z,
                          current_state.mode.c_str());
        ros::spinOnce();
        main_rate.sleep();
    }

    return 0;
}
