
#include <plan_manage/ego_replan_fsm.h>
#include <cmath>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/goal_reached_thresh", goal_reached_thresh_, 1.0);
    if (!std::isfinite(goal_reached_thresh_) || goal_reached_thresh_ <= 0.0)
    {
      ROS_WARN("Invalid fsm/goal_reached_thresh; using 1.0m");
      goal_reached_thresh_ = 1.0;
    }
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    // 等待前机首条轨迹的时间上限。
    nh.param("fsm/seq_start_timeout", seq_start_timeout_, 10.0);
    // 重规划超时上限。
    nh.param("fsm/replan_stuck_timeout", replan_stuck_timeout_, 1.0);
    // 新目标到达时，旧轨迹与里程计偏差超过门限则重新起步。
    nh.param("fsm/replan_from_odom_dist", replan_from_odom_dist_, 1.0);
    // 允许的正常跟随滞后: 位置差 <= dist + lag*|v轨迹|, 速度差 <= max(vel, 0.5*|v轨迹|)
    nh.param("fsm/replan_from_odom_lag", replan_from_odom_lag_, 0.35);
    nh.param("fsm/replan_from_odom_vel", replan_from_odom_vel_, 2.0);

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    // 运行时速度和加速度限制。
    speed_limit_sub_ = nh.subscribe("speed_limit", 1, &EGOReplanFSM::speedLimitCallback, this);

    // 运行时轨迹碰撞急停开关。
    disable_collision_check_sub_ = nh.subscribe("disable_collision_check", 1,
        &EGOReplanFSM::disableCollisionCheckCallback, this);

    // 运动目标末端速度。
    goal_vel_sub_ = nh.subscribe("goal_vel", 1, &EGOReplanFSM::goalVelCallback, this);

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &EGOReplanFSM::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
    }
    string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);

    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());

    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];

    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);





    // // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    // ros::Duration(0.001).sleep();
    // visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    // ros::Duration(0.001).sleep();
    // else
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    // 防止目标回调重入。
    if (in_plan_wp_cb_)
      return;
    in_plan_wp_cb_ = true;

    // This is the virtual XY planning plane shared with the projected lidar map.
    // The flight controller owns physical patrol/tracking altitude independently.
    Eigen::Vector3d planning_wp = next_wp;
    planner_manager_->pinToPlane(planning_wp);

    bool success = false;
    // 使用目标消息提供的末端速度。
    Eigen::Vector3d end_vel = have_goal_vel_ ? goal_vel_ : Eigen::Vector3d::Zero();
    if (planner_manager_->plane_mode_) end_vel(2) = 0.0;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), planning_wp, end_vel, Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = planning_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_ = end_vel;
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EMERGENCY_STOP)
      {
        // 新目标不覆盖 EMERGENCY_STOP，保留急停恢复流程。
      }
      else
      {
        // 限制回调中的等待时长。
        const ros::Time wait_start = ros::Time::now();
        while (exec_state_ != EXEC_TRAJ)
        {
          if ((ros::Time::now() - wait_start).toSec() > 0.5)
          {
            ROS_WARN("[ego fsm] wait EXEC_TRAJ timeout, replan anyway (state=%d)", int(exec_state_));
            break;
          }
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        // 旧轨迹过期、未执行或落后超过容差时，从实时里程计重建；普通跟踪滞后不重置。
        LocalTrajData *info = &planner_manager_->local_data_;
        const double t_cur = (ros::Time::now() - info->start_time_).toSec();
        bool from_odom = t_cur > info->duration_;
        double pos_dev = 0.0, vel_dev = 0.0, pos_tol = 0.0, vel_tol = 0.0;
        if (!from_odom)
        {
          Eigen::Vector3d dev = info->position_traj_.evaluateDeBoorT(t_cur) - odom_pos_;
          Eigen::Vector3d v_traj = info->velocity_traj_.evaluateDeBoorT(t_cur);
          Eigen::Vector3d dv = v_traj - odom_vel_;
          if (planner_manager_->plane_mode_)
          {
            dev(2) = 0.0;
            v_traj(2) = 0.0;
            dv(2) = 0.0;
          }
          pos_dev = dev.norm();
          vel_dev = dv.norm();
          pos_tol = replan_from_odom_dist_ + replan_from_odom_lag_ * v_traj.norm();
          vel_tol = std::max(replan_from_odom_vel_, 0.5 * v_traj.norm());
          from_odom = pos_dev > pos_tol || vel_dev > vel_tol;
        }
        if (from_odom)
          ROS_INFO("[drone %d][ego重规划] 新目标: 旧轨迹%s (位置差 %.1f/%.1fm, 速度差 %.1f/%.1fm/s), "
                   "从里程计重新起步", planner_manager_->pp_.drone_id,
                   t_cur > info->duration_ ? "已过期" : "未被执行", pos_dev, pos_tol, vel_dev, vel_tol);
        changeFSMExecState(from_odom ? GEN_NEW_TRAJ : REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    in_plan_wp_cb_ = false;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    planNextWaypoint(end_wp);
  }

  // 运行时速度消息为 [max_vel, max_acc]；只更新参数，不在回调中嵌套重规划。
  void EGOReplanFSM::speedLimitCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    if (msg->data.empty())
      return;

    const double v = msg->data[0];
    if (v <= 0.05 || v > 30.0)
    {
      ROS_WARN_THROTTLE(2.0, "[ego fsm] speed_limit %.2f out of range, ignored", v);
      return;
    }
    const double a = (msg->data.size() > 1 && msg->data[1] > 0.05) ? msg->data[1]
                                                                   : planner_manager_->pp_.max_acc_;

    if (fabs(v - planner_manager_->pp_.max_vel_) < 1e-3 && fabs(a - planner_manager_->pp_.max_acc_) < 1e-3)
      return;

    planner_manager_->setSpeedLimit(v, a);
    ROS_WARN("[ego fsm] speed limit -> max_vel=%.2f max_acc=%.2f", v, a);
  }

  // 运行时轨迹碰撞急停开关。
  void EGOReplanFSM::disableCollisionCheckCallback(const std_msgs::BoolConstPtr &msg)
  {
    if (msg->data == disable_collision_check_)
      return;
    disable_collision_check_ = msg->data;
    // 同时关掉 B 样条优化后端的深度障碍检查(否则友机被误标成障碍时,
    // "前 3 个控制点在障碍里"直接 return false, 规划失败卡在进场)。
    planner_manager_->setDisableCollisionCheck(msg->data);
    ROS_WARN("[ego fsm] 碰撞急停 -> %s", disable_collision_check_ ? "关闭(巡逻, 马路净空)" : "开启(跟踪)");
  }

  // 使用目标末端速度。
  void EGOReplanFSM::goalVelCallback(const geometry_msgs::TwistStampedConstPtr &msg)
  {
    goal_vel_(0) = msg->twist.linear.x;
    goal_vel_(1) = msg->twist.linear.y;
    goal_vel_(2) = msg->twist.linear.z;
    have_goal_vel_ = true;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;
    // nav_msgs/Odometry 的 twist 位于 child_frame，先旋转到世界系。
    if (!msg->child_frame_id.empty() && msg->child_frame_id != msg->header.frame_id)
    {
      const Eigen::Quaterniond q_body(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      if (q_body.norm() > 1e-6)
        odom_vel_ = q_body.normalized() * odom_vel_;
    }


    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
                msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    /* Fill up the buffer */
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;
    // planner_manager_->swarm_trajs_buf_[id].start_time_ = ros::Time::now(); // Un-reliable time sync

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;


    if (!have_odom_)
    {
      ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      ROS_ERROR("Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    // 记录急停开始时间，用于超时恢复。
    if (new_state == EMERGENCY_STOP && exec_state_ != EMERGENCY_STOP)
      emergency_stop_t0_ = ros::Time::now();

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    // 仅在 FSM 状态变化时输出日志。
    if (pre_s != int(new_state))
      ROS_INFO("[drone %d][ego fsm][%s] %s -> %s", planner_manager_->pp_.drone_id,
                pos_call.c_str(), state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      // 前机轨迹等待有超时，超时后允许独立启动。
      if (seq_start_wait_t0_.isZero())
        seq_start_wait_t0_ = ros::Time::now();
      const bool pre_agent_timeout =
          (ros::Time::now() - seq_start_wait_t0_).toSec() > seq_start_timeout_;
      if (pre_agent_timeout && !have_recv_pre_agent_)
      {
        ROS_WARN_THROTTLE(2.0,
            "[drone %d] 等前一架的首条轨迹超过 %.1fs, 不再等待, 自行起飞 "
            "(机间避障仍由 broadcast_bspline 保证)",
            planner_manager_->pp_.drone_id, seq_start_timeout_);
      }

      if (planner_manager_->pp_.drone_id <= 0 ||
          (planner_manager_->pp_.drone_id >= 1 && (have_recv_pre_agent_ || pre_agent_timeout)))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {


      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        // REPLAN 持续超时后从实时里程计以多项式重新初始化。
        if (continously_called_times_ > (int)(replan_stuck_timeout_ / 0.01))
        {
          ROS_WARN("[ego fsm] REPLAN_TRAJ 连续失败超过 %.1fs, 起始状态可能已退化, "
                    "改用里程计重新起步 (GEN_NEW_TRAJ)", replan_stuck_timeout_);
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_-1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          // A finished B-spline is not proof that the vehicle reached its goal.
          // Optimization can leave its endpoint short, and tracking may lag.
          // traj_server keeps publishing fresh hover commands after completion;
          // WAIT_TARGET here would strand fly_px4_1 outside its arrival radius.
          Eigen::Vector3d odom_goal_error = end_pt_ - odom_pos_;
          // In plane mode the controller owns altitude (e.g. tracking at 2.7m
          // while EGO plans XY motion at 4.5m). Check actual horizontal arrival.
          // A 3D planner must still reach its goal in all three coordinates.
          if (planner_manager_->plane_mode_) odom_goal_error(2) = 0.0;
          const double odom_goal_distance = odom_goal_error.norm();
          if (odom_goal_distance > goal_reached_thresh_)
          {
            ROS_WARN_THROTTLE(1.0,
                "[drone %d][ego goal] trajectory finished but odom gap=%.2fm > %.2fm; replan from odometry",
                planner_manager_->pp_.drone_id, odom_goal_distance, goal_reached_thresh_);
            // The old trajectory has expired. Regenerate from actual odometry
            // instead of sampling its endpoint as a new start state.
            changeFSMExecState(GEN_NEW_TRAJ, "GOAL_NOT_REACHED");
            break;
          }

          have_target_ = false;
          have_trigger_ = false;

          if ( target_type_ == TARGET_TYPE::PRESET_TARGET )
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        // 急停在速度接近零或等待超时后恢复。
        const double estop_sec = (ros::Time::now() - emergency_stop_t0_).toSec();
        if ((enable_fail_safe_ && odom_vel_.norm() < 0.1) || estop_sec > 3.0)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();


    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    // check lost of depth
    // 每次深度断流只锁存一次急停；数据恢复后重新允许下一次断流检测。
    if (map->getOdomDepthTimeout())
    {
      if (depth_lost_armed_ && exec_state_ != EMERGENCY_STOP)
      {
        ROS_ERROR_THROTTLE(1.0, "Depth Lost! EMERGENCY_STOP (深度恢复后会自动重新规划)");
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        depth_lost_armed_ = false;   // 本周期内不再重复急停, 破死循环
      }
    }
    else
    {
      depth_lost_armed_ = true;      // 深度恢复, 重新武装
    }

    // check trajectory
    // 关闭轨迹碰撞检查仍保留深度超时检查。
    if (disable_collision_check_)
      return;

    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = info->duration_ * 2 / 3;

    // 静态占据检查跳过轨迹起点后的膨胀余量 +0.3m，避免起点被膨胀带锁死。
    // 机间碰撞检查始终保留。
    const double self_clearance = map->getInflation() + 0.3;
    double committed = 0.0;
    Eigen::Vector3d p_prev = p_cur;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d p_t = info->position_traj_.evaluateDeBoorT(t);
      committed += (p_t - p_prev).norm();
      p_prev = p_t;

      bool occ = false;
      if (committed >= self_clearance)
        occ |= map->getInflateOccupancy(p_t);

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;
    // 重规划诊断记录起点、局部目标、终点、距离和成功标志。
    ROS_INFO("[drone %d][ego重规划] start=(%.1f,%.1f) v=%.1f -> local_tgt=(%.1f,%.1f) v_tgt=%.1f "
             "end=(%.1f,%.1f) dist_start_to_end=%.1f use_poly_init=%d rand_poly=%d success=%d",
             planner_manager_->pp_.drone_id,
             start_pt_(0), start_pt_(1), start_vel_.norm(),
             local_target_pt_(0), local_target_pt_(1), local_target_vel_.norm(),
             end_pt_(0), end_pt_(1), (end_pt_ - start_pt_).norm(),
             (int)flag_use_poly_init, (int)flag_randomPolyTraj, (int)plan_and_refine_success);

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_.publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
      }
      swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_.publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;

    // 旧全局进度超出规划视界时，重新对齐到全局轨迹最近点。
    {
      Eigen::Vector3d pos_stale = planner_manager_->global_data_.getPosition(planner_manager_->global_data_.last_progress_time_);
      double dist_stale = (pos_stale - start_pt_).norm();
      if (dist_stale >= planning_horizen_)
      {
        double resync_dist_min = 9999, resync_t_min = 0.0;
        for (double tt = 0.0; tt < planner_manager_->global_data_.global_duration_; tt += t_step)
        {
          double d = (planner_manager_->global_data_.getPosition(tt) - start_pt_).norm();
          if (d < resync_dist_min)
          {
            resync_dist_min = d;
            resync_t_min = tt;
          }
        }
        ROS_WARN("[drone %d][ego重规划] last_progress_time_滞后修复: old_t=%.2f dist=%.1f(>=horizen=%.1f) "
                 "-> resync_t=%.2f dist=%.1f start=(%.1f,%.1f)",
                 planner_manager_->pp_.drone_id, planner_manager_->global_data_.last_progress_time_, dist_stale,
                 planning_horizen_, resync_t_min, resync_dist_min, start_pt_(0), start_pt_(1));
        planner_manager_->global_data_.last_progress_time_ = resync_t_min;
      }
    }

    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      // planner_manager_->grid_map_;
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // 接近终点时使用运动目标的末端速度。
      local_target_vel_ = have_goal_vel_ ? goal_vel_ : Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
