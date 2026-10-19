#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    // obj_predictor_->init();
    // obj_pub_ = nh.advertise<visualization_msgs::Marker>("/dynamic/obj_prdi", 10); // zx-todo

    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    // A* 池尺寸与步长共同决定覆盖半径，须覆盖规划视界；平面模式减少 z 层预算。
    int pool_x, pool_y, pool_z;
    double a_star_step;
    nh.param("manager/a_star_pool_x", pool_x, 220);
    nh.param("manager/a_star_pool_y", pool_y, 220);
    nh.param("manager/a_star_pool_z", pool_z, 40);
    nh.param("manager/a_star_step", a_star_step, 0.25);

    {
      const double infl = grid_map_->getInflation();
      if (infl > 0.0 && a_star_step > infl)
      {
        ROS_WARN("[A*] step %.2f > obstacles_inflation %.2f, 有一步跨过障碍的风险, 已钳到 %.2f",
                 a_star_step, infl, infl);
        a_star_step = infl;
      }
      bspline_optimizer_->setAStarStep(a_star_step);

      const double cover_xy = 0.5 * std::min(pool_x, pool_y) * a_star_step;
      ROS_WARN("[A*] pool %dx%dx%d, step %.2f => 覆盖 x/y ±%.1fm, z ±%.1fm; "
               "planning_horizon %.1fm",
               pool_x, pool_y, pool_z, a_star_step, cover_xy,
               0.5 * pool_z * a_star_step, pp_.planning_horizen_);
      if (pp_.planning_horizen_ > 0.0 && cover_xy < pp_.planning_horizen_)
      {
        ROS_ERROR("[A*] 覆盖半径 %.1fm < planning_horizon %.1fm —— A* 会持续报 "
                  "Ran out of pool, 飞机会走走停停! 请调大 manager/a_star_pool_x|y "
                  "或 manager/a_star_step, 或调小 fsm/planning_horizon",
                  cover_xy, pp_.planning_horizen_);
      }
    }
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(pool_x, pool_y, pool_z));

    // plane_mode 固定全局轨迹、A* 和 B 样条的 z，实际高度由主控控制。
    nh.param("manager/plane_mode", plane_mode_, false);
    nh.param("manager/plane_z", plane_z_, 5.0);
    if (plane_mode_)
    {
      bspline_optimizer_->setPlaneMode(true, plane_z_);
      ROS_WARN("[规划] 平面模式已开启: 所有轨迹 z 恒为 %.2fm (A* 8邻域 + z梯度清零)",
               plane_z_);
    }

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    // 平面模式固定起终点 z。
    pinToPlane(start_pt);
    pinToPlane(local_target_pt);
    if (plane_mode_)
    {
      start_vel(2) = 0.0;
      start_acc(2) = 0.0;
      local_target_vel(2) = 0.0;
    }

    // 占据起点按八个方向有界搜索可用起点；固定的边界控制点无法由避障梯度推出。
    if (grid_map_->getInflateOccupancy(start_pt))
    {
      Eigen::Vector3d escape_dir = start_pt - local_target_pt;
      if (plane_mode_)
        escape_dir(2) = 0.0;
      double escape_dir_norm = escape_dir.norm();
      if (escape_dir_norm < 1e-3)
      {
        escape_dir = start_vel;
        if (plane_mode_)
          escape_dir(2) = 0.0;
        escape_dir_norm = escape_dir.norm();
      }
      if (escape_dir_norm < 1e-3)
        escape_dir = Eigen::Vector3d(1.0, 0.0, 0.0);
      else
        escape_dir /= escape_dir_norm;

      const double kEscapeStep = 0.1;      // 与 dyn_a_star.cpp 的 A* 步长一致
      const int kMaxEscapeStep = 50;       // 50 * 0.1 = 5m, 够穿过任何真实障碍

      bool escaped_ok = false;
      Eigen::Vector3d escaped_pt = start_pt;
      Eigen::Vector3d escaped_dir = escape_dir;
      int escaped_cnt = 0;

      for (int k = 0; k < 8 && !escaped_ok; ++k)
      {
        Eigen::Vector3d dir = escape_dir;
        if (k > 0)   // 主方向失败: 水平面内每 45 度换一个方向重试
        {
          const double ang = k * 45.0 * M_PI / 180.0;
          const double c = std::cos(ang), s = std::sin(ang);
          dir = Eigen::Vector3d(escape_dir(0) * c - escape_dir(1) * s,
                                escape_dir(0) * s + escape_dir(1) * c, 0.0);
        }
        Eigen::Vector3d p = start_pt;
        int cnt = 0;
        while (cnt < kMaxEscapeStep && grid_map_->getInflateOccupancy(p))
        {
          p += dir * kEscapeStep;
          pinToPlane(p);
          ++cnt;
        }
        if (!grid_map_->getInflateOccupancy(p))
        {
          escaped_ok = true;
          escaped_pt = p;
          escaped_dir = dir;
          escaped_cnt = cnt;
        }
      }

      if (!escaped_ok)
      {
        ROS_WARN_THROTTLE(0.2, "[drone %d][ego重规划] 起点在障碍物内, 8 个方向外推 %.1fm "
                                "均未脱离, 放弃本次重规划",
                           pp_.drone_id, kMaxEscapeStep * kEscapeStep);
        continous_failures_count_++;
        return false;
      }
      ROS_WARN("[drone %d][ego重规划] 起点在障碍物内, 沿 (%.2f,%.2f) 外推 %.2fm 到 "
               "(%.2f,%.2f,%.2f) 脱离占据",
               pp_.drone_id, escaped_dir(0), escaped_dir(1),
               escaped_cnt * kEscapeStep, escaped_pt(0), escaped_pt(1), escaped_pt(2));
      start_pt = escaped_pt;
    }

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      // 记录规划快速失败，便于诊断连续拒绝初值。
      ROS_WARN_THROTTLE(0.2, "[drone %d][ego重规划] Close to goal, dist=%.2f, "
                              "continous_failures=%d (本次不发新轨迹, 继续执行旧的)",
                         pp_.drone_id, (start_pt - local_target_pt).norm(), continous_failures_count_);
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt( local_target_pt );

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
        // 短距离初值按沿目标方向的起止速度计算三角/梯形时间，取较长者以限制速度和加速度。
        if (pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist && dist > 1e-3)
        {
          const Eigen::Vector3d dir = (local_target_pt - start_pt) / dist;
          const double vm = pp_.max_vel_, am = pp_.max_acc_;
          const double v0 = std::min(std::max(start_vel.dot(dir), 0.0), vm);
          const double v1 = std::min(std::max(local_target_vel.dot(dir), 0.0), vm);
          const double vp = sqrt(am * dist + 0.5 * (v0 * v0 + v1 * v1));
          double t_profile;
          if (vp < std::max(v0, v1))
            t_profile = 2.0 * dist / std::max(v0 + v1, 1e-6); // 来不及变速: 按平均速度
          else if (vp <= vm)
            t_profile = (2.0 * vp - v0 - v1) / am; // 三角形
          else
            t_profile = (2.0 * vm - v0 - v1) / am +
                        (dist - (2.0 * vm * vm - v0 * v0 - v1 * v1) / (2.0 * am)) / vm; // 梯形
          time = std::max(time, t_profile);
        }

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
          // 记录随机中点分支，便于诊断初始路径折返。
          ROS_WARN("[drone %d][ego重规划] 随机插点重规划: start=(%.1f,%.1f) mid_offset_pt=(%.1f,%.1f) "
                   "local_tgt=(%.1f,%.1f) continous_failures=%d",
                   pp_.drone_id, start_pt(0), start_pt(1), random_inserted_pt(0), random_inserted_pt(1),
                   local_target_pt(0), local_target_pt(1), continous_failures_count_);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        // 旧轨迹已过期时回退多项式初值，避免采样空弧长。
        if (t_cur > local_data_.duration_)
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
          continue;
        }

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        // 弧长点少于两个或总长接近零时回退多项式，避免下标下溢和采样死循环。
        if (pseudo_arc_length.size() < 2 || pseudo_arc_length.back() < 1e-3)
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
          continue;
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        int sample_guard = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
          ++sample_guard;
        } while (point_set.size() < 7 && sample_guard < 64); // If the start point is very close to end point, this will help

        // 采样 64 次仍不足七点时回退多项式初值，避免病态弧长导致死循环。
        if (point_set.size() < 7)
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
          continue;
        }

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = ros::Time::now() - t_start;
      //static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = ros::Time::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    static bool print_once = true;
    if ( print_once )
    {
      print_once = false;
      ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
    }



    t_refine = ros::Time::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt + t_refine).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    // Emergency hover must use the same virtual plane as regular trajectories.
    pinToPlane(stop_pos);
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if ( local_data_.start_time_.toSec() < 1e9 ) // It means my first planning has not started
      return false;
    
    double my_traj_start_time = local_data_.start_time_.toSec();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();

    double t_start = max( my_traj_start_time, other_traj_start_time );
    double t_end = min( my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_ );

    for ( double t=t_start; t<t_end; t+=0.03 )
    {
      if ( (local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance() )
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());


    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    // 平面模式从全局轨迹起终点固定 z。
    vector<Eigen::Vector3d> points;
    Eigen::Vector3d sp = start_pos, ep = end_pos;
    pinToPlane(sp);
    pinToPlane(ep);
    Eigen::Vector3d sv = start_vel, ev = end_vel;
    Eigen::Vector3d sa = start_acc, ea = end_acc;
    if (plane_mode_)
    {
      sv(2) = ev(2) = sa(2) = ea(2) = 0.0;
    }
    points.push_back(sp);
    points.push_back(ep);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, sv, ev, sa, ea, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(sp, sv, sa, ep, ev, ea, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
