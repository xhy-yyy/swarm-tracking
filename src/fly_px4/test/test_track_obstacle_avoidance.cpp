// 跟踪避障单测 + 离线闭环对比 (不依赖 ROS/PX4/Gazebo)。
// 闭环部分把 fly_px4_1 的站位/PID/斜率限幅原样复刻, 分别接"原方向采样"和
// "新净空场过滤", 用仿真二维雷达(512 线/20m)看真实墙距、视线遮挡和交回 EGO 次数。
#include <fly_px4/track_obstacle_avoidance.hpp>

#include <chrono>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(bool value, const std::string& name) {
    if (!value) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
    else std::cout << "PASS: " << name << '\n';
}
constexpr double kPi = 3.14159265358979323846;
double wrapRad(double a) { while (a > kPi) a -= 2 * kPi; while (a < -kPi) a += 2 * kPi; return a; }
double wrapDeg(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

// 真实几何 (多边形)
struct Poly { std::vector<std::pair<double, double>> v; };
Poly rect(double x0, double y0, double x1, double y1) {
    return Poly{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
}
Poly disc(double cx, double cy, double r) {
    Poly p;
    for (int i = 0; i < 12; ++i)
        p.v.emplace_back(cx + r * std::cos(i * kPi / 6), cy + r * std::sin(i * kPi / 6));
    return p;
}
bool inside(const Poly& p, double x, double y) {
    bool in = false;
    for (std::size_t i = 0, j = p.v.size() - 1; i < p.v.size(); j = i++) {
        const auto& a = p.v[i]; const auto& b = p.v[j];
        if ((a.second > y) != (b.second > y) &&
            x < (b.first - a.first) * (y - a.second) / (b.second - a.second) + a.first) in = !in;
    }
    return in;
}
double segDist(double px, double py, double ax, double ay, double bx, double by) {
    const double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    double t = l2 > 1e-12 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0.0;
    t = std::min(1.0, std::max(0.0, t));
    return std::hypot(px - ax - t * dx, py - ay - t * dy);
}
// 带符号: 在多边形内为负
double trueClearance(const std::vector<Poly>& w, double x, double y) {
    double best = 1e9;
    for (const auto& p : w) {
        double d = 1e9;
        for (std::size_t i = 0, j = p.v.size() - 1; i < p.v.size(); j = i++)
            d = std::min(d, segDist(x, y, p.v[j].first, p.v[j].second, p.v[i].first, p.v[i].second));
        best = std::min(best, inside(p, x, y) ? -d : d);
    }
    return best;
}
bool segCross(double ax, double ay, double bx, double by,
              double cx, double cy, double dx, double dy, double* t = nullptr) {
    const double rx = bx - ax, ry = by - ay, sx = dx - cx, sy = dy - cy;
    const double den = rx * sy - ry * sx;
    if (std::fabs(den) < 1e-12) return false;
    const double u = ((cx - ax) * sy - (cy - ay) * sx) / den;
    const double v = ((cx - ax) * ry - (cy - ay) * rx) / den;
    if (u < 0 || u > 1 || v < 0 || v > 1) return false;
    if (t) *t = u;
    return true;
}
// 视线: 本机->行人 (末端 0.3m 内的交点不算, 行人贴墙走)
bool occluded(const std::vector<Poly>& w, double ax, double ay, double bx, double by) {
    const double len = std::hypot(bx - ax, by - ay);
    for (const auto& p : w)
        for (std::size_t i = 0, j = p.v.size() - 1; i < p.v.size(); j = i++) {
            double t;
            if (segCross(ax, ay, bx, by, p.v[j].first, p.v[j].second,
                         p.v[i].first, p.v[i].second, &t) && (1.0 - t) * len > 0.3) return true;
        }
    return false;
}
std::vector<std::pair<double, double>> lidarScan(const std::vector<Poly>& w, double x, double y) {
    std::vector<std::pair<double, double>> pts;
    for (int k = 0; k < 512; ++k) {
        const double a = -3.14 + 6.28 * k / 511.0;
        const double ex = x + 20.0 * std::cos(a), ey = y + 20.0 * std::sin(a);
        double best = 2.0;
        for (const auto& p : w)
            for (std::size_t i = 0, j = p.v.size() - 1; i < p.v.size(); j = i++) {
                double t;
                if (segCross(x, y, ex, ey, p.v[j].first, p.v[j].second,
                             p.v[i].first, p.v[i].second, &t)) best = std::min(best, t);
            }
        if (best <= 1.0 && best * 20.0 >= 0.5)
            pts.emplace_back(x + best * 20.0 * std::cos(a), y + best * 20.0 * std::sin(a));
    }
    return pts;
}

// 原 ego 膨胀带 (grid 0.1m, 方形膨胀 1.5m, 只含最新一帧)
struct InflatedBand {
    double res = 0.1, infl = 1.5;
    int n = 0, ox = 0, oy = 0;
    std::vector<uint8_t> inf;
    void build(const std::vector<std::pair<double, double>>& scan, double cx, double cy) {
        const double half = 12.0;
        n = static_cast<int>(2 * half / res);
        ox = static_cast<int>(std::floor((cx - half) / res));
        oy = static_cast<int>(std::floor((cy - half) / res));
        std::vector<uint8_t> raw(n * n, 0), tmp(n * n, 0);
        inf.assign(n * n, 0);
        for (const auto& p : scan) {
            const int i = static_cast<int>(std::floor(p.first / res)) - ox;
            const int j = static_cast<int>(std::floor(p.second / res)) - oy;
            if (i >= 0 && j >= 0 && i < n && j < n) raw[j * n + i] = 1;
        }
        const int s = static_cast<int>(std::ceil(infl / res));
        for (int j = 0; j < n; ++j) {
            int cnt = 0;
            for (int i = -s; i < n + s; ++i) {
                if (i + s < n && i + s >= 0) cnt += raw[j * n + i + s];
                if (i - s - 1 >= 0 && i - s - 1 < n) cnt -= raw[j * n + i - s - 1];
                if (i >= 0 && i < n) tmp[j * n + i] = cnt > 0;
            }
        }
        for (int i = 0; i < n; ++i) {
            int cnt = 0;
            for (int j = -s; j < n + s; ++j) {
                if (j + s < n && j + s >= 0) cnt += tmp[(j + s) * n + i];
                if (j - s - 1 >= 0 && j - s - 1 < n) cnt -= tmp[(j - s - 1) * n + i];
                if (j >= 0 && j < n) inf[j * n + i] = cnt > 0;
            }
        }
    }
    void nearby(double cx, double cy, double r, std::vector<std::pair<float, float>>& out) const {
        out.clear();
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                if (!inf[j * n + i]) continue;
                const double x = (ox + i + 0.5) * res, y = (oy + j + 0.5) * res;
                if (std::hypot(x - cx, y - cy) <= r) out.emplace_back(x, y);
            }
    }
};

// fly_px4_1 原 avoidBySampling / rayBlocked 原样复刻 (yaml: lookahead 2.5, clear 0.55, 7, 0.4)
bool rayBlocked(double wx, double wy, double ux, double uy, double L, double clear,
                const std::vector<std::pair<float, float>>& pts) {
    const double ax = ux * L, ay = uy * L, len2 = ax * ax + ay * ay, c2 = clear * clear;
    for (const auto& p : pts) {
        const double px = p.first - wx, py = p.second - wy;
        double t = (len2 > 1e-9) ? (px * ax + py * ay) / len2 : 0.0;
        t = std::min(1.0, std::max(0.0, t));
        const double dx = px - t * ax, dy = py - t * ay;
        if (dx * dx + dy * dy < c2) return true;
    }
    return false;
}
bool oldAvoid(const InflatedBand& band, double wx, double wy, double dvx, double dvy,
              bool cmd_init, double cvx, double cvy, double& ox, double& oy) {
    ox = dvx; oy = dvy;
    const double speed = std::hypot(dvx, dvy);
    if (speed < 1e-3) return true;
    const double L = std::max(2.5, speed * 1.2 + 0.8);
    std::vector<std::pair<float, float>> pts;
    band.nearby(wx, wy, L + 0.55 + 0.5, pts);
    if (pts.empty()) return true;
    const double base = std::atan2(dvy, dvx);
    const double cur = (cmd_init && std::hypot(cvx, cvy) > 1e-3) ? std::atan2(cvy, cvx) : base;
    const double step = 15.0 * kPi / 180.0;
    double best_cost = 1e18, best_ang = base;
    bool found = false;
    for (int i = -7; i <= 7; ++i) {
        const double ang = base + i * step;
        if (rayBlocked(wx, wy, std::cos(ang), std::sin(ang), L, 0.55, pts)) continue;
        const double cost = std::fabs(wrapRad(ang - base)) + 0.4 * std::fabs(wrapRad(ang - cur));
        if (cost < best_cost) { best_cost = cost; best_ang = ang; found = true; }
    }
    if (!found) return false;
    ox = speed * std::cos(best_ang); oy = speed * std::sin(best_ang);
    return true;
}

// 行人脚本
struct Walker {
    std::vector<std::pair<double, double>> wp;
    double speed = 1.0;
    std::size_t seg = 0;
    double x = 0, y = 0, vx = 0, vy = 0;
    bool done = false;
    void reset() { seg = 0; x = wp[0].first; y = wp[0].second; done = false; }
    void step(double dt) {
        double left = speed * dt;
        vx = vy = 0;
        while (left > 1e-9 && seg + 1 < wp.size()) {
            const double dx = wp[seg + 1].first - x, dy = wp[seg + 1].second - y;
            const double d = std::hypot(dx, dy);
            if (d < 1e-9) { ++seg; continue; }
            const double m = std::min(left, d);
            x += dx / d * m; y += dy / d * m;
            vx = dx / d * speed; vy = dy / d * speed;
            left -= m;
            if (m >= d - 1e-9) ++seg;
        }
        if (seg + 1 >= wp.size()) done = true;
    }
};

struct Metrics {
    double min_clear = 1e9, t_lt15 = 0, t_lt10 = 0, t_lt20 = 0, t_occ = 0, d_err = 0, t = 0;
    int handoffs = 0, standoff_bad = 0;
    double t_intervene = 0, max_dcmd = 0, t_handoff = 0;
    int passes = 0;        // 飞机从行人身边绕到另一侧 (3s 内方位转 >120°, 距人 <7m)
    int jumps = 0;         // 站位点单拍跳变 >4m
    double t_blind = 0;    // 离人 <3.7m (相机下沿盲区)
    double max_speed = 0;
    double t_corridor = 0; // 站位处在走廊档
};

// OLD = 原方向采样 + 膨胀带; V1 = 净空场 (走廊模式关, 即上一版); V2 = 净空场 + 走廊模式
enum class Mode { OLD, V1, V2 };
// 与 patrol_routes.yaml 当前值一致 (standoff_radius / track_avoid_*)
struct LoopCfg {
    double radius = 5.5, d_hard = 1.0, d_safe = 2.5, lag = 0.2;
};

fly_px4::StandoffObstacleParams standoffParams(const LoopCfg& cfg, bool corridor) {
    // 与 fly_px4_1.cpp loadRoute 的派生关系一致
    fly_px4::StandoffObstacleParams sp;
    sp.corridor = corridor;
    sp.wall_min = std::min(sp.wall_clear, cfg.d_hard + 0.6);
    sp.corridor_clear = std::min(sp.wall_clear, std::max(sp.corridor_clear, cfg.d_hard + 0.3));
    sp.min_radius = 4.0;
    sp.corridor_cliff = std::max(cfg.d_hard + 0.2, std::min(sp.corridor_clear - 0.3, sp.wall_min));
    return sp;
}

// 一次闭环
Metrics runLoop(const std::vector<Poly>& world, Walker walker, double sx0, double sy0,
                Mode mode, double t_max, std::vector<std::pair<double, double>>* cmds = nullptr,
                double* us_per_tick = nullptr, const LoopCfg& cfg = LoopCfg()) {
    const bool use_new = mode != Mode::OLD;
    const double dt = 0.02;
    // PID (patrol_routes.yaml 当前值)
    const double kp = 0.5, ki = 0.0, kd = 0.1, err_tau = 0.1, ff_dead = 0.35;
    const double acc_max = 2.5, vmax = 3.0, lead_tau = 0.3;
    walker.reset();
    double x = sx0, y = sy0, avx = 0, avy = 0, lpx = 0, lpy = 0;
    bool vl_init = false, cmd_init = false;
    double ex = 0, ey = 0, exp = 0, eyp = 0, ix = 0, iy = 0;
    double cvx = 0, cvy = 0;
    bool az_valid = false; double az_prev = 0;
    bool lead_valid = false; double lvx = 0, lvy = 0;
    int cand = -1;
    fly_px4::SurfaceMemory mem(0.2);
    fly_px4::ClearanceField field;
    fly_px4::VelocityGuardParams gp; gp.acc = acc_max;
    gp.d_hard = cfg.d_hard; gp.d_safe = cfg.d_safe; gp.lag = cfg.lag;
    const fly_px4::StandoffObstacleParams sp = standoffParams(cfg, mode == Mode::V2);
    double corridor_ref = gp.d_safe, corridor_until = -1;
    std::deque<double> az_hist;
    double last_pass = -1e9, pgx = 0, pgy = 0;
    bool have_pg = false;
    InflatedBand band;
    bool guard_active = false; double blocked_since = -1, holdoff_until = -1;
    double last_block = -1e9, hold = 1.5;
    std::vector<std::pair<double, double>> scan;
    Metrics m;
    const double lpf_a = dt / (dt + 1.0 / (2 * kPi * 2.5));
    double t = 0, next_scan = 0, next_band = 0, next_field = 0;
    bool prev_ok = true;
    double work_us = 0; int ticks = 0;
    while (t < t_max && !walker.done) {
        walker.step(dt);
        t += dt;
        const double tx = walker.x, ty = walker.y, tvx = walker.vx, tvy = walker.vy;
        if (t >= next_scan) {
            scan = lidarScan(world, x, y);
            for (const auto& p : scan) mem.insert(p.first, p.second, t);
            next_scan = t + 1.0 / 30.0;
        }
        if (!use_new && t >= next_band) { band.build(scan, x, y); next_band = t + 0.11; }
        const auto w0 = std::chrono::steady_clock::now();
        if (use_new && t >= next_field) {
            mem.prune(t, 3.0, x, y, 22.0);
            field.build(mem, x, y, 14.0, t, 3.0, 0.8);
            next_field = t + 0.1;
        }
        // 站位 (selectStandoff 复刻; standoff_lead_sec=0)
        const double d_person = std::hypot(x - tx, y - ty);
        double r_eff = cfg.radius + (d_person < 4.0 ? 4.0 - d_person : 0.0);
        const double az0d = wrapDeg(std::atan2(y - ty, x - tx) * 180 / kPi);
        if (!az_valid) { az_prev = az0d; az_valid = true; }
        else {
            const double d = wrapDeg(az0d - az_prev);
            if (std::fabs(d) > 8.0) az_prev = wrapDeg(az_prev + (d - std::copysign(8.0, d)) * 0.5);
        }
        if (!lead_valid) { lvx = tvx; lvy = tvy; lead_valid = true; }
        else { const double a = dt / (dt + lead_tau); lvx += a * (tvx - lvx); lvy += a * (tvy - lvy); }
        static const double kOff[12] = {0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180};
        std::array<fly_px4::StandoffCandidate, 12> c;
        std::array<fly_px4::StandoffObstacleInfo, 12> info;
        std::array<double, 12> offs;
        for (int k = 0; k < 12; ++k) {
            const double az = (az_prev + kOff[k]) * kPi / 180;
            c[k].x = tx + r_eff * std::cos(az); c[k].y = ty + r_eff * std::sin(az);
            c[k].inside_bounds = true;
            offs[k] = kOff[k];
            if (use_new) info[k] = fly_px4::evaluateStandoffObstacles(field, c[k].x, c[k].y,
                                                                      tx, ty, x, y, sp);
        }
        // 与 selectStandoff 一致: 首选附近没有 A 档时才微调
        if (use_new && sp.corridor && !fly_px4::standoffNearATier(c, info, offs, cand, 3.5, sp)) {
            for (int k = 0; k < 12; ++k) {
                double rx, ry;
                if (!fly_px4::refineStandoffPoint(field, tx, ty, r_eff,
                        (az_prev + kOff[k]) * kPi / 180, x, y, tx, ty, sp, rx, ry)) continue;
                c[k].x = rx; c[k].y = ry;
                info[k] = fly_px4::evaluateStandoffObstacles(field, rx, ry, tx, ty, x, y, sp);
            }
        }
        bool corridor_tier = false;
        cand = use_new ? fly_px4::chooseObstacleAwareStandoff(c, info, offs, cand, 3.5, 0.5, sp,
                                                              nullptr, &corridor_tier)
                       : fly_px4::chooseStandoffCandidate(c, cand, 3.5, 0.5);
        const double gx = c[cand].x, gy = c[cand].y;
        if (corridor_tier) m.t_corridor += dt;
        if (have_pg && std::hypot(gx - pgx, gy - pgy) > 4.0) ++m.jumps;
        pgx = gx; pgy = gy; have_pg = true;
        if (trueClearance(world, gx, gy) < 1.5) ++m.standoff_bad;
        // PID (computeVelLoopCmd 复刻)
        double vx, vy;
        {
            const double eX = gx - x, eY = gy - y;
            if (!vl_init) { ex = eX; ey = eY; exp = eX; eyp = eY; ix = iy = 0; vl_init = true; }
            else { const double a = dt / (dt + err_tau); ex += a * (eX - ex); ey += a * (eY - ey); }
            ix = std::max(-2.0, std::min(2.0, ix + ex * dt));
            iy = std::max(-2.0, std::min(2.0, iy + ey * dt));
            const double dex = (ex - exp) / dt, dey = (ey - eyp) / dt;
            exp = ex; eyp = ey;
            double fx = lvx, fy = lvy;
            if (std::hypot(fx, fy) < ff_dead) fx = fy = 0;
            vx = fx + kp * ex + ki * ix + kd * dex;
            vy = fy + kp * ey + ki * iy + kd * dey;
            const double mg = std::hypot(vx, vy);
            if (mg > vmax) { vx *= vmax / mg; vy *= vmax / mg; }
        }
        // 避障
        bool ok = true;
        if (use_new) {
            if (t < holdoff_until) ok = false;
            else {
                fly_px4::VelocityGuardState gs;
                gs.px = x; gs.py = y; gs.v0x = avx; gs.v0y = avy;
                gs.vpx = cmd_init ? cvx : 0; gs.vpy = cmd_init ? cvy : 0;
                gs.vdx = vx; gs.vdy = vy; gs.has_person = true; gs.tx = tx; gs.ty = ty;
                gs.active = guard_active;
                if (sp.corridor && corridor_tier) {   // 与 guardVelLoop 一致
                    corridor_ref = std::max(sp.corridor_clear, info[cand].wall_clearance - 0.2);
                    corridor_until = t + 2.0;
                }
                if (t <= corridor_until) gs.safe_ref = corridor_ref;
                const auto r = fly_px4::guardVelocity(field, gp, gs);
                guard_active = r.intervened;
                if (r.intervened) m.t_intervene += dt;
                vx = r.vx; vy = r.vy;
                if (!r.blocked) blocked_since = -1;
                else {
                    if (blocked_since < 0) blocked_since = t;
                    if (t - blocked_since >= 0.6) {
                        hold = (t - last_block < 5.0) ? std::min(4.5, hold * 2.0) : 1.5;
                        last_block = t;
                        ok = false; holdoff_until = t + hold; blocked_since = -1;
                    }
                }
            }
        } else {
            ok = oldAvoid(band, x, y, vx, vy, cmd_init, cvx, cvy, vx, vy);
        }
        work_us += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - w0).count();
        ++ticks;
        if (!ok) {
            // 交回 EGO: 这里不模拟 EGO, 记次数并按悬停处理; PID 状态与真实代码一样清零
            if (prev_ok) ++m.handoffs;   // 计事件数, 不计拍数
            m.t_handoff += dt;
            vl_init = false; cmd_init = false; guard_active = false;
            vx = vy = 0;
            cvx = cvy = 0;
        } else {
            if (cmd_init) {
                double dvx = vx - cvx, dvy = vy - cvy;
                const double dm = std::hypot(dvx, dvy), lim = acc_max * dt;
                if (dm > lim) { dvx *= lim / dm; dvy *= lim / dm; }
                vx = cvx + dvx; vy = cvy + dvy;
            }
            m.max_dcmd = std::max(m.max_dcmd, std::hypot(vx - cvx, vy - cvy) / dt);
            cvx = vx; cvy = vy; cmd_init = true;
        }
        prev_ok = ok;
        if (cmds) cmds->emplace_back(vx, vy);
        // 执行: 指令低通 2.5Hz + PX4 速度环一阶 0.3s, 加速度 <=5
        lpx += lpf_a * (vx - lpx); lpy += lpf_a * (vy - lpy);
        double ax = (lpx - avx) / 0.3, ay = (lpy - avy) / 0.3;
        const double am = std::hypot(ax, ay);
        if (am > 5.0) { ax *= 5.0 / am; ay *= 5.0 / am; }
        avx += ax * dt; avy += ay * dt;
        x += avx * dt; y += avy * dt;
        // 指标
        const double cl = trueClearance(world, x, y);
        m.min_clear = std::min(m.min_clear, cl);
        if (cl < 1.5) m.t_lt15 += dt;
        if (cl < 1.0) m.t_lt10 += dt;
        if (cl < 2.0) m.t_lt20 += dt;
        if (occluded(world, x, y, tx, ty)) m.t_occ += dt;
        const double d_now = std::hypot(x - tx, y - ty);
        m.d_err += std::fabs(d_now - cfg.radius) * dt;
        if (d_now < 3.7) m.t_blind += dt;
        m.max_speed = std::max(m.max_speed, std::hypot(avx, avy));
        az_hist.push_back(std::atan2(y - ty, x - tx));
        if (az_hist.size() > 150) az_hist.pop_front();   // 3s
        if (az_hist.size() == 150 && d_now < 7.0 && t - last_pass > 3.0 &&
            std::fabs(wrapRad(az_hist.back() - az_hist.front())) > 120.0 * kPi / 180.0) {
            ++m.passes;
            last_pass = t;
        }
        m.t += dt;
    }
    if (m.t > 0) m.d_err /= m.t;
    if (us_per_tick && ticks) *us_per_tick = work_us / ticks;
    return m;
}

void printMetrics(const char* name, const char* method, const Metrics& m) {
    std::printf("  %-22s %-4s 绕到人另一侧 %d次  站位跳变 %d次  盲区 %4.1fs  最大速度 %.1f  走廊档 %4.1fs\n",
                name, method, m.passes, m.jumps, m.t_blind, m.max_speed, m.t_corridor);
    std::printf("  %-22s %-4s 最小墙距 %5.2fm  <1.5m %4.1fs  <1.0m %4.1fs  遮挡 %4.1fs/%4.1fs"
                "  距离误差 %4.2fm  交回EGO %3d次/%4.1fs  站位<1.5m %4d拍  介入 %4.1fs\n",
                name, method, m.min_clear, m.t_lt15, m.t_lt10, m.t_occ, m.t, m.d_err,
                m.handoffs, m.t_handoff, m.standoff_bad, m.t_intervene);
}

// 单测
void fieldBasics() {
    std::vector<std::pair<double, double>> pts;
    for (double y = -6; y <= 6; y += 0.1) pts.emplace_back(3.0, y);   // 墙 x=3
    pts.emplace_back(-4.0, 0.0);                                      // 细杆
    fly_px4::ClearanceField f;
    f.buildFromPoints(pts, 0.2, 0, 0, 12, 0.8);
    check(std::fabs(f.clearance(0, 0) - 3.0) < 0.25, "clearance to wall ~3m");
    check(std::fabs(f.clearance(-2.0, 0) - 2.0) < 0.25, "thin post counts for collision");
    check(f.wallClearance(-4.0, 0) > 6.5, "thin post is not a wall (no occlusion/wall cost)");
    check(f.segmentClearance(0, 0, 6, 0, 0, 0, true) < 0.2, "segment crossing wall has ~0 clearance");
    check(!std::isfinite(f.clearance(100, 100)), "outside window is unknown (not blocking)");
    fly_px4::ClearanceField empty;
    empty.buildFromPoints({}, 0.2, 0, 0, 12, 0.8);
    check(!std::isfinite(empty.clearance(0, 0)), "empty map is free");
}

void guardPassThrough() {
    fly_px4::ClearanceField f;
    std::vector<std::pair<double, double>> pts;
    for (double y = -6; y <= 6; y += 0.1) pts.emplace_back(8.0, y);
    f.buildFromPoints(pts, 0.2, 0, 0, 14, 0.8);
    fly_px4::VelocityGuardParams p;
    fly_px4::VelocityGuardState s;
    s.vdx = 0.0; s.vdy = 2.0; s.v0x = 0.0; s.v0y = 2.0; s.vpx = 0; s.vpy = 2.0;  // 平行墙 8m
    auto r = fly_px4::guardVelocity(f, p, s);
    check(!r.intervened && r.vx == s.vdx && r.vy == s.vdy, "far wall: PID velocity passes unchanged");
    s.vdx = 1.0; s.vdy = 0.0; s.v0x = 1.0; s.v0y = 0.0;
    r = fly_px4::guardVelocity(f, p, s);
    check(!r.intervened, "1m/s toward wall 8m away: no intervention yet");
}

void guardWall() {
    fly_px4::ClearanceField f;
    std::vector<std::pair<double, double>> pts;
    for (double y = -10; y <= 10; y += 0.1) pts.emplace_back(3.0, y);
    f.buildFromPoints(pts, 0.2, 0, 0, 14, 0.8);
    fly_px4::VelocityGuardParams p;
    fly_px4::VelocityGuardState s;
    s.vdx = 2.0; s.vdy = 0.0; s.v0x = 2.0; s.v0y = 0.0; s.vpx = 2.0; s.vpy = 0.0;
    auto r = fly_px4::guardVelocity(f, p, s);
    const auto e = fly_px4::evaluateGuardPath(f, p, 0, 0, 2.0, 0.0, r.vx, r.vy);
    check(r.intervened && r.vx < 1.0, "2m/s head-on at 3m: early brake/turn");
    check(e.min_all >= p.d_hard - 1e-6, "chosen path keeps hard clearance");
    // 平行墙 1.7m: 向外偏
    fly_px4::VelocityGuardState q;
    q.px = 1.3; q.vdx = 0.0; q.vdy = 1.5; q.v0y = 1.5; q.vpy = 1.5;
    r = fly_px4::guardVelocity(f, p, q);
    check(r.intervened && r.vx < -0.05 && r.vy > 0.8, "parallel wall at 1.7m: drift away, keep going");
}

void guardCorner() {
    // 凸角: 楼 x>=2, y>=2 ; 期望从 (0,0) 斜向 (4,4) 方向 -> 切角
    fly_px4::ClearanceField f;
    std::vector<std::pair<double, double>> pts;
    for (double v = 2; v <= 12; v += 0.1) { pts.emplace_back(2.0, v); pts.emplace_back(v, 2.0); }
    f.buildFromPoints(pts, 0.2, 0, 0, 14, 0.8);
    fly_px4::VelocityGuardParams p;
    fly_px4::VelocityGuardState s;
    s.px = -1.5; s.py = 0.5;
    s.vdx = 2.0; s.vdy = 0.6; s.v0x = 2.0; s.v0y = 0.6; s.vpx = 2.0; s.vpy = 0.6;
    const auto des = fly_px4::evaluateGuardPath(f, p, s.px, s.py, s.v0x, s.v0y, s.vdx, s.vdy);
    const auto r = fly_px4::guardVelocity(f, p, s);
    const auto e = fly_px4::evaluateGuardPath(f, p, s.px, s.py, s.v0x, s.v0y, r.vx, r.vy);
    check(r.intervened && e.min_all > des.min_all + 0.3 && e.min_all >= p.d_hard,
          "corner: chosen path keeps away from corner (not cutting it)");
}

void standoffEquivalence() {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-12, 12);
    const double offs_arr[12] = {0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180};
    std::array<double, 12> offs;
    for (int k = 0; k < 12; ++k) offs[k] = offs_arr[k];
    fly_px4::StandoffObstacleParams sp;
    int prev_old = -1, prev_new = -1, same = 0, total = 0;
    for (int it = 0; it < 4000; ++it) {
        std::array<fly_px4::StandoffCandidate, 12> c;
        std::array<fly_px4::StandoffObstacleInfo, 12> info;   // 全 +inf: 附近无障碍
        const double peer_x = u(rng), peer_y = u(rng);
        for (int k = 0; k < 12; ++k) {
            const double a = offs[k] * kPi / 180;
            c[k].x = 5 * std::cos(a); c[k].y = 5 * std::sin(a);
            c[k].inside_bounds = (it % 13) != k;
            c[k].peer_clearance = std::hypot(c[k].x - peer_x, c[k].y - peer_y);
        }
        prev_old = fly_px4::chooseStandoffCandidate(c, prev_old, 3.5, 0.5);
        prev_new = fly_px4::chooseObstacleAwareStandoff(c, info, offs, prev_new, 3.5, 0.5, sp);
        same += prev_old == prev_new;
        ++total;
    }
    check(same == total, "no obstacles: standoff choice identical to original (4000 random peer cases)");
}

void standoffWallAndLos() {
    // 行人 (0,0), 楼 x∈[1.5,10] y∈[-10,10] 在行人东侧; 本机在东北 -> 首选在楼里
    std::vector<std::pair<double, double>> pts;
    for (double y = -10; y <= 10; y += 0.1) pts.emplace_back(1.5, y);
    fly_px4::ClearanceField f;
    f.buildFromPoints(pts, 0.2, 0, 0, 14, 0.8);
    const double offs_arr[12] = {0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180};
    std::array<double, 12> offs;
    std::array<fly_px4::StandoffCandidate, 12> c;
    std::array<fly_px4::StandoffObstacleInfo, 12> info;
    fly_px4::StandoffObstacleParams sp;
    const double az0 = 20.0;
    for (int k = 0; k < 12; ++k) {
        offs[k] = offs_arr[k];
        const double a = (az0 + offs[k]) * kPi / 180;
        c[k].x = 5 * std::cos(a); c[k].y = 5 * std::sin(a); c[k].inside_bounds = true;
        info[k] = fly_px4::evaluateStandoffObstacles(f, c[k].x, c[k].y, 0, 0, 1.0, 4.0, sp);
    }
    const int pick = fly_px4::chooseObstacleAwareStandoff(c, info, offs, -1, 3.5, 0.5, sp);
    check(pick > 0 && c[pick].x < 1.5 - 2.4, "standoff behind wall replaced by one on person's side");
    check(info[pick].los_clearance >= sp.los_clear, "chosen standoff has clear line of sight");
    check(info[0].los_clearance < sp.los_clear, "original standoff was occluded by the wall");
}

void bandPointsCollisionOnly() {
    // 顶棚: 行人 (0,0) 在棚下; 棚的投影 x∈[-3.4,3.4] y∈[-9,9], 只有高度带点(檐口)
    std::vector<std::pair<double, double>> lidar, band;
    for (double y = -9; y <= 9; y += 0.1) { band.emplace_back(-3.4, y); band.emplace_back(3.4, y); }
    for (double x = -3.4; x <= 3.4; x += 0.1) { band.emplace_back(x, -9.0); band.emplace_back(x, 9.0); }
    // 立柱 (雷达看得到, 0.6m 粗, 按墙分类), 恰好挡在本机->棚内候选的直线上
    for (double a = 0; a < 2 * kPi; a += 0.3) lidar.emplace_back(0.6 + 0.3 * std::cos(a), 4.0 + 0.3 * std::sin(a));
    fly_px4::ClearanceField f;
    f.buildFromPoints(lidar, 0.2, 0, 0, 14, 0.8, {}, band);   // 檐口 = 上方结构证据
    check(std::fabs(f.clearance(5.4, 0) - 2.0) < 0.25, "band points count for collision clearance");
    check(f.wallClearance(5.4, 0) < 2.3, "band eave is a wall for wall clearance / crossing checks");
    check(f.hasOverhead(), "eave registered as overhead structure");
    check(f.losClearance(5.4, 0) > 4.0, "band eave never occludes the camera line of sight (only the pillar does)");
    check(f.losSegmentClearance(6.0, 0, 0, 0, 0, 0.4) > 2.5, "LOS under the eave stays clear");
    check(f.segmentClearance(6.0, 0, 0, 0, 0, 0, true) < 0.2, "path into the canopy crosses the eave");

    // 站位: 首选在棚里 (直飞要穿过檐口) -> 换到棚外, 且视线不判遮挡
    const double offs_arr[12] = {0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180};
    std::array<double, 12> offs;
    std::array<fly_px4::StandoffCandidate, 12> c;
    std::array<fly_px4::StandoffObstacleInfo, 12> info;
    fly_px4::StandoffObstacleParams sp;
    const double wx = 9.0, wy = 0.5;   // 本机在棚东侧外
    for (int k = 0; k < 12; ++k) {
        offs[k] = offs_arr[k];
        const double a = (90.0 + offs[k]) * kPi / 180;   // 首选 = 正北, 落在棚里
        c[k].x = 5.5 * std::cos(a); c[k].y = 5.5 * std::sin(a); c[k].inside_bounds = true;
        info[k] = fly_px4::evaluateStandoffObstacles(f, c[k].x, c[k].y, 0, 0, wx, wy, sp);
    }
    const int pick = fly_px4::chooseObstacleAwareStandoff(c, info, offs, -1, 3.5, 0.5, sp);
    check(pick >= 0 && std::fabs(c[pick].x) > 3.4 + 1.5, "standoff under the canopy replaced by one outside");
    check(info[0].band_blocked && !info[6].band_blocked, "candidate under canopy is band-blocked, outside one is not");
    for (int k = 0; k < 12; ++k)
        if (std::fabs(c[k].x) < 3.4 && std::fabs(c[k].y) < 9.0 && !info[k].band_blocked)
            check(false, "every candidate under the canopy is band-blocked (even behind a lidar pillar)");

    // 同一扫描高度上记住的楼墙 (只算碰撞, 非上方结构): 不硬排除, 仍可绕行
    fly_px4::ClearanceField remembered;
    remembered.buildFromPoints({}, 0.2, 0, 0, 14, 0.8, band);
    const auto ri = fly_px4::evaluateStandoffObstacles(remembered, c[0].x, c[0].y, 0, 0, wx, wy, sp);
    check(!ri.band_blocked && ri.view_clearance < sp.view_clear,
          "remembered same-height wall: soft (view) penalty only, not hard-excluded");
    check(pick >= 0 && info[pick].los_clearance >= sp.los_clear, "outside standoff keeps clear LOS under eave");

    // PID 速度过滤: 朝檐口 2m/s 冲 -> 提前制动/偏转
    fly_px4::VelocityGuardParams gp;
    fly_px4::VelocityGuardState s;
    s.px = 6.4; s.py = 0.0; s.vdx = -2.0; s.v0x = -2.0; s.vpx = -2.0;
    const auto r = fly_px4::guardVelocity(f, gp, s);
    check(r.intervened && r.vx > -1.0, "PID heading into eave is braked by band points");

    // 无高度带点时视线场与墙面场逐点一致 (原行为不变)
    std::vector<std::pair<double, double>> walls;
    for (double y = -6; y <= 6; y += 0.1) walls.emplace_back(3.0, y);
    walls.emplace_back(-4.0, 0.0);
    fly_px4::ClearanceField g;
    g.buildFromPoints(walls, 0.2, 0, 0, 12, 0.8);
    bool same = true;
    for (double x = -10; x <= 10; x += 0.37)
        for (double y = -10; y <= 10; y += 0.41) {
            const double a = g.losClearance(x, y), b = g.wallClearance(x, y);
            same &= (a == b) || (!std::isfinite(a) && !std::isfinite(b));
        }
    check(same, "no band points: LOS field identical to wall field");
}

// 2026-10-15: 雷达 10s 前扫到过的普通楼墙, 深度点一直重发 -> 旧逻辑 (雷达只记 3s) 把它
// 当屋檐; 有 lidar_seen_sec 后不算上方结构。雷达从没扫到过的檐口仍是上方结构。
void overheadNeedsLidarNeverSeen() {
    const double now = 100.0;
    fly_px4::SurfaceMemory mem(0.2);
    for (double y = -6; y <= 6; y += 0.1) {
        mem.insert(3.0, y, now - 10.0);                                   // 雷达 10s 前扫到的墙
        mem.insert(3.0, y, now, /*occludes=*/false, /*overhead=*/true);   // 深度现在仍在发
    }
    fly_px4::ClearanceField old_field, new_field;
    old_field.build(mem, 0, 0, 14, now, 3.0, 0.8);
    new_field.build(mem, 0, 0, 14, now, 3.0, 0.8, 15.0);
    check(old_field.hasOverhead(), "old: wall lidar saw 10s ago became overhead structure");
    check(!new_field.hasOverhead(), "new: wall lidar saw within 15s is not overhead structure");
    check(std::fabs(new_field.clearance(5.0, 0) - 2.0) < 0.25,
          "new: the remembered wall still counts for collision clearance");
    fly_px4::SurfaceMemory eave(0.2);
    for (double y = -6; y <= 6; y += 0.1) eave.insert(3.0, y, now, false, true);   // 雷达从没扫到
    fly_px4::ClearanceField eave_field;
    eave_field.build(eave, 0, 0, 14, now, 3.0, 0.8, 15.0);
    check(eave_field.hasOverhead(), "new: structure the lidar never saw stays overhead");
    fly_px4::SurfaceMemory stale(0.2);
    for (double y = -6; y <= 6; y += 0.1) {
        stale.insert(3.0, y, now - 20.0);
        stale.insert(3.0, y, now, false, true);
    }
    fly_px4::ClearanceField stale_field;
    stale_field.build(stale, 0, 0, 14, now, 3.0, 0.8, 15.0);
    check(stale_field.hasOverhead(), "new: lidar sighting older than the window does not count");
}

// 4m 通道 (x∈[0,10], 墙 y=±2), 行人在通道里, 本机在人身后
void corridorStandoff() {
    std::vector<std::pair<double, double>> pts;
    for (double x = 0; x <= 10.0; x += 0.1) { pts.emplace_back(x, 2.0); pts.emplace_back(x, -2.0); }
    for (double y = 2.0; y <= 12; y += 0.1) { pts.emplace_back(0.0, y); pts.emplace_back(10.0, y);
                                              pts.emplace_back(0.0, -y); pts.emplace_back(10.0, -y); }
    fly_px4::ClearanceField f;
    f.buildFromPoints(pts, 0.2, 5, 0, 14, 0.8);
    const double tx = 7.5, ty = 0.3, wx = 1.5, wy = 0.0, r = 5.5;
    const double offs_arr[12] = {0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180};
    const double az0 = std::atan2(wy - ty, wx - tx);
    std::array<double, 12> offs;
    for (int k = 0; k < 12; ++k) offs[k] = offs_arr[k];
    const auto ring = [&](const fly_px4::StandoffObstacleParams& sp,
                          std::array<fly_px4::StandoffCandidate, 12>& c,
                          std::array<fly_px4::StandoffObstacleInfo, 12>& info) {
        for (int k = 0; k < 12; ++k) {
            const double a = az0 + offs[k] * kPi / 180;
            c[k].x = tx + r * std::cos(a); c[k].y = ty + r * std::sin(a); c[k].inside_bounds = true;
            info[k] = fly_px4::evaluateStandoffObstacles(f, c[k].x, c[k].y, tx, ty, wx, wy, sp);
        }
        if (sp.corridor && !fly_px4::standoffNearATier(c, info, offs, -1, 3.5, sp))
            for (int k = 0; k < 12; ++k) {
                double rx, ry;
                if (!fly_px4::refineStandoffPoint(f, tx, ty, r, az0 + offs[k] * kPi / 180,
                                                  wx, wy, tx, ty, sp, rx, ry)) continue;
                c[k].x = rx; c[k].y = ry;
                info[k] = fly_px4::evaluateStandoffObstacles(f, rx, ry, tx, ty, wx, wy, sp);
            }
    };
    fly_px4::StandoffObstacleParams v1;
    v1.corridor = false;
    std::array<fly_px4::StandoffCandidate, 12> c;
    std::array<fly_px4::StandoffObstacleInfo, 12> info;
    ring(v1, c, info);
    const int p1 = fly_px4::chooseObstacleAwareStandoff(c, info, offs, -1, 3.5, 0.5, v1);
    check(p1 >= 0 && std::fabs(offs[p1]) >= 120 && c[p1].x > 10.0,
          "corridor: previous rule picks a point beyond the far end (other side of the person)");
    fly_px4::StandoffObstacleParams v2;
    v2.corridor_clear = 1.6;
    ring(v2, c, info);
    check(info[11].crosses_person, "corridor: far-side candidate is flagged as passing the person");
    bool tier_b = false;
    const int p2 = fly_px4::chooseObstacleAwareStandoff(c, info, offs, -1, 3.5, 0.5, v2,
                                                        nullptr, &tier_b);
    check(p2 == 0 && tier_b, "corridor: corridor mode keeps the standoff behind the person");
    check(std::fabs(c[p2].y) < 0.35 && info[p2].wall_clearance >= 1.6,
          "corridor: standoff is moved to the corridor centre line");
    check(std::fabs(std::hypot(c[p2].x - tx, c[p2].y - ty) - r) < 1.01,
          "corridor: refined standoff keeps the follow distance within 1m");
    // 保持: 下一拍仍选 0, 不在走廊档/外面的 A 档之间来回跳
    ring(v2, c, info);
    check(fly_px4::chooseObstacleAwareStandoff(c, info, offs, p2, 3.5, 0.5, v2) == 0,
          "corridor: corridor-tier standoff is kept on the next tick");
    // 2.6m 窄缝: 走廊档也不合格, 降级时不选穿越点
    std::vector<std::pair<double, double>> narrow;
    for (double x = 0; x <= 10.0; x += 0.1) { narrow.emplace_back(x, 1.3); narrow.emplace_back(x, -1.3); }
    fly_px4::ClearanceField fn;
    fn.buildFromPoints(narrow, 0.2, 5, 0, 14, 0.8);
    for (int k = 0; k < 12; ++k) {
        const double a = az0 + offs[k] * kPi / 180;
        c[k].x = tx + r * std::cos(a); c[k].y = ty + r * std::sin(a);
        info[k] = fly_px4::evaluateStandoffObstacles(fn, c[k].x, c[k].y, tx, ty, wx, wy, v2);
    }
    const int pn = fly_px4::chooseObstacleAwareStandoff(c, info, offs, -1, 3.5, 0.5, v2);
    check(pn >= 0 && !info[pn].crosses_person, "narrow gap: never picks a point past the person");
}

// 走廊档下速度过滤以站位自身离墙为期望净空: 沿中线前进不被刹停
void guardCorridorReference() {
    std::vector<std::pair<double, double>> pts;
    for (double x = -2; x <= 20.0; x += 0.1) { pts.emplace_back(x, 2.0); pts.emplace_back(x, -2.0); }
    fly_px4::ClearanceField f;
    f.buildFromPoints(pts, 0.2, 5, 0, 14, 0.8);
    fly_px4::VelocityGuardParams p;
    p.d_hard = 1.0; p.d_safe = 2.5;
    fly_px4::VelocityGuardState s;
    s.px = 2.0; s.vdx = 2.0; s.v0x = 2.0; s.vpx = 2.0;
    const auto def = fly_px4::guardVelocity(f, p, s);
    s.safe_ref = 1.7;
    const auto cor = fly_px4::guardVelocity(f, p, s);
    check(def.intervened, "corridor guard: default reference intervenes on the centre line");
    check(!cor.intervened && cor.vx == 2.0, "corridor guard: corridor reference passes PID through");
    s.py = 0.8; s.vdy = 1.0; s.v0y = 1.0; s.vpy = 1.0;     // 斜着冲向墙仍然拦
    const auto side = fly_px4::guardVelocity(f, p, s);
    check(side.intervened && side.vy < 0.8, "corridor guard: heading into a wall is still braked");
}

// 闭环对比
struct Scenario {
    const char* name;
    std::vector<Poly> world;
    Walker walker;
    double sx, sy, t_max;
};

void closedLoop() {
    std::vector<Scenario> sc;
    {   // 0 空旷马路 (1m/s, 90° 转向)
        Walker w; w.wp = {{0, 0}, {20, 0}, {20, 15}}; w.speed = 1.0;
        sc.push_back({"空旷 1m/s 转弯", {}, w, -5, 0, 60});
        w.speed = 2.0;
        sc.push_back({"空旷 2m/s 转弯", {}, w, -5, 0, 40});
    }
    {   // 2 马路边贴楼走: 楼南墙 y=3, 行人在 y=1.5; 本机从楼西北侧接近
        Walker w; w.wp = {{-4, 1.5}, {22, 1.5}}; w.speed = 1.0;
        sc.push_back({"贴墙行走 1m/s", {rect(0, 3, 15, 13)}, w, -7, 5.5, 40});
        w.speed = 2.0;
        sc.push_back({"贴墙行走 2m/s", {rect(0, 3, 15, 13)}, w, -7, 5.5, 25});
    }
    {   // 4 绕楼角: 沿西墙北上, 绕西北角沿北墙东行
        Walker w; w.wp = {{-2, -8}, {-2, 14}, {20, 14}}; w.speed = 1.0;
        sc.push_back({"绕楼角 1m/s", {rect(0, 0, 12, 12)}, w, -2, -13, 60});
        w.speed = 2.0;
        sc.push_back({"绕楼角 2m/s", {rect(0, 0, 12, 12)}, w, -2, -13, 35});
    }
    {   // 6 两楼夹道 (宽 6m) 穿行
        Walker w; w.wp = {{-8, 0}, {28, 0}}; w.speed = 1.0;
        sc.push_back({"6m 夹道 1m/s", {rect(0, 3, 20, 13), rect(0, -13, 20, -3)}, w, -13, 0, 45});
    }
    {   // 7 楼角 + 路灯杆, 行人沿楼角拐进院子
        Walker w; w.wp = {{-10, -2}, {6, -2}, {6, 10}}; w.speed = 1.2;
        sc.push_back({"拐进院子 1.2m/s",
                      {rect(-12, 0, 3.5, 12), rect(9, 0, 20, 12), disc(2, -5, 0.15)}, w, -15, -3, 45});
    }
    std::printf("\n闭环对比 (old=原方向采样+膨胀带, new=净空场站位+速度过滤):\n");
    double us_new = 0;
    for (std::size_t i = 0; i < sc.size(); ++i) {
        const auto& s = sc[i];
        std::vector<std::pair<double, double>> c_old, c_new;
        const Metrics mo = runLoop(s.world, s.walker, s.sx, s.sy, Mode::OLD, s.t_max, &c_old);
        double us = 0;
        const Metrics mn = runLoop(s.world, s.walker, s.sx, s.sy, Mode::V2, s.t_max, &c_new, &us);
        const Metrics m1 = runLoop(s.world, s.walker, s.sx, s.sy, Mode::V1, s.t_max);
        us_new = std::max(us_new, us);
        printMetrics(s.name, "old", mo);
        printMetrics(s.name, "v1", m1);
        printMetrics(s.name, "new", mn);
        if (s.world.empty()) {
            double max_diff = 0;
            for (std::size_t k = 0; k < std::min(c_old.size(), c_new.size()); ++k)
                max_diff = std::max(max_diff, std::hypot(c_old[k].first - c_new[k].first,
                                                         c_old[k].second - c_new[k].second));
            check(c_old.size() == c_new.size() && max_diff == 0.0,
                  std::string(s.name) + ": PID commands bit-identical to original");
            continue;
        }
        check(mn.min_clear >= LoopCfg().d_hard + 0.5,
              std::string(s.name) + ": new keeps >= d_hard+0.5 from walls");
        // 与上一版 (现场在用) 比; 对原方向采样的 0.1m 要求在 d_hard=1.0 下上一版也过不了
        check(mn.min_clear >= m1.min_clear - 0.15,
              std::string(s.name) + ": wall distance not worse than previous version");
        check(mn.t_lt15 <= mo.t_lt15 + 0.2, std::string(s.name) + ": less time within 1.5m of walls");
        check(mn.t_occ <= mo.t_occ + 0.3, std::string(s.name) + ": occlusion time not worse");
        check(mn.handoffs <= mo.handoffs, std::string(s.name) + ": no more EGO handoffs");
        check(mn.d_err <= mo.d_err + 0.6, std::string(s.name) + ": follow distance stays close to radius");
        check(mn.passes == 0, std::string(s.name) + ": never swings around to the far side");
    }
    std::printf("  新方法每拍计算 (含 10Hz 建场) 最大平均 %.0f us\n", us_new);
    check(us_new < 3000.0, "new avoidance per-tick cost well under the 20ms control period");
}
// 走廊 (两栋房子之间) 闭环: 上一版 vs 走廊模式
// 比赛图里房子间通道 3.9~4.4m (base1/2/3: 1_67/2_71、1_67/3_68、1_66/2_71)。
// 上一版要求站位离墙 2.8m, 通道里没有合格点, 只能去通道外 —— 常是行人另一侧,
// 飞机加速从人身边穿过去再掉头, 人进相机下沿盲区就丢了。
void corridorLoop() {
    std::vector<Scenario> sc;
    const auto houses = [](double gap, double len) {
        const double h = gap / 2;
        return std::vector<Poly>{rect(0, h, len, h + 10), rect(0, -h - 10, len, -h)};
    };
    {
        Walker w; w.wp = {{-8, 0.3}, {26, 0.3}}; w.speed = 1.0;
        sc.push_back({"4.0m 走廊直穿 1m/s", houses(4.0, 14), w, -13.5, 0.0, 45});
        w.speed = 2.0;
        sc.push_back({"4.0m 走廊直穿 2m/s", houses(4.0, 14), w, -13.5, 0.0, 25});
    }
    {   // 贴一侧墙走 (离墙 0.5m)
        Walker w; w.wp = {{-8, 1.5}, {26, 1.5}}; w.speed = 1.0;
        sc.push_back({"4.4m 走廊贴墙 1m/s", houses(4.4, 14), w, -13.5, -1.0, 45});
    }
    {   // 本机从斜后方进入
        Walker w; w.wp = {{-6, 0}, {26, 0}}; w.speed = 1.0;
        sc.push_back({"3.9m 走廊斜入 1m/s", houses(3.9, 14), w, -9.5, -5.5, 45});
    }
    {   // 出口后 90° 拐弯
        Walker w; w.wp = {{-8, 0}, {18, 0}, {18, 14}}; w.speed = 1.0;
        sc.push_back({"4.1m 走廊出口拐弯", houses(4.1, 14), w, -13.5, 0.0, 50});
    }
    {   // 走到通道中间停 4s 再原路返回 (迎着飞机)
        Walker w; w.wp = {{-8, 0}, {7, 0}, {7.01, 0}, {-12, 0}}; w.speed = 1.0;
        sc.push_back({"4.0m 走廊折返", houses(4.0, 14), w, -13.5, 0.0, 50});
    }
    {   // 2.6m 窄缝: 飞机不该钻进去, 更不该从人身边穿过去
        Walker w; w.wp = {{-8, 0}, {26, 0}}; w.speed = 1.0;
        sc.push_back({"2.6m 窄缝 1m/s", houses(2.6, 12), w, -13.5, 0.0, 45});
    }
    std::printf("\n走廊闭环 (v1=上一版, v2=走廊模式):\n");
    int v1_passes = 0;
    for (const auto& s : sc) {
        const Metrics m1 = runLoop(s.world, s.walker, s.sx, s.sy, Mode::V1, s.t_max);
        const Metrics m2 = runLoop(s.world, s.walker, s.sx, s.sy, Mode::V2, s.t_max);
        printMetrics(s.name, "v1", m1);
        printMetrics(s.name, "v2", m2);
        v1_passes += m1.passes;
        const std::string n(s.name);
        check(m2.passes == 0, n + ": v2 never swings around the person");
        check(m2.min_clear >= LoopCfg().d_hard + 0.2, n + ": v2 keeps >= d_hard+0.2 from walls");
        check(m2.t_blind <= m1.t_blind + 0.3, n + ": v2 blind-zone time not worse");
        check(m2.t_occ <= m1.t_occ + 0.5, n + ": v2 occlusion not worse");
        check(m2.handoffs <= m1.handoffs, n + ": v2 no more EGO handoffs");
        check(m2.jumps <= m1.jumps, n + ": v2 no more standoff jumps");
    }
    std::printf("  v1 合计绕到人另一侧 %d 次\n", v1_passes);
    check(v1_passes > 0, "previous version reproduces the corridor swing-around (test is meaningful)");
}

// 真实地图随机压力测试 (可选)
// 用法: test_track_obstacle_avoidance <polys.txt> [N]
// polys.txt 每行一个多边形 "x0 y0 x1 y1 ..." (patrol_world_geometry 导出的
// 跟踪高度碰撞轮廓)。行人在楼边随机走 (不穿楼), 本机从 5~8m 外起跟。
// 注意: 原方法"交回 EGO"在这里按悬停算 (不模拟 EGO), 所以它的贴墙指标偏乐观。
int stress(const char* path, int n) {
    std::vector<Poly> world;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        Poly p; double x, y;
        while (ss >> x >> y) p.v.emplace_back(x, y);
        if (p.v.size() >= 3) world.push_back(p);
    }
    std::printf("stress: %zu obstacles from %s, %d runs\n", world.size(), path, n);
    std::mt19937 rng(20261008);
    std::uniform_real_distribution<double> U(0, 1);
    const auto pathFree = [&](double ax, double ay, double bx, double by) {
        const double len = std::hypot(bx - ax, by - ay);
        for (double s = 0; s <= len; s += 0.25) {
            const double x = ax + (bx - ax) * s / len, y = ay + (by - ay) * s / len;
            if (trueClearance(world, x, y) < 0.8) return false;
        }
        return true;
    };
    struct Agg { int runs = 0, coll = 0; double lt10 = 0, lt15 = 0, occ = 0, t = 0, derr = 0,
                 minc = 1e9, thand = 0, lt20 = 0, blind = 0, corr = 0; int hand = 0, passes = 0,
                 jumps = 0; } ao, a1, an;
    int made = 0, tries = 0;
    while (made < n && tries < n * 200) {
        ++tries;
        // 选一栋楼, 在其外 1~4m 处起步
        const auto& b = world[rng() % world.size()];
        double area = 0;
        for (std::size_t i = 0, j = b.v.size() - 1; i < b.v.size(); j = i++)
            area += b.v[j].first * b.v[i].second - b.v[i].first * b.v[j].second;
        if (std::fabs(area) < 4.0) continue;   // 跳过路灯杆
        const auto& v0 = b.v[rng() % b.v.size()];
        const double a0 = U(rng) * 2 * kPi, r0 = 1.5 + 3.0 * U(rng);
        const double px = v0.first + r0 * std::cos(a0), py = v0.second + r0 * std::sin(a0);
        if (trueClearance(world, px, py) < 1.0) continue;
        Walker w; w.speed = U(rng) < 0.5 ? 1.0 : 2.0;
        w.wp.emplace_back(px, py);
        double cx = px, cy = py, heading = U(rng) * 2 * kPi, total = 0;
        for (int k = 0; k < 40 && total < 22.0 * w.speed + 10; ++k) {
            const double h = heading + (U(rng) - 0.5) * kPi;
            const double l = 4 + 8 * U(rng);
            const double nx = cx + l * std::cos(h), ny = cy + l * std::sin(h);
            if (!pathFree(cx, cy, nx, ny)) continue;
            w.wp.emplace_back(nx, ny); cx = nx; cy = ny; heading = h; total += l;
        }
        if (total < 20) continue;
        // 本机起点: 离人 5~8m, 离墙 >=3m, 能看到人
        bool ok = false; double sx = 0, sy = 0;
        for (int k = 0; k < 50 && !ok; ++k) {
            const double a = U(rng) * 2 * kPi, r = 5 + 3 * U(rng);
            sx = px + r * std::cos(a); sy = py + r * std::sin(a);
            ok = trueClearance(world, sx, sy) >= 3.0 && !occluded(world, sx, sy, px, py);
        }
        if (!ok) continue;
        ++made;
        const double tmax = total / w.speed + 5;
        const Metrics mo = runLoop(world, w, sx, sy, Mode::OLD, tmax);
        const Metrics m1 = runLoop(world, w, sx, sy, Mode::V1, tmax);
        const Metrics mn = runLoop(world, w, sx, sy, Mode::V2, tmax);
        for (auto pr : {std::make_pair(&ao, &mo), std::make_pair(&a1, &m1), std::make_pair(&an, &mn)}) {
            Agg& a = *pr.first; const Metrics& m = *pr.second;
            ++a.runs; a.coll += m.min_clear < 0.6; a.lt10 += m.t_lt10; a.lt15 += m.t_lt15;
            a.occ += m.t_occ; a.t += m.t; a.derr += m.d_err * m.t; a.hand += m.handoffs;
            a.thand += m.t_handoff; a.lt20 += m.t_lt20;
            a.passes += m.passes; a.jumps += m.jumps; a.blind += m.t_blind; a.corr += m.t_corridor;
            a.minc = std::min(a.minc, m.min_clear);
        }
    }
    for (auto pr : {std::make_pair("old", &ao), std::make_pair("v1", &a1), std::make_pair("v2", &an)}) {
        const Agg& a = *pr.second;
        std::printf("  %s 绕到人另一侧=%d次  站位跳变=%d次  盲区=%.1fs  走廊档=%.0fs\n",
                    pr.first, a.passes, a.jumps, a.blind, a.corr);
        std::printf("  %s runs=%d 碰撞(<0.6m)=%d 最小墙距=%.2fm  <1.0m=%.1fs  <1.5m=%.1fs  <2.0m=%.1fs  "
                    "遮挡=%.1f%%  距离误差=%.2fm  交回EGO=%d次/%.0fs  总时长=%.0fs\n",
                    pr.first, a.runs, a.coll, a.minc, a.lt10, a.lt15, a.lt20,
                    100.0 * a.occ / std::max(1e-9, a.t), a.derr / std::max(1e-9, a.t), a.hand,
                    a.thand, a.t);
    }
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc >= 2) return stress(argv[1], argc >= 3 ? std::atoi(argv[2]) : 60);
    fieldBasics();
    guardPassThrough();
    guardWall();
    guardCorner();
    standoffEquivalence();
    standoffWallAndLos();
    bandPointsCollisionOnly();
    overheadNeedsLidarNeverSeen();
    corridorStandoff();
    guardCorridorReference();
    closedLoop();
    corridorLoop();
    if (failures) { std::cerr << failures << " track obstacle avoidance checks failed\n"; return 1; }
    std::cout << "track obstacle avoidance checks passed\n";
    return 0;
}
