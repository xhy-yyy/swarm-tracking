#pragma once
// 跟踪避障：以原始障碍记忆计算墙面、视线和机体高度带净空。
// 站位检查包含边界与友机间距，速度守卫检查预测路径；上方结构另行约束站位。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "fly_px4/tracking_navigation_policy.hpp"

namespace fly_px4 {

// 原始表面记忆
class SurfaceMemory {
public:
    explicit SurfaceMemory(double resolution = 0.2)
        : res_(std::isfinite(resolution) && resolution >= 0.05 ? resolution : 0.2) {}

    double resolution() const { return res_; }
    std::size_t size() const { return cells_.size(); }
    void clear() { cells_.clear(); }

    // occludes=false: 只算碰撞的点 (机体高度带), 不参与视线遮挡判定
    // overhead=true : 上方结构证据 (深度相机/别的高度扫到的), 站位不能在它后面
    void insert(double x, double y, double stamp, bool occludes = true, bool overhead = false) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(stamp)) return;
        Cell& c = cells_[key(cellOf(x), cellOf(y))];
        c.t = std::max(c.t, stamp);
        if (occludes) c.t_occ = std::max(c.t_occ, stamp);
        if (overhead) c.t_over = std::max(c.t_over, stamp);
    }

    // 删掉过期(> memory_sec)或离本机太远的格 (keep_radius=inf 只按时间);
    // 时间回跳(仿真重置)时全清。
    void prune(double now, double memory_sec, double cx, double cy, double keep_radius) {
        const int icx = cellOf(cx), icy = cellOf(cy);
        const double r_cells = keep_radius / res_;
        const double r2 = r_cells * r_cells;
        for (auto it = cells_.begin(); it != cells_.end();) {
            const int ix = static_cast<int32_t>(it->first >> 32);
            const int iy = static_cast<int32_t>(it->first & 0xffffffffu);
            const double dx = ix - icx, dy = iy - icy;
            const double t = it->second.t;
            const bool expired = t < now - memory_sec || t > now + 1.0;
            if (expired || dx * dx + dy * dy > r2) it = cells_.erase(it);
            else ++it;
        }
    }

    // f(ix, iy, 最后看到的时刻, 最后被遮挡性来源看到的时刻, 最后作为上方结构
    // 证据看到的时刻; 从未则 -inf)
    template <class F>
    void forEach(F&& f) const {
        for (const auto& c : cells_)
            f(static_cast<int32_t>(c.first >> 32), static_cast<int32_t>(c.first & 0xffffffffu),
              c.second.t, c.second.t_occ, c.second.t_over);
    }

    int cellOf(double v) const { return static_cast<int>(std::floor(v / res_)); }

private:
    struct Cell {
        double t = -std::numeric_limits<double>::infinity();
        double t_occ = -std::numeric_limits<double>::infinity();
        double t_over = -std::numeric_limits<double>::infinity();
    };
    static uint64_t key(int ix, int iy) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
               static_cast<uint32_t>(iy);
    }
    double res_;
    std::unordered_map<uint64_t, Cell> cells_;
};

// 局部净空场
class ClearanceField {
public:
    static constexpr double kInf = std::numeric_limits<double>::infinity();

    // 碰撞记忆与较长的雷达历史分开，避免把普通墙面误判为上方结构。
    void build(const SurfaceMemory& mem, double cx, double cy, double half,
               double now, double memory_sec, double thin_extent,
               double lidar_seen_sec = 0.0) {
        res_ = mem.resolution();
        n_ = std::max(4, static_cast<int>(std::ceil(2.0 * half / res_)));
        ox_ = mem.cellOf(cx - half);
        oy_ = mem.cellOf(cy - half);
        const std::size_t total = static_cast<std::size_t>(n_) * n_;
        occ_.assign(total, 0);
        std::vector<uint8_t> occ_los(total, 0), over(total, 0);
        bool collision_only = false, any_over = false;
        const auto fresh = [&](double t) { return t >= now - memory_sec && t <= now + 1.0; };
        const double seen_sec = std::max(memory_sec, lidar_seen_sec);
        const auto lidar_seen = [&](double t) { return t >= now - seen_sec && t <= now + 1.0; };
        mem.forEach([&](int ix, int iy, double t, double t_occ, double t_over) {
            if (!fresh(t)) return;
            const int i = ix - ox_, j = iy - oy_;
            if (i < 0 || j < 0 || i >= n_ || j >= n_) return;
            occ_[idx(i, j)] = 1;
            if (fresh(t_occ)) occ_los[idx(i, j)] = 1;
            else collision_only = true;
            // 雷达当前也扫到的格不算"上方结构" (雷达自己就能挡)
            if (fresh(t_over) && !lidar_seen(t_occ)) { over[idx(i, j)] = 1; any_over = true; }
        });
        classify(occ_, thin_extent);
        all_.assign(total, 0.0);
        wall_.assign(total, 0.0);
        any_all_ = any_wall_ = false;
        for (std::size_t k = 0; k < total; ++k) {
            all_[k] = occ_[k] ? 0.0 : kBig;
            wall_[k] = occ_[k] == 2 ? 0.0 : kBig;
            any_all_ |= occ_[k] != 0;
            any_wall_ |= occ_[k] == 2;
        }
        if (any_all_) edt2d(all_);
        if (any_wall_) edt2d(wall_);
        // 视线场只认雷达墙面; 没有只算碰撞的点时与 wall 相同, 不重复计算
        separate_los_ = collision_only;
        any_los_ = false;
        if (separate_los_) {
            classify(occ_los, thin_extent);
            los_.assign(total, 0.0);
            for (std::size_t k = 0; k < total; ++k) {
                los_[k] = occ_los[k] == 2 ? 0.0 : kBig;
                any_los_ |= occ_los[k] == 2;
            }
            if (any_los_) edt2d(los_);
        }
        // 上方结构墙面: 合并分类后属于大块表面、且只有上方结构证据的格
        any_band_ = false;
        if (any_over) {
            band_.assign(total, 0.0);
            for (std::size_t k = 0; k < total; ++k) {
                band_[k] = (over[k] && occ_[k] == 2) ? 0.0 : kBig;
                any_band_ |= band_[k] == 0.0;
            }
            if (any_band_) edt2d(band_);
        }
        valid_ = true;
    }

    // 便于单测: 直接用点集建场; collision_only = 只算碰撞、不挡视线的点;
    // overhead = 同时是上方结构证据的只算碰撞点
    void buildFromPoints(const std::vector<std::pair<double, double>>& pts, double res,
                         double cx, double cy, double half, double thin_extent,
                         const std::vector<std::pair<double, double>>& collision_only = {},
                         const std::vector<std::pair<double, double>>& overhead = {}) {
        SurfaceMemory mem(res);
        for (const auto& p : pts) mem.insert(p.first, p.second, 0.0);
        for (const auto& p : collision_only) mem.insert(p.first, p.second, 0.0, false);
        for (const auto& p : overhead) mem.insert(p.first, p.second, 0.0, false, true);
        build(mem, cx, cy, half, 0.0, 1.0, thin_extent);
    }

    bool valid() const { return valid_; }
    void invalidate() { valid_ = false; }
    // 场内有雷达当前看不到的上方结构墙 (屋檐/顶棚边)
    bool hasOverhead() const { return valid_ && any_band_; }

    // 到任意表面 / 大块表面的距离 (m)。窗口外或窗口内没有表面 = +inf (未知不拦截)。
    double clearance(double x, double y) const { return lookup(all_, any_all_, x, y); }
    double wallClearance(double x, double y) const { return lookup(wall_, any_wall_, x, y); }
    // 到遮挡相机视线的雷达墙面的距离
    double losClearance(double x, double y) const {
        return separate_los_ ? lookup(los_, any_los_, x, y) : wallClearance(x, y);
    }
    // 贴墙代价用的等效净空: 细杆按 thin_bonus 打折 (同样 2.5m 期望下杆只要 1.8m)
    double effectiveClearance(double x, double y, double thin_bonus) const {
        return std::min(wallClearance(x, y), clearance(x, y) + thin_bonus);
    }

    // 线段 (a->b) 上的最小净空, 两端可各跳过一段 (行人贴墙走时末端本就靠墙)
    double segmentClearance(double ax, double ay, double bx, double by,
                            double skip_a, double skip_b, bool wall_only) const {
        return segmentMin(ax, ay, bx, by, skip_a, skip_b, [&](double x, double y) {
            return wall_only ? wallClearance(x, y) : clearance(x, y);
        });
    }
    // 相机视线线段离遮挡墙面的最小距离 (高度带点不算遮挡)
    double losSegmentClearance(double ax, double ay, double bx, double by,
                               double skip_a, double skip_b) const {
        return segmentMin(ax, ay, bx, by, skip_a, skip_b,
                          [&](double x, double y) { return losClearance(x, y); });
    }
    // 线段离上方结构墙(屋檐/顶棚边)的最小距离; ~0 = 中间隔着顶棚边
    double bandSegmentClearance(double ax, double ay, double bx, double by) const {
        return segmentMin(ax, ay, bx, by, 0.0, 0.0,
                          [&](double x, double y) { return lookup(band_, any_band_, x, y); });
    }

private:
    static constexpr double kBig = 1e12;

    std::size_t idx(int i, int j) const { return static_cast<std::size_t>(j) * n_ + i; }

    template <class F>
    double segmentMin(double ax, double ay, double bx, double by,
                      double skip_a, double skip_b, F&& clearance_at) const {
        if (!valid_) return kInf;
        const double len = std::hypot(bx - ax, by - ay);
        const double s0 = std::max(0.0, skip_a), s1 = len - std::max(0.0, skip_b);
        if (s1 <= s0 || len < 1e-6) return kInf;
        const double ux = (bx - ax) / len, uy = (by - ay) / len;
        const double step = 0.5 * res_;
        double best = kInf;
        for (double s = s0;; s += step) {
            const double ss = std::min(s, s1);
            best = std::min(best, clearance_at(ax + ux * ss, ay + uy * ss));
            if (ss >= s1) break;
        }
        return best;
    }

    double lookup(const std::vector<double>& g, bool any, double x, double y) const {
        if (!valid_ || !any || !std::isfinite(x) || !std::isfinite(y)) return kInf;
        const int i = static_cast<int>(std::floor(x / res_)) - ox_;
        const int j = static_cast<int>(std::floor(y / res_)) - oy_;
        if (i < 0 || j < 0 || i >= n_ || j >= n_) return kInf;
        const double d2 = g[idx(i, j)];
        if (d2 >= 0.5 * kBig) return kInf;
        // 格心到格心距离 -> 偏保守减半格
        return std::max(0.0, std::sqrt(d2) * res_ - 0.5 * res_);
    }

    // 连通块(5x5 邻域, 容忍雷达点间 1 格空隙)外接框够大的标 2(墙), 否则保持 1(细杆)
    void classify(std::vector<uint8_t>& occ, double thin_extent) const {
        std::vector<int> stack, members;
        std::vector<uint8_t> seen(occ.size(), 0);
        for (int j0 = 0; j0 < n_; ++j0) {
            for (int i0 = 0; i0 < n_; ++i0) {
                const std::size_t k0 = idx(i0, j0);
                if (!occ[k0] || seen[k0]) continue;
                int imin = i0, imax = i0, jmin = j0, jmax = j0;
                stack.assign(1, static_cast<int>(k0));
                members.clear();
                seen[k0] = 1;
                while (!stack.empty()) {
                    const int k = stack.back();
                    stack.pop_back();
                    members.push_back(k);
                    const int i = k % n_, j = k / n_;
                    imin = std::min(imin, i); imax = std::max(imax, i);
                    jmin = std::min(jmin, j); jmax = std::max(jmax, j);
                    for (int dj = -2; dj <= 2; ++dj) {
                        for (int di = -2; di <= 2; ++di) {
                            const int ii = i + di, jj = j + dj;
                            if (ii < 0 || jj < 0 || ii >= n_ || jj >= n_) continue;
                            const std::size_t kk = idx(ii, jj);
                            if (occ[kk] && !seen[kk]) {
                                seen[kk] = 1;
                                stack.push_back(static_cast<int>(kk));
                            }
                        }
                    }
                }
                const double extent = std::hypot(imax - imin + 1.0, jmax - jmin + 1.0) * res_;
                if (extent >= thin_extent)
                    for (int k : members) occ[k] = 2;
            }
        }
    }

    // Felzenszwalb-Huttenlocher 一维平方距离变换
    static void edt1d(const double* f, int n, double* d, int* v, double* z) {
        int k = 0;
        v[0] = 0;
        z[0] = -1e30;
        z[1] = 1e30;
        for (int q = 1; q < n; ++q) {
            double s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) /
                       (2.0 * (q - v[k]));
            while (s <= z[k]) {
                --k;
                s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) /
                    (2.0 * (q - v[k]));
            }
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = 1e30;
        }
        k = 0;
        for (int q = 0; q < n; ++q) {
            while (z[k + 1] < q) ++k;
            const double dq = q - v[k];
            d[q] = dq * dq + f[v[k]];
        }
    }

    void edt2d(std::vector<double>& g) const {
        std::vector<double> f(n_), d(n_), z(n_ + 1);
        std::vector<int> v(n_);
        for (int j = 0; j < n_; ++j) {
            for (int i = 0; i < n_; ++i) f[i] = g[idx(i, j)];
            edt1d(f.data(), n_, d.data(), v.data(), z.data());
            for (int i = 0; i < n_; ++i) g[idx(i, j)] = d[i];
        }
        for (int i = 0; i < n_; ++i) {
            for (int j = 0; j < n_; ++j) f[j] = g[idx(i, j)];
            edt1d(f.data(), n_, d.data(), v.data(), z.data());
            for (int j = 0; j < n_; ++j) g[idx(i, j)] = d[j];
        }
    }

    bool valid_ = false;
    bool any_all_ = false, any_wall_ = false, any_los_ = false, any_band_ = false;
    bool separate_los_ = false;
    double res_ = 0.2;
    int n_ = 0, ox_ = 0, oy_ = 0;
    std::vector<uint8_t> occ_;
    std::vector<double> all_, wall_, los_, band_;
};
// 站位候选: 墙面净空 + 视线 + 可达
struct StandoffObstacleInfo {
    double wall_clearance = ClearanceField::kInf;  // 站位点到墙/楼
    double any_clearance = ClearanceField::kInf;   // 站位点到任意表面(含细杆)
    double los_clearance = ClearanceField::kInf;   // 站位->行人视线线段到雷达墙面(高度带点不遮挡)
    double path_clearance = ClearanceField::kInf;  // 本机->站位直线到任意表面
    // 本机->站位直线到墙: ~0 = 中间隔着墙。雷达只扫到朝向本机的墙面, 楼内部
    // 在净空场里看起来是"空地", 站位点落进楼里只能靠这条识别出来。
    double view_clearance = ClearanceField::kInf;
    // 上方结构禁止棚下站位；普通墙面不作为同类硬排除。
    bool band_blocked = false;
    // 本机直飞到站位要从行人身边经过 (人进相机下沿盲区, 机头还得掉头 180°)
    bool crosses_person = false;
    // 走廊档的视线余量要求: 行人贴墙走时视线末端必然靠墙, 按人离墙距离放宽
    double los_required = 0.0;
};

struct StandoffObstacleParams {
    double wall_clear = 2.8;   // 站位点离墙期望净空 (> 速度过滤 d_safe, 悬停时不触发过滤)
    double wall_min = 2.1;     // 降级(没有合格候选)时也不选更近的
    double thin_clear = 2.0;   // 站位点离细杆
    double los_clear = 0.35;   // 视线离墙 (穿墙时 ~0)
    double los_skip = 0.4;     // 视线末端跳过 (行人贴墙走/定位误差)
    double path_clear = 1.2;   // 本机能直线飞到 (软约束, 选边用)
    double view_clear = 0.25;  // 本机->站位不穿墙 (硬约束: 防站位落在楼里/楼背面)
    double hysteresis = 0.3;   // 已选点放宽 / 回首选加严
    // 走廊档放宽期望净空，但仍遵守硬净空。
    bool corridor = true;
    double corridor_clear = 1.6;    // 走廊档: 站位离墙/离任意表面的下限
    double corridor_cliff = 1.3;    // 降级打分的离墙悬崖 (走廊模式下取代 wall_min)
    double cross_radius = 3.5;      // 本机->站位直线离行人比这近 = 要从人身边穿过去
    double cross_path_clear = 2.5;  // 穿到行人另一侧只允许在开阔处 (直飞净空)
    double refine_deg = 25.0;       // 站位微调: 沿以人为心的圆弧 ±refine_deg
    double refine_step_deg = 5.0;
    double refine_dr = 1.0;         // 半径 ±refine_dr (0.5m 一档)
    double min_radius = 4.0;        // 微调后离人不小于 (相机下沿盲区)
};

inline StandoffObstacleInfo evaluateStandoffObstacles(const ClearanceField& f,
        double cx, double cy, double tx, double ty, double wx, double wy,
        const StandoffObstacleParams& p) {
    StandoffObstacleInfo o;
    if (!f.valid()) return o;
    o.wall_clearance = f.wallClearance(cx, cy);
    o.any_clearance = f.clearance(cx, cy);
    o.los_clearance = f.losSegmentClearance(cx, cy, tx, ty, 0.0, p.los_skip);
    o.path_clearance = f.segmentClearance(wx, wy, cx, cy, 0.0, 0.0, false);
    o.view_clearance = f.segmentClearance(wx, wy, cx, cy, 0.0, 0.0, true);
    o.band_blocked = f.hasOverhead() && f.bandSegmentClearance(wx, wy, cx, cy) < p.view_clear;
    o.los_required = p.los_clear;
    if (p.corridor) {
        // 视线要求按行人自身离墙距离放宽 (穿墙的视线余量 ~0, 仍然过不了 0.08)
        const double person_los = f.losClearance(tx, ty);
        o.los_required = std::min(p.los_clear, std::max(0.08, 0.5 * person_los));
        // 穿越判据: 直线离行人最近距离 < cross_radius, 且比本机现在离人还近 0.5m 以上
        // (人冲近时沿同一方位外推的点不算穿越)
        const double vx = cx - wx, vy = cy - wy, l2 = vx * vx + vy * vy;
        double t = l2 > 1e-9 ? ((tx - wx) * vx + (ty - wy) * vy) / l2 : 0.0;
        t = std::min(1.0, std::max(0.0, t));
        const double dseg = std::hypot(tx - wx - t * vx, ty - wy - t * vy);
        const double d_now = std::hypot(tx - wx, ty - wy);
        o.crosses_person = dseg < std::min(p.cross_radius, d_now - 0.5);
    }
    return o;
}

// 常规候选失败后微调角度和半径以靠近通道中线，仍检查视线与上方结构；移动后返回 true。
inline bool refineStandoffPoint(const ClearanceField& f, double ox, double oy, double r,
        double az, double wx, double wy, double tx, double ty,
        const StandoffObstacleParams& p, double& x, double& y) {
    x = ox + r * std::cos(az);
    y = oy + r * std::sin(az);
    if (!p.corridor || !f.valid()) return false;
    if (f.wallClearance(x, y) >= p.wall_clear && f.clearance(x, y) >= p.thin_clear) return false;
    // 挪的目标比合格线多留 hysteresis, 别停在 A 档边缘 (边缘上一抖就换档)
    const double wall_goal = p.wall_clear + p.hysteresis, thin_goal = p.thin_clear + p.hysteresis;
    const auto quality = [&](double px, double py) {
        return std::min(1.0, std::min(f.wallClearance(px, py) / wall_goal,
                                      f.clearance(px, py) / thin_goal));
    };
    const double q0 = quality(x, y);
    struct Point { double cost, q, x, y; };
    std::vector<Point> grid;
    const int na = std::max(0, static_cast<int>(p.refine_deg / p.refine_step_deg + 1e-6));
    const double r_lo = std::max(p.min_radius, r - p.refine_dr);
    for (int i = -na; i <= na; ++i) {
        const double da = i * p.refine_step_deg;
        for (int k = -2; k <= 2; ++k) {
            const double rr = r + 0.5 * k * p.refine_dr;
            if (rr < r_lo - 1e-9) continue;
            const double a = az + da * M_PI / 180.0;
            const double px = ox + rr * std::cos(a), py = oy + rr * std::sin(a);
            const double q = quality(px, py);
            grid.push_back({-q + 0.004 * std::fabs(da) + 0.15 * std::fabs(rr - r), q, px, py});
        }
    }
    std::sort(grid.begin(), grid.end(),
              [](const Point& a, const Point& b) { return a.cost < b.cost; });
    const double person_los = f.losClearance(tx, ty);
    const double los_relaxed = std::min(p.los_clear, std::max(0.08, 0.5 * person_los));
    for (const double los_req : {p.los_clear, los_relaxed}) {
        int checked = 0;
        for (const auto& g : grid) {
            if (g.q <= q0 + 0.02 || ++checked > 20) break;   // 不比名义点好就别挪
            if (f.segmentClearance(wx, wy, g.x, g.y, 0.0, 0.0, true) < p.view_clear) continue;
            if (f.hasOverhead() && f.bandSegmentClearance(wx, wy, g.x, g.y) < p.view_clear)
                continue;
            if (f.losSegmentClearance(g.x, g.y, tx, ty, 0.0, p.los_skip) < los_req) continue;
            x = g.x;
            y = g.y;
            return true;
        }
        if (los_relaxed >= p.los_clear) break;
    }
    return false;
}

// 首选附近没有常规候选时才尝试走廊和中线细化。
template <std::size_t N>
bool standoffNearATier(const std::array<StandoffCandidate, N>& c,
        const std::array<StandoffObstacleInfo, N>& info,
        const std::array<double, N>& offsets_deg, int previous,
        double peer_hard, const StandoffObstacleParams& p) {
    const auto a_ok = [&](int i, double m) {
        const auto& o = info[i];
        return i >= 0 && i < static_cast<int>(N) && c[i].inside_bounds &&
            std::isfinite(c[i].x) && std::isfinite(c[i].y) &&
            c[i].peer_clearance >= peer_hard && !o.band_blocked &&
            !(p.corridor && o.crosses_person) &&
            std::fabs(offsets_deg[i]) <= 30.0 + 1e-6 &&
            o.wall_clearance >= p.wall_clear + m && o.any_clearance >= p.thin_clear + m &&
            o.los_clearance >= p.los_clear + 0.5 * m && o.view_clearance >= p.view_clear;
    };
    if (a_ok(previous, -p.hysteresis)) return true;
    for (int i = 0; i < static_cast<int>(N); ++i)
        if (a_ok(i, 0.0)) return true;
    return false;
}

// 候选遵守边界、友机间距和高度带硬约束，并保持站位滞回。
// 常规档优先，走廊档及降级评分保留硬约束和穿越惩罚。
template <std::size_t N>
int chooseObstacleAwareStandoff(const std::array<StandoffCandidate, N>& c,
        const std::array<StandoffObstacleInfo, N>& info,
        const std::array<double, N>& offsets_deg, int previous,
        double peer_hard, double peer_return_margin,
        const StandoffObstacleParams& p, bool* degraded = nullptr,
        bool* corridor_tier = nullptr) {
    if (degraded) *degraded = false;
    if (corridor_tier) *corridor_tier = false;
    const int n = static_cast<int>(N);
    const auto hard = [&](int i, double m) {
        return i >= 0 && i < n && c[i].inside_bounds &&
            std::isfinite(c[i].x) && std::isfinite(c[i].y) &&
            c[i].peer_clearance >= peer_hard + m && !info[i].band_blocked;
    };
    const auto clear_ok = [&](int i, double m) {
        const auto& o = info[i];
        return o.wall_clearance >= p.wall_clear + m &&
               o.any_clearance >= p.thin_clear + m &&
               o.los_clearance >= p.los_clear + 0.5 * m &&
               o.view_clearance >= p.view_clear;
    };
    const auto crossing = [&](int i) { return p.corridor && info[i].crosses_person; };
    // A 档 (穿越的只在开阔处才算) / B 档 (走廊档, 不许穿越)
    const auto a_ok = [&](int i, double m) {
        return clear_ok(i, m) &&
               (!crossing(i) || info[i].path_clearance >= p.cross_path_clear + m);
    };
    const auto b_ok = [&](int i, double m) {
        const auto& o = info[i];
        return p.corridor && !crossing(i) &&
               o.wall_clearance >= p.corridor_clear + m &&
               o.any_clearance >= p.corridor_clear + m &&
               o.los_clearance >= o.los_required + 0.5 * m &&
               o.view_clearance >= p.view_clear;
    };
    const auto steps = [&](int i) { return std::fabs(offsets_deg[i]) / 30.0; };
    const bool near_a = !p.corridor ||
        standoffNearATier(c, info, offsets_deg, previous, peer_hard, p);

    if (hard(previous, 0.0)) {
        const bool keep_a = a_ok(previous, -p.hysteresis);
        const bool keep_b = !keep_a && !near_a && b_ok(previous, -p.hysteresis);
        if (keep_a || keep_b) {
            if (previous != 0 && hard(0, std::max(0.0, peer_return_margin)) && !crossing(0)) {
                if (clear_ok(0, p.hysteresis)) return 0;
                if (keep_b && b_ok(0, p.hysteresis)) {   // 都是走廊档: 回到正对行人
                    if (corridor_tier) *corridor_tier = true;
                    return 0;
                }
            }
            if (corridor_tier) *corridor_tier = keep_b;
            return previous;
        }
    }
    int best = -1;
    bool best_b = false;
    double best_cost = ClearanceField::kInf;
    for (int i = 0; i < n; ++i) {
        if (!hard(i, 0.0)) continue;
        const bool ia = a_ok(i, 0.0), ib = !ia && b_ok(i, 0.0);
        if (!ia && !ib) continue;
        const double cost = steps(i) + (info[i].path_clearance < p.path_clear ? 2.0 : 0.0) +
                            (ib ? 1.5 : 0.0) + (crossing(i) ? 6.0 : 0.0);
        if (cost < best_cost - 1e-9) { best_cost = cost; best = i; best_b = ib; }
    }
    if (best >= 0) {
        if (corridor_tier) *corridor_tier = best_b;
        return best;
    }

    const double cliff = p.corridor ? std::min(p.wall_min, p.corridor_cliff) : p.wall_min;
    const double any_cliff = p.corridor ? std::min(1.5, p.corridor_cliff) : 1.5;
    const auto score = [&](int i) {
        const auto& o = info[i];
        double s = o.wall_clearance >= cliff
            ? std::min(1.0, o.wall_clearance / p.wall_clear) : -3.0;
        s += o.any_clearance >= p.thin_clear ? 0.5 : (o.any_clearance >= any_cliff ? 0.0 : -3.0);
        s += o.los_clearance >= (p.corridor ? o.los_required : p.los_clear) ? 1.0 : 0.0;
        s += o.view_clearance >= p.view_clear ? 2.0 : 0.0;   // 楼里/楼背面的点几乎不选
        s += o.path_clearance >= p.path_clear ? 0.5 : 0.0;
        if (crossing(i)) s -= 5.0;                            // 宁可在人身后等, 不从人身边穿
        return s - 0.05 * steps(i);
    };
    double best_score = -ClearanceField::kInf;
    for (int i = 0; i < n; ++i) {
        if (!hard(i, 0.0)) continue;
        const double sc = score(i);
        if (sc > best_score + 1e-9) { best_score = sc; best = i; }
    }
    if (best < 0) return -1;
    if (degraded) *degraded = true;
    if (hard(previous, 0.0) && score(previous) >= best_score - 0.15) best = previous;
    if (corridor_tier) *corridor_tier = p.corridor && !clear_ok(best, 0.0);
    return best;
}

// PID 速度安全过滤
struct VelocityGuardParams {
    double d_hard = 1.5;          // 预测路径离任意表面的最小允许净空 (m)
    double d_safe = 2.5;          // 期望离墙净空, 低于它开始计贴墙代价
    double thin_bonus = 0.5;      // 细杆的期望净空 = d_safe - thin_bonus
    double release_margin = 0.3;  // 介入后需路径净空 > d_safe+margin 才放回原速度
    double acc = 2.5;             // 指令斜率 (与 PID 末级斜率限幅一致)
    double lag = 0.3;             // 执行滞后 (PX4 速度环 + 指令低通)
    double horizon = 1.2;         // 到达新速度后再匀速前视的时间 (s)
    double brake_acc = 2.0;       // 制动尾段减速度 (m/s²)
    double max_turn_deg = 120.0;  // 最大偏转
    double turn_step_deg = 10.0;
    double los_ok = 0.6;          // 路径终点->行人视线离墙低于此值开始计遮挡代价
    double los_skip = 0.4;
    double w_clear = 20.0, w_los = 1.5, w_slow = 1.5, w_smooth = 0.3;
    double escape_speed = 0.6;    // 已经贴近(<d_hard+0.3)时补充的远离速度
};

struct VelocityGuardState {
    double px = 0.0, py = 0.0;    // 本机位置
    double v0x = 0.0, v0y = 0.0;  // 当前实际速度 (缺失时用上一拍指令)
    double vpx = 0.0, vpy = 0.0;  // 上一拍下发指令
    double vdx = 0.0, vdy = 0.0;  // PID 期望速度
    bool has_person = false;
    double tx = 0.0, ty = 0.0;    // 行人 (视线代价)
    bool active = false;          // 上一拍是否在介入 (滞回)
    // 走廊期望净空不超过可用宽度，下限为硬净空 +0.2m。
    double safe_ref = std::numeric_limits<double>::infinity();
};

struct VelocityGuardResult {
    double vx = 0.0, vy = 0.0;
    bool intervened = false;      // 修改了 PID 期望速度
    bool blocked = false;         // 找不到朝期望方向前进的安全速度
    double desired_clear = ClearanceField::kInf;  // 期望速度预测路径的最小(等效)净空
    double chosen_clear = ClearanceField::kInf;
};

struct GuardPathEval {
    double min_all = ClearanceField::kInf;      // 全路径到任意表面
    double min_all_lag = ClearanceField::kInf;  // 滞后段(所有候选共有)
    double min_eff = ClearanceField::kInf;      // 等效净空 (墙/细杆折算)
    double mean_def2 = 0.0, max_def = 0.0;      // 贴墙亏欠 (归一化)
    double ex = 0.0, ey = 0.0;                  // 前视终点 (制动尾段之前)
};

// 预测: 先按当前速度走 lag, 再以 acc 斜率转到 u, 匀速 horizon, 最后制动尾段。
inline GuardPathEval evaluateGuardPath(const ClearanceField& f, const VelocityGuardParams& p,
        double px, double py, double v0x, double v0y, double ux, double uy) {
    GuardPathEval e;
    const double dt = 0.05;
    double x = px, y = py, vx = v0x, vy = v0y, t = 0.0, sum = 0.0;
    int n = 0;
    const auto sample = [&](double sx, double sy, bool lag_part) {
        const double ca = f.clearance(sx, sy);
        const double ce = std::min(f.wallClearance(sx, sy), ca + p.thin_bonus);
        e.min_all = std::min(e.min_all, ca);
        if (lag_part) e.min_all_lag = std::min(e.min_all_lag, ca);
        e.min_eff = std::min(e.min_eff, ce);
        const double def = std::isfinite(ce) ? std::max(0.0, p.d_safe - ce) / p.d_safe : 0.0;
        sum += def * def;
        ++n;
        e.max_def = std::max(e.max_def, def);
    };
    const double acc = std::max(0.1, p.acc);
    const double t_end = p.lag + std::hypot(ux - v0x, uy - v0y) / acc + p.horizon;
    while (t < t_end - 1e-9) {
        if (t >= p.lag - 1e-9) {
            double dvx = ux - vx, dvy = uy - vy;
            const double dm = std::hypot(dvx, dvy), lim = acc * dt;
            if (dm > lim) { dvx *= lim / dm; dvy *= lim / dm; }
            vx += dvx; vy += dvy;
        }
        x += vx * dt; y += vy * dt; t += dt;
        sample(x, y, t <= p.lag + 1e-9);
    }
    e.ex = x; e.ey = y;
    const double s = std::hypot(vx, vy);
    if (s > 1e-3) {
        const double len = s * s / (2.0 * std::max(0.1, p.brake_acc));
        for (double l = 0.15; l < len + 0.15; l += 0.15) {
            const double ll = std::min(l, len);
            sample(x + vx / s * ll, y + vy / s * ll, false);
            if (ll >= len) break;
        }
    }
    e.mean_def2 = n ? sum / n : 0.0;
    return e;
}

// 期望速度的预测路径不进入 d_safe 时原样返回(不介入); 否则在 ±max_turn 方向 ×
// {1, .75, .5, .25} 速度里选代价最小的: 贴墙亏欠 + 偏离期望 + 减速 + 遮挡 + 平滑。
inline VelocityGuardResult guardVelocity(const ClearanceField& f, const VelocityGuardParams& p0,
                                         const VelocityGuardState& s) {
    VelocityGuardParams p = p0;
    if (s.safe_ref < p.d_safe) p.d_safe = std::max(p.d_hard + 0.2, s.safe_ref);
    VelocityGuardResult r;
    r.vx = s.vdx; r.vy = s.vdy;
    if (!f.valid()) return r;
    const GuardPathEval des = evaluateGuardPath(f, p, s.px, s.py, s.v0x, s.v0y, s.vdx, s.vdy);
    r.desired_clear = r.chosen_clear = des.min_eff;
    const double release = s.active ? p.release_margin : 0.0;
    if (des.min_all >= p.d_hard && des.min_eff >= p.d_safe + release) return r;

    r.intervened = true;
    const double sd = std::hypot(s.vdx, s.vdy);
    const double s_ref = std::max(sd, 0.5);
    const double c_now = f.clearance(s.px, s.py);
    // 已经贴近时只要求"不再更近", 保证总有可选的远离/停住
    const double d_req = std::min(p.d_hard, std::min(c_now, des.min_all_lag) - 0.05);

    std::vector<std::pair<double, double>> cand;
    cand.reserve(110);
    if (sd > 0.05) {
        const double base = std::atan2(s.vdy, s.vdx);
        const int K = std::max(0, static_cast<int>(p.max_turn_deg / p.turn_step_deg + 1e-6));
        const double step = p.turn_step_deg * M_PI / 180.0;
        for (int k = -K; k <= K; ++k)
            for (double sc : {1.0, 0.75, 0.5, 0.25})
                cand.emplace_back(sc * sd * std::cos(base + k * step),
                                  sc * sd * std::sin(base + k * step));
    }
    cand.emplace_back(0.0, 0.0);
    if (c_now < p.d_hard + 0.3 && std::isfinite(c_now)) {
        const double h = 0.3;
        const double gx = f.clearance(s.px + h, s.py) - f.clearance(s.px - h, s.py);
        const double gy = f.clearance(s.px, s.py + h) - f.clearance(s.px, s.py - h);
        if (std::isfinite(gx) && std::isfinite(gy) && std::hypot(gx, gy) > 1e-6) {
            const double a0 = std::atan2(gy, gx);
            for (double da : {-0.8, -0.4, 0.0, 0.4, 0.8})
                cand.emplace_back(p.escape_speed * std::cos(a0 + da),
                                  p.escape_speed * std::sin(a0 + da));
        }
    }

    double best_j = ClearanceField::kInf, best_vx = 0.0, best_vy = 0.0, best_clear = 0.0;
    double emerg_c = -1.0, emerg_vx = 0.0, emerg_vy = 0.0, emerg_clear = 0.0;
    for (const auto& u : cand) {
        const GuardPathEval e = evaluateGuardPath(f, p, s.px, s.py, s.v0x, s.v0y,
                                                  u.first, u.second);
        if (e.min_all > emerg_c) {
            emerg_c = e.min_all; emerg_vx = u.first; emerg_vy = u.second;
            emerg_clear = e.min_eff;
        }
        if (e.min_all < d_req) continue;
        const double su = std::hypot(u.first, u.second);
        const double dvx = u.first - s.vdx, dvy = u.second - s.vdy;
        double j = (dvx * dvx + dvy * dvy) / (s_ref * s_ref);
        if (su < sd) j += p.w_slow * ((sd - su) / s_ref) * ((sd - su) / s_ref);
        j += p.w_clear * (e.mean_def2 + e.max_def * e.max_def);
        if (s.has_person) {
            const double lc = f.losSegmentClearance(e.ex, e.ey, s.tx, s.ty, 0.0, p.los_skip);
            j += p.w_los * std::min(1.0, std::max(0.0, (p.los_ok - lc) / p.los_ok));
        }
        const double pvx = u.first - s.vpx, pvy = u.second - s.vpy;
        j += p.w_smooth * (pvx * pvx + pvy * pvy) / (s_ref * s_ref);
        if (j < best_j) {
            best_j = j; best_vx = u.first; best_vy = u.second; best_clear = e.min_eff;
        }
    }
    if (!std::isfinite(best_j)) {
        // 连停住都来不及保持 d_req: 取预测净空最大的 (紧急)
        r.vx = emerg_vx; r.vy = emerg_vy; r.chosen_clear = emerg_clear;
        r.blocked = true;
        return r;
    }
    r.vx = best_vx; r.vy = best_vy; r.chosen_clear = best_clear;
    if (sd > 0.3) {
        const double progress = (best_vx * s.vdx + best_vy * s.vdy) / sd;
        r.blocked = progress < 0.1 * sd;
    }
    return r;
}

}  // namespace fly_px4
