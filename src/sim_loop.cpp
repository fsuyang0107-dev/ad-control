#include "adc/sim_loop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace adc {

std::size_t FindNearest(const Trajectory& traj, const VehicleState& s) {
    if (traj.empty()) return 0;
    std::size_t best = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < traj.size(); ++i) {
        const double dx = traj[i].x - s.x;
        const double dy = traj[i].y - s.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return best;
}

std::size_t FindLookahead(const Trajectory& traj, std::size_t idx, double lookahead) {
    if (traj.size() < 2) return 0;
    if (idx >= traj.size() - 1) return traj.size() - 1;

    double acc = 0.0;
    for (std::size_t i = idx + 1; i < traj.size(); ++i) {
        acc += std::hypot(traj[i].x - traj[i - 1].x, traj[i].y - traj[i - 1].y);
        if (acc >= lookahead) return i;
    }
    return traj.size() - 1;
}

ControlCmd FeedForward(const TrajPoint& rp, const VehicleParams& p) {
    ControlCmd u;
    u.steer = std::atan(rp.curvature * p.wheelbase);
    u.steer = std::clamp(u.steer, -p.max_steer, p.max_steer);
    u.accel = 0.0;
    return u;
}

ControlCmd PurePursuitCmd(const Trajectory& traj, const VehicleState& s, std::size_t idx,
                          const VehicleParams& p, const CtrlParams& cp) {
    ControlCmd u;
    if (traj.empty()) return u;

    const double Ld = cp.lookahead_base + cp.lookahead_time * s.v;
    const std::size_t li = FindLookahead(traj, idx, Ld);

    const double dx = traj[li].x - s.x;
    const double dy = traj[li].y - s.y;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) { u.steer = 0.0; return u; }

    const double alpha = NormalizeAngle(std::atan2(dy, dx) - s.yaw);
    u.steer = std::atan2(2.0 * p.wheelbase * std::sin(alpha), dist);
    u.steer = std::clamp(u.steer, -p.max_steer, p.max_steer);
    u.accel = 0.0;
    return u;
}

ControlCmd StanleyCmd(const VehicleState& s, const TrajPoint& rp, const VehicleParams& p,
                      const CtrlParams& cp) {
    ControlCmd u;

    const double dx = s.x - rp.x;
    const double dy = s.y - rp.y;
    const double e_y = -dx * std::sin(rp.yaw) + dy * std::cos(rp.yaw);
    const double e_yaw = NormalizeAngle(s.yaw - rp.yaw);

    double steer = 0.0;
    if (cp.feedforward) steer += std::atan(rp.curvature * p.wheelbase);

    // 航向项:车头偏左 -> 右打
    steer -= e_yaw;

    // 横偏项:车偏左 -> 右打
    const double v = std::max(s.v, 1.0);
    steer -= std::atan(cp.stanley_gain * e_y / v);

    u.steer = std::clamp(steer, -p.max_steer, p.max_steer);
    u.accel = 0.0;
    return u;
}

ControlCmd ComputeControl(CtrlMode mode, const Trajectory& traj, const VehicleState& s,
                          const TrajPoint& rp, std::size_t idx, const SimConfig& cfg) {
    switch (mode) {
        case CtrlMode::kFeedForward:
            return FeedForward(rp, cfg.vehicle);
        case CtrlMode::kPurePursuit:
            return PurePursuitCmd(traj, s, idx, cfg.vehicle, cfg.ctrl);
        case CtrlMode::kStanley:
            return StanleyCmd(s, rp, cfg.vehicle, cfg.ctrl);
    }
    return FeedForward(rp, cfg.vehicle);
}

std::string CtrlModeName(CtrlMode m) {
    switch (m) {
        case CtrlMode::kFeedForward: return "feedforward";
        case CtrlMode::kPurePursuit: return "pure_pursuit";
        case CtrlMode::kStanley: return "stanley";
    }
    return "unknown";
}

std::vector<SimRecord> SimLoop::Run(const Trajectory& ref) const {
    std::vector<SimRecord> out;
    if (ref.empty()) return out;

    const double dt = cfg_.dt;
    if (dt <= 0.0) return out;

    const int steps = static_cast<int>(std::lround(cfg_.duration / dt));
    if (steps <= 0) return out;

    const TrajPoint& p0 = ref[0];
    VehicleState s;
    s.x = p0.x + cfg_.init_y_offset * (-std::sin(p0.yaw));
    s.y = p0.y + cfg_.init_y_offset * (std::cos(p0.yaw));
    s.yaw = p0.yaw + cfg_.init_yaw_offset;
    s.v = p0.v_ref;

    out.reserve(static_cast<std::size_t>(steps) + 1);

    for (int k = 0; k <= steps; ++k) {
        const std::size_t idx = FindNearest(ref, s);
        const TrajPoint& rp = ref[idx];

        const double dx = s.x - rp.x;
        const double dy = s.y - rp.y;
        const double e_y = -dx * std::sin(rp.yaw) + dy * std::cos(rp.yaw);
        const double e_yaw = NormalizeAngle(s.yaw - rp.yaw);
        const double e_v = s.v - rp.v_ref;

        const ControlCmd u = ComputeControl(cfg_.ctrl_mode, ref, s, rp, idx, cfg_);

        SimRecord rec;
        rec.t = k * dt;
        rec.state = s;
        rec.cmd = u;
        rec.e_y = e_y;
        rec.e_yaw = e_yaw;
        rec.e_v = e_v;
        rec.v_ref = rp.v_ref;
        rec.ref_index = idx;
        out.push_back(rec);

        model_.StepKinematic(s, u, dt);
        s.v = rp.v_ref;
    }
    return out;
}

}  // namespace adc
