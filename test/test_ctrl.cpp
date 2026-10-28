#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "adc/metrics.hpp"
#include "adc/sim_loop.hpp"
#include "adc/trajectory.hpp"
#include "adc/types.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Outcome {
    double rmse_y{0.0};
    double ss_max{0.0};
    double settle{0.0};
};

Outcome Analyze(const std::vector<adc::SimRecord>& r) {
    Outcome o;
    if (r.empty()) return o;
    double sum = 0.0;
    for (const auto& x : r) sum += x.e_y * x.e_y;
    o.rmse_y = std::sqrt(sum / r.size());

    for (std::size_t i = r.size() * 8 / 10; i < r.size(); ++i)
        o.ss_max = std::max(o.ss_max, std::abs(r[i].e_y));

    o.settle = r.back().t;
    for (std::size_t i = r.size(); i-- > 0;)
        if (std::abs(r[i].e_y) >= 0.05) { o.settle = r[i].t; break; }
    return o;
}

Outcome RunScenario(const adc::Trajectory& traj, adc::CtrlMode mode, double v,
                    double init_y = 0.0, double init_yaw = 0.0) {
    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = traj.length() / v;
    cfg.ctrl_mode = mode;
    cfg.init_y_offset = init_y;
    cfg.init_yaw_offset = init_yaw;
    adc::SimLoop loop(cfg);
    return Analyze(loop.Run(traj));
}

}  // namespace

TEST(CtrlModeName, ReturnsReadableNames) {
    EXPECT_EQ(adc::CtrlModeName(adc::CtrlMode::kFeedForward), "feedforward");
    EXPECT_EQ(adc::CtrlModeName(adc::CtrlMode::kPurePursuit), "pure_pursuit");
    EXPECT_EQ(adc::CtrlModeName(adc::CtrlMode::kStanley), "stanley");
}

TEST(FindLookahead, EmptyTrajectoryReturnsZero) {
    adc::Trajectory t;
    EXPECT_EQ(adc::FindLookahead(t, 0, 5.0), 0u);
}

TEST(FindLookahead, ReturnsPointAheadByLookaheadDistance) {
    const auto traj = adc::MakeStraight(20.0, 0.1, 10.0);
    const std::size_t idx = adc::FindLookahead(traj, 10, 5.0);
    ASSERT_LT(idx, traj.size());
    const double dist = std::hypot(traj[idx].x - traj[10].x, traj[idx].y - traj[10].y);
    EXPECT_NEAR(dist, 5.0, 0.11);
}

TEST(FindLookahead, ClampsAtTrajectoryEnd) {
    const auto traj = adc::MakeStraight(10.0, 0.1, 10.0);
    EXPECT_EQ(adc::FindLookahead(traj, traj.size() - 1, 5.0), traj.size() - 1);
    EXPECT_EQ(adc::FindLookahead(traj, traj.size() - 5, 100.0), traj.size() - 1);
}

TEST(PurePursuit, ZeroErrorOnStraightGivesZeroSteer) {
    const auto traj = adc::MakeStraight(20.0, 0.1, 15.0);
    adc::VehicleState s;
    s.x = 5.0;
    s.y = 0.0;
    s.yaw = 0.0;
    s.v = 15.0;
    adc::VehicleParams p;
    adc::CtrlParams cp;
    const std::size_t idx = adc::FindNearest(traj, s);
    EXPECT_NEAR(adc::PurePursuitCmd(traj, s, idx, p, cp).steer, 0.0, 1e-12);
}

TEST(PurePursuit, VehicleLeftOfPathSteersRight) {
    const auto traj = adc::MakeStraight(20.0, 0.1, 15.0);
    adc::VehicleState s;
    s.x = 5.0;
    s.y = 0.5;
    s.yaw = 0.0;
    s.v = 15.0;
    adc::VehicleParams p;
    adc::CtrlParams cp;
    const std::size_t idx = adc::FindNearest(traj, s);
    EXPECT_LT(adc::PurePursuitCmd(traj, s, idx, p, cp).steer, 0.0)
        << "车在左侧应当右打方向";
}

TEST(PurePursuit, VehicleRightOfPathSteersLeft) {
    const auto traj = adc::MakeStraight(20.0, 0.1, 15.0);
    adc::VehicleState s;
    s.x = 5.0;
    s.y = -0.5;
    s.yaw = 0.0;
    s.v = 15.0;
    adc::VehicleParams p;
    adc::CtrlParams cp;
    const std::size_t idx = adc::FindNearest(traj, s);
    EXPECT_GT(adc::PurePursuitCmd(traj, s, idx, p, cp).steer, 0.0)
        << "车在右侧应当左打方向";
}

TEST(Stanley, ZeroErrorOnStraightGivesZeroSteer) {
    adc::VehicleState s;
    s.yaw = 0.0;
    s.v = 15.0;
    adc::TrajPoint rp;
    adc::VehicleParams p;
    adc::CtrlParams cp;
    EXPECT_NEAR(adc::StanleyCmd(s, rp, p, cp).steer, 0.0, 1e-12);
}

TEST(Stanley, VehicleLeftOfPathSteersRight) {
    adc::VehicleState s;
    s.y = 0.5;
    s.yaw = 0.0;
    s.v = 15.0;
    adc::TrajPoint rp;
    adc::VehicleParams p;
    adc::CtrlParams cp;
    EXPECT_LT(adc::StanleyCmd(s, rp, p, cp).steer, 0.0);
}

TEST(Stanley, HeadingLeftOfPathSteersRight) {
    adc::VehicleState s;
    s.y = 0.0;
    s.yaw = 0.1;
    s.v = 15.0;
    adc::TrajPoint rp;
    adc::VehicleParams p;
    adc::CtrlParams cp;
    EXPECT_LT(adc::StanleyCmd(s, rp, p, cp).steer, 0.0);
}

TEST(Stanley, ZeroErrorOnCurveEqualsFeedForward) {
    adc::VehicleState s;
    s.yaw = 0.0;
    s.v = 15.0;
    adc::TrajPoint rp;
    rp.curvature = 0.02;
    adc::VehicleParams p;
    adc::CtrlParams cp;
    cp.feedforward = true;
    EXPECT_NEAR(adc::StanleyCmd(s, rp, p, cp).steer, adc::FeedForward(rp, p).steer, 1e-12);
}

TEST(Stanley, FeedforwardCanBeDisabled) {
    adc::VehicleState s;
    s.yaw = 0.0;
    s.v = 15.0;
    adc::TrajPoint rp;
    rp.curvature = 0.02;
    adc::VehicleParams p;
    adc::CtrlParams cp;
    cp.feedforward = false;
    EXPECT_NEAR(adc::StanleyCmd(s, rp, p, cp).steer, 0.0, 1e-12);
}

TEST(Controllers, SteerNeverExceedsPhysicalLimit) {
    adc::VehicleParams p;
    adc::CtrlParams cp;
    const auto traj = adc::MakeStraight(20.0, 0.1, 15.0);

    adc::VehicleState s;
    s.x = 5.0;
    s.y = 5.0;
    s.yaw = 0.5;
    s.v = 15.0;
    const std::size_t idx = adc::FindNearest(traj, s);
    EXPECT_LE(std::abs(adc::PurePursuitCmd(traj, s, idx, p, cp).steer), p.max_steer + 1e-12);

    adc::TrajPoint rp;
    rp.curvature = 10.0;
    EXPECT_LE(std::abs(adc::StanleyCmd(s, rp, p, cp).steer), p.max_steer + 1e-12);
}

TEST(ClosedLoop, PurePursuitConvergesOnStraight) {
    const auto traj = adc::MakeStraight(20.0, 0.1, 15.0);
    const auto r = RunScenario(traj, adc::CtrlMode::kPurePursuit, 15.0, 0.5);
    EXPECT_LT(r.rmse_y, 0.30);
    EXPECT_LT(r.ss_max, 0.02);
    EXPECT_LT(r.settle, 1.5);
}

TEST(ClosedLoop, StanleyConvergesOnStraight) {
    const auto traj = adc::MakeStraight(20.0, 0.1, 15.0);
    const auto r = RunScenario(traj, adc::CtrlMode::kStanley, 15.0, 0.5);
    EXPECT_LT(r.rmse_y, 0.30);
    EXPECT_LT(r.ss_max, 0.05);
    EXPECT_LT(r.settle, 1.5);
}

TEST(ClosedLoop, FeedbackBeatsFeedForwardOnDLC) {
    const auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);

    const auto ff = RunScenario(traj, adc::CtrlMode::kFeedForward, 15.0, 0.5);
    const auto pp = RunScenario(traj, adc::CtrlMode::kPurePursuit, 15.0, 0.5);
    const auto st = RunScenario(traj, adc::CtrlMode::kStanley, 15.0, 0.5);

    EXPECT_GT(ff.rmse_y, 0.40) << "前馈基线应保持在 0.4 以上";
    EXPECT_GT(ff.ss_max, 0.30) << "前馈不收敛";

    EXPECT_LT(pp.rmse_y, ff.rmse_y * 0.5);
    EXPECT_LT(st.rmse_y, ff.rmse_y * 0.5);

    EXPECT_LT(pp.ss_max, ff.ss_max * 0.1);
    EXPECT_LT(st.ss_max, ff.ss_max * 0.1);
}

TEST(ClosedLoop, FeedbackHandlesInitialHeadingError) {
    const auto traj = adc::MakeStraight(20.0, 0.1, 15.0);

    const auto ff = RunScenario(traj, adc::CtrlMode::kFeedForward, 15.0, 0.0, 0.1);
    const auto pp = RunScenario(traj, adc::CtrlMode::kPurePursuit, 15.0, 0.0, 0.1);
    const auto st = RunScenario(traj, adc::CtrlMode::kStanley, 15.0, 0.0, 0.1);

    EXPECT_GT(ff.rmse_y, 1.0);
    EXPECT_LT(pp.rmse_y, 0.20);
    EXPECT_LT(st.rmse_y, 0.20);
}

TEST(ClosedLoop, AllModesProduceFiniteResultsOnAllScenarios) {
    const double v = 15.0;
    const adc::Trajectory trajs[] = {
        adc::MakeStraight(20.0, 0.1, v),
        adc::MakeArc(25.0, kPi / 2.0, 0.1, v),
        adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, v),
        adc::MakeSine(50.0, 2.0, 2.0, 0.1, v),
    };
    for (const auto& traj : trajs) {
        for (auto mode : {adc::CtrlMode::kFeedForward, adc::CtrlMode::kPurePursuit,
                          adc::CtrlMode::kStanley}) {
            const auto r = RunScenario(traj, mode, v, 0.5);
            EXPECT_TRUE(std::isfinite(r.rmse_y)) << adc::CtrlModeName(mode);
            EXPECT_TRUE(std::isfinite(r.ss_max)) << adc::CtrlModeName(mode);
        }
    }
}

TEST(ClosedLoop, ZeroOffsetTracksWithinTolerance) {
    const double v = 15.0;
    const adc::Trajectory traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, v);
    for (auto mode : {adc::CtrlMode::kPurePursuit, adc::CtrlMode::kStanley}) {
        const auto r = RunScenario(traj, mode, v, 0.0);
        EXPECT_LT(r.rmse_y, 0.05) << adc::CtrlModeName(mode);
    }
}
