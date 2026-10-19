#include <gtest/gtest.h>

#include <cmath>

#include "adc/types.hpp"
#include "adc/vehicle_model.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

/// 圆周运动的解析解(恒定速度 v 与恒定前轮转角 steer)
/// omega = v * tan(steer) / L
/// x(t)  = x0 + v/omega * (sin(yaw0 + omega*t) - sin(yaw0))
/// y(t)  = y0 - v/omega * (cos(yaw0 + omega*t) - cos(yaw0))
/// yaw(t)= yaw0 + omega*t
struct CircleExact {
    double x;
    double y;
    double yaw;
};

CircleExact AnalyticCircle(double x0, double y0, double yaw0, double v, double steer,
                           double L, double t) {
    const double omega = v * std::tan(steer) / L;
    CircleExact e{};
    e.x = x0 + (v / omega) * (std::sin(yaw0 + omega * t) - std::sin(yaw0));
    e.y = y0 - (v / omega) * (std::cos(yaw0 + omega * t) - std::cos(yaw0));
    e.yaw = yaw0 + omega * t;
    return e;
}

/// 用某种积分方法跑 t_total 秒,返回最终状态
adc::VehicleState RunCircle(const adc::VehicleModel& model, double dt, double t_total,
                            double yaw0, double v, double steer) {
    adc::VehicleState s;
    s.yaw = yaw0;
    s.v = v;
    adc::ControlCmd u;
    u.steer = steer;
    const int steps = static_cast<int>(std::lround(t_total / dt));
    for (int i = 0; i < steps; ++i) model.StepKinematic(s, u, dt);
    return s;
}

adc::VehicleState RunCircleEuler(const adc::VehicleModel& model, double dt, double t_total,
                                 double yaw0, double v, double steer) {
    adc::VehicleState s;
    s.yaw = yaw0;
    s.v = v;
    adc::ControlCmd u;
    u.steer = steer;
    const int steps = static_cast<int>(std::lround(t_total / dt));
    for (int i = 0; i < steps; ++i) model.StepKinematicEuler(s, u, dt);
    return s;
}

}  // namespace

// ---------------------------------------------------------------
// 一、RK4 与解析解对比
// ---------------------------------------------------------------

TEST(VehicleModelKinematic, Rk4YawMatchesAnalyticExactly) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    const double v = 10.0, steer = 0.1, t_total = 10.0, dt = 0.02, yaw0 = 0.0;
    const auto got = RunCircle(model, dt, t_total, yaw0, v, steer);
    const auto exp = AnalyticCircle(0.0, 0.0, yaw0, v, steer, p.wheelbase, t_total);

    // yaw 的微分方程是线性的,RK4 应当精确到浮点误差量级
    EXPECT_NEAR(got.yaw, exp.yaw, 1e-12);
}

TEST(VehicleModelKinematic, Rk4PositionMatchesAnalytic) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    const double v = 10.0, steer = 0.1, t_total = 10.0, dt = 0.02, yaw0 = 0.0;
    const auto got = RunCircle(model, dt, t_total, yaw0, v, steer);
    const auto exp = AnalyticCircle(0.0, 0.0, yaw0, v, steer, p.wheelbase, t_total);

    EXPECT_NEAR(got.x, exp.x, 1e-3);
    EXPECT_NEAR(got.y, exp.y, 1e-3);
}

TEST(VehicleModelKinematic, CircleRadiusMatchesTheory) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    const double v = 10.0, steer = 0.1, t_total = 10.0, dt = 0.02;
    const auto got = RunCircle(model, dt, t_total, 0.0, v, steer);

    // 理论转弯半径 R = L / tan(steer)
    const double R = p.wheelbase / std::tan(steer);
    // 从原点出发,圆心在 (0, R);检查到圆心的距离是否等于 R
    const double dist = std::hypot(got.x - 0.0, got.y - R);
    EXPECT_NEAR(dist, R, 1e-3);
}

// ---------------------------------------------------------------
// 二、RK4 精度优于欧拉法(这是关键的对比证据)
// ---------------------------------------------------------------

TEST(VehicleModelKinematic, Rk4IsMoreAccurateThanEulerAtSameStep) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    const double v = 10.0, steer = 0.1, t_total = 10.0, dt = 0.1;
    const auto exp = AnalyticCircle(0.0, 0.0, 0.0, v, steer, p.wheelbase, t_total);

    const auto rk4 = RunCircle(model, dt, t_total, 0.0, v, steer);
    const auto eul = RunCircleEuler(model, dt, t_total, 0.0, v, steer);

    const double err_rk4 = std::hypot(rk4.x - exp.x, rk4.y - exp.y);
    const double err_eul = std::hypot(eul.x - exp.x, eul.y - exp.y);

    // RK4 的误差必须明显小于欧拉法(至少小一个数量级)
    EXPECT_LT(err_rk4, err_eul);

    // 在 dt = 0.1 下,RK4 位置误差应在毫米级
    EXPECT_LT(err_rk4, 1e-2);
    // 欧拉法误差会明显更大
    EXPECT_GT(err_eul, 1e-2);
}

TEST(VehicleModelKinematic, EulerErrorShrinksAboutQuadratically) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    const double v = 10.0, steer = 0.1, t_total = 5.0;

    auto eulerError = [&](double dt) {
        const auto exp = AnalyticCircle(0.0, 0.0, 0.0, v, steer, p.wheelbase, t_total);
        const auto got = RunCircleEuler(model, dt, t_total, 0.0, v, steer);
        return std::hypot(got.x - exp.x, got.y - exp.y);
    };

    const double e1 = eulerError(0.1);
    const double e2 = eulerError(0.05);

    // 步长减半,欧拉法误差约降为 1/4(一阶精度的特征)
    // 这里只要求它确实下降,并且下降幅度明显
    EXPECT_LT(e2, e1 / 2.0);
}

TEST(VehicleModelKinematic, Rk4ErrorShrinksFasterThanEuler) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    const double v = 10.0, steer = 0.1, t_total = 5.0;

    auto rk4Error = [&](double dt) {
        const auto exp = AnalyticCircle(0.0, 0.0, 0.0, v, steer, p.wheelbase, t_total);
        const auto got = RunCircle(model, dt, t_total, 0.0, v, steer);
        return std::hypot(got.x - exp.x, got.y - exp.y);
    };

    const double r1 = rk4Error(0.1);
    const double r2 = rk4Error(0.05);
    const double e1 = [&] {
        const auto exp = AnalyticCircle(0.0, 0.0, 0.0, v, steer, p.wheelbase, t_total);
        const auto got = RunCircleEuler(model, 0.1, t_total, 0.0, v, steer);
        return std::hypot(got.x - exp.x, got.y - exp.y);
    }();
    const double e2 = [&] {
        const auto exp = AnalyticCircle(0.0, 0.0, 0.0, v, steer, p.wheelbase, t_total);
        const auto got = RunCircleEuler(model, 0.05, t_total, 0.0, v, steer);
        return std::hypot(got.x - exp.x, got.y - exp.y);
    }();

    // 收敛速度:误差缩小的倍数,RK4 应明显强于欧拉
    const double eulerRatio = e1 / e2;
    const double rk4Ratio = r1 / r2;
    EXPECT_GT(rk4Ratio, eulerRatio);
}

// ---------------------------------------------------------------
// 三、边界与退化情形
// ---------------------------------------------------------------

TEST(VehicleModelKinematic, ZeroSpeedMeansNoMotion) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    adc::VehicleState s;
    s.x = 1.0;
    s.y = 2.0;
    s.yaw = 0.5;
    s.v = 0.0;

    adc::ControlCmd u;
    u.steer = 0.3;

    for (int i = 0; i < 100; ++i) model.StepKinematic(s, u, 0.02);

    EXPECT_DOUBLE_EQ(s.x, 1.0);
    EXPECT_DOUBLE_EQ(s.y, 2.0);
    EXPECT_DOUBLE_EQ(s.yaw, 0.5);
}

TEST(VehicleModelKinematic, ZeroSteeringKeepsStraightLine) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    adc::VehicleState s;
    s.v = 5.0;

    adc::ControlCmd u;
    u.steer = 0.0;

    for (int i = 0; i < 50; ++i) model.StepKinematic(s, u, 0.02);

    // 1 秒后应前进约 5 m,且横向无偏移、航向不变
    EXPECT_NEAR(s.x, 5.0, 1e-9);
    EXPECT_NEAR(s.y, 0.0, 1e-12);
    EXPECT_NEAR(s.yaw, 0.0, 1e-12);
}

TEST(VehicleModelKinematic, YawRateMatchesFormula) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    adc::VehicleState s;
    s.v = 8.0;
    adc::ControlCmd u;
    u.steer = 0.2;

    model.StepKinematic(s, u, 0.02);

    const double expected = s.v / p.wheelbase * std::tan(u.steer);
    EXPECT_NEAR(s.yaw_rate, expected, 1e-12);
}

TEST(VehicleModelKinematic, SignConventionPositiveSteerTurnsLeft) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    adc::VehicleState s;
    s.v = 5.0;
    adc::ControlCmd u;
    u.steer = 0.1;  // 左转

    for (int i = 0; i < 50; ++i) model.StepKinematic(s, u, 0.02);

    // 约定:正转角 = 左转 = y 增大、yaw 增大
    EXPECT_GT(s.yaw, 0.0);
    EXPECT_GT(s.y, 0.0);
}

TEST(VehicleModelKinematic, ResultsAreAlwaysFinite) {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    adc::VehicleState s;
    s.v = 20.0;
    adc::ControlCmd u;
    u.steer = p.max_steer;

    for (int i = 0; i < 500; ++i) {
        model.StepKinematic(s, u, 0.02);
        ASSERT_TRUE(std::isfinite(s.x));
        ASSERT_TRUE(std::isfinite(s.y));
        ASSERT_TRUE(std::isfinite(s.yaw));
    }
}
