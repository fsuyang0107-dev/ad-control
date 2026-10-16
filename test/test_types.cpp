#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "adc/types.hpp"

namespace {

// 判断一个物理量是否为有限数(不是 NaN、不是 inf)
bool IsFinite(double v) { return std::isfinite(v); }

}  // namespace

// ============================================================
// 一、VehicleParams:车辆几何与物理参数的不变量
// ============================================================

TEST(VehicleParams, WheelbaseConsistentWithAxleDistances) {
    adc::VehicleParams p;
    // 轴距必须等于"前轴到质心 + 质心到后轴"的距离
    EXPECT_DOUBLE_EQ(p.wheelbase, p.lf + p.lr);
}

TEST(VehicleParams, SteeringLimitIsPhysicallyReasonable) {
    adc::VehicleParams p;
    // 前轮转角必须在 (0, 90 度) 区间内,否则车辆模型没有意义
    EXPECT_GT(p.max_steer, 0.0);
    EXPECT_LT(p.max_steer, M_PI / 2.0);
    // 现实中乘用车最大前轮转角约 30~45 度
    EXPECT_LT(p.max_steer, 50.0 * M_PI / 180.0);
}

TEST(VehicleParams, MassAndInertiaArePositive) {
    adc::VehicleParams p;
    EXPECT_GT(p.mass, 0.0);
    EXPECT_GT(p.Iz, 0.0);
    EXPECT_TRUE(IsFinite(p.mass));
    EXPECT_TRUE(IsFinite(p.Iz));
}

TEST(VehicleParams, CorneringStiffnessIsPositive) {
    adc::VehicleParams p;
    EXPECT_GT(p.Cf, 0.0);
    EXPECT_GT(p.Cr, 0.0);
    EXPECT_TRUE(IsFinite(p.Cf));
    EXPECT_TRUE(IsFinite(p.Cr));
}

TEST(VehicleParams, AccelerationLimitsHaveCorrectSigns) {
    adc::VehicleParams p;
    EXPECT_GT(p.max_accel, 0.0);   // 加速上限应为正
    EXPECT_LT(p.max_decel, 0.0);   // 减速上限应为负
    EXPECT_TRUE(IsFinite(p.max_accel));
    EXPECT_TRUE(IsFinite(p.max_decel));
}

TEST(VehicleParams, MassDistributionIsGeometricallyConsistent) {
    adc::VehicleParams p;
    // 质心到前后轴的距离都必须为正(质心在前后轴之间)
    EXPECT_GT(p.lf, 0.0);
    EXPECT_GT(p.lr, 0.0);
    // 前轴距通常不小于后轴距的 1/3,也不大于 3 倍
    EXPECT_GT(p.lf / p.lr, 1.0 / 3.0);
    EXPECT_LT(p.lf / p.lr, 3.0);
}

// ============================================================
// 二、VehicleState:默认构造必须是"全零且有限"
// ============================================================

TEST(VehicleState, DefaultConstructionIsAllZero) {
    adc::VehicleState s;
    EXPECT_DOUBLE_EQ(s.x, 0.0);
    EXPECT_DOUBLE_EQ(s.y, 0.0);
    EXPECT_DOUBLE_EQ(s.yaw, 0.0);
    EXPECT_DOUBLE_EQ(s.v, 0.0);
    EXPECT_DOUBLE_EQ(s.yaw_rate, 0.0);
}

TEST(VehicleState, DefaultValuesAreFinite) {
    adc::VehicleState s;
    EXPECT_TRUE(IsFinite(s.x));
    EXPECT_TRUE(IsFinite(s.y));
    EXPECT_TRUE(IsFinite(s.yaw));
    EXPECT_TRUE(IsFinite(s.v));
    EXPECT_TRUE(IsFinite(s.yaw_rate));
}

// ============================================================
// 三、ControlCmd:默认"不动"
// ============================================================

TEST(ControlCmd, DefaultIsNoSteeringAndNoAcceleration) {
    adc::ControlCmd u;
    EXPECT_DOUBLE_EQ(u.steer, 0.0);
    EXPECT_DOUBLE_EQ(u.accel, 0.0);
}

// ============================================================
// 四、TrajPoint:默认值与有限性
// ============================================================

TEST(TrajPoint, DefaultConstructionIsAllZero) {
    adc::TrajPoint p;
    EXPECT_DOUBLE_EQ(p.x, 0.0);
    EXPECT_DOUBLE_EQ(p.y, 0.0);
    EXPECT_DOUBLE_EQ(p.yaw, 0.0);
    EXPECT_DOUBLE_EQ(p.v_ref, 0.0);
    EXPECT_DOUBLE_EQ(p.curvature, 0.0);
}

// ============================================================
// 五、SimConfig:默认仿真配置必须可用
// ============================================================

TEST(SimConfig, DefaultDtIsPositiveAndReasonable) {
    adc::SimConfig c;
    EXPECT_GT(c.dt, 0.0);
    // 控制周期通常不超过 100 ms,否则离散化误差过大
    EXPECT_LE(c.dt, 0.1);
}

TEST(SimConfig, DefaultDurationIsPositiveAndLongEnough) {
    adc::SimConfig c;
    EXPECT_GT(c.duration, 0.0);
    // 至少能跑 1 秒,否则做不出任何有意义的跟踪实验
    EXPECT_GE(c.duration, 1.0);
}

TEST(SimConfig, SimulationStepCountIsConsistent) {
    adc::SimConfig c;
    const double steps = c.duration / c.dt;
    EXPECT_GT(steps, 0.0);
    EXPECT_TRUE(IsFinite(steps));
    // 步数应当是整数(避免半个时间步)
    EXPECT_DOUBLE_EQ(steps, std::floor(steps));
}

TEST(SimConfig, DefaultScenarioIsNotEmpty) {
    adc::SimConfig c;
    EXPECT_FALSE(c.scenario.empty());
}

// ============================================================
// 六、SimRecord:能被默认构造并聚合到容器里
// ============================================================

TEST(SimRecord, DefaultConstructionIsFinite) {
    adc::SimRecord r;
    EXPECT_DOUBLE_EQ(r.t, 0.0);
    EXPECT_DOUBLE_EQ(r.e_y, 0.0);
    EXPECT_DOUBLE_EQ(r.e_yaw, 0.0);
    EXPECT_TRUE(IsFinite(r.t));
}

TEST(SimRecord, CanBeStoredInVector) {
    std::vector<adc::SimRecord> log;
    log.reserve(3);
    for (int i = 0; i < 3; ++i) {
        adc::SimRecord r;
        r.t = 0.1 * i;
        log.push_back(r);
    }
    ASSERT_EQ(log.size(), 3u);
    EXPECT_DOUBLE_EQ(log[0].t, 0.0);
    EXPECT_DOUBLE_EQ(log[2].t, 0.2);
}
