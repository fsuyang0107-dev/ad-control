#include <gtest/gtest.h>

#include <cmath>

#include "adc/trajectory.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
}

// ============================================================
// 一、基础属性
// ============================================================

TEST(Trajectory, EmptyTrajectoryHasZeroSize) {
    adc::Trajectory t;
    EXPECT_EQ(t.size(), 0u);
    EXPECT_TRUE(t.empty());
    EXPECT_DOUBLE_EQ(t.length(), 0.0);
}

TEST(Trajectory, PushBackIncreasesSize) {
    adc::Trajectory t;
    adc::TrajPoint p;
    p.x = 1.0;
    t.push_back(p);
    EXPECT_EQ(t.size(), 1u);
    EXPECT_DOUBLE_EQ(t.at(0).x, 1.0);
}

TEST(Trajectory, LengthIsSumOfSegments) {
    adc::Trajectory t = adc::MakeStraight(10.0, 0.1, 5.0);
    EXPECT_NEAR(t.length(), 10.0, 1e-6);
}

// ============================================================
// 二、直线:曲率必须恒为 0
// ============================================================

TEST(Straight, AllPointsHaveZeroCurvature) {
    const auto t = adc::MakeStraight(20.0, 0.1, 8.0);
    ASSERT_GE(t.size(), 201u);
    for (std::size_t i = 0; i < t.size(); ++i)
        EXPECT_NEAR(t[i].curvature, 0.0, 1e-12) << "index " << i;
}

TEST(Straight, YawIsZeroAndYIsConstant) {
    const auto t = adc::MakeStraight(10.0, 0.1, 5.0);
    ASSERT_EQ(t.size(), 101u);
    for (std::size_t i = 0; i < t.size(); ++i) {
        EXPECT_NEAR(t[i].y, 0.0, 1e-12);
        EXPECT_NEAR(t[i].yaw, 0.0, 1e-12);
    }
}

TEST(Straight, XIncreasesMonotonically) {
    const auto t = adc::MakeStraight(10.0, 0.1, 5.0);
    for (std::size_t i = 1; i < t.size(); ++i)
        EXPECT_GT(t[i].x, t[i - 1].x);
}

TEST(Straight, AdjacentSpacingMatchesDs) {
    const double ds = 0.1;
    const auto t = adc::MakeStraight(10.0, ds, 5.0);
    for (std::size_t i = 1; i < t.size(); ++i) {
        const double seg = std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
        EXPECT_NEAR(seg, ds, 1e-6) << "index " << i;
    }
}

// ============================================================
// 三、圆弧:曲率必须等于 1/R
// ============================================================

TEST(Arc, CurvatureEqualsInverseRadius) {
    const double R = 25.0;
    const auto t = adc::MakeArc(R, kPi / 2.0, 0.1, 10.0);
    ASSERT_GT(t.size(), 100u);
    for (std::size_t i = 0; i < t.size(); ++i)
        EXPECT_NEAR(t[i].curvature, 1.0 / R, 1e-9) << "index " << i;
}

TEST(Arc, StartsAtOriginHeadingForward) {
    const auto t = adc::MakeArc(20.0, kPi / 2.0, 0.1, 10.0);
    ASSERT_GT(t.size(), 0u);
    EXPECT_NEAR(t[0].x, 0.0, 1e-12);
    EXPECT_NEAR(t[0].y, 0.0, 1e-12);
    EXPECT_NEAR(t[0].yaw, 0.0, 1e-12);
}

TEST(Arc, QuarterCircleEndsAtExpectedPoint) {
    const double R = 25.0;
    const auto t = adc::MakeArc(R, kPi / 2.0, 0.05, 10.0);
    const auto& last = t[t.size() - 1];
    // 90 度圆弧:x = R, y = R, yaw = 90 度
    EXPECT_NEAR(last.x, R, 1e-6);
    EXPECT_NEAR(last.y, R, 1e-6);
    EXPECT_NEAR(last.yaw, kPi / 2.0, 1e-9);
}

TEST(Arc, AllPointsLieOnSameCircle) {
    const double R = 25.0;
    const auto t = adc::MakeArc(R, kPi, 0.1, 10.0);
    // 圆心在 (0, R)
    for (const auto& p : t.points()) {
        const double dist = std::hypot(p.x - 0.0, p.y - R);
        EXPECT_NEAR(dist, R, 1e-9);
    }
}

TEST(Arc, AdjacentSpacingMatchesDs) {
    const double ds = 0.1;
    const auto t = adc::MakeArc(25.0, kPi / 2.0, ds, 10.0);
    for (std::size_t i = 1; i < t.size(); ++i) {
        const double seg = std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
        EXPECT_NEAR(seg, ds, ds * 0.02) << "index " << i;
    }
}

// ============================================================
// 四、单移线:五次多项式端点性质
// ============================================================
TEST(LaneChange, StartsAtCenterlineAndEndsAtOffset) {
    const auto t = adc::MakeLaneChange(60.0, 3.5, 0.1, 15.0);
    EXPECT_NEAR(t[0].y, 0.0, 1e-12);
    EXPECT_NEAR(t[t.size()-1].y, 3.5, 1e-9);
}

TEST(LaneChange, MidpointIsHalfOffset) {
    const auto t = adc::MakeLaneChange(60.0, 3.5, 0.1, 15.0);
    EXPECT_NEAR(t[t.size()/2].y, 1.75, 1e-6);   // 五次多项式在 u=0.5 恰好是 0.5
}

// ============================================================
// 五、双移线:五次多项式端点性质
// ============================================================

TEST(DoubleLaneChange, StartsAndEndsOnCenterline) {
    const auto t = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    ASSERT_GT(t.size(), 10u);
    EXPECT_NEAR(t[0].y, 0.0, 1e-12);
    EXPECT_NEAR(t[t.size() - 1].y, 0.0, 1e-9);
}

TEST(DoubleLaneChange, StartsAndEndsHeadingStraight) {
    const auto t = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    EXPECT_NEAR(t[0].yaw, 0.0, 1e-12);
    EXPECT_NEAR(t[t.size()/2].yaw, 0.0, 1e-6);  // 拼接处也回正
    EXPECT_NEAR(t[t.size() - 1].yaw, 0.0, 1e-9);
}

TEST(DoubleLaneChange, StartsAndEndsWithZeroCurvature) {
    const auto t = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    EXPECT_NEAR(t[0].curvature, 0.0, 1e-12);
    EXPECT_NEAR(t[t.size() - 1].curvature, 0.0, 1e-9);
}

TEST(DoubleLaneChange, ReachesExpectedLateralOffset) {
    // 五次多项式在 u=0.5 处 y = offset / 2
    const double offset = 3.5;
    const auto t = adc::MakeDoubleLaneChange(60.0, offset, 0.1, 15.0);
    ASSERT_EQ(t.size(), 601u);          // 60/0.1 + 1
    const std::size_t mid = t.size() / 2;
    EXPECT_NEAR(t[mid/2].y, offset / 2.0, 1e-6);
}

TEST(DoubleLaneChange, CurvatureHasBothSigns) {
    // 双移线先向左弯(正曲率)再向右弯(负曲率)
    const auto t = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    double max_k = 0.0, min_k = 0.0;
    for (const auto& p : t.points()) {
        max_k = std::max(max_k, p.curvature);
        min_k = std::min(min_k, p.curvature);
    }
    EXPECT_GT(max_k, 0.0);
    EXPECT_LT(min_k, 0.0);
}

TEST(DoubleLaneChange, MaxCurvatureIsReasonable) {
    // 60 m 内横移 3.5 m,最大曲率应当在 0.01~0.05 1/m 量级
    const auto t = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    double max_abs_k = 0.0;
    for (const auto& p : t.points())
        max_abs_k = std::max(max_abs_k, std::abs(p.curvature));
    EXPECT_GT(max_abs_k, 0.005);
    EXPECT_LT(max_abs_k, 0.06);
}

TEST(DoubleLaneChange, CurvatureIsContinuousAtMidpoint) {
    const auto t = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    const std::size_t mid = t.size() / 2;
    // 中点两侧曲率都应当接近 0
    EXPECT_NEAR(t[mid-1].curvature, 0.0, 1e-3);
    EXPECT_NEAR(t[mid].curvature, 0.0, 1e-5);
    EXPECT_NEAR(t[mid+1].curvature, 0.0, 1e-3);
}

// ============================================================
// 五、正弦:连续变曲率
// ============================================================

TEST(Sine, StartsAtOrigin) {
    const auto t = adc::MakeSine(50.0, 2.0, 2.0, 0.1, 10.0);
    ASSERT_GT(t.size(), 10u);
    EXPECT_NEAR(t[0].x, 0.0, 1e-12);
    EXPECT_NEAR(t[0].y, 0.0, 1e-12);
}

TEST(Sine, CurvatureChangesSign) {
    const auto t = adc::MakeSine(50.0, 2.0, 2.0, 0.1, 10.0);
    double max_k = 0.0, min_k = 0.0;
    for (const auto& p : t.points()) {
        max_k = std::max(max_k, p.curvature);
        min_k = std::min(min_k, p.curvature);
    }
    EXPECT_GT(max_k, 1e-4);
    EXPECT_LT(min_k, -1e-4);
}

TEST(Sine, AdjacentSpacingIsApproximatelyUniform) {
    const double ds = 0.1;
    const auto t = adc::MakeSine(50.0, 2.0, 2.0, ds, 10.0);
    for (std::size_t i = 1; i < t.size(); ++i) {
        const double seg = std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
        EXPECT_NEAR(seg, ds, ds * 0.02) << "index " << i;
    }
}

// ============================================================
// 六、全局健全性
// ============================================================

TEST(Trajectory, AllPointsAreFinite) {
    for (const auto& t : {adc::MakeStraight(20.0, 0.1, 8.0),
                          adc::MakeArc(25.0, kPi / 2.0, 0.1, 10.0),
                          adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0),
                          adc::MakeSine(50.0, 2.0, 2.0, 0.1, 10.0)}) {
        for (const auto& p : t.points()) {
            EXPECT_TRUE(std::isfinite(p.x));
            EXPECT_TRUE(std::isfinite(p.y));
            EXPECT_TRUE(std::isfinite(p.yaw));
            EXPECT_TRUE(std::isfinite(p.curvature));
        }
    }
}

TEST(Trajectory, InvalidParametersProduceEmptyTrajectory) {
    EXPECT_TRUE(adc::MakeStraight(0.0, 0.1, 5.0).empty());
    EXPECT_TRUE(adc::MakeStraight(10.0, 0.0, 5.0).empty());
    EXPECT_TRUE(adc::MakeArc(-1.0, kPi, 0.1, 5.0).empty());
    EXPECT_TRUE(adc::MakeDoubleLaneChange(0.0, 3.5, 0.1, 15.0).empty());
    EXPECT_TRUE(adc::MakeSine(50.0, 2.0, 0.0, 0.1, 10.0).empty());
}
