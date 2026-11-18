#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "adc/metrics.hpp"
#include "adc/types.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;

adc::SimRecord MakeRec(double t, double e_y, double e_yaw, double accel = 0.0) {
    adc::SimRecord r;
    r.t = t;
    r.e_y = e_y;
    r.e_yaw = e_yaw;
    r.state.accel = accel;   // jerk 用实际加速度
    return r;
}
}  // namespace

TEST(NormalizeAngle, ZeroStaysZero) {
    EXPECT_DOUBLE_EQ(adc::NormalizeAngle(0.0), 0.0);
}

TEST(NormalizeAngle, LeavesInRangeValuesAlone) {
    EXPECT_NEAR(adc::NormalizeAngle(0.5), 0.5, 1e-12);
    EXPECT_NEAR(adc::NormalizeAngle(-0.5), -0.5, 1e-12);
}

TEST(NormalizeAngle, WrapsValuesBeyondPi) {
    EXPECT_NEAR(adc::NormalizeAngle(2.0 * kPi), 0.0, 1e-12);
    EXPECT_NEAR(adc::NormalizeAngle(3.0 * kPi), kPi, 1e-12);
    EXPECT_NEAR(adc::NormalizeAngle(-3.0 * kPi), kPi, 1e-12);
}

TEST(NormalizeAngle, HandlesSixRadians) {
    EXPECT_NEAR(adc::NormalizeAngle(6.0), 6.0 - 2.0 * kPi, 1e-12);
}

TEST(NormalizeAngle, ResultAlwaysWithinPi) {
    for (double a = -20.0; a <= 20.0; a += 0.37) {
        const double n = adc::NormalizeAngle(a);
        EXPECT_LE(n, kPi + 1e-12);
        EXPECT_GE(n, -kPi - 1e-12);
    }
}

TEST(Metrics, EmptyInputGivesZeroes) {
    const auto m = adc::ComputeMetrics({});
    EXPECT_EQ(m.steps, 0u);
    EXPECT_DOUBLE_EQ(m.rmse_y, 0.0);
    EXPECT_DOUBLE_EQ(m.max_abs_y, 0.0);
}

TEST(Metrics, ZeroErrorsGiveZeroRmse) {
    std::vector<adc::SimRecord> recs;
    for (int i = 0; i < 10; ++i) recs.push_back(MakeRec(0.1 * i, 0.0, 0.0));
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_EQ(m.steps, 10u);
    EXPECT_NEAR(m.rmse_y, 0.0, 1e-15);
    EXPECT_NEAR(m.max_abs_y, 0.0, 1e-15);
}

TEST(Metrics, ConstantErrorGivesThatRmse) {
    std::vector<adc::SimRecord> recs;
    for (int i = 0; i < 10; ++i) recs.push_back(MakeRec(0.1 * i, 0.3, 0.0));
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_NEAR(m.rmse_y, 0.3, 1e-12);
    EXPECT_NEAR(m.max_abs_y, 0.3, 1e-12);
}

TEST(Metrics, AlternatingSignsDoNotCancelOut) {
    std::vector<adc::SimRecord> recs;
    for (int i = 0; i < 10; ++i) {
        recs.push_back(MakeRec(0.1 * i, (i % 2 == 0) ? 0.3 : -0.3, 0.0));
    }
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_NEAR(m.rmse_y, 0.3, 1e-12);
    EXPECT_NEAR(m.max_abs_y, 0.3, 1e-12);
}

TEST(Metrics, MaxCapturesSingleSpike) {
    std::vector<adc::SimRecord> recs;
    for (int i = 0; i < 100; ++i) recs.push_back(MakeRec(0.02 * i, 0.01, 0.0));
    recs[50].e_y = 1.0;
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_NEAR(m.max_abs_y, 1.0, 1e-12);
    EXPECT_GT(m.rmse_y, 0.05);
    EXPECT_LT(m.rmse_y, 0.15);
}

TEST(Metrics, YawErrorIsTrackedSeparately) {
    std::vector<adc::SimRecord> recs;
    for (int i = 0; i < 10; ++i) recs.push_back(MakeRec(0.1 * i, 0.0, 0.2));
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_NEAR(m.rmse_y, 0.0, 1e-15);
    EXPECT_NEAR(m.rmse_yaw, 0.2, 1e-12);
}

TEST(Metrics, JerkComputedFromAccelSteps) {
    std::vector<adc::SimRecord> recs;
    recs.push_back(MakeRec(0.0, 0.0, 0.0, 0.0));
    recs.push_back(MakeRec(0.1, 0.0, 0.0, 1.0));
    recs.push_back(MakeRec(0.2, 0.0, 0.0, 1.0));
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_NEAR(m.max_jerk, 10.0, 1e-9);
}

TEST(Csv, WritesHeaderAndRows) {
    std::vector<adc::SimRecord> recs;
    for (int i = 0; i < 5; ++i) recs.push_back(MakeRec(0.1 * i, 0.01 * i, 0.0));

    const std::string path = "/tmp/adc_test_metrics.csv";
    ASSERT_TRUE(adc::WriteCsv(recs, path));

    std::FILE* f = std::fopen(path.c_str(), "r");
    ASSERT_NE(f, nullptr);
    char line[512];
    int lines = 0;
    while (std::fgets(line, sizeof(line), f) != nullptr) ++lines;
    std::fclose(f);

    EXPECT_EQ(lines, 6);
}

TEST(Csv, HeaderHasExpectedColumns) {
    std::vector<adc::SimRecord> recs{MakeRec(0.0, 0.0, 0.0)};
    const std::string path = "/tmp/adc_test_header.csv";
    ASSERT_TRUE(adc::WriteCsv(recs, path));

    std::FILE* f = std::fopen(path.c_str(), "r");
    ASSERT_NE(f, nullptr);
    char header[512];
    ASSERT_NE(std::fgets(header, sizeof(header), f), nullptr);
    std::fclose(f);

    const std::string h(header);
    for (const char* col : {"t", "x", "y", "yaw", "e_y", "e_yaw", "v_ref"}) {
        EXPECT_NE(h.find(col), std::string::npos) << "缺少列: " << col;
    }
}

TEST(Csv, ReturnsFalseForInvalidPath) {
    std::vector<adc::SimRecord> recs{MakeRec(0.0, 0.0, 0.0)};
    EXPECT_FALSE(adc::WriteCsv(recs, "/nonexistent_dir_xyz/out.csv"));
}
