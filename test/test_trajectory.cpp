#include <gtest/gtest.h>
#include "adc/trajectory.hpp"

TEST(TrajectorySmoke, EmptyTrajectoryHasZeroSize) {
    adc::Trajectory t;
    EXPECT_EQ(t.size(), 0u);
}

TEST(TrajectorySmoke, PushBackIncreasesSize) {
    adc::Trajectory t;
    adc::TrajPoint p;
    p.x = 1.0;
    t.push_back(p);
    EXPECT_EQ(t.size(), 1u);
    EXPECT_DOUBLE_EQ(t.at(0).x, 1.0);
}
