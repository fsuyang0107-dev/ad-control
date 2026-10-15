#pragma once

namespace adc {
struct TrajPoint {
    double x{0.0}, y{0.0}, yaw{0.0};
    double v_ref{0.0};
    double curvature{0.0};
};
}  // namespace adc
