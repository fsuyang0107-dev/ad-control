#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "adc/types.hpp"
#include "adc/vehicle_model.hpp"

namespace {

double AnalyticX(double x0, double y0, double yaw0, double v, double steer, double L, double t) {
    const double w = v * std::tan(steer) / L;
    return x0 + (v / w) * (std::sin(yaw0 + w * t) - std::sin(yaw0));
}

double AnalyticY(double x0, double y0, double yaw0, double v, double steer, double L, double t) {
    const double w = v * std::tan(steer) / L;
    return y0 - (v / w) * (std::cos(yaw0 + w * t) - std::cos(yaw0));
}

}  // namespace

int main() {
    adc::VehicleParams p;
    adc::VehicleModel model(p);

    const double v = 10.0;
    const double steer = 0.1;
    const double t_total = 10.0;

    const double ex = AnalyticX(0, 0, 0, v, steer, p.wheelbase, t_total);
    const double ey = AnalyticY(0, 0, 0, v, steer, p.wheelbase, t_total);

    std::printf("%-8s %-10s %16s %16s %10s\n",
                "dt", "method", "x_error(m)", "y_error(m)", "ratio");
    std::printf("----------------------------------------------------------------------\n");

    double prev_euler = 0.0;

    for (double dt : {0.1, 0.05, 0.02, 0.01}) {
        const int steps = static_cast<int>(std::lround(t_total / dt));

        adc::VehicleState s_rk4;
        s_rk4.v = v;
        adc::ControlCmd u;
        u.steer = steer;

        adc::VehicleState s_eul = s_rk4;

        for (int i = 0; i < steps; ++i) {
            model.StepKinematic(s_rk4, u, dt);
            model.StepKinematicEuler(s_eul, u, dt);
        }

        const double err_rk4 = std::hypot(s_rk4.x - ex, s_rk4.y - ey);
        const double err_eul = std::hypot(s_eul.x - ex, s_eul.y - ey);

        const double ratio = (prev_euler > 0.0) ? prev_euler / err_eul : 0.0;
        prev_euler = err_eul;

        std::printf("%-8.3f %-10s %16.3e %16.3e\n", dt, "Euler", err_eul, 0.0);
        std::printf("%-8.3f %-10s %16.3e %16.3e %10.2f\n", dt, "RK4", err_rk4, 0.0, ratio);
    }

    return 0;
}
