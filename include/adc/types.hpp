#pragma once
#include <string>
namespace adc {
struct VehicleState {
    double x{0.0}, y{0.0};
    double yaw{0.0};
    double v{0.0};
    double yaw_rate{0.0};
    double curvature{0.0};
};

struct ControlCmd{
    double steer{0.0};//Front wheel angle,left is positive
    double accel{0.0};
};
struct VehicleParams{
    double wheelbase{2.7};
    double mass{1500.0};
    double lf{1.2};//center of mass to front axle
    double lr{1.5};
    double Cf{80000.0};//Front axle cornering stiffness N/rad
    double Cr{80000.0};
    double Iz{2500.0};//moment of inertia
    double max_steer{0.5236};//rad=30degree
    double max_accel{3.0};
    double max_decel{-5.0};
};
struct TrajPoint{
    double x{0.0},y{0.0},yaw{0.0};
    double v_ref{0.0};
    double curvature{0.0};
};
struct SimRecord{
    double t{0.0};
    VehicleState state;
    ControlCmd cmd;
    double e_y{0.0};
    double e_yaw{0.0};
};
struct SimConfig{
    double dt{0.02};//50Hz
    double duration{20.0};
    std::string scenario{"straight"};
};
}  // namespace adc
