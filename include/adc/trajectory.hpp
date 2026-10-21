#pragma once

#include <cstddef>
#include <vector>

#include "adc/types.hpp"

namespace adc {

/// 参考轨迹:一串带位置、朝向、参考速度、曲率的点
class Trajectory {
public:
    std::size_t size() const { return pts_.size(); }
    bool empty() const { return pts_.empty(); }
    void push_back(const TrajPoint& p) { pts_.push_back(p); }
    const TrajPoint& at(std::size_t i) const { return pts_.at(i); }
    const TrajPoint& operator[](std::size_t i) const { return pts_[i]; }
    const std::vector<TrajPoint>& points() const { return pts_; }
    void reserve(std::size_t n) { pts_.reserve(n); }

    /// 沿轨迹的累计长度,m
    double length() const;

private:
    std::vector<TrajPoint> pts_;
};

// ---- 四种标准场景生成器 ----

/// 直线:从原点沿 +x 方向
Trajectory MakeStraight(double length, double ds, double v_ref);

/// 圆弧:左转(正曲率),半径 radius
Trajectory MakeArc(double radius, double sweep_rad, double ds, double v_ref);

///单移线
Trajectory MakeLaneChange(double length, double offset, double ds, double v_ref);

/// 双移线:沿 x 方向前进 length,横向偏移 offset(五次多项式)
Trajectory MakeDoubleLaneChange(double length, double offset, double ds, double v_ref);

/// 正弦:s 形,振幅 amplitude,periods 个完整周期
Trajectory MakeSine(double length, double amplitude, double periods, double ds,
                    double v_ref);

}  // namespace adc
