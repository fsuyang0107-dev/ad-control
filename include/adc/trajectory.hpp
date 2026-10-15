#pragma once
#include <vector>
#include "adc/types.hpp"

namespace adc {
class Trajectory {
public:
    std::size_t size() const { return pts_.size(); }
    void push_back(const TrajPoint& p) { pts_.push_back(p); }
    const TrajPoint& at(std::size_t i) const { return pts_.at(i); }
    const std::vector<TrajPoint>& points() const { return pts_; }
private:
    std::vector<TrajPoint> pts_;
};
}  // namespace adc
