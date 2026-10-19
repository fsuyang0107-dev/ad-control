#pragma once 
#include<cstddef>
#include<functional>
#include<vector>
namespace adc {

/// 四阶龙格-库塔(RK4)单步积分。
/// s  : 状态向量,原地更新
/// dt : 步长
/// f  : 导数函数,输入状态返回 d(state)/dt
inline void Rk4Step(std::vector<double>& s, double dt,
                    const std::function<std::vector<double>(const std::vector<double>&)>& f) {
    const std::size_t n = s.size();
    std::vector<double> tmp(n);

    const std::vector<double> k1 = f(s);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = s[i] + 0.5 * dt * k1[i];

    const std::vector<double> k2 = f(tmp);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = s[i] + 0.5 * dt * k2[i];

    const std::vector<double> k3 = f(tmp);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = s[i] + dt * k3[i];

    const std::vector<double> k4 = f(tmp);
    for (std::size_t i = 0; i < n; ++i)
        s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
}

}  // namespace adc
