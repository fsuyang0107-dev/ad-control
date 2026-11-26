#include "adc/mpc.hpp"

#include <algorithm>
#include <cmath>

namespace adc {
namespace {

using Mat4 = std::vector<std::vector<double>>;   // 4x4
using Pow = std::vector<Mat4>;                   // Ad^0 .. Ad^N

Mat4 Zero4() { return Mat4(4, std::vector<double>(4, 0.0)); }

/// out = M · v
void MulVec(const Mat4& M, const double v[4], double out[4]) {
    for (int i = 0; i < 4; ++i) {
        double sum = 0.0;
        for (int m = 0; m < 4; ++m) sum += M[i][m] * v[m];
        out[i] = sum;
    }
}

/// out = M1 · M2
void MulMat(const Mat4& M1, const Mat4& M2, Mat4& out) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int m = 0; m < 4; ++m) sum += M1[i][m] * M2[m][j];
            out[i][j] = sum;
        }
}

void ToArray(const MpcState& x, double out[4]) {
    out[0] = x.e_y;
    out[1] = x.e_yaw;
    out[2] = x.v_y;
    out[3] = x.r;
}

}  // namespace

void MpcLinearize(const VehicleParams& p, double vx, double A[4][4], double B[4],
                  double d0[4]) {
    // 线性化工作点下限:避免 1/v 病态
    const double v = std::max(vx, 2.0);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) A[i][j] = 0.0;
        B[i] = 0.0;
        d0[i] = 0.0;
    }

    const double a11 = -(p.Cf + p.Cr) / (p.mass * v);
    const double a12 = -v - (p.lf * p.Cf - p.lr * p.Cr) / (p.mass * v);
    const double a21 = -(p.lf * p.Cf - p.lr * p.Cr) / (p.Iz * v);
    const double a22 = -(p.lf * p.lf * p.Cf + p.lr * p.lr * p.Cr) / (p.Iz * v);

    // ė_y = v·e_yaw + (v_y − l_r·r)
    //   ★ e_y 定义在后轴,而 v_y 是重心侧向速度,
    //     后轴横向速度 = v_y − l_r·r(刚体运动学,不是笔误)
    A[0][1] = v;
    A[0][2] = 1.0;
    A[0][3] = -p.lr;
    // ė_yaw = r − v·κ      ← 曲率作为已知扰动进入
    A[1][3] = 1.0;
    d0[1] = -v;
    // v̇_y = a11·v_y + a12·r + (C_f/m)·δ
    A[2][2] = a11;
    A[2][3] = a12;
    B[2] = p.Cf / p.mass;
    // ṙ = a21·v_y + a22·r + (l_f·C_f/I_z)·δ
    A[3][2] = a21;
    A[3][3] = a22;
    B[3] = p.lf * p.Cf / p.Iz;
}

bool CholeskySolve(std::vector<std::vector<double>>& H, std::vector<double>& b,
                   std::vector<double>& u) {
    const int n = static_cast<int>(H.size());
    u.assign(n, 0.0);

    for (int j = 0; j < n; ++j) {
        double sum = H[j][j];
        for (int k = 0; k < j; ++k) sum -= H[j][k] * H[j][k];
        if (sum <= 1e-14) return false;
        H[j][j] = std::sqrt(sum);
        for (int i = j + 1; i < n; ++i) {
            double s2 = H[i][j];
            for (int k = 0; k < j; ++k) s2 -= H[i][k] * H[j][k];
            H[i][j] = s2 / H[j][j];
        }
    }

    std::vector<double> y(n, 0.0);
    for (int i = 0; i < n; ++i) {
        double s = b[i];
        for (int k = 0; k < i; ++k) s -= H[i][k] * y[k];
        y[i] = s / H[i][i];
    }
    for (int i = n - 1; i >= 0; --i) {
        double s = y[i];
        for (int k = i + 1; k < n; ++k) s -= H[k][i] * u[k];
        u[i] = s / H[i][i];
    }
    return true;
}

namespace {

/// 已分解的对称正定系统:Cholesky 因子存下三角
struct Factor {
    std::vector<std::vector<double>> L;
};

bool Factorize(const std::vector<std::vector<double>>& H, Factor& f) {
    const int n = static_cast<int>(H.size());
    f.L.assign(n, std::vector<double>(n, 0.0));
    for (int j = 0; j < n; ++j) {
        double sum = H[j][j];
        for (int k = 0; k < j; ++k) sum -= f.L[j][k] * f.L[j][k];
        if (sum <= 1e-14) return false;
        f.L[j][j] = std::sqrt(sum);
        for (int i = j + 1; i < n; ++i) {
            double s2 = H[i][j];
            for (int k = 0; k < j; ++k) s2 -= f.L[i][k] * f.L[j][k];
            f.L[i][j] = s2 / f.L[j][j];
        }
    }
    return true;
}

void SolveFactored(const Factor& f, const std::vector<double>& b,
                   std::vector<double>& x) {
    const int n = static_cast<int>(f.L.size());
    x.assign(n, 0.0);
    std::vector<double> y(n, 0.0);
    for (int i = 0; i < n; ++i) {
        double s = b[i];
        for (int k = 0; k < i; ++k) s -= f.L[i][k] * y[k];
        y[i] = s / f.L[i][i];
    }
    for (int i = n - 1; i >= 0; --i) {
        double s = y[i];
        for (int k = i + 1; k < n; ++k) s -= f.L[k][i] * x[k];
        x[i] = s / f.L[i][i];
    }
}

}  // namespace

bool BoxQp(const std::vector<std::vector<double>>& H, const std::vector<double>& f,
           const std::vector<double>& lb, const std::vector<double>& ub,
           std::vector<double>& u, int max_iter, double tol) {
    const int n = static_cast<int>(H.size());
    if (n == 0) return false;

    // min 0.5uᵀHu + fᵀu + I_box(z),s.t. u = z
    const double rho = 1.0;
    std::vector<std::vector<double>> Hp = H;
    for (int i = 0; i < n; ++i) Hp[i][i] += rho;

    Factor fac;
    if (!Factorize(Hp, fac)) return false;

    std::vector<double> z(n, 0.0), lam(n, 0.0), rhs(n, 0.0), uu(n, 0.0);
    bool converged = false;

    for (int it = 0; it < max_iter; ++it) {
        for (int i = 0; i < n; ++i) rhs[i] = -f[i] + rho * (z[i] - lam[i]);
        SolveFactored(fac, rhs, uu);

        double rnorm = 0.0, snorm = 0.0, zprev_norm = 0.0;
        for (int i = 0; i < n; ++i) {
            const double zn = std::clamp(uu[i] + lam[i], lb[i], ub[i]);
            snorm += (zn - z[i]) * (zn - z[i]);
            zprev_norm += z[i] * z[i];
            z[i] = zn;
            lam[i] += uu[i] - z[i];
            rnorm += (uu[i] - z[i]) * (uu[i] - z[i]);
        }
        if (std::sqrt(rnorm) < tol && std::sqrt(snorm) < tol) {
            converged = true;
            break;
        }
    }
    u = z;
    return converged;
}

ControlCmd LateralMpc::Compute(const VehicleState& s, const MpcState& x,
                               const std::vector<double>& kappa, const VehicleParams& p,
                               MpcInfo* info) {
    ControlCmd cmd;
    const int N = std::max(1, params_.horizon);
    const double tau = params_.tau;

    // ---- 1. 线性化 + 前向欧拉离散化 ----
    double A[4][4], B[4], d0[4];
    MpcLinearize(p, s.v, A, B, d0);

    Mat4 Ad = Zero4();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) Ad[i][j] = A[i][j] * tau + (i == j ? 1.0 : 0.0);

    double Bd[4], Dd[4];
    for (int i = 0; i < 4; ++i) {
        Bd[i] = B[i] * tau;
        Dd[i] = d0[i] * tau;
    }

    // ---- 2. Ad 的幂 ----
    Pow Adk(N + 1, Zero4());
    for (int i = 0; i < 4; ++i) Adk[0][i][i] = 1.0;
    for (int k = 1; k <= N; ++k) MulMat(Adk[k - 1], Ad, Adk[k]);

    // ---- 3. 自由响应与输入敏感度 ----
    double x0[4];
    ToArray(x, x0);

    std::vector<std::vector<double>> free_x(N, std::vector<double>(4, 0.0));
    std::vector<std::vector<std::vector<double>>> S(
        N, std::vector<std::vector<double>>(N, std::vector<double>(4, 0.0)));

    for (int k = 0; k < N; ++k) {
        double tmp[4];
        MulVec(Adk[k + 1], x0, tmp);
        for (int i = 0; i < 4; ++i) free_x[k][i] = tmp[i];

        // 曲率扰动(已知参考)
        for (int j = 0; j <= k; ++j) {
            const double kk =
                kappa.empty() ? 0.0 : kappa[std::min<std::size_t>(j, kappa.size() - 1)];
            double acc[4];
            MulVec(Adk[k - j], Dd, acc);
            for (int i = 0; i < 4; ++i) free_x[k][i] += acc[i] * kk;
        }

        // 输入的敏感度
        for (int j = 0; j <= k; ++j) {
            double acc[4];
            MulVec(Adk[k - j], Bd, acc);
            for (int i = 0; i < 4; ++i) S[k][j][i] = acc[i];
        }
    }

    // ---- 4. H, f ----
    std::vector<std::vector<double>> H(N, std::vector<double>(N, 0.0));
    std::vector<double> f(N, 0.0);

    for (int k = 0; k < N; ++k) {
        const bool terminal = (k == N - 1);
        const double sc = terminal ? params_.qf_scale : 1.0;
        const double qv[4] = {params_.q_ey * sc, params_.q_eyaw * sc, params_.q_vy * sc,
                              params_.q_r * sc};

        for (int a = 0; a <= k; ++a) {
            double fa = 0.0;
            for (int i = 0; i < 4; ++i) fa += S[k][a][i] * qv[i] * free_x[k][i];
            f[a] += fa;
            for (int b = 0; b <= k; ++b) {
                double hab = 0.0;
                for (int i = 0; i < 4; ++i) hab += S[k][a][i] * qv[i] * S[k][b][i];
                H[a][b] += hab;
            }
        }
    }
    for (int i = 0; i < N; ++i) H[i][i] += params_.r_steer;

    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            const double m = 0.5 * (H[i][j] + H[j][i]);
            H[i][j] = m;
            H[j][i] = m;
        }

    // ---- 5. 求解 ----
    std::vector<double> negf(N, 0.0);
    for (int i = 0; i < N; ++i) negf[i] = -f[i];

    std::vector<double> u;
    bool ok = false;
    if (params_.use_constraints) {
        // |δ_k| ≤ max_steer(在优化里处理,而非事后裁剪)
        std::vector<double> lb(N, -p.max_steer), ub(N, p.max_steer);
        ok = BoxQp(H, f, lb, ub, u);
        if (!ok && static_cast<int>(u.size()) != N) {
            // ADMM 未收敛但没有解 → 退回无约束
            std::vector<std::vector<double>> Hc = H;
            ok = CholeskySolve(Hc, negf, u);
        }
    } else {
        std::vector<std::vector<double>> Hc = H;
        ok = CholeskySolve(Hc, negf, u);
    }

    double cost = 0.0;
    for (int i = 0; i < N; ++i) {
        cost += f[i] * u[i];
        for (int j = 0; j < N; ++j) cost += 0.5 * u[i] * H[i][j] * u[j];
    }

    const double raw = u.empty() ? 0.0 : u[0];
    const double clamped = std::clamp(raw, -p.max_steer, p.max_steer);

    if (info) {
        info->ok = ok;
        info->cost = cost;
        info->steer = raw;
        info->saturated = (std::abs(raw) > p.max_steer);
    }

    cmd.steer = clamped;
    cmd.accel = 0.0;
    return cmd;
}

}  // namespace adc
