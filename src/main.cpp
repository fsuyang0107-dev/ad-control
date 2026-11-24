#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "adc/metrics.hpp"
#include "adc/sim_loop.hpp"
#include "adc/trajectory.hpp"
#include "adc/types.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

void PrintUsage() {
    std::printf(
        "用法: adc_sim [选项]\n"
        "  --scenario <name>   场景: straight | arc | dlc | sine (默认 dlc)\n"
        "  --ctrl <name>       控制器: ff | pp | stanley (默认 pp)\n"
        "  --model <kind>      车辆模型: kin | dyn (默认 kin)\n"
        "  --compare           依次跑三种控制器并输出对比表\n"
        "  --dt <sec>          仿真步长 (默认 0.01)\n"
        "  --v <m/s>           参考速度 (默认 15)\n"
        "  --init-y <m>        初始横向偏差 (默认 0)\n"
        "  --init-yaw <rad>    初始航向偏差 (默认 0)\n"
        "  --csv <path>        输出 CSV 路径 (可选)\n"
        "  --help              显示本帮助\n");
}

adc::Trajectory BuildScenario(const std::string& name, double v) {
    const double ds = 0.1;
    if (name == "straight") return adc::MakeStraight(20.0, ds, v);
    if (name == "arc") return adc::MakeArc(25.0, kPi / 2.0, ds, v);
    if (name == "sine") return adc::MakeSine(50.0, 2.0, 2.0, ds, v);
    return adc::MakeDoubleLaneChange(60.0, 3.5, ds, v);
}

/// 沿速度剖面累加行程时间
double TravelTime(const adc::Trajectory& t) {
    if (t.size() < 2) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 1; i < t.size(); ++i) {
        const double ds = std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
        sum += ds / std::max(0.5 * (t[i].v_ref + t[i - 1].v_ref), 0.5);
    }
    return sum;
}

bool ParseMode(const std::string& s, adc::CtrlMode& out) {
    if (s == "ff" || s == "feedforward") { out = adc::CtrlMode::kFeedForward; return true; }
    if (s == "pp" || s == "pure_pursuit") { out = adc::CtrlMode::kPurePursuit; return true; }
    if (s == "stanley") { out = adc::CtrlMode::kStanley; return true; }
    return false;
}

struct RunOutcome {
    adc::Metrics metrics;
    double ss_max{0.0};
    double settle{0.0};
    std::vector<adc::SimRecord> records;
};

RunOutcome RunOne(const adc::Trajectory& traj, const adc::SimConfig& base_cfg,
                  adc::CtrlMode mode) {
    adc::SimConfig cfg = base_cfg;
    cfg.ctrl_mode = mode;
    adc::SimLoop loop(cfg);
    RunOutcome out;
    out.records = loop.Run(traj);
    out.metrics = adc::ComputeMetrics(out.records);

    if (!out.records.empty()) {
        const std::size_t ss_start = out.records.size() * 8 / 10;
        for (std::size_t i = ss_start; i < out.records.size(); ++i)
            out.ss_max = std::max(out.ss_max, std::abs(out.records[i].e_y));

        out.settle = out.records.back().t;
        for (std::size_t i = out.records.size(); i-- > 0;)
            if (std::abs(out.records[i].e_y) >= 0.05) { out.settle = out.records[i].t; break; }
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenario = "dlc";
    std::string csv_path;
    adc::CtrlMode mode = adc::CtrlMode::kPurePursuit;
    bool compare = false;
    double dt = 0.01;
    double v = 15.0;
    double init_y = 0.0;
    double init_yaw = 0.0;
    double a_lat_max = 0.0;
    adc::ModelKind model = adc::ModelKind::kKinematic;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto nextd = [&](double& dst) { if (i + 1 < argc) dst = std::atof(argv[++i]); };
        if (a == "--scenario" && i + 1 < argc) scenario = argv[++i];
        else if (a == "--ctrl" && i + 1 < argc) {
            if (!ParseMode(argv[++i], mode)) {
                std::fprintf(stderr, "未知控制器: %s\n", argv[i]);
                return 1;
            }
        } else if (a == "--compare") compare = true;
        else if (a == "--dt") nextd(dt);
        else if (a == "--v") nextd(v);
        else if (a == "--init-y") nextd(init_y);
        else if (a == "--init-yaw") nextd(init_yaw);
        else if (a == "--alat") nextd(a_lat_max);
        else if (a == "--model" && i + 1 < argc) {
            const std::string m = argv[++i];
            if (m == "dyn" || m == "dynamic") model = adc::ModelKind::kDynamic;
            else if (m == "kin" || m == "kinematic") model = adc::ModelKind::kKinematic;
            else { std::fprintf(stderr, "未知模型: %s\n", m.c_str()); return 1; }
        }
        else if (a == "--csv" && i + 1 < argc) csv_path = argv[++i];
        else if (a == "--help" || a == "-h") { PrintUsage(); return 0; }
    }

    adc::Trajectory traj = BuildScenario(scenario, v);
    if (a_lat_max > 0.0) {
        adc::ApplySpeedLimit(traj, v, a_lat_max, 4.0, 2.0);
    }
    if (traj.empty()) {
        std::fprintf(stderr, "错误: 场景 %s 生成了空轨迹\n", scenario.c_str());
        return 1;
    }

    adc::SimConfig cfg;
    cfg.dt = dt;
    cfg.scenario = scenario;
    cfg.init_y_offset = init_y;
    cfg.init_yaw_offset = init_yaw;
    cfg.model = model;
    cfg.duration = TravelTime(traj);

    std::printf("场景: %s | 轨迹长度: %.3f m | v: %.2f m/s | 时长: %.3f s\n", scenario.c_str(),
                traj.length(), v, cfg.duration);
    std::printf("步长: %.4f s | 模型: %s | 初始横向偏差: %.3f m | 初始航向偏差: %.4f rad\n\n",
                dt, model == adc::ModelKind::kDynamic ? "dynamic" : "kinematic", init_y,
                init_yaw);

    if (compare) {
        std::printf("%-14s %10s %10s %10s %10s %10s %14s\n", "控制器", "rmse_y", "max|e_y|",
                    "稳态|e_y|", "rmse_yaw", "调节时间", "最大转角");
        std::printf("%s\n", std::string(88, '-').c_str());
        for (auto m : {adc::CtrlMode::kFeedForward, adc::CtrlMode::kPurePursuit,
                       adc::CtrlMode::kStanley}) {
            const auto r = RunOne(traj, cfg, m);
            double max_steer = 0.0;
            for (const auto& rec : r.records)
                max_steer = std::max(max_steer, std::abs(rec.cmd.steer));
            std::printf("%-14s %10.5f %10.5f %10.5f %10.5f %8.2fs %10.2f deg\n",
                        adc::CtrlModeName(m).c_str(), r.metrics.rmse_y, r.metrics.max_abs_y,
                        r.ss_max, r.metrics.rmse_yaw, r.settle, max_steer * 180.0 / kPi);
        }
        std::printf("\n稳态|e_y| 为最后 20%% 时段的最大横向误差;调节时间为 |e_y| 最后一次超过 5 cm 的时刻。\n");
        return 0;
    }

    const auto r = RunOne(traj, cfg, mode);
    std::printf("控制器: %s\n\n", adc::CtrlModeName(mode).c_str());
    std::printf("=== 指标 ===\n");
    adc::PrintMetrics(r.metrics);
    std::printf("  稳态|e_y|  : %.6f m\n", r.ss_max);
    std::printf("  调节时间   : %.3f s\n", r.settle);

    if (!csv_path.empty()) {
        if (adc::WriteCsv(r.records, csv_path)) {
            std::printf("\nCSV 已写入: %s(%zu 行)\n", csv_path.c_str(), r.records.size());
        } else {
            std::fprintf(stderr, "\n写入 CSV 失败: %s\n", csv_path.c_str());
            return 1;
        }
    }
    return 0;
}
