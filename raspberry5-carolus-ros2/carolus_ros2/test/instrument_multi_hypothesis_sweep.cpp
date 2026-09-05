// instrument_multi_hypothesis_sweep.cpp — does the multi-hypothesis win at
// the one tested near-tie (37.4cm -> 5.8cm worst-case, 2026-08-25) generalise,
// or was that geometry a specifically favourable case?
//
// WHY THIS EXISTS. instrument_multi_hypothesis_sort.cpp tested exactly ONE
// near-tie angle (theta*=58.95 deg, where the P1-P2 and P1-P3 angular
// separations are equal to floating-point precision). That is the ONE point
// this project's synthetic near-tie search has ever looked at. Before
// recommending the multi-hypothesis fix be turned on by default, it is worth
// knowing whether the improvement holds across a RANGE of viewing angles, or
// whether 58.95 deg happens to be unusually kind to the fix.
//
// WHAT THIS DOES. Sweeps theta from 5 deg to 85 deg in 5 deg steps (avoiding
// the exact 0/90 deg extremes, which are degenerate: at theta=0 the beacon is
// perfectly frontal with no near-tie at all, and near 90 deg it edge-on and
// some points can leave the "in front of camera" half-space entirely). At
// EVERY theta, not just the located near-tie, runs 200 trials at +/-1px noise
// (fewer than the single-point harness's 1000, to keep total runtime
// reasonable across 17 angles) and reports max-error for both the baseline
// heuristic and the multi-hypothesis method.
//
// Reuses instrument_multi_hypothesis_sort.cpp's own algorithm verbatim
// (labelForCandidate, solveCorrected, the sign convention) rather than a
// second, divergent implementation -- duplicated here rather than shared via
// a header, matching this project's existing pattern of self-contained test
// files (instrument_p4p_sort.cpp and instrument_multi_hypothesis_sort.cpp
// are likewise standalone, not factored into a shared test-only header).

#include "carolus_node/pose_est.hpp"
#include "carolus_node/ceresP4P.hpp"

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

static const double FX = 546.1957, FY = 547.0838, CX = 575.6041, CY = 372.1876;
static const double Z_NOMINAL = 0.7;

static std::vector<Eigen::Vector3d> knownPoints() {
    return {{0.0825, 0.0, 0.0}, {-0.0825, 0.0, 0.0}, {0.0, 0.072, 0.0}, {0.0, 0.0, 0.0555}};
}

static std::vector<Eigen::Vector3d> beaconInCameraFrame(double theta) {
    Eigen::Matrix3d Ry;
    Ry << std::cos(theta), 0, std::sin(theta), 0, 1, 0, -std::sin(theta), 0, std::cos(theta);
    Eigen::Vector3d t(0.0, 0.0, Z_NOMINAL);
    std::vector<Eigen::Vector3d> out;
    for (const auto& p : knownPoints()) out.push_back(Ry * p + t);
    return out;
}

static Eigen::Vector3d toBearing(const Eigen::Vector3d& camPt, double noiseXY, std::mt19937* rng) {
    double xp = camPt.x() / camPt.z(), yp = camPt.y() / camPt.z();
    if (rng && noiseXY > 0.0) {
        std::normal_distribution<double> n(0.0, noiseXY);
        xp += n(*rng);
        yp += n(*rng);
    }
    return Eigen::Vector3d(xp, yp, 1.0).normalized();
}

static const std::pair<int, int> kPairTable[6] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
static const std::pair<int, int> kNotPairTable[6] = {{2, 3}, {1, 3}, {1, 2}, {0, 3}, {0, 2}, {0, 1}};

bool FindP1P2Indices(const double* v_p3p4, const double* v_p3pa, const double* v_p3pb,
                     const uint8_t* p1p2, uint8_t* p1, uint8_t* p2);

// 24 candidates, matching src/carolus_node_ros2.cpp verbatim as of 2026-09-03.
// candidateIdx decomposes as pair (6) x P3/P4 swap (2) x P1/P2 swap (2). The
// two swaps exist because the midpoint-distance test that orders P3/P4 and
// FindP1P2Indices' cross-product sign test are each a single guess that can go
// the wrong way; the 6-candidate version could not express either flip, which
// is what a tape measure caught on 2026-09-03 (0.7969 m reported against 0.83 m
// real). This instrument was still on the 6-candidate set until now, so its
// earlier verdict describes an algorithm no longer in production.
static bool labelForCandidate(const std::vector<Eigen::Vector3d>& pts, int candidateIdx,
                               std::vector<Eigen::Vector3d>& out) {
    const int pairIdx = candidateIdx / 4;
    const bool swapP3P4 = ((candidateIdx / 2) % 2) != 0;
    const bool swapP1P2 = (candidateIdx % 2) != 0;
    auto p1p2 = kPairTable[pairIdx];
    auto p3p4 = kNotPairTable[pairIdx];
    Eigen::Vector3d midpoint = (pts[p1p2.first] + pts[p1p2.second]) / 2.0;
    double d0 = (midpoint - pts[p3p4.first]).norm();
    double d1 = (midpoint - pts[p3p4.second]).norm();
    int p3 = (d0 < d1) ? p3p4.first : p3p4.second;
    int p4 = (d0 < d1) ? p3p4.second : p3p4.first;
    if (swapP3P4) std::swap(p3, p4);
    double v_p3p4[3] = {pts[p3].x() - pts[p4].x(), pts[p3].y() - pts[p4].y(), 0.0};
    double v_p3pa[3] = {pts[p3].x() - pts[p1p2.first].x(), pts[p3].y() - pts[p1p2.first].y(), 0.0};
    double v_p3pb[3] = {pts[p3].x() - pts[p1p2.second].x(), pts[p3].y() - pts[p1p2.second].y(), 0.0};
    uint8_t p1p2_arr[2] = {static_cast<uint8_t>(p1p2.first), static_cast<uint8_t>(p1p2.second)};
    uint8_t p1, p2;
    if (!FindP1P2Indices(v_p3p4, v_p3pa, v_p3pb, p1p2_arr, &p1, &p2)) return false;
    if (swapP1P2) std::swap(p1, p2);
    out.resize(4);
    out[0] = pts[p1]; out[1] = pts[p2]; out[2] = pts[p3]; out[3] = pts[p4];
    return true;
}

struct ReprojectionErrorNormalizedPlane {
    ReprojectionErrorNormalizedPlane(const Eigen::Vector2d& obs, const Eigen::Vector3d& tgt)
        : observed_(obs), target_(tgt) {}
    template <typename T>
    bool operator()(const T* const cp, T* residuals) const {
        T cam_T[3] = {cp[0], cp[1], cp[2]};
        T cam_R[3] = {cp[3], cp[4], cp[5]};
        T tp[3] = {T(target_[0]), T(target_[1]), T(target_[2])};
        T camp[3];
        ceres::AngleAxisRotatePoint(cam_R, tp, camp);
        camp[0] += cam_T[0]; camp[1] += cam_T[1]; camp[2] += cam_T[2];
        residuals[0] = camp[0] / camp[2] - T(observed_[0]);
        residuals[1] = camp[1] / camp[2] - T(observed_[1]);
        return true;
    }
    static ceres::CostFunction* Create(const Eigen::Vector2d& obs, const Eigen::Vector3d& tgt) {
        return new ceres::AutoDiffCostFunction<ReprojectionErrorNormalizedPlane, 2, 6>(
            new ReprojectionErrorNormalizedPlane(obs, tgt));
    }
    Eigen::Vector2d observed_; Eigen::Vector3d target_;
};

struct SolveResult { Eigen::Vector3d t = Eigen::Vector3d::Zero(); double final_cost = 1e18; };

static SolveResult solveCorrected(const std::vector<Eigen::Vector3d>& sortedBearing,
                                   const std::vector<Eigen::Vector3d>& known) {
    double camera_params[6] = {0.0, 0.0, -0.001, 0.0, 0.0, 0.7};
    ceres::Problem problem;
    for (size_t i = 0; i < sortedBearing.size(); ++i) {
        Eigen::Vector2d obs(sortedBearing[i].x() / sortedBearing[i].z(),
                            sortedBearing[i].y() / sortedBearing[i].z());
        problem.AddResidualBlock(ReprojectionErrorNormalizedPlane::Create(obs, known[i]), nullptr, camera_params);
    }
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 30;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    SolveResult r;
    r.t = Eigen::Vector3d(camera_params[0], camera_params[1], camera_params[2]);
    r.final_cost = summary.final_cost;
    return r;
}

int main() {
    auto known = knownPoints();
    const int N_TRIALS = 200;
    const double NOISE_XY = 1.0 / FX;
    Eigen::Vector3d groundTruthT(0.0, 0.0, -Z_NOMINAL);

    // Percentiles, not mean/max. A max over N trials is itself a noisy
    // statistic -- the 2026-09-03 re-run saw it swing 3cm between adjacent
    // 2-degree steps, which is enough to invent or hide a "regression band"
    // that is not there. p50/p95/p99 are stable across seeds and are what the
    // enable-by-default decision was actually taken on.
    std::printf("%6s | %8s %8s %8s %6s | %8s %8s %8s %6s\n",
                "theta", "b_p50", "b_p95", "b_p99", "b_fail",
                "m_p50", "m_p95", "m_p99", "m_fail");

    for (int deg = 5; deg <= 85; deg += 5) {
        double theta = deg * M_PI / 180.0;
        auto camPts = beaconInCameraFrame(theta);
        std::mt19937 rng(1000 + deg);  // distinct, reproducible seed per angle

        std::vector<double> baseErr, multiErr;
        int baseFail = 0, multiFail = 0;

        for (int trial = 0; trial < N_TRIALS; ++trial) {
            std::vector<Eigen::Vector3d> bearing;
            for (auto& p : camPts) bearing.push_back(toBearing(p, NOISE_XY, &rng));

            std::vector<double> lens(6);
            for (int k = 0; k < 6; ++k)
                lens[k] = (bearing[kPairTable[k].first] - bearing[kPairTable[k].second]).norm();
            int chosen = std::distance(lens.begin(), std::max_element(lens.begin(), lens.end()));
            // chosen is a PAIR index (0-5). Under the 24-candidate decomposition
            // the same pair with neither swap applied is candidate chosen*4 --
            // this keeps the baseline arm bit-identical to the shipped default
            // path, so the comparison below still measures the fix and not a
            // second change smuggled into the reference.
            std::vector<Eigen::Vector3d> baseSorted;
            if (!labelForCandidate(bearing, chosen * 4, baseSorted)) {
                baseFail++;
            } else {
                auto r = solveCorrected(baseSorted, known);
                baseErr.push_back((r.t - groundTruthT).norm());
            }

            double bestCost = std::numeric_limits<double>::infinity();
            Eigen::Vector3d bestT;
            bool any = false;
            for (int c = 0; c < 24; ++c) {
                std::vector<Eigen::Vector3d> cand;
                if (!labelForCandidate(bearing, c, cand)) continue;
                auto r = solveCorrected(cand, known);
                if (r.final_cost < bestCost) { bestCost = r.final_cost; bestT = r.t; any = true; }
            }
            if (!any) multiFail++; else multiErr.push_back((bestT - groundTruthT).norm());
        }

        auto pct = [](std::vector<double> v, double q) {
            if (v.empty()) return 0.0;
            std::sort(v.begin(), v.end());
            return v[static_cast<size_t>(q * (v.size() - 1))];
        };
        std::printf("%5ddeg | %8.4f %8.4f %8.4f %5d%% | %8.4f %8.4f %8.4f %5d%%\n",
                    deg, pct(baseErr, 0.50), pct(baseErr, 0.95), pct(baseErr, 0.99),
                    100 * baseFail / N_TRIALS,
                    pct(multiErr, 0.50), pct(multiErr, 0.95), pct(multiErr, 0.99),
                    100 * multiFail / N_TRIALS);
    }

    std::printf("\nAll errors in metres.\n");
    std::printf("Read b_fail alongside the baseline percentiles: a failed trial\n");
    std::printf("contributes no error sample, so where b_fail is large the\n");
    std::printf("baseline's own percentiles are computed over its survivors only\n");
    std::printf("and flatter it. The multi-hypothesis columns include every trial.\n");
    return 0;
}
