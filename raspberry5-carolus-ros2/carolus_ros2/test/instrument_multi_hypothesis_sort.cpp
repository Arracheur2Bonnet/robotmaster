// instrument_multi_hypothesis_sort.cpp — testing a candidate fix for BUG-131,
// synthetically, before touching pose_est.cpp.
//
// WHY THIS EXISTS (2026-08-25). BUG-131 (pose_est.cpp's
// SortTargetsUsingTetrahedronGeometry) picks its P1/P2 assignment by the
// LARGEST pairwise separation among the four candidate points -- angular
// separation, when called (as production does) on unit bearing vectors, not
// the beacon's real fixed 3D distances. Confirmed unstable on real hardware
// 2026-08-25 (journal.md, session 6): a real beacon at 60cm gave 8 distinct
// pose clusters over 40 stationary samples, while 90cm/120cm on the same rig
// were each rock-stable -- geometry-dependent, wider than the one synthetic
// near-tie point tested in instrument_p4p_sort.cpp.
//
// THE CANDIDATE FIX TESTED HERE: don't guess which of the 6 possible
// point-pairs is "P1-P2" from an unreliable 2D/angular cue at all. Generate
// ALL 6 candidate labelings, solve the REAL P4P cost function for each, and
// keep whichever one the solver itself says fits best (lowest final_cost).
// This is the standard way multi-hypothesis correspondence problems are
// resolved in PnP-family literature: let the actual reprojection residual
// decide, rather than a cheap pre-solve heuristic that can be wrong exactly
// when it matters (near a real or apparent tie).
//
// WHAT THIS PROGRAM DOES NOT DO. It does not modify pose_est.cpp or
// ceresP4P.cpp. The per-candidate labeling logic below is a direct,
// side-by-side reimplementation of SortTargetsUsingTetrahedronGeometry's own
// algorithm (FindMidpoint / Midpoint2P3P4 / FindP1P2Indices, the latter two
// reused unchanged from pose_est.hpp), parameterised by WHICH of the 6 pairs
// to treat as P1-P2 instead of always picking the max-distance one -- so any
// difference in results traces to the selection strategy, not to a
// reimplementation bug. It is not linked into carolus_node_ros2 or any
// shipped target. Per this project's standing rule, no change is made to
// pose_est.cpp until this is verified.

#include "carolus_node/pose_est.hpp"
#include "carolus_node/ceresP4P.hpp"

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
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

static double angularSep(const Eigen::Vector3d& a3, const Eigen::Vector3d& b3) {
    Eigen::Vector3d a(a3.x() / a3.z(), a3.y() / a3.z(), 1.0);
    Eigen::Vector3d b(b3.x() / b3.z(), b3.y() / b3.z(), 1.0);
    return (a.normalized() - b.normalized()).norm();
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

// FindP1P2Indices has external linkage from pose_est.cpp but is not declared
// in pose_est.hpp (only used internally there today) -- forward-declared
// here rather than editing the shared header for a still-unverified test.
bool FindP1P2Indices(const double* v_p3p4, const double* v_p3pa, const double* v_p3pb,
                     const uint8_t* p1p2, uint8_t* p1, uint8_t* p2);

static const std::pair<int, int> kPairTable[6] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
static const std::pair<int, int> kNotPairTable[6] = {{2, 3}, {1, 3}, {1, 2}, {0, 3}, {0, 2}, {0, 1}};

// Direct reimplementation of SortTargetsUsingTetrahedronGeometry's own
// labeling steps, parameterised by a FORCED p1p2 choice instead of always
// picking argmax. Reuses the project's own FindP1P2Indices/cross/signum
// (pose_est.hpp, unchanged) for the one step that isn't pure arithmetic.
static bool labelForCandidate(const std::vector<Eigen::Vector3d>& pts, int candidateIdx,
                               std::vector<Eigen::Vector3d>& out) {
    auto p1p2 = kPairTable[candidateIdx];
    auto p3p4 = kNotPairTable[candidateIdx];
    Eigen::Vector3d midpoint = (pts[p1p2.first] + pts[p1p2.second]) / 2.0;
    double d0 = (midpoint - pts[p3p4.first]).norm();
    double d1 = (midpoint - pts[p3p4.second]).norm();
    int p3 = (d0 < d1) ? p3p4.first : p3p4.second;
    int p4 = (d0 < d1) ? p3p4.second : p3p4.first;

    double v_p3p4[3] = {pts[p3].x() - pts[p4].x(), pts[p3].y() - pts[p4].y(), 0.0};
    double v_p3pa[3] = {pts[p3].x() - pts[p1p2.first].x(), pts[p3].y() - pts[p1p2.first].y(), 0.0};
    double v_p3pb[3] = {pts[p3].x() - pts[p1p2.second].x(), pts[p3].y() - pts[p1p2.second].y(), 0.0};
    uint8_t p1p2_arr[2] = {static_cast<uint8_t>(p1p2.first), static_cast<uint8_t>(p1p2.second)};
    uint8_t p1, p2;
    if (!FindP1P2Indices(v_p3p4, v_p3pa, v_p3pb, p1p2_arr, &p1, &p2)) return false;

    out.resize(4);
    out[0] = pts[p1];
    out[1] = pts[p2];
    out[2] = pts[p3];
    out[3] = pts[p4];
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
        camp[0] += cam_T[0];
        camp[1] += cam_T[1];
        camp[2] += cam_T[2];
        residuals[0] = camp[0] / camp[2] - T(observed_[0]);
        residuals[1] = camp[1] / camp[2] - T(observed_[1]);
        return true;
    }
    static ceres::CostFunction* Create(const Eigen::Vector2d& obs, const Eigen::Vector3d& tgt) {
        return new ceres::AutoDiffCostFunction<ReprojectionErrorNormalizedPlane, 2, 6>(
            new ReprojectionErrorNormalizedPlane(obs, tgt));
    }
    Eigen::Vector2d observed_;
    Eigen::Vector3d target_;
};

struct SolveResult {
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    double final_cost = 1e18;
    bool converged = false;
    bool ok = false;
};

static SolveResult solveCorrected(const std::vector<Eigen::Vector3d>& sortedBearing,
                                   const std::vector<Eigen::Vector3d>& known) {
    double camera_params[6] = {0.0, 0.0, -0.001, 0.0, 0.0, 0.7};
    ceres::Problem problem;
    for (size_t i = 0; i < sortedBearing.size(); ++i) {
        Eigen::Vector2d obs(sortedBearing[i].x() / sortedBearing[i].z(),
                            sortedBearing[i].y() / sortedBearing[i].z());
        problem.AddResidualBlock(ReprojectionErrorNormalizedPlane::Create(obs, known[i]), nullptr,
                                 camera_params);
    }
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 30;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    SolveResult r;
    r.ok = true;
    r.t = Eigen::Vector3d(camera_params[0], camera_params[1], camera_params[2]);
    r.final_cost = summary.final_cost;
    r.converged = (summary.termination_type == ceres::CONVERGENCE);
    return r;
}

int main() {
    auto known = knownPoints();

    // Reuse the exact near-tie geometry from instrument_p4p_sort.cpp (already
    // verified there: two angular separations equal to floating-point
    // precision at this theta) rather than re-deriving it -- same beacon,
    // same near-tie, so results are directly comparable to that harness.
    double lo = 0.0, hi = M_PI / 2.0 - 1e-3;
    auto diff = [&](double theta) {
        auto pts = beaconInCameraFrame(theta);
        return angularSep(pts[0], pts[1]) - angularSep(pts[0], pts[2]);
    };
    for (int i = 0; i < 60; ++i) {
        double mid = 0.5 * (lo + hi);
        if (diff(mid) > 0) lo = mid; else hi = mid;
    }
    double thetaStar = 0.5 * (lo + hi);
    std::printf("theta* = %.4f rad (%.2f deg), diff=%.3e\n", thetaStar,
                thetaStar * 180.0 / M_PI, diff(thetaStar));

    const int N_TRIALS = 1000;
    const double NOISE_XY = 1.0 / FX;  // ~1px equivalent, same as instrument_p4p_sort.cpp
    std::mt19937 rng(42);

    auto camPtsStar = beaconInCameraFrame(thetaStar);
    // Sign convention verified in instrument_p4p_sort.cpp the same day: for a
    // beacon truly placed at (0,0,+Z_NOMINAL) in front of the camera, the
    // solver's OWN camera_params[0..2] (fed straight into this harness's t,
    // no ROS-side transpose/basis-change applied) comes out with a NEGATIVE
    // z -- confirmed there at t_mean=(...,-0.695) to (...,-0.70) for this
    // exact geometry. Comparing against a POSITIVE truth here first (a real
    // bug, not the beacon's actual behaviour) produced a spurious ~1.4 m
    // "error" for every candidate alike, which is exactly 2*Z_NOMINAL and
    // was the tell that this was a sign mismatch in the test, not a real
    // finding.
    Eigen::Vector3d groundTruthT(0.0, 0.0, -Z_NOMINAL);

    // Baseline: the PRODUCTION heuristic (max angular separation), replicated
    // faithfully via labelForCandidate with the SAME argmax selection
    // pose_est.cpp itself performs.
    int baseline_correct_group = -1;  // filled once, all trials share thetaStar
    {
        std::vector<double> lens(6);
        for (int k = 0; k < 6; ++k)
            lens[k] = angularSep(camPtsStar[kPairTable[k].first], camPtsStar[kPairTable[k].second]);
        baseline_correct_group = std::distance(lens.begin(), std::max_element(lens.begin(), lens.end()));
    }

    int baseline_flip_count[6] = {0};
    int baseline_fail = 0;
    std::vector<double> baseline_err;  // |solved_t - truth| per trial

    int multi_fail = 0;
    std::vector<double> multi_err;
    int multi_pick_matches_true_labeling = 0;  // did the winning candidate equal candidateIdx that
                                               // gives the TRUE (untouched) point order 0,1,2,3?

    double baseline_solve_seconds = 0.0, multi_solve_seconds = 0.0;

    for (int trial = 0; trial < N_TRIALS; ++trial) {
        std::vector<Eigen::Vector3d> bearing;
        for (auto& p : camPtsStar) bearing.push_back(toBearing(p, NOISE_XY, &rng));

        // --- Baseline: production's own argmax-angular-separation choice ---
        std::vector<double> lens(6);
        for (int k = 0; k < 6; ++k)
            lens[k] = (bearing[kPairTable[k].first] - bearing[kPairTable[k].second]).norm();
        int chosen = std::distance(lens.begin(), std::max_element(lens.begin(), lens.end()));
        baseline_flip_count[chosen]++;

        std::vector<Eigen::Vector3d> baseline_sorted;
        if (!labelForCandidate(bearing, chosen, baseline_sorted)) {
            baseline_fail++;
        } else {
            auto t0 = std::chrono::steady_clock::now();
            SolveResult r = solveCorrected(baseline_sorted, known);
            baseline_solve_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            baseline_err.push_back((r.t - groundTruthT).norm());
        }

        // --- Multi-hypothesis: solve ALL 6, keep lowest final_cost ---
        SolveResult best;
        int bestCandidate = -1;
        int validCandidates = 0;
        auto tm0 = std::chrono::steady_clock::now();
        for (int c = 0; c < 6; ++c) {
            std::vector<Eigen::Vector3d> sorted_c;
            if (!labelForCandidate(bearing, c, sorted_c)) continue;
            validCandidates++;
            SolveResult r = solveCorrected(sorted_c, known);
            if (r.final_cost < best.final_cost) {
                best = r;
                bestCandidate = c;
            }
        }
        multi_solve_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - tm0).count();
        if (validCandidates == 0) {
            multi_fail++;
        } else {
            multi_err.push_back((best.t - groundTruthT).norm());
            if (bestCandidate == baseline_correct_group) multi_pick_matches_true_labeling++;
        }
    }

    std::printf("\n=== Baseline (production argmax-angular heuristic) ===\n");
    std::printf("  sort failures: %d / %d\n", baseline_fail, N_TRIALS);
    std::printf("  candidate-group selection counts:\n");
    for (int k = 0; k < 6; ++k)
        if (baseline_flip_count[k] > 0)
            std::printf("    group (%d,%d): %d trials%s\n", kPairTable[k].first, kPairTable[k].second,
                        baseline_flip_count[k], (k == baseline_correct_group) ? "  <- the TRUE longest edge" : "");
    if (!baseline_err.empty()) {
        double mean = std::accumulate(baseline_err.begin(), baseline_err.end(), 0.0) / baseline_err.size();
        double mx = *std::max_element(baseline_err.begin(), baseline_err.end());
        std::printf("  |solved_t - truth|: mean=%.4f m, max=%.4f m (n=%zu)\n", mean, mx, baseline_err.size());
    }

    std::printf("\n=== Multi-hypothesis (solve all 6, keep lowest final_cost) ===\n");
    std::printf("  outright failures (no candidate labeled successfully): %d / %d\n", multi_fail, N_TRIALS);
    std::printf("  winning candidate == true-longest-edge group: %d / %d (%.1f%%)\n",
                multi_pick_matches_true_labeling, N_TRIALS - multi_fail,
                100.0 * multi_pick_matches_true_labeling / std::max(1, N_TRIALS - multi_fail));
    if (!multi_err.empty()) {
        double mean = std::accumulate(multi_err.begin(), multi_err.end(), 0.0) / multi_err.size();
        double mx = *std::max_element(multi_err.begin(), multi_err.end());
        std::printf("  |solved_t - truth|: mean=%.4f m, max=%.4f m (n=%zu)\n", mean, mx, multi_err.size());
    }

    std::printf("\n=== Timing (single-threaded, this machine, not a real-time guarantee) ===\n");
    std::printf("  baseline: %.3f ms/trial (1 solve)\n", 1000.0 * baseline_solve_seconds / N_TRIALS);
    std::printf("  multi-hypothesis: %.3f ms/trial (up to 6 solves) -- %.2fx baseline\n",
                1000.0 * multi_solve_seconds / N_TRIALS,
                multi_solve_seconds / std::max(1e-12, baseline_solve_seconds));

    std::printf("\n=== Verdict ===\n");
    std::printf("  This test measures ACCURACY (vs the known true pose), not just stability.\n");
    std::printf("  Baseline sort failures include the disambiguation step failing (near-tie\n");
    std::printf("  in 2D projected space for FindP1P2Indices), which multi-hypothesis also\n");
    std::printf("  inherits per-candidate -- a candidate that fails to label is simply skipped,\n");
    std::printf("  not counted as a failure unless ALL 6 fail.\n");

    return 0;
}
