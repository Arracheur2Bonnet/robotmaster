// instrument_sort_input_scale.cpp -- does the ROS2 port's `.normalized()` cause
// BUG-131, or is the correspondence-sort instability independent of it?
//
// ===================== RESULT, 2026-08-27: HYPOTHESIS REFUTED =====================
// The two representations are equivalent for this geometry. `pair_diff` is 0/200
// at 16 of the 17 swept angles and 5/200 at 60 deg; the 48-53% failure band at
// 60-85 deg is present, identical, in BOTH. Removing `.normalized()` changes
// nothing. The `--verify-baseline` gate passed, so the comparison is sound.
//
// BUT the `argmax_ok` diagnostic column added alongside it found something
// sharper, and it reframes BUG-131 entirely:
//
//   5-55 deg : argmax picks the TRUE P1-P2 pair 200/200.
//   60 deg   : 16/200.
//   65-85 deg: 0/200 -- WRONG EVERY SINGLE TRIAL.
//
// This is not instability. Past a threshold the heuristic is DETERMINISTICALLY
// wrong. Verified geometrically, noise-free, in closed form: the P1-P2 edge
// (0.1650 m, along X) is foreshortened by rotation about Y, while the P3-P4
// edge (0.0909 m, in the YZ plane) is not. Their PROJECTED lengths cross over
// at ~58 deg (55 deg: 0.1365 vs 0.1202; 60 deg: 0.1191 vs 0.1222). Beyond that
// the longest projected edge is P3-P4, and the sort's founding assumption --
// "the longest projected edge is P1-P2" -- is simply false.
//
// This also explains the 58.95 deg "near-tie" the 2026-08-25 harness located:
// that angle is not an arbitrary near-tie, it IS this crossover, the one angle
// where the two projected lengths are equal. And it explains the earlier
// sweep's three regimes exactly: at 45-55 deg argmax is already right 200/200
// so multi-hypothesis can only lose (the regression band); at 60-85 deg argmax
// is always wrong so trying all 6 can only win.
//
// Consequence, stated as a measured operating envelope and NOT as a proposed
// fix: with this beacon's geometry the sort is exact below ~55 deg of viewing
// angle and unusable above ~60 deg.
// =================================================================================
//
// WHY THIS EXISTS (audit finding, 2026-08-27). Reading the two nodes side by
// side found that ROS1 and ROS2 feed DIFFERENT things to the same
// SortTargetsUsingTetrahedronGeometry:
//
//   ROS1 production path (fov:true -- set by all three config profiles):
//     undistortAstrobeeFov() returns `norm * focal_length`, i.e. PIXEL-SCALE
//     coordinates with the principal point already removed. The node then
//     builds Eigen::Vector3d(point.x, point.y, 1.0) with NO .normalized(),
//     and the `* fx` conversion below it is deliberately commented out
//     because the scaling already happened inside the undistortion.
//
//   ROS2 (carolus_node_ros2.cpp:224):
//     cv::undistortPoints() returns NORMALIZED-PLANE coordinates, and the node
//     builds Eigen::Vector3d(p.x, p.y, 1.0).normalized() -- UNIT BEARING
//     VECTORS.
//
// The sort computes (v_i - v_j).norm() and takes argmax over the 6 pairs
// (pose_est.cpp:111). Fed unit vectors that is a chord length, 2*sin(theta/2),
// a purely ANGULAR quantity with the radial scale divided away per-point. Fed
// pixel-scale points it is a plain image-plane DISTANCE. Those are not the same
// ordering in general, because normalisation divides each point by its OWN norm
// and points further from the optical axis shrink more than points near it.
//
// WHAT WEAKENS THE HYPOTHESIS, STATED UP FRONT so the result is not read as
// more than it is: FindP1P2Indices (pose_est.cpp:61) -- the function whose
// failure produces the 48-53% `b_fail` band at 60-85 deg -- is purely
// SIGN-based (signum of a sum of cross-product components) with no magnitude
// threshold anywhere, so it is scale-invariant by construction. Any difference
// this harness finds must therefore come from the argmax picking a DIFFERENT
// pair, not from FindP1P2Indices behaving differently on larger numbers. For a
// beacon near the optical axis the four per-point norms differ by under ~1.5%,
// so the argmax ordering may well not change at all. Measuring is cheap; the
// prediction is genuinely uncertain.
//
// CONTROLLED COMPARISON. The noise is drawn ONCE per point per trial, in the
// same order and from the same per-angle seed as
// instrument_multi_hypothesis_sweep.cpp, and BOTH representations are derived
// from that same draw. The solver, the cost function, the ground truth and the
// trial count are all identical. The ONLY variable is what the sort sees.
//
// THE CHECK THAT CAN FAIL: the `base_*` columns must reproduce
// instrument_multi_hypothesis_sweep.cpp's own numbers exactly (same seeds, same
// draws). --verify-baseline asserts that against the 2026-08-25 recorded values
// and exits non-zero if they drift, so a harness that quietly diverged cannot
// pass its result off as a comparison.

#include "carolus_node/pose_est.hpp"
#include "carolus_node/ceresP4P.hpp"

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
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

// One noisy observation, in normalized-plane coordinates. Both representations
// under test are derived from THIS, so the two variants see identical noise.
// The two n(*rng) draws happen in the same order as
// instrument_multi_hypothesis_sweep.cpp's toBearing(), which is what makes the
// baseline column reproducible against it.
static Eigen::Vector2d observeNormPlane(const Eigen::Vector3d& camPt, double noiseXY, std::mt19937* rng) {
    double xp = camPt.x() / camPt.z(), yp = camPt.y() / camPt.z();
    if (rng && noiseXY > 0.0) {
        std::normal_distribution<double> n(0.0, noiseXY);
        xp += n(*rng);
        yp += n(*rng);
    }
    return Eigen::Vector2d(xp, yp);
}

// ROS2 today: unit bearing vector. Sort sees angular separation.
static Eigen::Vector3d asUnitBearing(const Eigen::Vector2d& np) {
    return Eigen::Vector3d(np.x(), np.y(), 1.0).normalized();
}

// ROS1 production (fov:true): pixel-scale, principal point already removed,
// z left at exactly 1.0. Sort sees image-plane distance.
static Eigen::Vector3d asPixelScale(const Eigen::Vector2d& np) {
    return Eigen::Vector3d(np.x() * FX, np.y() * FY, 1.0);
}

static const std::pair<int, int> kPairTable[6] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
static const std::pair<int, int> kNotPairTable[6] = {{2, 3}, {1, 3}, {1, 2}, {0, 3}, {0, 2}, {0, 1}};

bool FindP1P2Indices(const double* v_p3p4, const double* v_p3pa, const double* v_p3pb,
                     const uint8_t* p1p2, uint8_t* p1, uint8_t* p2);

// Direct port of SortTargetsUsingTetrahedronGeometry's labeling for ONE chosen
// pair, returning the permutation as indices so the caller can apply it to
// whichever representation it is carrying.
static bool labelForCandidate(const std::vector<Eigen::Vector3d>& pts, int candidateIdx, int outIdx[4]) {
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
    outIdx[0] = p1; outIdx[1] = p2; outIdx[2] = p3; outIdx[3] = p4;
    return true;
}

static int argmaxPair(const std::vector<Eigen::Vector3d>& pts) {
    std::vector<double> lens(6);
    for (int k = 0; k < 6; ++k)
        lens[k] = (pts[kPairTable[k].first] - pts[kPairTable[k].second]).norm();
    return std::distance(lens.begin(), std::max_element(lens.begin(), lens.end()));
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

// Solves from normalized-plane observations, permuted by outIdx. Identical for
// both variants -- only the permutation differs, which is the whole point.
static SolveResult solveFromNormPlane(const std::vector<Eigen::Vector2d>& np, const int idx[4],
                                       const std::vector<Eigen::Vector3d>& known) {
    double camera_params[6] = {0.0, 0.0, -0.001, 0.0, 0.0, 0.7};
    ceres::Problem problem;
    for (int i = 0; i < 4; ++i) {
        problem.AddResidualBlock(ReprojectionErrorNormalizedPlane::Create(np[idx[i]], known[i]),
                                 nullptr, camera_params);
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

struct AngleResult {
    int deg;
    double baseMean, baseMax; int baseFailPct;
    double pixMean, pixMax;  int pixFailPct;
    int pairDisagreements;   // trials where the two argmax choices differed
    int argmaxCorrect;       // trials where argmax picked the TRUE P1-P2 pair
};

int main(int argc, char** argv) {
    bool verifyBaseline = (argc > 1 && std::strcmp(argv[1], "--verify-baseline") == 0);

    auto known = knownPoints();
    const int N_TRIALS = 200;
    const double NOISE_XY = 1.0 / FX;
    Eigen::Vector3d groundTruthT(0.0, 0.0, -Z_NOMINAL);

    std::vector<AngleResult> results;

    for (int deg = 5; deg <= 85; deg += 5) {
        double theta = deg * M_PI / 180.0;
        auto camPts = beaconInCameraFrame(theta);
        std::mt19937 rng(1000 + deg);  // SAME seed as instrument_multi_hypothesis_sweep.cpp

        std::vector<double> baseErr, pixErr;
        int baseFail = 0, pixFail = 0, disagree = 0, argmaxOk = 0;

        for (int trial = 0; trial < N_TRIALS; ++trial) {
            std::vector<Eigen::Vector2d> np;
            for (auto& p : camPts) np.push_back(observeNormPlane(p, NOISE_XY, &rng));

            std::vector<Eigen::Vector3d> unitPts, pixPts;
            for (auto& v : np) { unitPts.push_back(asUnitBearing(v)); pixPts.push_back(asPixelScale(v)); }

            const int chosenUnit = argmaxPair(unitPts);
            const int chosenPix  = argmaxPair(pixPts);
            if (chosenUnit != chosenPix) disagree++;
            // DIAGNOSIS ONLY, not a proposed fix. knownPoints() puts P1 at
            // index 0 and P2 at index 1, and camPts preserves that order, so
            // the geometrically correct answer is always pair {0,1} =
            // kPairTable index 0. Counting how often argmax lands there
            // separates "the sort chose wrong" from "the sort chose right and
            // FindP1P2Indices still refused to label it".
            if (chosenUnit == 0) argmaxOk++;

            int idx[4];
            if (!labelForCandidate(unitPts, chosenUnit, idx)) baseFail++;
            else baseErr.push_back((solveFromNormPlane(np, idx, known).t - groundTruthT).norm());

            if (!labelForCandidate(pixPts, chosenPix, idx)) pixFail++;
            else pixErr.push_back((solveFromNormPlane(np, idx, known).t - groundTruthT).norm());
        }

        auto meanOf = [](const std::vector<double>& v) {
            return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        };
        auto maxOf = [](const std::vector<double>& v) {
            return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
        };
        results.push_back({deg, meanOf(baseErr), maxOf(baseErr), 100 * baseFail / N_TRIALS,
                           meanOf(pixErr), maxOf(pixErr), 100 * pixFail / N_TRIALS, disagree, argmaxOk});
    }

    if (verifyBaseline) {
        // The 2026-08-25 sweep's own recorded base_mean/base_max/b_fail, for the
        // angles it printed individually. If this harness's unit-bearing path
        // has drifted from that one, the comparison below is meaningless and
        // this must fail loudly rather than print a reassuring table.
        struct Ref { int deg; double mean, max; int fail; };
        const Ref refs[] = {
            {5, 0.0282, 0.0425, 0}, {45, 0.0198, 0.0424, 0}, {50, 0.0174, 0.0446, 0},
            {55, 0.0146, 0.0368, 0}, {60, 0.0359, 0.0497, 15}, {65, 0.0795, 0.1092, 50},
            {70, 0.1521, 0.1907, 48}, {75, 0.2440, 0.2877, 50}, {80, 0.3413, 0.3904, 53},
            {85, 0.4290, 0.4750, 51},
        };
        int bad = 0;
        std::printf("=== baseline reproduction check against the 2026-08-25 sweep ===\n");
        for (const auto& r : refs) {
            auto it = std::find_if(results.begin(), results.end(),
                                   [&](const AngleResult& a) { return a.deg == r.deg; });
            if (it == results.end()) { std::printf("  %2ddeg MISSING\n", r.deg); bad++; continue; }
            bool ok = std::fabs(it->baseMean - r.mean) < 5e-4
                   && std::fabs(it->baseMax - r.max) < 5e-4
                   && it->baseFailPct == r.fail;
            std::printf("  %2ddeg  mean %.4f vs %.4f   max %.4f vs %.4f   fail %d%% vs %d%%   %s\n",
                        r.deg, it->baseMean, r.mean, it->baseMax, r.max,
                        it->baseFailPct, r.fail, ok ? "OK" : "DRIFT");
            if (!ok) bad++;
        }
        if (bad) {
            std::printf("\nRESULT: %d angle(s) DRIFTED. The baseline column does not reproduce the\n", bad);
            std::printf("recorded sweep, so nothing below it is a valid comparison. Do not report.\n");
            return 1;
        }
        std::printf("\nRESULT: baseline reproduces the recorded sweep exactly. Comparison is valid.\n\n");
    }

    std::printf("%6s  %10s  %10s  %8s  %10s  %10s  %8s  %10s  %10s\n",
                "theta", "base_mean", "base_max", "b_fail", "pix_mean", "pix_max", "p_fail",
                "pair_diff", "argmax_ok");
    for (const auto& r : results) {
        std::printf("%5ddeg  %10.4f  %10.4f  %7d%%  %10.4f  %10.4f  %7d%%  %8d/200  %8d/200\n",
                    r.deg, r.baseMean, r.baseMax, r.baseFailPct,
                    r.pixMean, r.pixMax, r.pixFailPct, r.pairDisagreements, r.argmaxCorrect);
    }

    std::printf("\nbase_* = unit bearing vectors (what ROS2 feeds the sort today).\n");
    std::printf("pix_*  = pixel-scale, principal point removed (what ROS1's fov:true path feeds it).\n");
    std::printf("pair_diff = trials where the two representations' argmax picked a DIFFERENT pair.\n");
    std::printf("If pair_diff is 0 everywhere, the representations are equivalent for this\n");
    std::printf("geometry and the .normalized() hypothesis is refuted for it. All errors in metres.\n");
    std::printf("argmax_ok = trials where the sort picked the TRUE P1-P2 pair (index 0). Diagnosis\n");
    std::printf("only: it separates a wrong CHOICE from a correct choice FindP1P2Indices refused.\n");
    return 0;
}
