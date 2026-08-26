// instrument_p4p_sort.cpp — standalone diagnostic for BUG-131 / BUG-088.
//
// WHY THIS EXISTS (2026-08-24). Four real-hardware sessions (Pi 5 twice, a
// second bare-metal PC twice) found Carolus's pose responding to only 1.43%
// of a real 30cm beacon displacement, with poses clustering in a small
// number of discrete, repeatable groups rather than either tracking the
// target or drifting continuously. Reading ceresP4P.cpp and pose_est.cpp
// found two candidate mechanisms:
//
//   (A) ceresP4P.cpp's Ceres problem is solved from an IDENTICAL fixed
//       initial guess on every frame (camera_params[6] =
//       {0,0,-0.001,0,0,0.7}), no warm start -- this is BUG-088, filed
//       2026-07-31, never confirmed for lack of exactly this kind of test.
//   (B) pose_est.cpp's SortTargetsUsingTetrahedronGeometry picks its
//       reference edge by comparing UNIT BEARING VECTORS (angular
//       separation from the camera), not the beacon's true fixed 3D
//       dimensions -- so ordinary measurement noise near an angular
//       near-tie can flip which points are labelled P1/P2/P3/P4.
//
// A THIRD, more fundamental candidate was found while building this harness,
// by hand-checking units before trusting anything else: in
// ReprojectionErrorWithAnalyticDiff (ceresP4P.cpp), `observed_point_` is a
// unit-bearing-vector component (magnitude ~0.0-0.15 for a centred beacon)
// but `predicted_point` is computed in PIXEL space (`xp*fx+cx`, dominated by
// cx~576, cy~372). At the true, correct pose for this project's verified
// real beacon geometry, this residual is ~372-640, not ~0 -- confirmed by
// hand in Python before writing a line of this file. This is (C) below.
//
// WHAT THIS PROGRAM DOES. Links the REAL, unmodified pose_est.cpp and
// ceresP4P.cpp directly (both are genuinely ROS-free, confirmed empty NEEDED
// list on the compiled library elsewhere in this project) -- no
// reimplementation, no ROS2, no camera, no beacon. It:
//   Step 0: prints the production residual at the true pose, confirming (C)
//           numerically inside the actual code path rather than trusting the
//           hand calculation alone.
//   Step 1: finds, by root-finding (not guessing), the beacon orientation at
//           which two of the six pairwise ANGULAR separations become equal
//           -- the precise condition (B) depends on.
//   Step 2: at that near-tie orientation, sweeps small (sub-pixel-scale)
//           random perturbations across many synthetic trials, running the
//           REAL SortTargetsUsingTetrahedronGeometry and TWO solves per
//           trial -- the production (mismatched-units) cost function, and a
//           harness-local "corrected" one comparing normalized-plane
//           coordinates on both sides -- so (B) and (C)'s contributions are
//           visible separately rather than confounded.
//
// WHAT THIS PROGRAM DOES NOT DO. It does not modify camera_params' initial
// guess, does not modify SortTargetsUsingTetrahedronGeometry or
// ReprojectionErrorWithAnalyticDiff, and is not linked into carolus_node_ros2
// or any shipped target. Per this project's own standing rule (a comment
// already in ceresP4P.cpp since 2026-08-13), no change to the solver or the
// sort is made without a before/after measurement -- this IS that
// measurement, not the fix.

#include "carolus_node/pose_est.hpp"
#include "carolus_node/ceresP4P.hpp"

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <numeric>
#include <random>
#include <vector>

// ---------------------------------------------------------------------------
// Real, verified project constants -- copied from logitech_1080p.yaml and
// technical-ros2.tex's own synthetic-test convention, not re-guessed here.
// known_points: confirmed correct against a real physical beacon to within
// tape-measure resolution (0.1-0.2 cm), 2026-08-24.
// ---------------------------------------------------------------------------
static const double FX = 546.1957, FY = 547.0838, CX = 575.6041, CY = 372.1876;
static const double Z_NOMINAL = 0.7;  // metres, matches the project's own synthetic-test depth

static std::vector<Eigen::Vector3d> knownPoints() {
    return {
        {0.0825, 0.0, 0.0},
        {-0.0825, 0.0, 0.0},
        {0.0, 0.072, 0.0},
        {0.0, 0.0, 0.0555},
    };
}

// Rotate the beacon about the WORLD Y AXIS by theta, then place its centroid
// at (0,0,Z_NOMINAL) in front of the camera. P3=(0,0.072,0) sits exactly ON
// this rotation axis, so it stays fixed as theta sweeps -- P1/P2/P4 move.
// This gives a clean single-parameter family to root-find a near-tie in.
static std::vector<Eigen::Vector3d> beaconInCameraFrame(double theta) {
    Eigen::Matrix3d Ry;
    Ry << std::cos(theta), 0, std::sin(theta),
                        0, 1,               0,
         -std::sin(theta), 0, std::cos(theta);
    Eigen::Vector3d t(0.0, 0.0, Z_NOMINAL);
    std::vector<Eigen::Vector3d> out;
    for (const auto& p : knownPoints()) out.push_back(Ry * p + t);
    return out;
}

// Angular separation (radians) between two points' unit bearing vectors, as
// SortTargetsUsingTetrahedronGeometry actually measures "distance" between
// them -- NOT their 3D Euclidean distance.
static double angularSep(const Eigen::Vector3d& camA, const Eigen::Vector3d& camB) {
    Eigen::Vector3d a(camA.x() / camA.z(), camA.y() / camA.z(), 1.0);
    Eigen::Vector3d b(camB.x() / camB.z(), camB.y() / camB.z(), 1.0);
    return (a.normalized() - b.normalized()).norm();  // chord length; monotonic in angle
}

// Project a 3D camera-frame point to a unit bearing vector, exactly as
// carolus_node_ros2.cpp:201 does: Eigen::Vector3d(p.x, p.y, 1.0).normalized(),
// where p.x,p.y are normalized-PLANE coordinates (X/Z, Y/Z) -- the output of
// cv::undistortPoints, not raw pixels.
static Eigen::Vector3d toBearing(const Eigen::Vector3d& camPt, double noiseXY = 0.0,
                                  std::mt19937* rng = nullptr) {
    double xp = camPt.x() / camPt.z();
    double yp = camPt.y() / camPt.z();
    if (rng && noiseXY > 0.0) {
        std::normal_distribution<double> n(0.0, noiseXY);
        xp += n(*rng);
        yp += n(*rng);
    }
    return Eigen::Vector3d(xp, yp, 1.0).normalized();
}

// ---------------------------------------------------------------------------
// Harness-local "corrected" cost function: predicted_point stays in
// normalized-plane coordinates (no *fx+cx), compared directly against the
// SAME normalized-plane representation of the observation -- like for like,
// unlike production's pixel-vs-unit-vector mismatch. AutoDiff is safe here:
// the analytic Jacobian this project already ships (compute_jacobian.h)
// explicitly ignores c_x/c_y ("(void)c_x; (void)c_y;"), confirming they only
// ever shifted the residual's VALUE, never its gradient direction -- dropping
// them changes no derivative this harness needs to get right by hand.
// ---------------------------------------------------------------------------
struct ReprojectionErrorNormalizedPlane {
    ReprojectionErrorNormalizedPlane(const Eigen::Vector2d& observed_plane_xy,
                                      const Eigen::Vector3d& target)
        : observed_(observed_plane_xy), target_(target) {}

    template <typename T>
    bool operator()(const T* const camera_params, T* residuals) const {
        T camera_T[3] = {camera_params[0], camera_params[1], camera_params[2]};
        T camera_R[3] = {camera_params[3], camera_params[4], camera_params[5]};
        T target_point[3] = {T(target_[0]), T(target_[1]), T(target_[2])};
        T camera_point[3];
        ceres::AngleAxisRotatePoint(camera_R, target_point, camera_point);
        camera_point[0] += camera_T[0];
        camera_point[1] += camera_T[1];
        camera_point[2] += camera_T[2];
        T xp = camera_point[0] / camera_point[2];
        T yp = camera_point[1] / camera_point[2];
        residuals[0] = xp - T(observed_[0]);
        residuals[1] = yp - T(observed_[1]);
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& observed_plane_xy,
                                        const Eigen::Vector3d& target) {
        return new ceres::AutoDiffCostFunction<ReprojectionErrorNormalizedPlane, 2, 6>(
            new ReprojectionErrorNormalizedPlane(observed_plane_xy, target));
    }

    Eigen::Vector2d observed_;
    Eigen::Vector3d target_;
};

// The six pairwise index combinations, in the same order pose_est.cpp's own
// idx_lookup_table uses. Hoisted to file scope so both the sweep loop and the
// results block can name the same table rather than each keeping its own copy.
static const std::pair<int, int> kPairTable[6] = {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}};

struct SolveResult {
    bool ok = false;
    Eigen::Vector3d t;
    double final_cost = -1.0;
    bool converged = false;
    int iterations = -1;
};

// Same fixed initial guess as production (camera_params[6] =
// {0,0,-0.001,0,0,0.7}), same solver options (DOGLEG, 30 iters, DENSE_SCHUR)
// -- everything held identical to production EXCEPT the cost function, so
// the corrected-vs-production comparison isolates (C) cleanly.
static SolveResult solveCorrected(const std::vector<Eigen::Vector3d>& sortedBearing,
                                   const std::vector<Eigen::Vector3d>& known) {
    double camera_params[6] = {0.000, 0.000, -0.001, 0.0, 0.0, 0.7};
    ceres::Problem problem;
    for (size_t i = 0; i < sortedBearing.size(); ++i) {
        // Compare like for like: both sides normalized-plane, so re-derive
        // xp,yp from the bearing vector rather than reusing its unit-vector
        // (renormalized) x,y -- for points near the image centre the two are
        // close, but using x/z of the bearing vector itself keeps this
        // exact rather than approximate.
        Eigen::Vector2d observed_plane(sortedBearing[i].x() / sortedBearing[i].z(),
                                        sortedBearing[i].y() / sortedBearing[i].z());
        problem.AddResidualBlock(
            ReprojectionErrorNormalizedPlane::Create(observed_plane, known[i]), nullptr,
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
    r.iterations = static_cast<int>(summary.iterations.size());
    return r;
}

int main() {
    auto known = knownPoints();
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << FX, 0, CX, 0, FY, CY, 0, 0, 1);

    // ---- Step 0: confirm the units mismatch inside the REAL code path ----
    std::printf("=== Step 0: production residual at the TRUE pose (should be ~0 if units matched) ===\n");
    {
        auto camPts = beaconInCameraFrame(0.0);  // frontal view, exact true pose is (0,0,0)/(0,0,Z)
        for (size_t i = 0; i < camPts.size(); ++i) {
            double xp = camPts[i].x() / camPts[i].z();
            double yp = camPts[i].y() / camPts[i].z();
            Eigen::Vector3d bearing = toBearing(camPts[i]);
            double residual_x = (xp * FX + CX) - bearing.x();
            double residual_y = (yp * FY + CY) - bearing.y();
            std::printf("  point %zu: observed_point_=(%.5f,%.5f)  predicted_point(true pose)=(%.2f,%.2f)"
                        "  residual=(%.2f,%.2f)\n",
                        i, bearing.x(), bearing.y(), xp * FX + CX, yp * FY + CY, residual_x, residual_y);
        }
    }

    // ---- Step 1: root-find the angular near-tie orientation ----
    std::printf("\n=== Step 1: sweeping beacon tilt theta, six angular separations ===\n");
    auto angularSeps = [&](double theta) {
        auto pts = beaconInCameraFrame(theta);
        std::vector<std::pair<std::pair<int,int>, double>> seps;
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j)
                seps.push_back({{i, j}, angularSep(pts[i], pts[j])});
        return seps;
    };
    for (double deg = 0; deg <= 85; deg += 5) {
        double theta = deg * M_PI / 180.0;
        auto seps = angularSeps(theta);
        std::printf("  theta=%5.1fdeg: ", deg);
        for (auto& s : seps) std::printf("(%d,%d)=%.4f  ", s.first.first, s.first.second, s.second);
        std::printf("\n");
    }

    // Root-find where sep(0,1) [P1-P2, the true-longest 3D edge] crosses
    // sep(0,2) [P1-P3] -- bisection, since angularSeps(0,1) is known to start
    // above angularSeps(0,2) at theta=0 (verified by the printed sweep above)
    // and P1-P2 foreshortens toward 0 as theta->90deg while P1-P3 (P3 fixed
    // on the rotation axis) does not collapse the same way.
    double lo = 0.0, hi = M_PI / 2.0 - 1e-3;
    auto diff = [&](double theta) {
        auto pts = beaconInCameraFrame(theta);
        return angularSep(pts[0], pts[1]) - angularSep(pts[0], pts[2]);
    };
    if (diff(lo) <= 0 || diff(hi) >= 0) {
        std::printf("\n!! No sign change found in (0,90deg) for the (0,1)/(0,2) pair -- "
                    "diff(lo)=%.6f diff(hi)=%.6f. Aborting: near-tie geometry assumption failed.\n",
                    diff(lo), diff(hi));
        return 1;
    }
    for (int iter = 0; iter < 60; ++iter) {
        double mid = 0.5 * (lo + hi);
        if (diff(mid) > 0) lo = mid; else hi = mid;
    }
    double thetaStar = 0.5 * (lo + hi);
    std::printf("\n=== Near-tie found: theta*=%.4f rad (%.2f deg), diff=%.3e ===\n",
                thetaStar, thetaStar * 180.0 / M_PI, diff(thetaStar));

    // ---- Step 2: sweep perturbations at theta*, run the REAL sort + BOTH solves ----
    const int N_TRIALS = 1000;
    // Perturbation scale: +/-1 px equivalent at this focal length.
    const double NOISE_XY = 1.0 / FX;
    std::printf("\n=== Step 2: %d trials at theta*, noise=+/-%.5f (normalized-plane, ~1px) ===\n",
                N_TRIALS, NOISE_XY);

    std::mt19937 rng(42);
    std::vector<int> flipCount(6, 0);
    // group -> list of solved t (production, corrected)
    std::vector<std::vector<Eigen::Vector3d>> prodByGroup(6), corrByGroup(6), ros1ByGroup(6), exactByGroup(6);
    std::vector<std::vector<double>> prodCostByGroup(6), corrCostByGroup(6), ros1CostByGroup(6), exactCostByGroup(6);
    int sortFailures = 0;

    auto camPtsStar = beaconInCameraFrame(thetaStar);

    for (int trial = 0; trial < N_TRIALS; ++trial) {
        std::vector<Eigen::Vector3d> bearing;
        for (auto& p : camPtsStar) bearing.push_back(toBearing(p, NOISE_XY, &rng));

        std::vector<Eigen::Vector3d> sorted(4);
        if (!SortTargetsUsingTetrahedronGeometry(bearing, sorted)) {
            sortFailures++;
            continue;
        }

        // Identify which p1p2_table_idx this corresponds to by matching the
        // sorted output back to index pairs in the ORIGINAL (unperturbed at
        // theta*) camera-frame points -- match by nearest bearing vector
        // rather than assuming order, since the sort permutes.
        auto matchIndex = [&](const Eigen::Vector3d& b) {
            int best = -1; double bestD = 1e9;
            for (int i = 0; i < 4; ++i) {
                double d = (b - toBearing(camPtsStar[i])).norm();
                if (d < bestD) { bestD = d; best = i; }
            }
            return best;
        };
        int i0 = matchIndex(sorted[0]), i1 = matchIndex(sorted[1]);
        int lo_i = std::min(i0, i1), hi_i = std::max(i0, i1);
        int groupIdx = -1;
        for (int k = 0; k < 6; ++k) if (kPairTable[k].first == lo_i && kPairTable[k].second == hi_i) groupIdx = k;
        if (groupIdx < 0) groupIdx = 5;  // shouldn't happen; bucket defensively rather than crash
        flipCount[groupIdx]++;

        // Production solve (real CobrasFumantes, untouched) -- exactly what the
        // ROS2 node does today: hands the solver unit bearing vectors.
        CameraPose prodPose;
        std::vector<cv::Point2f> dummyUndistorted;  // dead parameter, production never reads it
        CobrasFumantes solver(cameraMatrix, 2);
        solver.computeAndValidatePosesWithRefinement(sorted, known, dummyUndistorted, prodPose);
        if (prodPose.t.allFinite()) {
            prodByGroup[groupIdx].push_back(prodPose.t);
            prodCostByGroup[groupIdx].push_back(prodPose.solver_final_cost);
        }

        // ROS1-EQUIVALENT solve: same untouched CobrasFumantes, but the sorted
        // points are first pushed back into pixel space the way the ROS1 node
        // does it (carolus_astrobee.cpp: `sortedImagePoints[i](0) * fx + cx`),
        // a step the ROS2 port dropped. This is the candidate FIX, tested here
        // rather than applied to the node, per this project's own rule against
        // changing solver behaviour without a before/after measurement.
        std::vector<Eigen::Vector3d> sortedPix = sorted;
        for (auto& p : sortedPix) {
            p(0) = p(0) * FX + CX;
            p(1) = p(1) * FY + CY;
        }
        CameraPose ros1Pose;
        CobrasFumantes solver1(cameraMatrix, 2);
        solver1.computeAndValidatePosesWithRefinement(sortedPix, known, dummyUndistorted, ros1Pose);
        if (ros1Pose.t.allFinite()) {
            ros1ByGroup[groupIdx].push_back(ros1Pose.t);
            ros1CostByGroup[groupIdx].push_back(ros1Pose.solver_final_cost);
        }

        // EXACT-PIXEL solve: same as (D), but WITHOUT the `.normalized()` step
        // first -- i.e. x/z, y/z scaled by fx,fy and offset by cx,cy, which is
        // what a pinhole projection actually is. ROS1 normalizes before
        // rescaling, which multiplies every coordinate by 1/sqrt(x^2+y^2+1) --
        // a ~1% shrink for a beacon near the image centre. Included to find out
        // whether restoring exact ROS1 parity or fixing the maths outright is
        // the better target, rather than assuming the inherited version is right.
        std::vector<Eigen::Vector3d> sortedExact(4);
        for (int i = 0; i < 4; ++i) {
            // undo the unit-normalisation: recover x/z, y/z from the bearing vector
            double xz = sorted[i].x() / sorted[i].z();
            double yz = sorted[i].y() / sorted[i].z();
            sortedExact[i] = Eigen::Vector3d(xz * FX + CX, yz * FY + CY, 1.0);
        }
        CameraPose exactPose;
        CobrasFumantes solver2(cameraMatrix, 2);
        solver2.computeAndValidatePosesWithRefinement(sortedExact, known, dummyUndistorted, exactPose);
        if (exactPose.t.allFinite()) {
            exactByGroup[groupIdx].push_back(exactPose.t);
            exactCostByGroup[groupIdx].push_back(exactPose.solver_final_cost);
        }

        // Corrected solve (harness-local cost function).
        SolveResult corr = solveCorrected(sorted, known);
        if (corr.t.allFinite()) {
            corrByGroup[groupIdx].push_back(corr.t);
            corrCostByGroup[groupIdx].push_back(corr.final_cost);
        }
    }

    std::printf("\n=== Results: %d trials, %d sort failures ===\n", N_TRIALS, sortFailures);
    std::printf("--- (B) Does the sort flip? Trials per p1p2 group ---\n");
    int groupsSeen = 0;
    for (int k = 0; k < 6; ++k) {
        if (flipCount[k] == 0) continue;
        groupsSeen++;
        std::printf("  group (%d,%d): %d trials (%.1f%%)\n", kPairTable[k].first, kPairTable[k].second,
                    flipCount[k], 100.0 * flipCount[k] / N_TRIALS);
    }
    std::printf("  => %d distinct group(s) selected. %s\n", groupsSeen,
                groupsSeen > 1 ? "THE SORT FLIPS under this noise."
                               : "Sort did NOT flip -- hypothesis (B) unsupported at this noise level.");

    // Per-group pose spread: the mechanism predicts tight clusters WITHIN a
    // group and real separation BETWEEN groups.
    auto reportGroup = [&](const char* label,
                           const std::vector<std::vector<Eigen::Vector3d>>& byGroup,
                           const std::vector<std::vector<double>>& costByGroup) {
        std::printf("\n--- %s: solved t per group ---\n", label);
        for (int k = 0; k < 6; ++k) {
            if (byGroup[k].empty()) continue;
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (const auto& t : byGroup[k]) mean += t;
            mean /= static_cast<double>(byGroup[k].size());
            Eigen::Vector3d var = Eigen::Vector3d::Zero();
            for (const auto& t : byGroup[k]) {
                Eigen::Vector3d d = t - mean;
                var += d.cwiseProduct(d);
            }
            var /= static_cast<double>(byGroup[k].size());
            double cmean = std::accumulate(costByGroup[k].begin(), costByGroup[k].end(), 0.0) /
                           static_cast<double>(costByGroup[k].size());
            std::printf("  group (%d,%d) n=%3zu  t_mean=(%9.4f,%9.4f,%9.4f)  "
                        "t_std=(%.2e,%.2e,%.2e)  cost_mean=%.4e\n",
                        kPairTable[k].first, kPairTable[k].second, byGroup[k].size(),
                        mean.x(), mean.y(), mean.z(),
                        std::sqrt(var.x()), std::sqrt(var.y()), std::sqrt(var.z()), cmean);
        }
    };
    reportGroup("(A) PRODUCTION cost function (pixel-space vs unit-vector, the suspected mismatch)",
                prodByGroup, prodCostByGroup);
    reportGroup("(C) CORRECTED cost function (normalized-plane both sides)",
                corrByGroup, corrCostByGroup);
    reportGroup("(D) ROS1-EQUIVALENT: untouched solver, sorted points pushed back to PIXEL space "
                "(the *fx+cx step the ROS2 port dropped) -- THE CANDIDATE FIX",
                ros1ByGroup, ros1CostByGroup);
    reportGroup("(E) EXACT PIXEL: same as (D) but skipping ROS1's .normalized() shrink "
                "-- true pinhole projection",
                exactByGroup, exactCostByGroup);

    std::printf("\n--- Ground truth for comparison ---\n");
    std::printf("  The beacon centroid sits at (0,0,%.3f) in the camera frame at theta*.\n", Z_NOMINAL);
    std::printf("  A correct solver should return t near that. Compare both tables above.\n");

    return 0;
}
