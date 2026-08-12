/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-06-28 00:03:46
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-10-24 00:49:33
 * @ Description:
 * POSE ESTIMATION IMPLEMENTATION USING 4 POINTS
 */

#ifndef POSE_ESTIMATION_HPP
#define POSE_ESTIMATION_HPP
#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

struct CameraPose {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    // BUG-087 (2026-08-03) — solver diagnostics, surfaced but NOT acted upon.
    //
    // Until now `ceres::Solve()` filled a Solver::Summary that was never read:
    // a solve that hit the iteration cap without converging produced a
    // perfectly finite, entirely plausible pose that was published as truth,
    // guarded only by an allFinite() NaN/Inf test downstream. Same "silent
    // partial failure" shape already recorded twice on this project (BUG-076,
    // BUG-078).
    //
    // These fields are ADDITIVE and defaulted: nothing that constructs a
    // CameraPose today needs changing, and the published pose is byte-for-byte
    // what it was before. The point is to find out HOW OFTEN non-convergence
    // happens before deciding whether to reject on it — rejecting blind could
    // silence the pipeline entirely if it turns out to be common.
    bool   solver_converged  = false;   // ceres termination_type == CONVERGENCE
    double solver_final_cost = -1.0;    // summary.final_cost, -1 if unset
    int    solver_iterations = -1;      // iterations used, -1 if unset
};

bool SortTargetsUsingTetrahedronGeometry(const std::vector<Eigen::Vector3d>& candidate_target_list, std::vector<Eigen::Vector3d>& targets_out);

inline void cross(const double* a, const double* b, double* result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}


inline double signum(double x) {
    return (x > 0) - (x < 0);
}

#endif // POSE_ESTIMATION_HPP
