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
