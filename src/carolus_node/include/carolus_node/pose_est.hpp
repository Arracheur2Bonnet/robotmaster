/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-06-28 00:03:46
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-07-03 19:12:29
 * @ Description:
 * POSE ESTIMATION IMPLEMENTATION USING 4 POINTS
 */

#ifndef POSE_ESTIMATION_HPP
#define POSE_ESTIMATION_HPP
#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "p34p.hpp" 

using namespace lambdatwist;

bool SortTargetsUsingTetrahedronGeometry(const std::vector<Eigen::Vector3d>& candidate_target_list, std::vector<Eigen::Vector3d>& targets_out);
void computeAndValidatePoses(
    const std::vector<Eigen::Vector3d>& imagePoints,
    const std::vector<Eigen::Vector3d>& knownPoints,
    const cv::Mat& cameraMatrix,
    const std::vector<cv::Point2f>& undistortedPoints,
    CameraPose& bestPose,
    double& minError);

void computeAndValidatePosesWithRefinement(
    const std::vector<Eigen::Vector3d>& imagePoints,
    const std::vector<Eigen::Vector3d>& knownPoints,
    const cv::Mat& cameraMatrix,
    const std::vector<cv::Point2f>& undistortedPoints,
    CameraPose& bestPose);

inline void cross(const double* a, const double* b, double* result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}


inline double signum(double x) {
    return (x > 0) - (x < 0);
}

#endif // POSE_ESTIMATION_HPP
