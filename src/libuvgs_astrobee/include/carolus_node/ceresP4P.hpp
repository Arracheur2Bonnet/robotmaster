/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-08-09 00:34:04
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-10-24 01:14:27
 * @ Description:
 * BASE CLASSES FOR CERES OPERATION
 *
 * 
 * SUPPORTING FX ANF FY FOR FIT-116 METHOD 
 * 
 */

#ifndef POSE_SOLVER_H
#define POSE_SOLVER_H

#pragma once
#include "compute_jacobian.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "pose_est.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

//REPETITIVE FUNCTION TO COMPUTE ROTATION MATRIX, NEED TO CLEAN UP LATER
inline Eigen::Matrix3d computeRotationMatrix(double phi, double theta, double psi) {
    Eigen::Matrix3d Rx, Ry, Rz;

    Rx << 1, 0, 0,
          0, cos(phi), -sin(phi),
          0, sin(phi), cos(phi);

    Ry << cos(theta), 0, sin(theta),
          0, 1, 0,
          -sin(theta), 0, cos(theta);

    Rz << cos(psi), -sin(psi), 0,
          sin(psi), cos(psi), 0,
          0, 0, 1;

    return Rz * Ry * Rx;
}



struct ReprojectionErrorWithAnalyticDiff : public ceres::SizedCostFunction<2, 6> {
    ReprojectionErrorWithAnalyticDiff(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, 
                                      const double focalx_length, const double focaly_length, 
                                      const double cx, const double cy)
        : observed_point_(observed_point), target_(target), focalx_length_(focalx_length),
          focaly_length_(focaly_length), cx_(cx), cy_(cy) {}

    virtual ~ReprojectionErrorWithAnalyticDiff() = default;

    virtual bool Evaluate(const double* const* parameters, double* residuals, double** jacobians) const override {
        const double* camera_params = parameters[0];

        // Extract camera parameters
        Eigen::Matrix<double, 3, 1> camera_T(camera_params[0], camera_params[1], camera_params[2]);
        Eigen::Matrix<double, 3, 1> camera_R(camera_params[3], camera_params[4], camera_params[5]);
        Eigen::Matrix<double, 3, 1> known_point_G = target_;

        // Calculate residuals
        Eigen::Matrix<double, 2, 1> predicted_point;
        {
            double camera_point[3];
            double target_point[3] = { target_[0], target_[1], target_[2] };
            ceres::AngleAxisRotatePoint(camera_R.data(), target_point, camera_point);

            camera_point[0] += camera_T[0];
            camera_point[1] += camera_T[1];
            camera_point[2] += camera_T[2];

            // Pinhole projection
            double xp = camera_point[0] / camera_point[2];
            double yp = camera_point[1] / camera_point[2];

            // Apply intrinsics
            predicted_point[0] = xp * focalx_length_ + cx_;
            predicted_point[1] = yp * focaly_length_ + cy_;

            // Compute residuals
            residuals[0] = predicted_point[0] - observed_point_[0];
            residuals[1] = predicted_point[1] - observed_point_[1];
        }

        // If Jacobians are requested
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            Eigen::Matrix<double, 2, 6> J = sym::JacobianMatrix<double>(
                camera_T, camera_R, known_point_G, focalx_length_, focaly_length_, cx_, cy_
            );

            // Copy the Jacobian to the output
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 6; ++j) {
                    jacobians[0][i * 6 + j] = J(i, j);
                }
            }
        }

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, 
                                       const double focalx_length, const double focaly_length, 
                                       const double cx, const double cy) {
        return new ReprojectionErrorWithAnalyticDiff(observed_point, target, focalx_length, focaly_length, cx, cy);
    }

    Eigen::Vector2d observed_point_;
    Eigen::Vector3d target_;
    double focalx_length_;
    double focaly_length_;
    double cx_;
    double cy_;
};






class CobrasFumantes {
public:
    CobrasFumantes(const cv::Mat& cameraMatrix, const int& measType)
        : cameraMatrix_(cameraMatrix), measType_(measType) {}

    void computeAndValidatePosesWithRefinement(
            const std::vector<Eigen::Vector3d>& sortedImagePoints,
            const std::vector<Eigen::Vector3d>& knownPoints,
            const std::vector<cv::Point2f>& undistortedPoints,
            CameraPose& bestPose) const;

private:
    cv::Mat cameraMatrix_;
    int measType_;
};

#endif // POSE_SOLVER_H
