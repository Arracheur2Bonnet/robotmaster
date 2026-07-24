/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-08-09 00:34:04
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-08-12 12:56:39
 * @ Description:
 * BASE CLASSES FOR CERES OPERATION
 * KINDA MESSY RIGHT NOW, BUT IT WORKS
 * 
 * SUPPORTING FX ANF FY FOR FIT-116 METHOD 
 * 
 */

#ifndef POSE_SOLVER_H
#define POSE_SOLVER_H

#pragma once
#include "p34p.hpp"
#include "compute_jacobian.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

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


//P4P WITH AUTO DIFF
// struct ReprojectionErrorWithAutoDiff {
//     ReprojectionErrorWithAutoDiff(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, const double focalx_length, 
//         const double focaly_length, const double cx, const double cy)
//         : observed_point_(observed_point), target_(target), focalx_length_(focalx_length),
//           focaly_length_(focaly_length), cx_(cx), cy_(cy) {}

//     template <typename T>
//     bool operator()(const T* const camera_params, T* residuals) const {
       
//         T XL = camera_params[0];
//         T YL = camera_params[1];
//         T ZL = camera_params[2];
//         T phi = camera_params[3];
//         T theta = camera_params[4];
//         T psi = camera_params[5];

//         T angle_axis[3] = {phi, theta, psi};
//         T camera_point[3];

 
//         T target_point[3];
//         for (int i = 0; i < 3; ++i) {
//             target_point[i] = T(target_[i]);
//         }

//         //ROTATE TO CAMERA FRAME
//         //CHECK STABILITY ISSUES --> COULD MOVE TO QUATERNIONS
//         ceres::AngleAxisRotatePoint(angle_axis, target_point, camera_point);

//         camera_point[0] += XL;
//         camera_point[1] += YL;
//         camera_point[2] += ZL;

//         //PINHOLE PROJECT BABY 
//         T xp = camera_point[0] / camera_point[2];
//         T yp = camera_point[1] / camera_point[2];

//         //APPLY INTRINSICS
//         T predicted_x =  xp*focalx_length_ + T(cx_);
//         T predicted_y =  yp *focaly_length_ + T(cy_);

//         //COMPUTE RESIDUALS
//         residuals[0] = predicted_x - T(observed_point_[0]);
//         residuals[1] = predicted_y - T(observed_point_[1]);

//         return true;
//     }

//     static ceres::CostFunction* Create(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, const double focalx_length, const double focaly_length, const double cx, const double cy) {
//     return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithAutoDiff, 2, 6>(
//         new ReprojectionErrorWithAutoDiff(observed_point, target, focalx_length, focaly_length, cx, cy)));
// }

//     Eigen::Vector2d observed_point_;
//     Eigen::Vector3d target_;
//     double focalx_length_;
//     double focaly_length_;
//     double cx_;
//     double cy_;
// };



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



struct ReprojectionErrorWithAutoDiff {
    ReprojectionErrorWithAutoDiff(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, const double focalx_length, 
        const double focaly_length, const double cx, const double cy)
        : observed_point_(observed_point), target_(target), focalx_length_(focalx_length),
          focaly_length_(focaly_length), cx_(cx), cy_(cy) {}

    template <typename T>
    bool operator()(const T* const camera_params, T* residuals) const {
       
        T XL = camera_params[0];
        T YL = camera_params[1];
        T ZL = camera_params[2];
        T phi = camera_params[3];
        T theta = camera_params[4];
        T psi = camera_params[5];

        T angle_axis[3] = {phi, theta, psi};
        T camera_point[3];

        T target_point[3];
        for (int i = 0; i < 3; ++i) {
            target_point[i] = T(target_[i]);
        }

        // Rotate to camera frame
        ceres::AngleAxisRotatePoint(angle_axis, target_point, camera_point);

        camera_point[0] += XL;
        camera_point[1] += YL;
        camera_point[2] += ZL;

        // Pinhole projection
        T xp = camera_point[0] / camera_point[2];
        T yp = camera_point[1] / camera_point[2];

        // Apply intrinsics
        T predicted_x = xp * focalx_length_ + T(cx_);
        T predicted_y = yp * focaly_length_ + T(cy_);

        // Compute residuals
        residuals[0] = predicted_x - T(observed_point_[0]);
        residuals[1] = predicted_y - T(observed_point_[1]);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, const double focalx_length, const double focaly_length, const double cx, const double cy) {
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithAutoDiff, 2, 6>(
            new ReprojectionErrorWithAutoDiff(observed_point, target, focalx_length, focaly_length, cx, cy)));
    }

    Eigen::Vector2d observed_point_;
    Eigen::Vector3d target_;
    double focalx_length_;
    double focaly_length_;
    double cx_;
    double cy_;
};



// struct ReprojectionErrorFIT : public ceres::SizedCostFunction<2, 7> {  // 7 parameters: 3 for translation, 4 for quaternion
//     ReprojectionErrorFIT(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, const double focalx_length, 
//         const double focaly_length, const double cx, const double cy)
//         : observed_point_(observed_point), target_(target), focalx_length_(focalx_length),
//           focaly_length_(focaly_length), cx_(cx), cy_(cy) {}

//     bool Evaluate(const double* const* parameters, double* residuals, double** jacobians) const override {
//         // SE(3) parameters (translation + quaternion rotation)
//         Eigen::Map<const Eigen::Vector3d> t(parameters[0]);  // Translation vector

//         // Constraint: T(2) (i.e., Z component) must be negative
//         if (t(2) >= 0) {
//             // Apply a large penalty if the Z component is not negative
//             residuals[0] = residuals[1] = 1e5;  // Large penalty value
//             if (jacobians != nullptr) {
//                 std::fill(jacobians[0], jacobians[0] + 14, 0.0);  // Set Jacobians to zero
//             }
//             return true;
//         }

//         // Extract quaternion components
//         double qw = parameters[0][3];
//         double qx = parameters[0][4];
//         double qy = parameters[0][5];
//         double qz = parameters[0][6];

//         // Ensure quaternion is normalized
//         Eigen::Quaterniond q(qw, qx, qy, qz);
//         q.normalize();

//         // Convert quaternion to rotation matrix
//         Eigen::Matrix3d R = q.toRotationMatrix();

//         // Transform target point into the camera frame
//         Eigen::Vector3d camera_point = R * target_ + t;

//         // Pinhole projection
//         double xp = camera_point(0) / camera_point(2);
//         double yp = camera_point(1) / camera_point(2);

//         // Apply intrinsics
//         double predicted_x = xp * focalx_length_ + cx_;
//         double predicted_y = yp * focaly_length_ + cy_;

//         // Compute residuals
//         residuals[0] = predicted_x - observed_point_(0);
//         residuals[1] = predicted_y - observed_point_(1);

//         if (jacobians != nullptr) {
//             // Call the autogenerated symbolic Jacobian function
//             Eigen::Matrix<double, 2, 7> jacobian_matrix;
//             sym::ComputeJacobian(q.w(), q.x(), q.y(), q.z(), t.x(), t.y(), t.z(),
//                                  focalx_length_, focaly_length_, target_.x(), target_.y(), target_.z(),
//                                  cx_, cy_, observed_point_(0), observed_point_(1), &jacobian_matrix);

//             // Copy the Jacobian to the output
//             std::copy(jacobian_matrix.data(), jacobian_matrix.data() + 14, jacobians[0]);
//         }

//         return true;
//     }

//     static std::unique_ptr<ceres::CostFunction> Create(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, const double focalx_length, const double focaly_length, const double cx, const double cy) {
//         return std::make_unique<ReprojectionErrorFIT>(observed_point, target, focalx_length, focaly_length, cx, cy);
//     }

//     Eigen::Vector2d observed_point_;
//     Eigen::Vector3d target_;
//     double focalx_length_;
//     double focaly_length_;
//     double cx_;
//     double cy_;
// };





// struct ReprojectionErrorP4P : public ceres::SizedCostFunction<2, 6> {
//     ReprojectionErrorP4P(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target,
//                          const double focalx_length, const double focaly_length,
//                          const double cx, const double cy)
//         : observed_point_(observed_point), target_(target), focalx_length_(focalx_length),
//           focaly_length_(focaly_length), cx_(cx), cy_(cy) {}

//     bool Evaluate(const double* const* parameters, double* residuals, double** jacobians) const override {
//         // Extract translation and axis-angle rotation from parameters
//            Eigen::Vector3d translation(parameters[0][0], parameters[0][1], parameters[0][2]);
//     double angle_axis[3] = {parameters[0][3], parameters[0][4], parameters[0][5]};

//     // Rotate the target point using ceres::AngleAxisRotatePoint
//     double camera_point[3];
//     ceres::AngleAxisRotatePoint(angle_axis, target_.data(), camera_point);
//     double magnitudeAngle = sqrt(angle_axis[0] * angle_axis[0] + angle_axis[1] * angle_axis[1] + angle_axis[2] * angle_axis[2]);
//     Eigen::Vector3d axis = Eigen::Vector3d(angle_axis[0], angle_axis[1], angle_axis[2]);
//     // Apply translation
//     axis.normalize();

//     camera_point[0] += translation[0];
//     camera_point[1] += translation[1];
//     camera_point[2] += translation[2];

//     // Compute residuals using pinhole projection
//     double xp = camera_point[0] / camera_point[2];
//     double yp = camera_point[1] / camera_point[2];

//     double predicted_x = xp * focalx_length_ + cx_;
//     double predicted_y = yp * focaly_length_ + cy_;

//     residuals[0] = predicted_x - observed_point_.x();
//     residuals[1] = predicted_y - observed_point_.y();

//     // Compute the 4x6 Jacobian using the SymForce-generated function
//     if (jacobians != nullptr && jacobians[0] != nullptr) {
//         Eigen::Matrix<double, 4, 6> jacobian_matrix = sym::JacobianMatrix<double>(
//             sym::Pose3<double>(sym::Rot3<double>::FromAngleAxis(magnitudeAngle, axis), translation), 
//             target_, focalx_length_, focaly_length_
//         );
//         std::copy(jacobian_matrix.data(), jacobian_matrix.data() + 24, jacobians[0]);
//     }

//     return true;
// }
//     static std::unique_ptr<ceres::CostFunction> Create(const Eigen::Vector2d& observed_point,
//                                                        const Eigen::Vector3d& target,
//                                                        const double focalx_length,
//                                                        const double focaly_length,
//                                                        const double cx, const double cy) {
//         return std::make_unique<ReprojectionErrorP4P>(observed_point, target, focalx_length, focaly_length, cx, cy);
//     }

//     Eigen::Vector2d observed_point_;
//     Eigen::Vector3d target_;
//     double focalx_length_;
//     double focaly_length_;
//     double cx_;
//     double cy_;
// };
struct ReprojectionErrorWithNumericDiff {
    ReprojectionErrorWithNumericDiff(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, const double focalx_length, 
        const double focaly_length, const double cx, const double cy)
        : observed_point_(observed_point), target_(target), focalx_length_(focalx_length),
          focaly_length_(focaly_length), cx_(cx), cy_(cy) {}

    template <typename T>
    bool operator()(const T* const camera_params, T* residuals) const {
       
        T XL = camera_params[0];
        T YL = camera_params[1];
        T ZL = camera_params[2];
        T phi = camera_params[3];
        T theta = camera_params[4];
        T psi = camera_params[5];

        T angle_axis[3] = {phi, theta, psi};
        T camera_point[3];

        T target_point[3];
        for (int i = 0; i < 3; ++i) {
            target_point[i] = T(target_[i]);
        }

        ceres::AngleAxisRotatePoint(angle_axis, target_point, camera_point);

        camera_point[0] += XL;
        camera_point[1] += YL;
        camera_point[2] += ZL;

        T xp = camera_point[0] / camera_point[2];
        T yp = camera_point[1] / camera_point[2];

        T predicted_x = xp * focalx_length_ + T(cx_);
        T predicted_y = yp * focaly_length_ + T(cy_);

        residuals[0] = predicted_x - T(observed_point_[0]);
        residuals[1] = predicted_y - T(observed_point_[1]);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& observed_point, const Eigen::Vector3d& target, const double focalx_length, const double focaly_length, const double cx, const double cy) {
        return (new ceres::NumericDiffCostFunction<ReprojectionErrorWithNumericDiff, ceres::CENTRAL, 2, 6>(
            new ReprojectionErrorWithNumericDiff(observed_point, target, focalx_length, focaly_length, cx, cy)));
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
            lambdatwist::CameraPose& bestPose) const;

private:
    cv::Mat cameraMatrix_;
    int measType_;
};

#endif // POSE_SOLVER_H
