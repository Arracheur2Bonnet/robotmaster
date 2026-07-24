#ifndef P34P_HPP
#define P34P_HPP
#pragma once
#include <Eigen/Dense>
#include <vector>
//https://github.com/vlarsson/lambdatwist

namespace lambdatwist {

struct CameraPose {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};

void compute_eig3x3known0(const Eigen::Matrix3d &M, Eigen::Matrix3d &E, double &sig1, double &sig2);


void refine_lambda(double &lambda1, double &lambda2, double &lambda3,
                   const double a12, const double a13, const double a23,
                   const double b12, const double b13, const double b23);

int p3p(const std::vector<Eigen::Vector3d> &x, const std::vector<Eigen::Vector3d> &X, std::vector<CameraPose> *output);

} // namespace lambdatwist

#endif // P34P_HPP
