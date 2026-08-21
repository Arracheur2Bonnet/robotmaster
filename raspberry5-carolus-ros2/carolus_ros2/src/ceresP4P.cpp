/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-08-09 00:36:15
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-10-24 01:25:04
 * @ Description:
 * COBRAS FUMANTES
 * CERES P4P IMPLEMENTATION OUTER CLASS
 * 
 */


#include "carolus_node/ceresP4P.hpp"



void CobrasFumantes::computeAndValidatePosesWithRefinement(
    const std::vector<Eigen::Vector3d>& sortedImagePoints,
    const std::vector<Eigen::Vector3d>& knownPoints,
    const std::vector<cv::Point2f>& undistortedPoints,
    CameraPose& bestPose) const
{
    // SET UP CERES PROBLEM
    ceres::Problem problem;

    // SETUP INITIAL GUESS
    //
    // 2026-08-13 -- this line is BYTE-IDENTICAL to the one in the LIMO
    // integration of this same inherited node, and their write-up documents a
    // real, reproducible artefact caused by the last coefficient (0.7, the
    // initial guess for the rotation about Z).
    //
    // What they observed: driving a square in front of the beacon produced
    // CURVED tracks where the lateral translations should have been straight.
    // They first blamed fisheye distortion and ruled it out -- the curve is
    // identical with fisheye correction enabled.
    //
    // Why it happens: Ceres minimises reprojection error by gradient descent
    // from this fixed starting vector, and stops at whatever minimum it
    // reaches. A poor initial guess can settle in a LOCAL minimum rather than
    // the global one, and the resulting pose is then wrong in a smooth,
    // plausible-looking way rather than obviously broken.
    //
    // Their tuning rule, quoted rather than adopted: DECREASE the coefficient
    // (they suggest trying 0.2) and re-check that Carolus stays reliable;
    // increasing it makes the curvature worse. It must NOT be set to 0.0 --
    // the solve fails outright -- and if it is too low Carolus stops finding
    // the beacon at all.
    //
    // NOT CHANGED HERE, deliberately: this value has never been swept on the
    // RoboMaster, and the curved-track artefact has never been looked for in
    // our own data. Changing a solver initial guess on inherited advice,
    // without a before/after measurement on this robot, would be exactly the
    // kind of unverified edit this project's rules exist to prevent. It is
    // recorded here so the knob is known, and is a candidate explanation for
    // BUG-088 (pose jump from a fixed initial guess with no warm start).
    double camera_params[6] = {0.000, 0.000, -0.001, 0.0, 0.0, 0.7};  // Changed size to 6, as we are using only 6 parameters

    double focalX_length = cameraMatrix_.at<double>(0, 0);  // FX
    double focalY_length = cameraMatrix_.at<double>(1, 1);  // FY
    double cx = cameraMatrix_.at<double>(0, 2);
    double cy = cameraMatrix_.at<double>(1, 2);

    // TEMP VECTORS
    Eigen::Vector2d observed_point;
    Eigen::Vector3d target_point;

    // ANALYTIC 
        for (size_t i = 0; i < sortedImagePoints.size(); ++i) {
            observed_point = sortedImagePoints[i].head<2>();
            target_point = knownPoints[i];

            auto cost_function = ReprojectionErrorWithAnalyticDiff::Create(observed_point, target_point, focalX_length, focalY_length, cx, cy);
            problem.AddResidualBlock(cost_function, nullptr, camera_params);
        }
    

    // SET UP SOLVER OPTIONS
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG; // SEEMS BETTER THAN LEVENBERG-MARQUARDT
    options.max_num_iterations = 30;
    options.linear_solver_type = ceres::DENSE_SCHUR; // FEEL FREE TO TRY DENSE_QR
    options.minimizer_progress_to_stdout = false;

    // SOLVE IT!
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // UPDATE BEST POSE
    bestPose.R = computeRotationMatrix(camera_params[3], camera_params[4], camera_params[5]);
    bestPose.t = Eigen::Vector3d(camera_params[0], camera_params[1], camera_params[2]);

    // BUG-087 (2026-08-03) — surface what Ceres already told us.
    // `summary` was filled on every solve and never read, so a run that hit
    // max_num_iterations without converging returned a finite, plausible pose
    // indistinguishable from a good one. The pose written above is UNCHANGED;
    // only the diagnostics are new. Deciding whether to reject on them comes
    // after we know how often non-convergence actually occurs.
    bestPose.solver_converged  = (summary.termination_type == ceres::CONVERGENCE);
    bestPose.solver_final_cost = summary.final_cost;
    bestPose.solver_iterations = static_cast<int>(summary.iterations.size());
}
