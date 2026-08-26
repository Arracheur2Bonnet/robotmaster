// carolus_node_ros2.cpp
//
// Minimal ROS2 (rclcpp) wrapper around carolus_core, built 2026-08-18 to
// test Hector's ROS-portability question directly rather than argue it:
// does the same ROS-free detection/solver code, extracted from
// CarolusRexNode the same day, actually compile and run under a different
// middleware with no algorithm changes?
//
// Deliberately NOT a full port of CarolusRexNode. Scope, stated so nobody
// mistakes this for production-ready: single camera topic, plumb_bob
// (radtan) undistortion only (the FOV/Astrobee-specific undistortion path
// stays in carolus_astrobee.cpp, orthogonal to what this proves), no FIFO
// outlier filter, no ff_msgs (dead even on the ROS1 side: that node
// advertises an ff_msgs publisher, then overwrites it with the real /pose
// publisher on the very next line, so nothing has ever consumed it), synchronous
// callback instead of the ROS1 node's producer/consumer queue. What IS
// exercised end to end: image -> BeaconDetector -> CobrasFumantes -> pose,
// the exact chain the extraction was about.

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// cv_bridge's header was renamed .h -> .hpp mid-transition across ROS2
// distros (deprecated in 3.3.0, removed in 4.0.0) -- confirmed empirically
// on this project's own two test targets, not assumed from a changelog:
// this file's first build attempt used .hpp unconditionally because that is
// what the Jazzy container (built first) actually has; it then failed on
// Humble, which as installed here only ships .h. __has_include picks
// whichever is present rather than hardcoding one distro's answer.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "carolus_node/beacon_detector.hpp"
#include "carolus_node/pose_est.hpp"
#include "carolus_node/ceresP4P.hpp"

class CarolusRos2Node : public rclcpp::Node {
public:
    CarolusRos2Node() : Node("carolus_ros2") {
        // fx/fy/cx/cy/distortion/known_points below are what the synthetic-
        // beacon test in technical-ros2.tex relies on (it passes no
        // --params-file, so it needs these to already be right). Not this
        // project's real camera values -- those come from
        // config/logitech_1080p.yaml on every documented real-camera run.
        // image_threshold's default (190) is never actually relied on
        // anywhere in this document: every launch command, synthetic or
        // real, passes its own -p image_threshold override.
        // BUG-088 (2026-08-25) -- warm-start the solver from the previous
        // converged pose instead of the same fixed vector every frame.
        // Parameterised (not hardcoded on) so the exact same binary can A/B
        // it: relaunch with -p warm_start:=false to reproduce the old,
        // always-fixed-start behaviour for comparison.
        declare_parameter("warm_start", true);
        warm_start_ = get_parameter("warm_start").as_bool();

        // BUG-131 (2026-08-25) -- try all 6 possible point-pair labelings and
        // keep whichever the solver itself fits best, instead of trusting
        // SortTargetsUsingTetrahedronGeometry's single argmax-angular-
        // separation guess (which is confirmed unstable near real and
        // apparent ties, both synthetically and on real hardware the same
        // day). OFF by default: verified synthetically only so far
        // (test/instrument_multi_hypothesis_sort.cpp -- worst-case error
        // 37.4cm -> 5.8cm at a near-tie, cost +0.22ms/frame, negligible
        // against this camera's ~10-30 Hz), NOT yet hardware-tested, so it
        // does not change default behaviour until it has been.
        declare_parameter("multi_hypothesis_sort", false);
        multi_hypothesis_sort_ = get_parameter("multi_hypothesis_sort").as_bool();

        declare_parameter("fx", 546.1957);
        declare_parameter("fy", 547.0838);
        declare_parameter("cx", 575.6041);
        declare_parameter("cy", 372.1876);
        declare_parameter("distortion", std::vector<double>{-0.1479, 0.1452, -0.0023, 0.0025});
        declare_parameter("known_points", std::vector<double>{
            0.0825, 0.0, 0.0, -0.0825, 0.0, 0.0, 0.0, 0.072, 0.0, 0.0, 0.0, 0.0555});
        declare_parameter("kernel_size_gaussian", 3);
        declare_parameter("kernel_size_morph", 3);
        declare_parameter("image_threshold", 190);
        declare_parameter("min_area", 8.0);
        declare_parameter("max_area", 1800.0);
        declare_parameter("saturation_threshold", 80);
        declare_parameter("lb_hue", 90.0);
        declare_parameter("ub_hue", 140.0);
        declare_parameter("max_distance_lim", 1000.0);
        declare_parameter("min_circularity", 0.6);
        declare_parameter("camera_topic", std::string("/camera/color/image_raw"));
        declare_parameter("qos_profile", std::string("sensor_data"));

        double fx = get_parameter("fx").as_double();
        double fy = get_parameter("fy").as_double();
        double cx = get_parameter("cx").as_double();
        double cy = get_parameter("cy").as_double();
        auto dist = get_parameter("distortion").as_double_array();
        auto kp = get_parameter("known_points").as_double_array();
        min_circularity_ = get_parameter("min_circularity").as_double();

        camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        dist_coeffs_ = (cv::Mat_<double>(1, 4) << dist[0], dist[1], dist[2], dist[3]);
        for (size_t i = 0; i + 2 < kp.size(); i += 3) {
            known_points_.emplace_back(kp[i], kp[i + 1], kp[i + 2]);
        }

        detector_ = std::make_unique<BeaconDetector>(
            get_parameter("kernel_size_gaussian").as_int(),
            get_parameter("kernel_size_morph").as_int(),
            get_parameter("image_threshold").as_int(),
            get_parameter("min_area").as_double(),
            get_parameter("max_area").as_double(),
            get_parameter("saturation_threshold").as_int(),
            get_parameter("lb_hue").as_double(),
            get_parameter("ub_hue").as_double(),
            get_parameter("max_distance_lim").as_double(),
            [this](LogLevel level, const std::string& msg) {
                switch (level) {
                    case LogLevel::INFO:  RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());  break;
                    case LogLevel::WARN:  RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());  break;
                    case LogLevel::ERROR: RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); break;
                }
            });

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);

        // Plain rclcpp subscription, not image_transport -- kept as a member so
        // its callback can outlive this ctor.
        //
        // QoS is passed EXPLICITLY and is not left to the default. Unlike ROS1,
        // a ROS2 subscription only connects if its profile is compatible with
        // the publisher's; an incompatible pair delivers nothing at all, with
        // no error and no warning -- the callback simply never fires and the
        // node looks perfectly healthy. ROS2's default is RELIABLE, while
        // camera drivers normally publish BEST_EFFORT (the sensor-data preset),
        // and a RELIABLE subscriber cannot receive from a BEST_EFFORT publisher.
        // BEST_EFFORT on this side is the strictly more compatible choice: it
        // connects to publishers of either kind. Overridable so the other
        // profile can actually be tested against a real driver rather than
        // chosen from convention.
        const std::string qos_name = get_parameter("qos_profile").as_string();
        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
        if (qos_name == "default") {
            qos = rmw_qos_profile_default;
        } else if (qos_name != "sensor_data") {
            RCLCPP_WARN(get_logger(),
                        "unknown qos_profile '%s', falling back to sensor_data "
                        "(accepted: sensor_data | default)", qos_name.c_str());
        }
        RCLCPP_INFO(get_logger(), "image subscription QoS profile: %s",
                    (qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                        ? "sensor_data (best effort)" : "default (reliable)");

        // Why plain rather than image_transport::create_subscription(...,
        // "raw", qos), as an earlier version of this code did: on the Pi 5
        // (Jazzy, aarch64) that plugin never delivered a frame to the
        // callback, though the same code worked fine via image_transport on
        // the lab PC (Humble, x86_64), not reproduced here. Nothing in this
        // node used any image_transport feature beyond the "raw" transport
        // it was hardcoded to anyway, so nothing is lost by not depending on it.
        rclcpp::QoS ros_qos(rclcpp::QoSInitialization::from_rmw(qos), qos);
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            get_parameter("camera_topic").as_string(), ros_qos,
            std::bind(&CarolusRos2Node::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "carolus_ros2 up, camera_topic=%s",
                    get_parameter("camera_topic").as_string().c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        // Request bgr8 EXPLICITLY rather than passing msg->encoding through.
        // The original version of this callback did the latter, which matched
        // the synthetic-beacon test (published as bgr8) but broke silently
        // against a real driver: this project's live webcam publishes rgb8
        // (found 2026-08-20, ros-humble-usb-cam + yuyv2rgb pixel_format), and
        // cv::COLOR_BGR2HSV on RGB data swaps R and B -- it does not crash, it
        // rotates every hue reading, which would have pushed a real blue beacon
        // outside lb_hue/ub_hue silently -- the failure shape this project has
        // hit repeatedly: runs fine, looks healthy, publishes nothing. Asking cv_bridge for
        // bgr8 unconditionally makes the source encoding the driver's problem,
        // not this callback's, and removes the channel-count branch below --
        // toCvShare converts (Bayer/YUV/RGB/mono all handle it) whenever the
        // source encoding differs from the target, or shares the buffer
        // untouched when it's already bgr8.
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception (encoding=%s): %s",
                         msg->encoding.c_str(), e.what());
            return;
        }

        const cv::Mat& image = cv_ptr->image;
        cv::Mat imageMono, imageHSV;
        cv::cvtColor(image, imageMono, cv::COLOR_BGR2GRAY);
        cv::cvtColor(image, imageHSV, cv::COLOR_BGR2HSV);

        cv::Mat preprocessed = detector_->preprocessImage(imageMono);
        auto blobsOpt = detector_->findAndCalcContours(preprocessed, imageHSV, 1);
        if (!blobsOpt) {
            return;
        }
        auto bestBlobs = detector_->selectBlobs(blobsOpt.value(), min_circularity_);
        if (bestBlobs.size() != 4) {
            return;
        }

        std::vector<cv::Point2f> distorted;
        for (const auto& b : bestBlobs) distorted.emplace_back(b.blob.x, b.blob.y);
        std::vector<cv::Point2f> undistorted;
        cv::undistortPoints(distorted, undistorted, camera_matrix_, dist_coeffs_);

        std::vector<Eigen::Vector3d> imagePoints;
        for (const auto& p : undistorted) {
            imagePoints.emplace_back(Eigen::Vector3d(p.x, p.y, 1.0).normalized());
        }
        const double* prior = (warm_start_ && has_prior_) ? prior_params_ : nullptr;
        CameraPose bestPose;
        int winningCandidate = -1;  // -1 = the single-guess path; 0..5 under multi-hypothesis

        if (multi_hypothesis_sort_) {
            if (!solveMultiHypothesis(imagePoints, undistorted, prior, bestPose, winningCandidate)) {
                RCLCPP_ERROR(get_logger(), "multi_hypothesis_sort: all 6 candidate labelings failed.");
                return;
            }
        } else {
            std::vector<Eigen::Vector3d> sorted(4);
            if (!SortTargetsUsingTetrahedronGeometry(imagePoints, sorted)) {
                RCLCPP_ERROR(get_logger(), "Failed to sort targets using tetrahedron geometry.");
                return;
            }

            // BUG-132 (2026-08-25) — push the sorted points back into PIXEL
            // space before handing them to the solver. This step existed in
            // the ROS1 node all along (carolus_astrobee.cpp, under the
            // comment "UNDISTORTED POINTS ARE NORMALIZED, CONVERT BACK TO
            // ORIGINAL IMAGE SPACE") and was lost in the ROS2 port, because
            // there it sat inside a loop whose other job was filling the
            // Astrobee ff_msgs landmark array -- which this node correctly
            // does not need, so the whole loop went, coordinate conversion
            // included.
            //
            // Why it matters: ReprojectionErrorWithAnalyticDiff computes
            // `predicted_point = xp*fx + cx` (pixels, dominated by
            // cx~576/cy~372) and subtracts `observed_point_` from it.
            // Without this loop observed_point_ is a UNIT BEARING VECTOR
            // component, magnitude ~0.1 -- so the residual at the TRUE pose
            // was ~372-640 instead of ~0, and minimising it drove the pose
            // toward cancelling cx/cy rather than toward matching the
            // beacon. Measured, synthetic harness, 1000 trials
            // (test/instrument_p4p_sort.cpp): solved |t| = 11.4 m against a
            // true 0.700 m before this loop; 0.6977 m (0.33% error) after it.
            pixelSpaceConvert(sorted);

            // Second constructor arg is measType -- verified dead: stored as
            // measType_ in ceresP4P.hpp but never read anywhere in
            // ceresP4P.cpp. Value has no effect; kept only because the
            // constructor requires one.
            CobrasFumantes solver(camera_matrix_, 2);
            // BUG-088 (2026-08-25) -- warm-start from the last CONVERGED
            // solve rather than the same fixed vector every frame.
            // `has_prior_` starts false (first frame, and any frame after a
            // stretch with no prior convergence, falls back to the original
            // fixed default via nullptr) -- never warm-starts from a guess
            // we have no evidence is good. The prior is only updated below
            // on solver_converged==true, so a bad solve can't poison the
            // next attempt.
            solver.computeAndValidatePosesWithRefinement(sorted, known_points_, undistorted, bestPose, prior);
        }

        if (!bestPose.R.allFinite() || !bestPose.t.allFinite()) {
            RCLCPP_ERROR(get_logger(), "Pose solve produced non-finite result.");
            return;
        }

        if (bestPose.solver_converged) {
            for (int i = 0; i < 6; ++i) prior_params_[i] = bestPose.solved_params[i];
            has_prior_ = true;
        }

        RCLCPP_INFO(get_logger(), "[P4P] final_cost=%.6g iterations=%d converged=%d warm_start=%s candidate=%d",
                    bestPose.solver_final_cost, bestPose.solver_iterations,
                    static_cast<int>(bestPose.solver_converged),
                    (warm_start_ && has_prior_) ? "prior" : "fixed", winningCandidate);

        Eigen::Quaterniond q(bestPose.R);
        geometry_msgs::msg::PoseStamped out;
        out.header = msg->header;
        out.pose.position.x = bestPose.t(0);
        out.pose.position.y = bestPose.t(1);
        out.pose.position.z = bestPose.t(2);
        out.pose.orientation.x = q.x();
        out.pose.orientation.y = q.y();
        out.pose.orientation.z = q.z();
        out.pose.orientation.w = q.w();
        pose_pub_->publish(out);
    }

    // BUG-132's conversion, factored out so both the single-guess path and
    // solveMultiHypothesis (which needs it once per candidate) call the
    // identical logic rather than a second copy that could drift.
    void pixelSpaceConvert(std::vector<Eigen::Vector3d>& pts) const {
        const double k_fx = camera_matrix_.at<double>(0, 0);
        const double k_fy = camera_matrix_.at<double>(1, 1);
        const double k_cx = camera_matrix_.at<double>(0, 2);
        const double k_cy = camera_matrix_.at<double>(1, 2);
        for (auto& p : pts) {
            p(0) = p(0) * k_fx + k_cx;
            p(1) = p(1) * k_fy + k_cy;
        }
    }

    // BUG-131 (2026-08-25). Direct reimplementation of
    // SortTargetsUsingTetrahedronGeometry's own labeling steps (FindMidpoint
    // / Midpoint2P3P4 inlined here since they are pose_est.cpp-local; the
    // disambiguation step, FindP1P2Indices, is the project's own unchanged
    // function, now declared in pose_est.hpp), parameterised by a FORCED
    // p1p2 pair instead of always picking the max-angular-separation one.
    // Verified synthetically against this exact algorithm in
    // test/instrument_multi_hypothesis_sort.cpp before being wired in here.
    static bool labelForCandidate(const std::vector<Eigen::Vector3d>& pts, int candidateIdx,
                                  std::vector<Eigen::Vector3d>& out) {
        static const std::pair<int, int> kPairTable[6] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
        static const std::pair<int, int> kNotPairTable[6] = {{2, 3}, {1, 3}, {1, 2}, {0, 3}, {0, 2}, {0, 1}};
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

    // Tries all 6 candidate labelings of `imagePoints` (unit bearing
    // vectors, same input SortTargetsUsingTetrahedronGeometry itself takes),
    // solving the REAL production CobrasFumantes for each (same warm-start
    // prior offered to every candidate, since they all explain the same
    // frame), and keeps whichever has the lowest final_cost -- exactly the
    // criterion verified in test/instrument_multi_hypothesis_sort.cpp
    // (worst-case error 37.4cm -> 5.8cm at a synthetic near-tie; ~0.3ms/frame
    // for up to 6 solves, negligible against this camera's frame budget).
    // Returns false only if every one of the 6 candidates fails to label
    // (the FindP1P2Indices disambiguation itself hits its own 2D near-tie) --
    // a stricter failure condition than the single-guess path, by design.
    bool solveMultiHypothesis(const std::vector<Eigen::Vector3d>& imagePoints,
                              const std::vector<cv::Point2f>& undistorted, const double* prior,
                              CameraPose& bestPose, int& winningCandidate) const {
        bestPose.solver_final_cost = -1.0;
        double bestCost = std::numeric_limits<double>::infinity();
        winningCandidate = -1;
        for (int c = 0; c < 6; ++c) {
            std::vector<Eigen::Vector3d> candidate;
            if (!labelForCandidate(imagePoints, c, candidate)) continue;
            pixelSpaceConvert(candidate);
            CameraPose trial;
            CobrasFumantes solver(camera_matrix_, 2);
            solver.computeAndValidatePosesWithRefinement(candidate, known_points_, undistorted, trial, prior);
            if (!trial.R.allFinite() || !trial.t.allFinite()) continue;
            if (trial.solver_final_cost < bestCost) {
                bestCost = trial.solver_final_cost;
                bestPose = trial;
                winningCandidate = c;
            }
        }
        return winningCandidate != -1;
    }

    std::unique_ptr<BeaconDetector> detector_;
    cv::Mat camera_matrix_, dist_coeffs_;
    std::vector<Eigen::Vector3d> known_points_;
    double min_circularity_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // BUG-088 warm-start state. has_prior_ starts false; prior_params_'s
    // initial value is never read while that holds, but is set to the same
    // constant the solver has always started from, in case that ever changes.
    bool warm_start_ = true;
    bool has_prior_ = false;
    double prior_params_[6] = {0.0, 0.0, -0.001, 0.0, 0.0, 0.7};

    // BUG-131 (2026-08-25) -- see the constructor's declare_parameter for
    // scope/status. Off by default, not yet hardware-validated.
    bool multi_hypothesis_sort_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarolusRos2Node>());
    rclcpp::shutdown();
    return 0;
}
