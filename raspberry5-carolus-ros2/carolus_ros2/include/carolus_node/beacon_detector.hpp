/**
 * @ Description:
 * BEACON DETECTION -- BLOB PREPROCESSING, CONTOUR EXTRACTION, AND SELECTION.
 *
 * Extracted 2026-08-18 from CarolusRexNode (carolus_astrobee.cpp), where these
 * five methods lived as private members of the ROS node class. Their
 * signatures were already pure OpenCV/Eigen (no ROS types), but their bodies
 * called ROS_INFO/ROS_ERROR directly -- real coupling, not just an
 * appearance of one. That coupling is removed here via an injected logger
 * callback (default: no-op), so this class links against Ceres/Eigen/OpenCV
 * only. Motivation: make the target middleware (ROS1, ROS2 Humble, or a
 * newer ROS2 distro) a later choice instead of baking it into the algorithm.
 *
 * Member-variable coupling was checked before this extraction, not assumed
 * clean: the only state these methods touch is the 9 config scalars taken
 * as constructor arguments below, all loaded once at startup in the
 * original code and never reassigned. Runtime/stateful members
 * (cameraMatrix_, distCoeffs_, knownPoints_, the image queue/threading
 * state) are untouched by these methods and stay out of scope here --
 * they belong to CarolusRexNode's processBlobs/imageCallback.
 */

#ifndef BEACON_DETECTOR_HPP
#define BEACON_DETECTOR_HPP
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "carolus_node/carolus_types.hpp"

enum class LogLevel { INFO, WARN, ERROR };

class BeaconDetector {
public:
    using LogFn = std::function<void(LogLevel, const std::string&)>;

    BeaconDetector(
        int kernel_size_gaussian,
        int kernel_size_morph,
        int image_threshold,
        double min_area,
        double max_area,
        int saturation_threshold,
        double lb_hue,
        double ub_hue,
        double max_distance_lim,
        LogFn logger = [](LogLevel, const std::string&) {}
    );

    // Gaussian blur -> threshold -> morphological close. Genuinely ROS-free
    // in the original code too, body included -- moved unchanged.
    cv::Mat preprocessImage(const cv::Mat& image) const;

    // Colour path: contours -> per-contour area/circularity/hue filter.
    std::optional<std::vector<BlobCarolus>> findAndCalcContours(
        const cv::Mat& image, const cv::Mat& originalImageHSV, int num_threads) const;

    // Mono path: same contour/area/circularity filter, no hue extraction.
    // This is the method that runs by default (`mono` defaults to true in
    // the wrapper) -- easy to miss since it wasn't in the wrapper's public
    // "selection" surface, but it is the common case, not an edge case.
    std::optional<std::vector<BlobCarolus>> findAndCalcContoursMono(
        const cv::Mat& image, int num_threads) const;

    // 4-of-N combinatorial search minimising combined distance+area variance
    // (colour path -- also folds hue into the search).
    std::vector<BlobCarolus> selectBlobs(
        const std::vector<BlobCarolus>& blobs, double min_circularity) const;

    // Same combinatorial search, mono variant (adds a minimum-spacing floor
    // the colour path does not have -- moved as-is, not reconciled).
    std::vector<BlobCarolus> selectBlobsMono(
        const std::vector<BlobCarolus>& blobs, double min_circularity) const;

private:
    int kernel_size_gaussian_;
    int kernel_size_morph_;
    int image_threshold_;
    double min_area_;
    double max_area_;
    int saturation_threshold_;
    double lb_hue_;
    double ub_hue_;
    double max_distance_lim_;
    LogFn log_;
};

#endif  // BEACON_DETECTOR_HPP
