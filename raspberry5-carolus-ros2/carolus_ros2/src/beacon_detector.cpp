#include "carolus_node/beacon_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <limits>
#include <mutex>
#include <numeric>

namespace {
// printf-style formatting, matching what ROS_INFO/ROS_ERROR did internally
// for the call sites moved here -- keeps log text byte-identical once routed
// through the wrapper's logger.
std::string format(const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    return std::string(buf);
}
}  // namespace

BeaconDetector::BeaconDetector(
    int kernel_size_gaussian,
    int kernel_size_morph,
    int image_threshold,
    double min_area,
    double max_area,
    int saturation_threshold,
    double lb_hue,
    double ub_hue,
    double max_distance_lim,
    LogFn logger)
    : kernel_size_gaussian_(kernel_size_gaussian),
      kernel_size_morph_(kernel_size_morph),
      image_threshold_(image_threshold),
      min_area_(min_area),
      max_area_(max_area),
      saturation_threshold_(saturation_threshold),
      lb_hue_(lb_hue),
      ub_hue_(ub_hue),
      max_distance_lim_(max_distance_lim),
      log_(std::move(logger)) {}

cv::Mat BeaconDetector::preprocessImage(const cv::Mat& image) const {
    cv::Mat blurred, thresholded;
    cv::GaussianBlur(image, blurred, cv::Size(kernel_size_gaussian_, kernel_size_gaussian_), 0);
    double threshValue = image_threshold_;
    cv::threshold(blurred, thresholded, threshValue, 255, cv::THRESH_BINARY);
    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(kernel_size_morph_, kernel_size_morph_));
    cv::morphologyEx(thresholded, thresholded, cv::MORPH_CLOSE, morph_kernel, cv::Point(-1, -1), 1);
    return thresholded;
}

std::optional<std::vector<BlobCarolus>> BeaconDetector::findAndCalcContours(
    const cv::Mat& image, const cv::Mat& originalImageHSV, int num_threads) const {
    std::vector<std::vector<cv::Point>> contours;

    auto start = std::chrono::high_resolution_clock::now();
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    log_(LogLevel::INFO, format("Time to find contours: %f seconds", duration.count()));

    if (contours.empty() || contours.size() > 1000) {
        log_(LogLevel::ERROR, format("ERROR: %lu contours found", contours.size()));
        return std::nullopt;
    }

    std::vector<BlobCarolus> blobs;
    blobs.reserve(contours.size());

    std::vector<std::future<void>> futures;
    std::mutex blobs_mutex;
    int num_threads_used = std::min(num_threads, static_cast<int>(contours.size()));
    int contours_per_thread = contours.size() / num_threads_used;
    int remainder = contours.size() % num_threads_used;

    int start_idx = 0;

    for (int t = 0; t < num_threads_used; ++t) {
        int end_idx = start_idx + contours_per_thread + (t < remainder ? 1 : 0);
        futures.emplace_back(std::async(std::launch::async, [start_idx, end_idx, &contours, &blobs, &blobs_mutex, &originalImageHSV, this]() {
            for (int i = start_idx; i < end_idx; ++i) {
                const auto& contour = contours[i];

                if (contours[i].size() < 5 || contour.empty()) {
                    continue;
                }

                cv::Moments moments = cv::moments(contour);
                if (moments.m00 < min_area_ || moments.m00 > max_area_) {
                    continue;
                }
                double perimeter = cv::arcLength(contour, true);
                double circularity = (4 * CV_PI * moments.m00) / (perimeter * perimeter);
                double x = moments.m10 / moments.m00;
                double y = moments.m01 / moments.m00;

                cv::Rect boundingRect = cv::boundingRect(contour);
                cv::Mat BlobRegion = originalImageHSV(boundingRect);

                cv::Mat maskHSV;
                cv::inRange(BlobRegion, cv::Scalar(0, saturation_threshold_, 0), cv::Scalar(180, 255, 255), maskHSV);
                cv::Scalar meanHSV;
                std::vector<cv::Mat> hsvChannels;
                cv::split(BlobRegion, hsvChannels);
                cv::Mat hueChannel = hsvChannels[0];

                double sumSin = 0.0;
                double sumCos = 0.0;
                int count = 0;

                for (int r = 0; r < hueChannel.rows; ++r) {
                    for (int c = 0; c < hueChannel.cols; ++c) {
                        if (maskHSV.at<uchar>(r, c) != 0) {
                            double hue = hueChannel.at<uchar>(r, c) * 2.0 * CV_PI / 180.0;
                            sumSin += std::sin(hue);
                            sumCos += std::cos(hue);
                            ++count;
                        }
                    }
                }

                double meanAngle = std::atan2(sumSin / count, sumCos / count) * 180.0 / CV_PI;
                if (meanAngle < 0) meanAngle += 360.0;
                meanHSV[0] = meanAngle / 2.0;

                if (std::isnan(meanHSV[0]) || std::isinf(meanHSV[0])) {
                    continue;
                }
                if (meanHSV[0] < 27) {
                    meanHSV[0] = 180 - meanHSV[0];
                }
                if (meanHSV[0] < lb_hue_ || meanHSV[0] > ub_hue_) {
                    continue;
                }

                BlobCarolus blobCarolus;
                blobCarolus.blob = {x, y};
                blobCarolus.properties = {perimeter, moments.m00, circularity, meanHSV[0], boundingRect};

                std::lock_guard<std::mutex> lock(blobs_mutex);
                blobs.emplace_back(std::move(blobCarolus));
            }
        }));
        start_idx = end_idx;
    }

    for (auto& fut : futures) {
        fut.get();
    }

    return blobs;
}

std::optional<std::vector<BlobCarolus>> BeaconDetector::findAndCalcContoursMono(
    const cv::Mat& image, int num_threads) const {
    std::vector<std::vector<cv::Point>> contours;

    auto start = std::chrono::high_resolution_clock::now();
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    log_(LogLevel::INFO, format("Time to find contours: %f seconds", duration.count()));

    if (contours.empty() || contours.size() > 1000) {
        log_(LogLevel::ERROR, format("ERROR: %lu contours found", contours.size()));
        return std::nullopt;
    }

    std::vector<BlobCarolus> blobs;
    blobs.reserve(contours.size());

    std::vector<std::future<void>> futures;
    std::mutex blobs_mutex;
    int num_threads_used = std::min(num_threads, static_cast<int>(contours.size()));
    int contours_per_thread = contours.size() / num_threads_used;
    int remainder = contours.size() % num_threads_used;

    int start_idx = 0;

    for (int t = 0; t < num_threads_used; ++t) {
        int end_idx = start_idx + contours_per_thread + (t < remainder ? 1 : 0);
        futures.emplace_back(std::async(std::launch::async, [this, t, start_idx, end_idx, contours_per_thread, remainder, &contours, &blobs, &blobs_mutex]() {
            for (int i = start_idx; i < end_idx; ++i) {
                const auto& contour = contours[i];

                cv::Moments moments = cv::moments(contour);
                if (moments.m00 < min_area_ || moments.m00 > max_area_) {
                    continue;
                }
                double perimeter = cv::arcLength(contour, true);
                double circularity = (4 * CV_PI * moments.m00) / (perimeter * perimeter);
                double x = moments.m10 / moments.m00;
                double y = moments.m01 / moments.m00;

                cv::Rect boundingRect = cv::boundingRect(contour);

                BlobCarolus blobCarolus;
                blobCarolus.blob = {x, y};
                blobCarolus.properties = {perimeter, moments.m00, circularity, 180, boundingRect};

                std::lock_guard<std::mutex> lock(blobs_mutex);
                blobs.emplace_back(std::move(blobCarolus));
            }
        }));
        start_idx = end_idx;
    }

    for (auto& fut : futures) {
        fut.get();
    }

    return blobs;
}

std::vector<BlobCarolus> BeaconDetector::selectBlobs(
    const std::vector<BlobCarolus>& blobs, double min_circularity) const {
    std::vector<BlobCarolus> filtered_blobs;
    filtered_blobs.reserve(blobs.size());

    if (blobs.size() < 4) {
        log_(LogLevel::ERROR, "Not enough blobs < 4.");
        return {};
    }

    for (const auto& blob : blobs) {
        if (blob.properties.circularity >= min_circularity) {
            filtered_blobs.emplace_back(blob);
        }
    }

    if (filtered_blobs.size() < 4) {
        log_(LogLevel::ERROR, "Not enough blobs with required circularity.");
        return {};
    }

    double min_variation = std::numeric_limits<double>::max();
    std::vector<BlobCarolus> best_group;
    double max_distance_best_group;

    for (size_t i = 0; i < filtered_blobs.size() - 3; ++i) {
        for (size_t j = i + 1; j < filtered_blobs.size() - 2; ++j) {
            for (size_t k = j + 1; k < filtered_blobs.size() - 1; ++k) {
                for (size_t l = k + 1; l < filtered_blobs.size(); ++l) {
                    double dist_ij = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[j].blob.x, filtered_blobs[i].blob.y - filtered_blobs[j].blob.y);
                    double dist_ik = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[k].blob.x, filtered_blobs[i].blob.y - filtered_blobs[k].blob.y);
                    double dist_il = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[l].blob.x, filtered_blobs[i].blob.y - filtered_blobs[l].blob.y);
                    double dist_jk = std::hypot(filtered_blobs[j].blob.x - filtered_blobs[k].blob.x, filtered_blobs[j].blob.y - filtered_blobs[k].blob.y);
                    double dist_jl = std::hypot(filtered_blobs[j].blob.x - filtered_blobs[l].blob.x, filtered_blobs[j].blob.y - filtered_blobs[l].blob.y);
                    double dist_kl = std::hypot(filtered_blobs[k].blob.x - filtered_blobs[l].blob.x, filtered_blobs[k].blob.y - filtered_blobs[l].blob.y);

                    std::vector<double> distances = {dist_ij, dist_ik, dist_il, dist_jk, dist_jl, dist_kl};
                    double max_distance = *std::max_element(distances.begin(), distances.end());
                    double mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
                    double distance_variance = std::accumulate(distances.begin(), distances.end(), 0.0,
                        [mean_distance](double sum, double distance) {
                            return sum + std::pow(distance - mean_distance, 2);
                        }) / distances.size();

                    if (max_distance > max_distance_lim_) {
                        continue;
                    }

                    std::vector<double> areas = {
                        filtered_blobs[i].properties.m00,
                        filtered_blobs[j].properties.m00,
                        filtered_blobs[k].properties.m00,
                        filtered_blobs[l].properties.m00
                    };
                    double mean_area = std::accumulate(areas.begin(), areas.end(), 0.0) / areas.size();
                    double area_variance = std::accumulate(areas.begin(), areas.end(), 0.0,
                        [mean_area](double sum, double area) {
                            return sum + std::pow(area - mean_area, 2);
                        }) / areas.size();

                    std::vector<double> hues = {
                        filtered_blobs[i].properties.hue,
                        filtered_blobs[j].properties.hue,
                        filtered_blobs[k].properties.hue,
                        filtered_blobs[l].properties.hue
                    };
                    double mean_hue = std::accumulate(hues.begin(), hues.end(), 0.0) / areas.size();
                    double intensity_hues = std::accumulate(hues.begin(), hues.end(), 0.0,
                        [mean_hue](double sum, double hues) {
                            return sum + std::pow(hues - mean_hue, 2);
                        }) / hues.size();
                    (void)intensity_hues;  // computed, unused in the combined score -- matches the original

                    double combined_variance = distance_variance + 5 * area_variance;

                    if (combined_variance < min_variation) {
                        min_variation = combined_variance;
                        best_group = {filtered_blobs[i], filtered_blobs[j], filtered_blobs[k], filtered_blobs[l]};
                        max_distance_best_group = max_distance;
                    }
                }
            }
        }
    }
    log_(LogLevel::INFO, format("max distance = %f", max_distance_best_group));
    return best_group;
}

std::vector<BlobCarolus> BeaconDetector::selectBlobsMono(
    const std::vector<BlobCarolus>& blobs, double min_circularity) const {
    std::vector<BlobCarolus> filtered_blobs;
    filtered_blobs.reserve(blobs.size());

    if (blobs.size() < 4) {
        log_(LogLevel::ERROR, "Not enough blobs < 4.");
        return {};
    }

    for (const auto& blob : blobs) {
        if (blob.properties.circularity >= min_circularity) {
            filtered_blobs.emplace_back(blob);
        }
    }

    if (filtered_blobs.size() < 4) {
        log_(LogLevel::ERROR, "Not enough blobs with required circularity.");
        return {};
    }

    double min_variation = std::numeric_limits<double>::max();
    std::vector<BlobCarolus> best_group;

    for (size_t i = 0; i < filtered_blobs.size() - 3; ++i) {
        for (size_t j = i + 1; j < filtered_blobs.size() - 2; ++j) {
            for (size_t k = j + 1; k < filtered_blobs.size() - 1; ++k) {
                for (size_t l = k + 1; l < filtered_blobs.size(); ++l) {
                    double dist_ij = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[j].blob.x, filtered_blobs[i].blob.y - filtered_blobs[j].blob.y);
                    double dist_ik = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[k].blob.x, filtered_blobs[i].blob.y - filtered_blobs[k].blob.y);
                    double dist_il = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[l].blob.x, filtered_blobs[i].blob.y - filtered_blobs[l].blob.y);
                    double dist_jk = std::hypot(filtered_blobs[j].blob.x - filtered_blobs[k].blob.x, filtered_blobs[j].blob.y - filtered_blobs[k].blob.y);
                    double dist_jl = std::hypot(filtered_blobs[j].blob.x - filtered_blobs[l].blob.x, filtered_blobs[j].blob.y - filtered_blobs[l].blob.y);
                    double dist_kl = std::hypot(filtered_blobs[k].blob.x - filtered_blobs[l].blob.x, filtered_blobs[k].blob.y - filtered_blobs[l].blob.y);

                    std::vector<double> distances = {dist_ij, dist_ik, dist_il, dist_jk, dist_jl, dist_kl};
                    double max_distance = *std::max_element(distances.begin(), distances.end());
                    double min_distance = *std::min_element(distances.begin(), distances.end());
                    double mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
                    double distance_variance = std::accumulate(distances.begin(), distances.end(), 0.0,
                        [mean_distance](double sum, double distance) {
                            return sum + std::pow(distance - mean_distance, 2);
                        }) / distances.size();

                    if (max_distance > max_distance_lim_) {
                        continue;
                    }
                    if (min_distance < 100) {
                        continue;
                    }

                    std::vector<double> areas = {
                        filtered_blobs[i].properties.m00,
                        filtered_blobs[j].properties.m00,
                        filtered_blobs[k].properties.m00,
                        filtered_blobs[l].properties.m00
                    };
                    double mean_area = std::accumulate(areas.begin(), areas.end(), 0.0) / areas.size();
                    double area_variance = std::accumulate(areas.begin(), areas.end(), 0.0,
                        [mean_area](double sum, double area) {
                            return sum + std::pow(area - mean_area, 2);
                        }) / areas.size();

                    double combined_variance = distance_variance + 1.5 * area_variance;

                    if (combined_variance < min_variation) {
                        min_variation = combined_variance;
                        best_group = {filtered_blobs[i], filtered_blobs[j], filtered_blobs[k], filtered_blobs[l]};
                    }
                }
            }
        }
    }

    return best_group;
}
