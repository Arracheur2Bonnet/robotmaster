#include "carolus_node/pose_filter.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

PoseFilter::PoseFilter(std::size_t filter_size,
                       double translation_threshold,
                       double rotation_threshold,
                       std::int64_t max_age_ns,
                       int reject_limit,
                       LogFn logger)
    : filter_size_(filter_size),
      translation_threshold_(translation_threshold),
      rotation_threshold_(rotation_threshold),
      max_age_ns_(max_age_ns),
      reject_limit_(reject_limit),
      log_(std::move(logger)) {}

void PoseFilter::reset() {
    queue_.clear();
    reject_count_ = 0;
    has_accepted_ = false;
    last_accepted_ns_ = 0;
}

CameraPose PoseFilter::filter(const CameraPose& new_pose, std::int64_t timestamp_ns) {
    // Staleness reset. Integer subtraction, so the comparison against
    // max_age_ns_ is exact -- no rounding can move a pose across the boundary,
    // which is the property a double seconds representation could not give.
    // Guarded on has_accepted_ rather than on a zero timestamp: 0 is a legal
    // instant, "nothing accepted yet" is not the same statement.
    if (!queue_.empty() && has_accepted_ &&
        (timestamp_ns - last_accepted_ns_) > max_age_ns_) {
        queue_.clear();
        reject_count_ = 0;
    }

    // Re-tested, not assumed: the block above may have just emptied the window,
    // and the original fell straight through to back() here.
    if (!queue_.empty()) {
        // BY VALUE. The reject-limit path below clears the window, which would
        // leave a reference dangling before it is returned.
        const CameraPose last_pose = queue_.back();

        const double translation_diff = (last_pose.t - new_pose.t).norm();
        if (translation_diff > translation_threshold_) {
            log_(LogLevel::WARN,
                 "Current pose translation is too different. Ignoring current pose.");
            reject_count_++;
            if (reject_count_ > reject_limit_) {
                queue_.clear();
                reject_count_ = 0;
            }
            return last_pose;
        }

        const Eigen::Quaterniond q_last(last_pose.R);
        const Eigen::Quaterniond q_new(new_pose.R);
        const double angle_diff = q_last.angularDistance(q_new);
        if (angle_diff > rotation_threshold_) {
            log_(LogLevel::WARN,
                 "Current pose rotation is too different. Ignoring current pose.");
            reject_count_++;
            if (reject_count_ > reject_limit_) {
                queue_.clear();
                reject_count_ = 0;
            }
            return last_pose;
        }
    }

    queue_.push_back(new_pose);
    last_accepted_ns_ = timestamp_ns;
    has_accepted_ = true;
    if (queue_.size() > filter_size_) {
        queue_.pop_front();
    }

    // Average translation: arithmetic mean over the window.
    Eigen::Vector3d avg_t = Eigen::Vector3d::Zero();
    for (const auto& pose : queue_) {
        avg_t += pose.t;
    }
    avg_t /= static_cast<double>(queue_.size());

    // Average rotation: incremental SLERP with weight 1/(count+1), which walks
    // the running mean toward each successive sample. Kept exactly as the
    // original -- this is a behaviour-preserving extraction, and the choice of
    // averaging scheme is not what is under change here.
    Eigen::Quaterniond q_avg;
    bool initialized = false;
    int count = 0;
    for (const auto& pose : queue_) {
        const Eigen::Quaterniond q_pose(pose.R);
        if (!initialized) {
            q_avg = q_pose;
            initialized = true;
        } else {
            const double w = 1.0 / (count + 1);
            q_avg = q_avg.slerp(w, q_pose);
        }
        count++;
    }

    CameraPose filtered_pose;
    filtered_pose.R = q_avg.normalized().toRotationMatrix();
    filtered_pose.t = avg_t;
    return filtered_pose;
}
