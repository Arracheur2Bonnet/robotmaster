/**
 * @ Description:
 * FIFO OUTLIER-REJECTION FILTER OVER THE LAST N POSES.
 *
 * Extracted 2026-08-19 from CarolusRexNode::getFilteredPose
 * (carolus_astrobee.cpp), the second step of making the Carolus core
 * middleware-agnostic after beacon_detector.hpp. The filtering maths was
 * already plain Eigen; the only ROS types in it were the ros::Time parameter
 * and the ros::Duration built from max_time_fifo, plus two ROS_WARN calls.
 * Those are the three things this file removes.
 *
 * WHY int64_t NANOSECONDS AND NOT double SECONDS.
 * The obvious "generic scalar timestamp" is a double holding seconds since
 * epoch, and that is what this project first proposed. It does not work, and
 * the reason is arithmetic rather than stylistic: at a 2026-era epoch value
 * (~1.787e9 s) an IEEE-754 binary64 has a spacing of 238.4185791015625 ns
 * between adjacent representable values, so `t + 1ns == t` evaluates true.
 * A ROS timestamp cannot round-trip through it. A signed 64-bit nanosecond
 * count carries both wire formats losslessly with room to spare:
 *
 *     ROS1 max (uint32 s, uint32 ns)  =  4 294 967 295 999 999 999 ns
 *     ROS2 min (int32 s,  uint32 ns)  = -2 147 483 648 000 000 000 ns
 *     INT64_MAX                       =  9 223 372 036 854 775 807 ns
 *
 * CLOCK DOMAIN IS THE CALLER'S CONTRACT. This class does no conversion and
 * reads no clock. Every timestamp handed to filter() must come from the same
 * clock as the previous one; mixing acquisition time with wall time, or ROS
 * simulated time with steady time, produces meaningless ages and this class
 * cannot detect it. The ROS1 wrapper passes the image acquisition stamp.
 *
 * FOUR DEFECTS IN THE ORIGINAL ARE FIXED HERE rather than carried over. They
 * are listed because the extraction is otherwise behaviour-preserving, and a
 * reader diffing this against the old node needs to know which differences
 * are deliberate:
 *   1. The staleness test compared old-minus-new, so it was negative on any
 *      forward-moving stream and never fired -- not even across the long gaps
 *      it exists for.
 *   2. Clearing the window on staleness fell through to a back() call on the
 *      just-emptied container. Undefined behaviour, reached only if (1) were
 *      fixed alone -- the two defects masked each other.
 *   3. On the reject-limit path the window was cleared and a reference into
 *      it then returned. Also undefined behaviour, and unlike (2) this one is
 *      reachable in normal operation: reject_limit consecutive outliers is
 *      all it takes. Fixed by holding the last pose by value.
 *   4. The translation-rejection branch logged the word "rotation", so the
 *      log could not distinguish which gate had fired.
 */

#ifndef CAROLUS_POSE_FILTER_HPP
#define CAROLUS_POSE_FILTER_HPP
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

#include "carolus_node/beacon_detector.hpp"  // LogLevel
#include "carolus_node/pose_est.hpp"         // CameraPose

class PoseFilter {
public:
    using LogFn = std::function<void(LogLevel, const std::string&)>;

    /**
     * @param filter_size           window length in poses (original: filter_size, 7)
     * @param translation_threshold per-pose translation rejection gate, metres (0.5)
     * @param rotation_threshold    per-pose rotation rejection gate, radians (0.3)
     * @param max_age_ns            window staleness reset, NANOSECONDS
     *                              (original: max_time_fifo = 5.0 s = 5'000'000'000)
     * @param reject_limit          consecutive rejections before the window is
     *                              dropped entirely (5)
     * @param logger                defaults to a no-op so this class links with
     *                              zero ROS symbols and runs standalone
     */
    PoseFilter(std::size_t filter_size,
               double translation_threshold,
               double rotation_threshold,
               std::int64_t max_age_ns,
               int reject_limit,
               LogFn logger = [](LogLevel, const std::string&) {});

    /**
     * Feed one solved pose. Returns the window average when accepted, or the
     * last accepted pose when rejected -- same contract as the original.
     */
    CameraPose filter(const CameraPose& new_pose, std::int64_t timestamp_ns);

    /** Acquisition stamp of the most recent ACCEPTED pose, nanoseconds.
     *  The ROS1 node publishes this as the pose header stamp rather than
     *  now(), so it must stay observable from outside the filter. */
    std::int64_t lastAcceptedTimestampNs() const { return last_accepted_ns_; }

    /** False until the first pose is accepted. Distinguishes "no pose yet"
     *  from a genuine timestamp of 0, which a bare zero cannot. */
    bool hasAcceptedPose() const { return has_accepted_; }

    std::size_t windowSize() const { return queue_.size(); }
    void reset();

private:
    std::size_t   filter_size_;
    double        translation_threshold_;
    double        rotation_threshold_;
    std::int64_t  max_age_ns_;
    int           reject_limit_;
    LogFn         log_;

    std::deque<CameraPose> queue_;
    std::int64_t last_accepted_ns_ = 0;
    bool         has_accepted_     = false;
    int          reject_count_     = 0;
};

#endif  // CAROLUS_POSE_FILTER_HPP
