/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-06-28 00:53:33
 * @ Modified by: Hugo MAS
 * @ Modified time: 2025-12-10 15:53:12 PM
 * @ Description:
 * POSE ESTIMATION NODE FROM A 4 POINT TARGET NODE USING ROS
 * (NOT USING CV_BRIDGE AS IT MAY NOT BE COMPATIBLE WITH RESOURCE CONSTRAINED/CUSTOMS SYSTEMS)
 * 
 * ---------------------------------------------------------
 * SAY MY NAME WHEN YOU PRAY TO THE SKIES, SEE CAROLUS RISE
 * ---------------------------------------------------------
 * 
 */

#include <ros/ros.h>
#include <utility>
#include <chrono>
#include <vector>
#include <iostream>
#include <optional>
#include <thread>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <condition_variable>
#include <atomic>
#include <limits>
#include <image_transport/image_transport.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ff_msgs/VisualLandmark.h>
#include <ff_msgs/VisualLandmarks.h>
#include "carolus_node/carolus_types.hpp"
#include "carolus_node/pose_est.hpp"
#include "carolus_node/ceresP4P.hpp"
#include "carolus_node/beacon_detector.hpp"
#include "carolus_node/pose_filter.hpp"

class CarolusRexNode {
public:
    CarolusRexNode(ros::NodeHandle& nh) :
        nh_(nh),
        image_transport_(nh_),
        num_threads_(1),
        min_circularity_(0.4),
        saturation_threshold_(15),
        keep_running_(true),
        //INITIALIZE THE QUEUE PARAMS
        max_queue_size_(10), //PLAY WITH THIS NUMBER BASED ON MEMORY USAGE
        curr_queue_size_(0),
        fisheye(false),
        fov(true),
        mono(true),
        _bot_name("wannabee")

    {
        auto private_nh_ = ros::NodeHandle("~");
        //CONTROL PARAMETERS
        private_nh_.param("queue_size", max_queue_size_int_, 10);
        max_queue_size_ = static_cast<size_t>(max_queue_size_int_);
        private_nh_.param("num_threads", num_threads_, 1);
        private_nh_.param("min_circularity", min_circularity_, 0.5);
        private_nh_.param("nav_cam", nav_cam_, true);
        private_nh_.param("dock_cam", dock_cam_, false);
        private_nh_.param("sci_cam_compressed", sci_cam_compressed_, false);
        private_nh_.param("sci_cam", sci_cam_, false);
        private_nh_.param("saturation_threshold", saturation_threshold_, 15);
        private_nh_.param("fisheye", fisheye, false);
        private_nh_.param("fov", fov, true);
        private_nh_.param("mono", mono, true);
        // BUG-088 (ported to ROS1 2026-08-27) — warm-start the solver from the
        // previous converged pose instead of the same fixed vector every frame.
        // Parameterised rather than hardcoded so the exact same binary can A/B
        // it: relaunch with _warm_start:=false to reproduce the old,
        // always-fixed-start behaviour. Matches carolus_node_ros2.cpp, where
        // this was measured on hardware (2026-08-25): a real 90->120 cm beacon
        // move gave 213.7 mm of 300 mm (71.2%) with the correct sign, against
        // 55.3% with the wrong sign across independent restarts.
        private_nh_.param("warm_start", warm_start_, true);
        // Default true since 2026-09-03 -- see the member's own comment for
        // why. Set false to restore the single-guess sort.
        private_nh_.param("multi_hypothesis_sort", multi_hypothesis_sort_, true);
        private_nh_.param("bot_name", _bot_name, std::string("wannabee"));
        private_nh_.param("frame_id_conv", frame_id_conv, false);
        // TIMESTAMP SOURCE FOR THE PUBLISHED POSE (added 2026-08-04).
        // true  -> stamp the pose with the ACQUISITION time of the image it was
        //          computed from (PoseFilter::lastAcceptedTimestampNs()).
        // false -> legacy behaviour: ros::Time::now() at publication time.
        //
        // The legacy behaviour is wrong for any fusion consumer and silently so.
        // A filter (robot_localization, MINS, any EKF) reads header.stamp to
        // place the measurement in time; stamping with now() asserts the pose
        // describes the present, when it describes an image captured N ms ago.
        // That injects a systematic time offset no filter can detect. It also
        // makes the pose's true latency unmeasurable from ROS alone, and it
        // corrupts every TF lookup done at the pose's stamp -- beacon_absolute_
        // pose.py composes the pose with the gimbal angle at the WRONG instant.
        //
        // Kept as a parameter so the previous behaviour can be restored in one
        // launch argument if a clock-skew problem surfaces between machines.
        private_nh_.param("stamp_from_acquisition", stamp_from_acquisition_, true);
        //PREPROCESSING PARAMETERS
        private_nh_.param("kernel_size_gaussian", kernel_size_gaussian_, 3);
        private_nh_.param("kernel_size_morph", kernel_size_morph_, 3);
        private_nh_.param("image_threshold", image_threshold_, 250);
        //BLOB SELECTION PARAMETERS
        private_nh_.param("min_area", min_area_, 30.0);
        private_nh_.param("max_area", max_area_, 2500.0);
        private_nh_.param("max_distance_lim", max_distance_lim_, 500.0);
        private_nh_.param("lb_hue", lb_hue_, 130.0);
        private_nh_.param("ub_hue", ub_hue_, 160.0);
        //TOPIC PARAMETERS
        private_nh_.param("dock_cam_topic", dock_cam_topic_, std::string("/hw/cam_dock"));
        private_nh_.param("nav_cam_topic", nav_cam_topic_, std::string("/hw/nav_cam"));
        private_nh_.param("sci_cam_topic", sci_cam_topic_, std::string("/hw/cam_sci/"));

        private_nh_.param("processed_image_topic", processed_image_topic_, std::string("/postprocessed/image"));
        // pose_topic param no longer loaded -- its only use (the dead
        // ff_msgs::VisualLandmarks advertise, BUG-001) was removed below.
        //BENCHTEST
        private_nh_.param("benchtest", benchtest, false);
        //FIFO
        private_nh_.param("filter_size", filter_size_int_, 7);
        filter_size_ = static_cast<size_t>(filter_size_int_);
        private_nh_.param("translation_threshold", translation_threshold_, 0.5); //m
        private_nh_.param("rotation_threshold", rotation_threshold_, 0.3); //rd
        private_nh_.param("fifo_on", fifo, true);
        private_nh_.param("max_time_fifo", max_time_fifo, 5.0);//seconds
        private_nh_.param("reject_limit", reject_limit, 5);



        if (fov && fisheye){
            ROS_ERROR("Cannot have both Fisheye and FOV enabled, disabling Fisheye");
            fisheye = false;
        }
        if (!fov && !fisheye){
            ROS_WARN("Using RADTAN distortion model, consider enabling FOV for ASTROBEE better performance");
        }
        //LOAD USER PREFERENCES AND PARAMETERS

 


        //BEACON POINTS
        std::vector<double> known_points_vector;
        if (private_nh_.getParam("known_points", known_points_vector)) {
            if (known_points_vector.size() % 3 != 0) {
                ROS_WARN("Invalid known points size, expected multiple of 3");
            } else {
                knownPoints_.clear();
                for (size_t i = 0; i < known_points_vector.size(); i += 3) {
                    knownPoints_.emplace_back(known_points_vector[i], known_points_vector[i+1], known_points_vector[i+2]);
                }
            }
        } else {
            //DEFAULT SMALL TARGETS
            // knownPoints_ = {
            //     {0.055, 0.0, 0.0},
            //     {-0.055, 0.0, 0.0},
            //     {0.0, 0.048, 0.0},
            //     {0.0, 0.0, 0.037}
            // };
            //std::cout << "Vector contents:\n" << knownPoints_ << std::endl;

            knownPoints_ = {
                {0.0825, 0.0, 0.0},
                {-0.0825, 0.0, 0.0},
                {0.0, 0.0, 0.0555},
                {0.0, 0.072, 0.0}
            };

        }
        if (nav_cam_) {
            ROS_INFO("NAV CAM ENABLED");
            if (dock_cam_ || sci_cam_compressed_ || sci_cam_) {
                ROS_ERROR("Only one camera can be enabled at a time. Disabling all except NAV CAM.");
                dock_cam_ = false;
                sci_cam_compressed_ = false;
                sci_cam_ = false;
            }
        } else if (dock_cam_) {
            ROS_INFO("DOCK CAM ENABLED");
            if (nav_cam_ || sci_cam_compressed_ || sci_cam_) {
                ROS_ERROR("Only one camera can be enabled at a time. Disabling all except DOCK CAM.");
                nav_cam_ = false;
                sci_cam_compressed_ = false;
                sci_cam_ = false;
            }
        } else if (sci_cam_compressed_) {
            ROS_INFO("SCI CAM COMPRESSED ENABLED");
            if (nav_cam_ || dock_cam_ || sci_cam_) {
                ROS_ERROR("Only one camera can be enabled at a time. Disabling all except SCI CAM COMPRESSED.");
                nav_cam_ = false;
                dock_cam_ = false;
                sci_cam_ = false;
            }
        } else if (sci_cam_) {
            ROS_INFO("SCI CAM ENABLED");
            if (nav_cam_ || dock_cam_ || sci_cam_compressed_) {
                ROS_ERROR("Only one camera can be enabled at a time. Disabling all except SCI CAM.");
                nav_cam_ = false;
                dock_cam_ = false;
                sci_cam_compressed_ = false;
            }
        } else {
            ROS_ERROR("No camera is enabled. Enabling NAV CAM by default.");
            nav_cam_ = true;
        }
            std::vector<double> distCoeffs_vector;

        if (_bot_name == "wannabee"){
            ROS_INFO("WANNABEE BOT SELECTED");

            //CAMERA PROPERTIES (PINHOLE MODEL)
            if (nav_cam_){
                //NAVCAM PARAMTERS FROM WANNABEE
                private_nh_.param("fx", fx, 562.121846);
                private_nh_.param("fy", fy, 565.312154);
                private_nh_.param("cx", cx, 659.564484);
                private_nh_.param("cy", cy, 373.599988);
                private_nh_.getParam("distortion", distCoeffs_vector);

                if (distCoeffs_vector.size() != 4) {
                    // BUG-117 (2026-08-17): this branch was silently swallowing a
                    // real configuration error -- a `distortion` param that failed
                    // to load as a 4-element list (e.g. loaded as a YAML string
                    // instead of a sequence) fell through to here with no signal
                    // anywhere, and the node ran on placeholder Astrobee values
                    // instead of the robot's own measured distortion. Warn loudly.
                    ROS_WARN("distortion param did not load as 4 elements (got %zu) "
                             "-- using placeholder Astrobee coefficients instead of "
                             "the robot's own. Check the YAML is a list, not a "
                             "quoted string.", distCoeffs_vector.size());
                    distCoeffs_vector = {-0.092775, 0.014312, 0.000103, 0.000220};
                }
            }
            if (dock_cam_){
                //NAVCAM PARAMTERS FROM WANNABEE
                private_nh_.param("fx", fx, 562.121846);
                private_nh_.param("fy", fy, 565.312154);
                private_nh_.param("cx", cx, 659.564484);
                private_nh_.param("cy", cy, 373.599988);
                private_nh_.getParam("distortion", distCoeffs_vector);

                if (distCoeffs_vector.size() != 4) {
                    ROS_WARN("distortion param did not load as 4 elements (got %zu) "
                             "-- using placeholder Astrobee coefficients instead of "
                             "the robot's own. Check the YAML is a list, not a "
                             "quoted string.", distCoeffs_vector.size());
                    distCoeffs_vector = {-0.092775, 0.014312, 0.000103, 0.000220};
                }
            }
            if (sci_cam_compressed_){
                //SCICAM COMPRESSED PARAMETERS FROM WANNABEE PLACEHOLDER
                private_nh_.param("fx", fx, 875.9435630126997);
                private_nh_.param("fy", fy, 875.5638319050095);
                private_nh_.param("cx", cx, 576.0009986724265);
                private_nh_.param("cy", cy, 342.73010114675753);
                private_nh_.getParam("distortion", distCoeffs_vector);

                if (distCoeffs_vector.size() != 4) {
                    // ROS_WARN("Using default distortion coefficients, expected 4 elements.");
                    distCoeffs_vector = {0.0, 0.0, 0.0, 0.0};
                }
            }
            if (sci_cam_){
                //SCICAM PARAMETERS FROM WANNABEE PLACEHOLDER
                private_nh_.param("fx", fx, 875.9435630126997);
                private_nh_.param("fy", fy, 875.5638319050095);
                private_nh_.param("cx", cx, 576.0009986724265);
                private_nh_.param("cy", cy, 342.73010114675753);
                private_nh_.getParam("distortion", distCoeffs_vector);

                if (distCoeffs_vector.size() != 4) {
                    // ROS_WARN("Using default distortion coefficients, expected 4 elements.");
                    distCoeffs_vector = {0.0, 0.0, 0.0, 0.0};
                }
            }
        } if (_bot_name == "bsharp") {
            ROS_INFO("BSHARP BOT SELECTED");

            // CAMERA PROPERTIES (PINHOLE MODEL)

            if (nav_cam_) {
                // NAVCAM PARAMETERS FROM BSHARP
                private_nh_.param("fx", fx, 603.78877);
                private_nh_.param("fy", fy, 602.11334);
                private_nh_.param("cx", cx, 575.92329);
                private_nh_.param("cy", cy, 495.30887);
                private_nh_.getParam("distortion", distCoeffs_vector);

                if (distCoeffs_vector.size() != 4) {
                    // ROS_WARN("Using default distortion coefficients, expected 4 elements.");
                    distCoeffs_vector = {0.993591, 0.0, 0.0, 0.0};
                }
            }

            if (dock_cam_) {
                // DOCKCAM PARAMETERS FROM BSHARP
                private_nh_.param("fx", fx, 753.50986);
                private_nh_.param("fy", fy, 751.15119);
                private_nh_.param("cx", cx, 565.35452);
                private_nh_.param("cy", cy, 483.81274);
                private_nh_.getParam("distortion", distCoeffs_vector);

                if (distCoeffs_vector.size() != 4) {
                    // ROS_WARN("Using default distortion coefficients, expected 4 elements.");
                    distCoeffs_vector = {1.00447, 0.0, 0.0, 0.0};
                }
            }
            if (sci_cam_compressed_) {
                // SCICAM COMPRESSED PARAMETERS PLACEHOLDER
                private_nh_.param("fx", fx, 875.9435630126997);
                private_nh_.param("fy", fy, 875.5638319050095);
                private_nh_.param("cx", cx, 576.0009986724265);
                private_nh_.param("cy", cy, 342.73010114675753);
                private_nh_.getParam("distortion", distCoeffs_vector);

                if (distCoeffs_vector.size() != 4) {
                    // ROS_WARN("Using default distortion coefficients, expected 4 elements.");
                    distCoeffs_vector = {0.0, 0.0, 0.0, 0.0};
                }
            }
            if (sci_cam_){
                //SCICAM PARAMETERS FROM BSHARP PLACEHOLDER
                private_nh_.param("fx", fx, 875.9435630126997);
                private_nh_.param("fy", fy, 875.5638319050095);
                private_nh_.param("cx", cx, 576.0009986724265);
                private_nh_.param("cy", cy, 342.73010114675753);
                private_nh_.getParam("distortion", distCoeffs_vector);

                if (distCoeffs_vector.size() != 4) {
                    // ROS_WARN("Using default distortion coefficients, expected 4 elements.");
                    distCoeffs_vector = {0.0, 0.0, 0.0, 0.0};
                }
            }
        
        }

       //D455 STRAIGHT FROM KALIBR
        // nh_.param("fx", fx, 423.84596179);
        // nh_.param("fy", fy, 422.96425442);
        // nh_.param("cx", cx, 423.75095666);
        // nh_.param("cy", cy, 248.55000177);
        // nh_.getParam("distortion", distCoeffs_vector);

        // if (distCoeffs_vector.size() != 4) {
        //     ROS_ERROR("Invalid distortion coefficients size, expected 4 elements, using default values.");
        //     distCoeffs_vector = {-0.04360337, 0.03103359, -0.00098949, 0.00150547};
        // }

        // if (distCoeffs_vector.size() != 4) {
        //     ROS_ERROR("Invalid distortion coefficients size, expected 4 elements, using default values.");
        //     distCoeffs_vector = {-0.02701573, 0.02348154, -0.00106455, -0.00364093};
        // }
        // nh_.param("fx", fx, 875.9435630126997);
        // nh_.param("fy", fy, 875.5638319050095);
        // nh_.param("cx", cx, 576.0009986724265);
        // nh_.param("cy", cy, 342.73010114675753);
        // nh_.getParam("distortion", distCoeffs_vector);

        // if (distCoeffs_vector.size() != 4) {
        //     ROS_ERROR("Invalid distortion coefficients size, expected 4 elements, using default values.");
        //     distCoeffs_vector = {0.0, 0.0, 0.0, 0.0};
        // }

        //CONSTRUCT CAMERA MATRIX AND DISTORTION COEFFICIENTS USING OPENCV FORMAT
        cameraMatrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                                0, fy, cy,
                                                0, 0, 1);

        distCoeffs_ = (cv::Mat_<double>(1, 4) << distCoeffs_vector[0], distCoeffs_vector[1], distCoeffs_vector[2], distCoeffs_vector[3]);

        if (fov){
                preCalculateFov(distCoeffs_);

        }

        // BeaconDetector holds no ROS dependency (2026-08-18 extraction, see
        // beacon_detector.hpp's own header comment). Constructed here, once
        // the 9 config scalars above are loaded, rather than in this class's
        // member-initializer list -- those scalars come from private_nh_.param()
        // calls in this constructor's body, not compile-time constants.
        // Logger forwards to ROS_INFO/WARN/ERROR so today's log output is
        // unchanged; a std-string payload goes through "%s" rather than
        // being interpreted as a format string itself.
        detector_ = std::make_unique<BeaconDetector>(
            kernel_size_gaussian_, kernel_size_morph_, image_threshold_,
            min_area_, max_area_, saturation_threshold_, lb_hue_, ub_hue_,
            max_distance_lim_,
            [](LogLevel level, const std::string& msg) {
                switch (level) {
                    case LogLevel::INFO:  ROS_INFO("%s", msg.c_str());  break;
                    case LogLevel::WARN:  ROS_WARN("%s", msg.c_str());  break;
                    case LogLevel::ERROR: ROS_ERROR("%s", msg.c_str()); break;
                }
            });

        // The outlier filter, likewise moved into carolus_core. max_time_fifo
        // stays a double in seconds at the ROS parameter boundary (that is what
        // the launch files hold and what operators edit); it is converted once,
        // here, into the integer nanoseconds the core works in. 5.0 s becomes
        // exactly 5'000'000'000 ns, so the reset boundary is an exact integer
        // comparison rather than a floating-point one.
        pose_filter_ = std::make_unique<PoseFilter>(
            filter_size_, translation_threshold_, rotation_threshold_,
            static_cast<std::int64_t>(max_time_fifo * 1e9), reject_limit,
            [](LogLevel level, const std::string& msg) {
                switch (level) {
                    case LogLevel::INFO:  ROS_INFO("%s", msg.c_str());  break;
                    case LogLevel::WARN:  ROS_WARN("%s", msg.c_str());  break;
                    case LogLevel::ERROR: ROS_ERROR("%s", msg.c_str()); break;
                }
            });

        //SETUP SUBSCRIBER AND PUBLISHERS
        //TEMP: LEAVING AS DEFAULT, CANNOT BE MODIFIED IN THE LAUNCH FILE 
        //      EITHER MODIFY THE CODE OR REMAP THE TOPICS FOR NOW

        //TODO: ADD NODLET OPTION TO MODIFY TOPICS AND LAUNCH MULTIPLE INSTANCES OF CAROLUSREXNODE
        if (nav_cam_){
            image_sub_ = image_transport_.subscribe(nav_cam_topic_, 10, &CarolusRexNode::imageCallback, this);
        }
        if (dock_cam_){
            image_sub_ = image_transport_.subscribe(dock_cam_topic_, 10, &CarolusRexNode::imageCallback, this);
        }
        if (sci_cam_compressed_){
            image_sub_ = image_transport_.subscribe(sci_cam_topic_, 10, &CarolusRexNode::imageCallback, this, image_transport::TransportHints("compressed"));
        }
        if (sci_cam_){
            image_sub_ = image_transport_.subscribe(sci_cam_topic_, 10, &CarolusRexNode::imageCallback, this);
        }
        image_pub_ = image_transport_.advertise(processed_image_topic_, 10);
        // ff_msgs::VisualLandmarks advertise on pose_topic_ removed 2026-08-18
        // (BUG-001, documented from the project's first week): this line
        // immediately overwrote pose_pub_ with the /pose advertise below, so
        // it was dead from day one -- confirmed again here via a repo-wide
        // grep for /loc/ar/features and ff_msgs::VisualLandmarks, no
        // subscriber found in any tracked .py/.cpp/.launch file. Static
        // check only, the robot was off; a live `rostopic info` re-check is
        // still owed next session as the final confirmation.
        // NOTE: ff_msgs stays a real dependency of this file regardless --
        // processBlobs() (below, untouched by this extraction) still builds
        // and populates a full ff_msgs::VisualLandmarks/visual_landmarks_vec_
        // every frame, and that object is ALSO never published, only its
        // .header field gets copied out. Same dead-work shape as this line,
        // found while reading this constructor, deliberately not touched
        // here -- out of scope for a beacon-detection extraction, logged as
        // a separate finding instead (journal.md 2026-08-18).
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose", 10);
        process_thread_ = std::thread(&CarolusRexNode::processImages, this);
        ROS_INFO("============================================");
        ROS_INFO("A P4P POSE ESTIMATION NODE");
        ROS_INFO("@1822 TROPICAL EMPIRE. All rights reserved.");
        ROS_INFO("============================================");
    }

    ~CarolusRexNode() {
        keep_running_ = false;
        cv_.notify_all();
        if (process_thread_.joinable()) {
            process_thread_.join();
        }
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (curr_queue_size_.load() >= max_queue_size_) {
            ROS_WARN("Image queue is full, dropping oldest image.");
            producer_image_queue_.pop();
            producer_image_queue_.push(msg);
        } else {
            producer_image_queue_.push(msg);
            curr_queue_size_.fetch_add(1, std::memory_order_relaxed);
            }
        }
        //NOW NOTIFY THE PROCESSING THREAD
        cv_.notify_one();
    }

    void processImages() {
        while (ros::ok() && keep_running_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            //WAIT FOR NEW IMAGE
            cv_.wait(lock, [this] { return !producer_image_queue_.empty() || !keep_running_; });
            if (!keep_running_) break;
            
            //ATOMICALY SWAP QUEUES
            std::swap(consumer_image_queue_, producer_image_queue_);
            //NOW UNLOCK
            lock.unlock();
            
            //PROCESS ALL IMAGES IN THE QUEUE
            while (!consumer_image_queue_.empty()) {
                auto msg = consumer_image_queue_.front();
                consumer_image_queue_.pop();
                curr_queue_size_.fetch_sub(1, std::memory_order_relaxed);

                //COLLECT TIMESTAMP
                auto timestamp = msg->header.stamp;
                cv::Mat image = convertImageMessageToMat(msg);
                if (image.empty() || !image.data) {
                    ROS_ERROR("Empty or invalid cv::Mat from image message, skipping processing");
                    continue;
                }
                if (!imagesizeSet){
                    imagesize_ = Eigen::Vector2d(static_cast<double>(image.cols), static_cast<double>(image.rows));
                    imagesizeSet = true;
                }

                if (image.empty()) {
                    ROS_ERROR("Failed to convert image message to cv::Mat. RGB8, BGR8 or MONO8 encoding expected.");
                    continue;
                }
            //TODO: ADD COMPILATION FLAG TO ENABLE/DISABLE COLOR PROCESSING
                //BEGIN PREPROCESSING

                //TODO:: REORGANIZE IMAGES FOR MONO CASE BETTER NO NEED TO CREATE A SECOND IMAGE....
                cv::Mat imageMono;
                if (image.channels() == 3) {
                    cv::cvtColor(image, imageMono, cv::COLOR_BGR2GRAY);
                    //CONVERT ORIGINAL TO HSV
                    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
                } else { //HOT FIX FOR MONO8 IMAGES
                    imageMono = image.clone();
                    // cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
                    // cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
                }
                cv::Mat preprocessedImage = detector_->preprocessImage(imageMono);

                //BEGIN BLOB DETECTION AND PROCESSING
                std::optional<std::vector<BlobCarolus>> blobCarolusVec;
                if (mono){

                    blobCarolusVec = detector_->findAndCalcContoursMono(preprocessedImage, num_threads_);

                } else{

                    blobCarolusVec = detector_->findAndCalcContours(preprocessedImage, image, num_threads_);
                    
                }
                if (blobCarolusVec) {
                    std::vector<BlobCarolus> best_blobs;

                    if (mono){
                       best_blobs  = detector_->selectBlobsMono(blobCarolusVec.value(), min_circularity_);
                    } else{
                        best_blobs = detector_->selectBlobs(blobCarolusVec.value(), min_circularity_);
                    }
                    //IF FOUND BLOBS, DRAW THEM ON THE IMAGE AND PUBLISH
                    if (!best_blobs.empty()) {
                        if (benchtest){
                            cv::Mat coloredPreprocessedImage;
                            cv::cvtColor(preprocessedImage, coloredPreprocessedImage, cv::COLOR_GRAY2BGR);
                            //GET HUE COLOR IN BGR FOR EACH BLOB
                            int compteur=0;
                            for (const auto& blob : best_blobs) {
                                    compteur++;
                                    double hue = blob.properties.hue;
                                    cv::Mat hsvColor(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255)); // Full saturation and value
                                    cv::Mat bgrColor;
                                    cv::cvtColor(hsvColor, bgrColor, cv::COLOR_HSV2BGR);
                                    cv::Vec3b bgr = bgrColor.at<cv::Vec3b>(0, 0);

                                    //OPENCV CONVERSION
                                    cv::Scalar color(bgr[0], bgr[1], bgr[2]);

                                    ROS_INFO("Parameter of blob number %d",compteur);

                                    ROS_INFO("x=%f",blob.blob.x);
                                    ROS_INFO("y=%f",blob.blob.y);

                                    ROS_INFO("Circularity: %f",blob.properties.circularity);

                                    ROS_INFO("HUE: %f", blob.properties.hue);
                                    ROS_INFO("Area: %f", blob.properties.m00);

                                    ROS_INFO("----------------------------");
                                    cv::circle(coloredPreprocessedImage, cv::Point(blob.blob.x, blob.blob.y), 5, color, -1);
                            }

                            auto resultIMG_msg = convertMatToImageMessage(coloredPreprocessedImage, msg->header);
                            // ROS_INFO("Publishing processed image...");
                            image_pub_.publish(resultIMG_msg);
                        }
                        std::vector<Blob> blobs;
                        blobs.reserve(best_blobs.size());
                        for (const auto& blobCarolus : best_blobs) {
                            blobs.emplace_back(blobCarolus.blob);
                        }
                        processBlobs(blobs, timestamp);
                    }
                } else {
                    ROS_INFO("No valid contours found.");
                }
                //RELEASE IMAGE MEMORY
                image.release();
                imageMono.release();
                preprocessedImage.release();
            }
        }
    }
    // Thin forwarder onto PoseFilter (carolus_core, ROS-free). The filtering
    // maths, the window and the rejection gates all live there now; what stays
    // here is the one thing that is genuinely ROS: turning a ros::Time into the
    // integer nanosecond count the core speaks. See pose_filter.hpp for why the
    // core takes int64 nanoseconds rather than double seconds.
    CameraPose getFilteredPose(const CameraPose& new_pose, const ros::Time& timestamp) {
        return pose_filter_->filter(new_pose, static_cast<std::int64_t>(timestamp.toNSec()));
    }

 void processBlobs(const std::vector<Blob>& blobs, const ros::Time& timestamp) {
    if (blobs.size() < 4) {
        ROS_ERROR("Not enough blobs to calculate 6DoF state.");
        return;
    }

    std::vector<cv::Point2f> distortedPoints;
    distortedPoints.reserve(blobs.size());
    for (const auto& blob : blobs) {
        distortedPoints.emplace_back(blob.x, blob.y);
    }

    std::vector<cv::Point2f> undistortedPoints;
    if (!fov) {
        if (fisheye){
        //FISHEYE CAMERA MODEL
        cv::fisheye::undistortPoints(distortedPoints, undistortedPoints, cameraMatrix_, distCoeffs_);
        } else { //RADTAN
        cv::undistortPoints(distortedPoints, undistortedPoints, cameraMatrix_, distCoeffs_);
        }
    } else {
        //FOV
        undistortedPoints = undistortAstrobeeFov(distortedPoints, imagesize_);
        
    }

   

    std::vector<Eigen::Vector3d> imagePoints;
    std::vector<Eigen::Vector3d> sortedImagePoints(4);




    CameraPose bestPose;
    int measType_ = 2;

    if (!fov){
         //UNDISTORTED POINTS ARE NORMALIZED, CONVERT BACK TO ORIGINAL IMAGE SPACE
        imagePoints.reserve(undistortedPoints.size());
        for (const auto& point : undistortedPoints) {
        imagePoints.emplace_back(Eigen::Vector3d(point.x, point.y, 1.0).normalized());
        }

        const double* prior = (warm_start_ && has_prior_) ? prior_params_ : nullptr;

        if (multi_hypothesis_sort_) {
            int winningCandidate = -1;
            if (!solveMultiHypothesis(imagePoints, undistortedPoints, cameraMatrix_, true,
                                      measType_, prior, bestPose, sortedImagePoints,
                                      winningCandidate)) {
                ROS_ERROR("multi_hypothesis_sort: all 24 candidate labelings failed.");
                return;
            }
        } else {
            bool success = SortTargetsUsingTetrahedronGeometry(imagePoints, sortedImagePoints);
            if (!success) {
                ROS_ERROR("Failed to sort targets using tetrahedron geometry.");
                return;
            }
            for (int i = 0; i < sortedImagePoints.size(); i++) {
                sortedImagePoints[i](0) = sortedImagePoints[i](0) * fx +cx;
                sortedImagePoints[i](1) = sortedImagePoints[i](1) * fy +cy;
            }
            //COBRAS FUMANTES POSE SOLVER
            //THE SNAKE IS GOING TO SMOKE
            CobrasFumantes poseSolver(cameraMatrix_, measType_);
            poseSolver.computeAndValidatePosesWithRefinement(sortedImagePoints, knownPoints_,
                                                            undistortedPoints, bestPose, prior);
        }

        // Populated from whichever labeling was actually used, after the solve
        // rather than during the sort. It used to run ABOVE the `if (!success)`
        // check, so a failed sort left this member holding landmarks derived
        // from an unsuccessfully-sorted vector. Nothing was published on that
        // frame (the check returns immediately after), so this was latent
        // rather than active; moving it below the check removes the state
        // entirely instead of relying on the next frame to overwrite it.
        for (int i = 0; i < sortedImagePoints.size(); i++) {
            //GOTTA CREATE THE LANDMARK MESSAGE AS PER ASTORBBE DEFS
            visual_landmarks_vec_[i].u = sortedImagePoints[i](0);
            visual_landmarks_vec_[i].v = sortedImagePoints[i](1);
            visual_landmarks_vec_[i].x = knownPoints_[i](0);
            visual_landmarks_vec_[i].y = knownPoints_[i](1);
            visual_landmarks_vec_[i].z = knownPoints_[i](2);
        }
    } else {
        imagePoints.reserve(undistortedPoints.size());
        for (const auto& point : undistortedPoints) {
        imagePoints.emplace_back(Eigen::Vector3d(point.x, point.y, 1.0));
        }

        const double* prior = (warm_start_ && has_prior_) ? prior_params_ : nullptr;

        if (multi_hypothesis_sort_) {
            int winningCandidate = -1;
            // toPixelSpace=false: undistortAstrobeeFov already returns points
            // in the space camMatrixAstrobee expects, which is why the
            // original code's two scaling lines here were commented out.
            if (!solveMultiHypothesis(imagePoints, undistortedPoints, camMatrixAstrobee, false,
                                      measType_, prior, bestPose, sortedImagePoints,
                                      winningCandidate)) {
                ROS_ERROR("multi_hypothesis_sort: all 24 candidate labelings failed.");
                return;
            }
        } else {
            bool success = SortTargetsUsingTetrahedronGeometry(imagePoints, sortedImagePoints);
            if (!success) {
                ROS_ERROR("Failed to sort targets using tetrahedron geometry.");
                return;
            }
            //COBRAS FUMANTES POSE SOLVER
            //THE SNAKE IS GOING TO SMOKE
            CobrasFumantes poseSolver(camMatrixAstrobee, measType_);
            poseSolver.computeAndValidatePosesWithRefinement(sortedImagePoints, knownPoints_,
                                                            undistortedPoints, bestPose, prior);
        }

        for (int i = 0; i < sortedImagePoints.size(); i++) {
        //GOTTA CREATE THE LANDMARK MESSAGE AS PER ASTORBBE DEFS
            visual_landmarks_vec_[i].u = sortedImagePoints[i](0);
            visual_landmarks_vec_[i].v = sortedImagePoints[i](1);
            visual_landmarks_vec_[i].x = knownPoints_[i](0);
            visual_landmarks_vec_[i].y = knownPoints_[i](1);
            visual_landmarks_vec_[i].z = knownPoints_[i](2);
        }
    }
   





    // BUG-087 (2026-08-03) — report non-convergence instead of hiding it.
    // Deliberately a WARNING and not a rejection: the pose is still published
    // exactly as before, because we do not yet know how often this fires. If it
    // turns out to be rare, rejecting becomes the right fix; if it is common,
    // rejecting would silence the pipeline. Throttled so a persistent failure
    // cannot flood the log at frame rate.
    // BUG-088 (ported to ROS1 2026-08-27) — refresh the prior ONLY from a
    // converged solve. A non-converged solve's parameter vector is wherever the
    // optimiser happened to stop, which would poison every subsequent frame.
    if (bestPose.solver_converged) {
        for (int i = 0; i < 6; ++i) prior_params_[i] = bestPose.solved_params[i];
        has_prior_ = true;
    }

    if (!bestPose.solver_converged) {
        ROS_WARN_THROTTLE(2.0,
            "[P4P] solver did NOT converge (iterations=%d, final_cost=%.6g) — "
            "pose still published, treat with suspicion",
            bestPose.solver_iterations, bestPose.solver_final_cost);
    }

    // 2026-08-10 — log the residual on EVERY solve, not only on the failures.
    //
    // `solver_final_cost` was already computed (ceresP4P.cpp:70) and thrown away
    // unless the solve failed to converge, which discards the useful case: a
    // solve that converges cleanly to a WRONG answer. Round 08's arbitration
    // identified this residual as the only instrument available, without new
    // hardware, that separates two explanations of the same symptom — a rigid
    // lever arm (residual stays flat as the gimbal moves; the geometry is
    // consistent, our frame model is not) from a viewing-angle-dependent blob
    // centroid bias (residual grows with viewing angle; the measurements
    // themselves are being corrupted). The 2026-08-10 pixel-geometry check
    // could not tell those apart, because it re-used the same blob centroids.
    //
    // Throttled to 2 s: this is a trend to plot against gimbal angle across a
    // calibration sweep, not a per-frame value anyone reads live — and the
    // launcher's log path has a measured ceiling worth respecting (BUG-098).
    ROS_INFO_THROTTLE(2.0, "[P4P] final_cost=%.6g iterations=%d converged=%d",
                      bestPose.solver_final_cost, bestPose.solver_iterations,
                      static_cast<int>(bestPose.solver_converged));

    if (bestPose.R.allFinite() && bestPose.t.allFinite()) {
        CameraPose filteredPose = getFilteredPose(bestPose, timestamp);
        if (fifo){
            bestPose = filteredPose;
        }
        std::stringstream ssR;
        // Maybe a transposition is needed
        ssR << (bestPose.R.transpose()).format(Eigen::IOFormat()); 
        // ssR << bestPose.R.format(Eigen::IOFormat());
        std::stringstream sst;
        sst << bestPose.t.transpose().format(Eigen::IOFormat());

        //ROS_INFO("Rotation matrix R:\n%s", ssR.str().c_str());
        //ROS_INFO("Translation vector t:\n%s", sst.str().c_str());

        double roll, pitch, yaw, xExtracted, yExtracted, zExtracted;
        extractRPY(bestPose.R, roll, pitch, yaw);
        extractXYZ(bestPose.t, xExtracted, zExtracted, yExtracted);


        //PUB ASTROBEE POSE

        ff_msgs::VisualLandmarks PoseAstrobee;
        geometry_msgs::PoseStamped PoseAstrobeeMsgs;

        // See stamp_from_acquisition_ in the parameter block for why this is not
        // ros::Time::now(). The filter's lastAcceptedTimestampNs() is the
        // acquisition time of the most recent image that contributed to this
        // pose; the published pose may be an average over
        // the outlier-filter window, and the most recent contributing
        // measurement is the conventional stamp for such an average.
        // Falls back to now() if no pose has been accepted yet, so the very
        // first publication can never carry a zero timestamp.
        if (stamp_from_acquisition_ && pose_filter_->hasAcceptedPose()) {
            PoseAstrobee.header.stamp.fromNSec(
                static_cast<uint64_t>(pose_filter_->lastAcceptedTimestampNs()));
        } else {
            PoseAstrobee.header.stamp = ros::Time::now();
        }
        if (frame_id_conv){
            if (_bot_name == "wannabee") {
                PoseAstrobee.header.frame_id = "wannabee/body";
            } else { //default to bsharp
                PoseAstrobee.header.frame_id = "bsharp/body";
            }
        } else {
            PoseAstrobee.header.frame_id = "body";
        }

        PoseAstrobee.landmarks = std::vector<ff_msgs::VisualLandmark>(std::begin(visual_landmarks_vec_), std::end(visual_landmarks_vec_));
        if (dock_cam_){
            PoseAstrobee.camera_id = 0; //DOCKING
        } else {
            PoseAstrobee.camera_id = 1; //NAVCAM
        }
        PoseAstrobee.runtime = timestamp.toSec();//NOT SURE HERE NOT FILLED ON MARKER TRACKING


        



        //REMOVE STATIC CAST --> EIGEN ARE ALREADY DOUBLE
        // PoseAstrobee.pose.position.x = bestPose.t(0);
        // PoseAstrobee.pose.position.y = bestPose.t(1);
        // PoseAstrobee.pose.position.z = bestPose.t(2);

        PoseAstrobeeMsgs.pose.position.x=bestPose.t(0);
        PoseAstrobeeMsgs.pose.position.y=bestPose.t(1);
        PoseAstrobeeMsgs.pose.position.z=bestPose.t(2);


        //TRANSPOSE TO GET THE CORRECT ROTATION MATRIX --> IF NOT TRANSPOSED, THE ROTATION MATRIX IS INVERTED
        //DOUBLE CHECK THIS ITS 5AM
        Eigen::Quaterniond q(bestPose.R.transpose()); 
        // PoseAstrobee.pose.orientation.x = q.x();
        // PoseAstrobee.pose.orientation.y = q.y();
        // PoseAstrobee.pose.orientation.z = q.z();
        // PoseAstrobee.pose.orientation.w = q.w();

        PoseAstrobeeMsgs.pose.orientation.x=q.x();
        PoseAstrobeeMsgs.pose.orientation.y=q.y();
        PoseAstrobeeMsgs.pose.orientation.z=q.z();
        PoseAstrobeeMsgs.pose.orientation.w=q.w();

        // Publish the pose

        PoseAstrobeeMsgs.header=PoseAstrobee.header;

        pose_pub_.publish(PoseAstrobeeMsgs);
    } else {
        ROS_ERROR("No valid pose found with the required constraints.");
    }
}


//NOT SUPPORTING YUV422 IMAGE IN THIS DEBUG VERSION, SINCE BENCHTEST BAGS ARE IN RGB8 OR BGR8
//SUPPORTING ASTROBEE BAYER IMAGES
    // cv::Mat convertImageMessageToMat(const sensor_msgs::ImageConstPtr& msg) {
    //     cv::Mat mat;
    //     if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
    //         mat = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    //     } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
    //         mat = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    //     } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
    //         cv::Mat rgb(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    //         cv::cvtColor(rgb, mat, cv::COLOR_RGB2BGR);
    //     } else if (msg->encoding == "bayer_grbg8"){ //HAVE TO USE STRING HERE...
    //         cv::Mat bayer(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    //         cv::cvtColor(bayer, mat, cv::COLOR_BayerGR2BGR);
    //     } else {
    //         ROS_ERROR("Unsupported encoding type: %s", msg->encoding.c_str());
    //         return cv::Mat();
    //     }
    //     return mat.clone(); //FULL COPY CORRECTS ANY MEMORY ALIGNMENT ISSUES
    // }

    //ATTEMPTING TO REDUCE MEMORY USAGE BY NOT COPYING THE IMAGE
    cv::Mat convertImageMessageToMat(const sensor_msgs::ImageConstPtr& msg) {
    if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        return cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        return cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        cv::Mat rgb(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0]), msg->step);
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    } else if (msg->encoding == "bayer_grbg8") {
        cv::Mat bayer(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(&msg->data[0]), msg->step);
        cv::Mat bgr;
        cv::cvtColor(bayer, bgr, cv::COLOR_BayerGR2BGR);
        return bgr;
    } else {
        ROS_ERROR("Unsupported encoding type: %s", msg->encoding.c_str());
        return cv::Mat(); //EMPTY MAT
    }
}
    void preCalculateFov (const cv::Mat distCfs) {
        distortion_precalc1_ = 1 / distCfs.at<double>(0, 0);
        distortion_precalc2_ = 2 * tan(distCfs.at<double>(0, 0) / 2);
        fov_distortion_coeff = distCfs.at<double>(0, 0);
        camMatrixAstrobee = (cv::Mat_<double>(3, 3) << fx, 0, 0,
                                        0, fy, 0,
                                        0, 0, 1);
}

    
    //AT THE EDGE OF MADNESS, IN TIME OF SADNESS, AN IMORTAL SOLDIER FINDS HIS HOME!
    std::vector<cv::Point2f> undistortAstrobeeFov(const std::vector<cv::Point2f>& distortedPoints, const Eigen::Vector2d& image_size) {
        // std::vector<Eigen::Vector3d> 

        Eigen::Vector2d focal_length_(fx, fy);
        Eigen::Vector2d optical_offset_(cx, cy);

        //NOW DIVIDE IMAGE SIZE BY 2
        Eigen::Vector2d distorted_half_size_ = Eigen::Vector2d(image_size(0), image_size(1)) / 2.0;

        std::vector<cv::Point2f> undistortedPoints;
        undistortedPoints.reserve(distortedPoints.size());

        //UNDISTORT POINTS ACCORDING TO ASTROBEE FOV MODEL
        for (const auto& distortedPoint : distortedPoints) {
            Eigen::Vector2d distorted_c(distortedPoint.x, distortedPoint.y);
            //CONVERT TO IMAGE CENTER COOORDINATE FRAME
            distorted_c -= distorted_half_size_;

            //NORMALIZE THE DISTORTED POINTS AND UNDISTORT THEM
            Eigen::Vector2d norm = (distorted_c - (optical_offset_ - distorted_half_size_)).cwiseQuotient(focal_length_);
            double rd = norm.norm();
            double ru = tan(rd * fov_distortion_coeff) / distortion_precalc2_;
            double conv = 1.0;
            if (rd > 1e-5) {
                conv = ru / rd;
            }
            Eigen::Vector2d undistorted_c = conv * norm.cwiseProduct(focal_length_);

            //THIS IS BAD PRACTICE, I DON'T NEED THE POINTS IN CV FORMAT, BUT FOR NOW WE'LL SEE IF THIS WORKS...
            undistortedPoints.emplace_back(undistorted_c.x(), undistorted_c.y());
        }

    return undistortedPoints;
}

void extractRPY(const Eigen::Matrix3d& rotationMatrix, double& roll, double& pitch, double& yaw) {
    yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));   
    roll = asin(-rotationMatrix(2, 0)); //roll                  
    pitch = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    std::cout << "Roll  : " << roll * 180.0 / M_PI << "°" << std::endl;
    std::cout << "Pitch : " << pitch * 180.0 / M_PI << "°" << std::endl;
    std::cout << "Yaw   : " << yaw * 180.0 / M_PI << "°" << std::endl;
}

void extractXYZ(const Eigen::Vector3d& translationVector, double& xExtracted, double& zExtracted, double& yExtracted) {
    xExtracted = translationVector(0, 0);   
    yExtracted = translationVector(1, 0);                    
    zExtracted = translationVector(2, 0); 
    std::cout << "X  : " << xExtracted << "m" << std::endl;
    std::cout << "Y : " << yExtracted  << "m" << std::endl;
    std::cout << "Z   : " << zExtracted << "m" << std::endl;
}

        
        




    sensor_msgs::ImagePtr convertMatToImageMessage(const cv::Mat& mat, const std_msgs::Header& header) {
        sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
        msg->header = header;
        msg->height = mat.rows;
        msg->width = mat.cols;
        msg->encoding = sensor_msgs::image_encodings::BGR8;
        msg->is_bigendian = false;
        msg->step = mat.step;
        msg->data.assign(mat.datastart, mat.dataend);
        return msg;
}

//=========================================================
//DO NOT MESS WITH THESE UNLESS YOU KNOW WHAT YOU ARE DOING
    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pose_pub_;
    std::unique_ptr<BeaconDetector> detector_;  // constructed once config scalars are loaded, see ctor body
    std::unique_ptr<PoseFilter> pose_filter_;   // idem -- holds the FIFO window and its gates
    std::queue<sensor_msgs::ImageConstPtr> producer_image_queue_;
    std::queue<sensor_msgs::ImageConstPtr> consumer_image_queue_;
    std::mutex queue_mutex_;
    std::thread process_thread_;
    bool keep_running_;
    std::condition_variable cv_;


//=========================================================
//ROS LAUNCH MODIFIABLE PARAMETERS 
    //EXECUTION PARAMETERS
    int num_threads_;
    //BLOB FILTERING PARAMETERS
    double min_circularity_;
    int saturation_threshold_;
    //CAMERA AND BEACON PARAMETERS
    std::vector<Eigen::Vector3d> knownPoints_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    bool fisheye;
    bool mono;
    bool fov;
    // BUG-088 warm-start state (ported to ROS1 2026-08-27). has_prior_ starts
    // false, so the very first frame — and any frame after a non-converged
    // solve — falls back to the fixed guess below, which is byte-identical to
    // ceresP4P.cpp's own default. Deliberately NOT reset on beacon loss: a
    // stale prior from seconds ago is still a far better starting point than
    // the fixed vector, and a wrong prior costs iterations, not correctness.
    bool warm_start_;
    bool has_prior_ = false;
    double prior_params_[6] = {0.0, 0.0, -0.001, 0.0, 0.0, 0.7};

    // BUG-131. SortTargetsUsingTetrahedronGeometry commits to ONE labeling of
    // the four blobs: it picks the widest-separated pair as P1/P2, derives
    // P3/P4 by which is closer to that pair's midpoint, and orders P1 vs P2 by
    // FindP1P2Indices' 2D cross-product sign. Each of those three steps is a
    // single guess, and when any one goes the wrong way the correct labeling
    // is simply unreachable -- the solver then fits the wrong correspondence
    // perfectly well and reports a confidently wrong pose, with no warning and
    // no failure to converge.
    //
    // The alternative below enumerates all 24 assignments of four blobs to the
    // four model points, solves the real production CobrasFumantes for each,
    // and keeps the lowest final_cost.
    bool multi_hypothesis_sort_;

    // Enumerates one of the 24 labelings. candidateIdx decomposes as
    // pair (6) x P3/P4 swap (2) x P1/P2 swap (2): the pair choice is
    // enumerated, and the two derived orderings each get their alternative
    // offered as well. Byte-for-byte the same algorithm as the ROS2 node's
    // (raspberry5-carolus-ros2/carolus_ros2/src/carolus_node_ros2.cpp) -- if
    // one is ever changed, change the other.
    static bool labelForCandidate(const std::vector<Eigen::Vector3d>& pts, int candidateIdx,
                                  std::vector<Eigen::Vector3d>& out) {
        static const std::pair<int, int> kPairTable[6] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
        static const std::pair<int, int> kNotPairTable[6] = {{2, 3}, {1, 3}, {1, 2}, {0, 3}, {0, 2}, {0, 1}};
        const int pairIdx = candidateIdx / 4;
        const bool swapP3P4 = ((candidateIdx / 2) % 2) != 0;
        const bool swapP1P2 = (candidateIdx % 2) != 0;
        auto p1p2 = kPairTable[pairIdx];
        auto p3p4 = kNotPairTable[pairIdx];
        Eigen::Vector3d midpoint = (pts[p1p2.first] + pts[p1p2.second]) / 2.0;
        double d0 = (midpoint - pts[p3p4.first]).norm();
        double d1 = (midpoint - pts[p3p4.second]).norm();
        int p3 = (d0 < d1) ? p3p4.first : p3p4.second;
        int p4 = (d0 < d1) ? p3p4.second : p3p4.first;
        if (swapP3P4) std::swap(p3, p4);

        double v_p3p4[3] = {pts[p3].x() - pts[p4].x(), pts[p3].y() - pts[p4].y(), 0.0};
        double v_p3pa[3] = {pts[p3].x() - pts[p1p2.first].x(), pts[p3].y() - pts[p1p2.first].y(), 0.0};
        double v_p3pb[3] = {pts[p3].x() - pts[p1p2.second].x(), pts[p3].y() - pts[p1p2.second].y(), 0.0};
        uint8_t p1p2_arr[2] = {static_cast<uint8_t>(p1p2.first), static_cast<uint8_t>(p1p2.second)};
        uint8_t p1, p2;
        if (!FindP1P2Indices(v_p3p4, v_p3pa, v_p3pb, p1p2_arr, &p1, &p2)) return false;
        if (swapP1P2) std::swap(p1, p2);

        out.resize(4);
        out[0] = pts[p1];
        out[1] = pts[p2];
        out[2] = pts[p3];
        out[3] = pts[p4];
        return true;
    }

    // Solves all 24 labelings and returns the best-fitting one. `K` and
    // `toPixelSpace` differ between this node's two branches -- the non-FOV
    // path solves in pixel space against cameraMatrix_, the FOV path solves
    // in the normalized space undistortAstrobeeFov already returns, against
    // camMatrixAstrobee -- so both are passed in rather than assumed. Every
    // candidate is offered the same warm-start prior, since they all explain
    // the same frame. bestSorted comes back so the caller can populate the
    // Astrobee landmark message from the labeling that actually won, not from
    // a stale one.
    //
    // Returns false only if all 24 fail to label, which is a stricter failure
    // condition than the single-guess path's, by design.
    bool solveMultiHypothesis(const std::vector<Eigen::Vector3d>& imagePoints,
                              const std::vector<cv::Point2f>& undistortedPoints,
                              const cv::Mat& K, bool toPixelSpace, int measType,
                              const double* prior, CameraPose& bestPose,
                              std::vector<Eigen::Vector3d>& bestSorted,
                              int& winningCandidate) const {
        bestPose.solver_final_cost = -1.0;
        double bestCost = std::numeric_limits<double>::infinity();
        winningCandidate = -1;
        for (int c = 0; c < 24; ++c) {
            std::vector<Eigen::Vector3d> candidate;
            if (!labelForCandidate(imagePoints, c, candidate)) continue;
            if (toPixelSpace) {
                for (auto& p : candidate) {
                    p(0) = p(0) * K.at<double>(0, 0) + K.at<double>(0, 2);
                    p(1) = p(1) * K.at<double>(1, 1) + K.at<double>(1, 2);
                }
            }
            CameraPose trial;
            CobrasFumantes solver(K, measType);
            solver.computeAndValidatePosesWithRefinement(candidate, knownPoints_, undistortedPoints,
                                                        trial, prior);
            if (!trial.R.allFinite() || !trial.t.allFinite()) continue;
            if (trial.solver_final_cost < bestCost) {
                bestCost = trial.solver_final_cost;
                bestPose = trial;
                bestSorted = candidate;
                winningCandidate = c;
            }
        }
        return winningCandidate != -1;
    }
    bool dock_cam_;
    bool nav_cam_;
    bool sci_cam_;
    bool sci_cam_compressed_;
    std::string _bot_name;
    double fx, fy, cx, cy;
    double fov_distortion_coeff;
    //FILTERING PARAMETERS
    double min_area_;
    double max_area_;
    double max_distance_lim_;
    int kernel_size_gaussian_;
    int kernel_size_morph_;
    int image_threshold_;
    double lb_hue_;
    double ub_hue_;
    //TOPIC NAMES
    std::string nav_cam_topic_, dock_cam_topic_, sci_cam_topic_, processed_image_topic_;
    //BENCHTEST
    bool benchtest;
    //FIFO
    bool fifo;
    int filter_size_int_;
    size_t filter_size_; 
    double translation_threshold_; 
    double rotation_threshold_; //radians
    double max_time_fifo; //seconds
    int reject_limit;
    // Stamp the published pose with the image's acquisition time rather than
    // the publication time (2026-08-04). See the parameter block for why.
    bool stamp_from_acquisition_;


    //QUEUE STUFF
    size_t max_queue_size_;
    int max_queue_size_int_;
    std::atomic<size_t> curr_queue_size_; 

    //FOV ASTROBEE
    double distortion_precalc1_;
    double distortion_precalc2_;
    cv::Mat camMatrixAstrobee;
    bool imagesizeSet = false;
    Eigen::Vector2d imagesize_;

    //ASTROBEE MSGS
    ff_msgs::VisualLandmark visual_landmarks_vec_[4];
    bool frame_id_conv;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "carolus_astrobee_rex");
    ros::NodeHandle nh;

    CarolusRexNode node(nh);

    ros::spin();

    return 0;
}

