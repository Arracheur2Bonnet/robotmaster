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
#include <mutex>
#include <queue>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <condition_variable>
#include <atomic>
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
        _bot_name("wannabee"),
        reject_count(0)

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
        private_nh_.param("bot_name", _bot_name, std::string("wannabee"));
        private_nh_.param("frame_id_conv", frame_id_conv, false);
        // TIMESTAMP SOURCE FOR THE PUBLISHED POSE (added 2026-08-04).
        // true  -> stamp the pose with the ACQUISITION time of the image it was
        //          computed from (last_valid_timestamp).
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
        private_nh_.param("pose_topic", pose_topic_, std::string("/loc/ar/features"));
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
                    // ROS_WARN("Using default distortion coefficients, expected 4 elements.");
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
                    // ROS_WARN("Using default distortion coefficients, expected 4 elements.");
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
        pose_pub_ = nh_.advertise<ff_msgs::VisualLandmarks>(pose_topic_, 10);
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
    cv::Mat preprocessImage(const cv::Mat& image) {
        //TODO: REORGANIZE THIS FOR BETTER PERFORMANCE AND USER SELECTION OF PREPROCESSING STEPS
        cv::Mat blurred, thresholded;

        //D455 GOLDEN STANDARD?
        cv::GaussianBlur(image, blurred, cv::Size(kernel_size_gaussian_, kernel_size_gaussian_), 0);
        double threshValue = image_threshold_;//250;//250
        cv::threshold(blurred, thresholded, threshValue, 255, cv::THRESH_BINARY);
        cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(kernel_size_morph_, kernel_size_morph_));
        cv::morphologyEx(thresholded, thresholded, cv::MORPH_CLOSE, morph_kernel, cv::Point(-1, -1), 1);
        // cv::morphologyEx(thresholded, thresholded, cv::MORPH_DILATE, morph_kernel, cv::Point(-1, -1), 1);
        return thresholded;



        // double threshValue1 = 220;
        // cv::threshold(image, thresholded, threshValue1, 255, cv::THRESH_BINARY);
        // cv::equalizeHist(image, image);
        // double eq_clip_limit = 10.0;
        //   cv::Size eq_win_size = cv::Size(8, 8);
        //   cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
        //   clahe->apply(image, image);
        //     cv::GaussianBlur(image, image, cv::Size(3, 3), 1);
        // cv::equalizeHist(image, image);
        // cv::GaussianBlur(image, image, cv::Size(3, 3), 1);
        // double threshValue = 240;
        // cv::threshold(image, thresholded, threshValue, 255, cv::THRESH_BINARY);


        // // double threshValue = 240;
        // // cv::threshold(image, thresholded, threshValue, 255, cv::THRESH_BINARY);
        // // cv::GaussianBlur(thresholded, blurred, cv::Size(3, 3), 0);

        // cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        // // cv::Mat morph_kernel2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        // // // cv::morphologyEx(thresholded, thresholded, cv::MORPH_OPEN, morph_kernel, cv::Point(-1, -1), 0);
        // cv::morphologyEx(thresholded, thresholded, cv::MORPH_CLOSE, morph_kernel, cv::Point(-1, -1), 2);
        // // // // cv::morphologyEx(thresholded, thresholded, cv::MORPH_OPEN, morph_kernel, cv::Point(-1, -1), 0);
        
        // // cv::morphologyEx(thresholded, thresholded, cv::MORPH_DILATE, morph_kernel, cv::Point(-1, -1), 1);
        // return thresholded;
    }

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
                cv::Mat preprocessedImage = preprocessImage(imageMono);

                //BEGIN BLOB DETECTION AND PROCESSING
                std::optional<std::vector<BlobCarolus>> blobCarolusVec;
                if (mono){

                    blobCarolusVec = findAndCalcContoursMono(preprocessedImage, num_threads_);

                } else{

                    blobCarolusVec = findAndCalcContours(preprocessedImage, image, num_threads_);
                    
                }
                if (blobCarolusVec) {
                    std::vector<BlobCarolus> best_blobs;

                    if (mono){
                       best_blobs  = selectBlobsMono(blobCarolusVec.value(), min_circularity_);
                    } else{
                        best_blobs = selectBlobs(blobCarolusVec.value(), min_circularity_);
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
    CameraPose getFilteredPose(const CameraPose& new_pose, const ros::Time& timestamp) {
        if (!pose_queue_.empty()) {
            if (last_valid_timestamp - timestamp > ros::Duration(max_time_fifo)) {
                pose_queue_.clear();
            }
            const CameraPose& last_pose = pose_queue_.back();

            // Check translation difference
            double translation_diff = (last_pose.t - new_pose.t).norm();
            if (translation_diff > translation_threshold_) {
                ROS_WARN("Current pose rotation is too different. Ignoring current pose.");
                reject_count++;
                if (reject_count > reject_limit) {
                    pose_queue_.clear();
                    reject_count = 0;
                }
                return last_pose;
            }

            // Check rotation difference using quaternion SLERP
            Eigen::Quaterniond q_last(last_pose.R);
            Eigen::Quaterniond q_new(new_pose.R);
            double angle_diff = q_last.angularDistance(q_new);
            if (angle_diff > rotation_threshold_) {
                ROS_WARN("Current pose rotation is too different. Ignoring current pose.");
                reject_count++;
                if (reject_count > reject_limit) {
                    pose_queue_.clear();
                    reject_count = 0;
                }
                return last_pose;
            }
        }

        pose_queue_.push_back(new_pose);
        last_valid_timestamp = timestamp;
        if (pose_queue_.size() > filter_size_) {
            pose_queue_.pop_front();
        }

        // Compute average translation
        Eigen::Vector3d avg_t = Eigen::Vector3d::Zero();
        for (const auto& pose : pose_queue_) {
            avg_t += pose.t;
        }
        avg_t /= pose_queue_.size();

       // Compute average rotation using SLERP
        Eigen::Quaterniond q_avg;
        bool initialized = false;
        int count = 0;

        for (const auto& pose : pose_queue_) {
            Eigen::Quaterniond q_pose(pose.R);
            if (!initialized) {
                q_avg = q_pose;
                initialized = true;
            } else {
                double w = 1.0 / (count + 1);
                q_avg = q_avg.slerp(w, q_pose);
            }
            count++;
        }
        // Convert quaternion back to rotation matrix
        Eigen::Matrix3d avg_R = q_avg.normalized().toRotationMatrix();

        CameraPose filtered_pose;
        filtered_pose.R = avg_R;
        filtered_pose.t = avg_t;
        return filtered_pose;
    }

    std::optional<std::vector<BlobCarolus>> findAndCalcContours(const cv::Mat &image, const cv::Mat &originalImageHSV, int num_threads) {
        std::vector<std::vector<cv::Point>> contours;

        auto start = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT COUNTOURS START
        //FIND CONTOURS IN THE IMAGE
        cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        auto end = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT COUNTOURS END
        std::chrono::duration<double> duration = end - start;
        ROS_INFO("Time to find contours: %f seconds", duration.count());

        //CHECK IF CONTOURS ARE VALID
        if (contours.empty() || contours.size() > 1000) {
            ROS_ERROR("ERROR: %lu contours found", contours.size());
            //return std::nullopt;
            return std::nullopt;
        }

        //PREPARE FOR BLOB PROPERTIES CALCULATION
        std::vector<BlobCarolus> blobs;
        blobs.reserve(contours.size());

        start = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT BLOBS START

        //PARALLELIZE BLOB CALCULATIONS
        std::vector<std::future<void>> futures;
        std::mutex blobs_mutex;
        int num_threads_used = std::min(num_threads, static_cast<int>(contours.size()));
        int contours_per_thread = contours.size() / num_threads_used;
        int remainder = contours.size() % num_threads_used;

        int start_idx = 0;

        for (int t = 0; t < num_threads_used; ++t) {
            int end_idx = start_idx + contours_per_thread + (t < remainder ? 1 : 0);
            futures.emplace_back(std::async(std::launch::async, [start_idx, end_idx, &contours, &blobs, &blobs_mutex, &originalImageHSV, this]() {

        // int contours_per_thread = std::max(1, static_cast<int>(contours.size() / num_threads));
        // int remainder = contours.size() % num_threads;

        // for (int t = 0; t < num_threads; ++t) {
        //     futures.emplace_back(std::async(std::launch::async, [this, t, contours_per_thread, remainder, &contours, &blobs, &blobs_mutex, &originalImageHSV]() {
        //         int start_idx = t * contours_per_thread + std::min(t, remainder);
        //         int end_idx = start_idx + contours_per_thread + (t < remainder ? 1 : 0);


                for (int i = start_idx; i < end_idx; ++i) {
                    const auto &contour = contours[i];

                    if (contours[i].size() < 5 || contour.empty()) {
                        continue;
                    }

                    //CHECK AREA OF CONTOUR (m00)
                    cv::Moments moments = cv::moments(contour);
                    if (moments.m00 < min_area_|| moments.m00 > max_area_) { //30 2500
                        continue;
                    }
                    //CALCULATE BLOB PROPERTIES
                    double perimeter = cv::arcLength(contour, true);
                    double circularity = (4 * CV_PI * moments.m00) / (perimeter * perimeter);
                    double x = moments.m10 / moments.m00; //CENTROID X
                    double y = moments.m01 / moments.m00; //CENTROID Y
                    
                    //HUE EXTRACTION PROCESS
                    //BOUND THE CONTOUR
                    cv::Rect boundingRect = cv::boundingRect(contour);
                
                    cv::Mat BlobRegion = originalImageHSV(boundingRect);


                    //DECLARE HSV MASK TO FILTER SATURATION
                    cv::Mat maskHSV;
                    //FILTER OUT BAD PIXELS
                    cv::inRange(BlobRegion, cv::Scalar(0, saturation_threshold_, 0), cv::Scalar(180, 255, 255), maskHSV);
                    //CALCULATE HUE --> meanHSV[0] 0-180
                    cv::Scalar meanHSV;
                    // if (!options.blob_config.circular_mean_hue) {
                    //     meanHSV = cv::mean(BlobRegion, maskHSV);
                    //     if (meanHSV[0] < options.blob_config.lb_hue || meanHSV[0] > options.blob_config.ub_hue) {
                    //         continue;
                    //     }
                    // } else { //CIRCULAR MEAN ALGORITHM
                        std::vector<cv::Mat> hsvChannels;
                        cv::split(BlobRegion, hsvChannels);
                        cv::Mat hueChannel = hsvChannels[0];

                        double sumSin = 0.0;
                        double sumCos = 0.0;
                        int count = 0;

                        for (int i = 0; i < hueChannel.rows; ++i) {
                            for (int j = 0; j < hueChannel.cols; ++j) {
                                if (maskHSV.at<uchar>(i, j) != 0) {  
                                double hue = hueChannel.at<uchar>(i, j) * 2.0 * CV_PI / 180.0;
                                sumSin += std::sin(hue);
                                sumCos += std::cos(hue);
                                ++count;
                                }
                            }
                        }  

                        double meanAngle = std::atan2(sumSin / count, sumCos / count) * 180.0 / CV_PI;
                        if (meanAngle < 0) meanAngle += 360.0;
                            meanHSV[0] = meanAngle/ 2.0;

                        
                        //CHECK HUE CONDITIONS PER BLOB COLOR
                        if (std::isnan(meanHSV[0]) || std::isinf(meanHSV[0])) {
                            continue;
                        }
                        if (meanHSV[0] < 27){ //WRAP VALUE ADJUST AS NEEDED
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

        for (auto &fut : futures) {
            fut.get();
        }

        end = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT BLOBS END
        duration = end - start;
        // ROS_INFO("Calc time: %f seconds", duration.count());

        return blobs;
    }

    std::optional<std::vector<BlobCarolus>> findAndCalcContoursMono(const cv::Mat &image, int num_threads) {
        std::vector<std::vector<cv::Point>> contours;

        auto start = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT COUNTOURS START
        //FIND CONTOURS IN THE IMAGE
        cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        auto end = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT COUNTOURS END
        std::chrono::duration<double> duration = end - start;
        ROS_INFO("Time to find contours: %f seconds", duration.count());

        //CHECK IF CONTOURS ARE VALID
        if (contours.empty() || contours.size() > 1000) {
            ROS_ERROR("ERROR: %lu contours found", contours.size());
            return std::nullopt;
        }

        //PREPARE FOR BLOB PROPERTIES CALCULATION
        std::vector<BlobCarolus> blobs;
        blobs.reserve(contours.size());

        start = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT BLOBS START
        //PARALLELIZE BLOB CALCULATIONS
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
                    const auto &contour = contours[i];

                    //CHECK AREA OF CONTOUR (m00)
                    cv::Moments moments = cv::moments(contour);
                    if (moments.m00 < min_area_|| moments.m00 > max_area_) { //30 2500
                        continue;
                    }
                    //CALCULATE BLOB PROPERTIES
                    double perimeter = cv::arcLength(contour, true);
                    double circularity = (4 * CV_PI * moments.m00) / (perimeter * perimeter);
                    double x = moments.m10 / moments.m00; //CENTROID X
                    double y = moments.m01 / moments.m00; //CENTROID Y
                    

                    //TODO: ADD CIRCULAR MEAN ALGORITHM FOR HUE EXTRACTION AND CHECK BEHAVIOR WHEN DEALING WITH MONO8->HSV
                    //BOUND THE CONTOUR
                    cv::Rect boundingRect = cv::boundingRect(contour);

                    //CHECK HUE CONDITIONS PER BLOB COLOR


                    BlobCarolus blobCarolus;
                    blobCarolus.blob = {x, y};
                    blobCarolus.properties = {perimeter, moments.m00, circularity, 180, boundingRect};

                    std::lock_guard<std::mutex> lock(blobs_mutex);
                    blobs.emplace_back(std::move(blobCarolus));
                }
            }));
            start_idx = end_idx;
        }

        for (auto &fut : futures) {
            fut.get();
        }

        end = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT BLOBS END
        duration = end - start;
        // ROS_INFO("Calc time: %f seconds", duration.count());

        return blobs;
    }


    //SELECT BLOBS BASED ON CIRCULARITY AND VARIANCE --> NEEDS CLEANING AND OPTIMIZATION
    std::vector<BlobCarolus> selectBlobs(const std::vector<BlobCarolus>& blobs, double min_circularity) {
        std::vector<BlobCarolus> filtered_blobs;
        filtered_blobs.reserve(blobs.size());

        if (blobs.size() < 4) {
            ROS_ERROR("Not enough blobs < 4.");
            return {};
        }

        for (const auto& blob : blobs) {
            if (blob.properties.circularity >= min_circularity) {
                filtered_blobs.emplace_back(blob);
            }
        }

        if (filtered_blobs.size() < 4) {
            ROS_ERROR("Not enough blobs with required circularity.");
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

                        //std::cout << max_distance << ", i=" << i << ",j=" << j << ",k=" << k << ",l=" << l << std::endl;
                        if (max_distance > max_distance_lim_) {//500
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

                        double combined_variance = distance_variance + 5*area_variance; //+ intensity_hues;

                        if (combined_variance < min_variation) {
                            min_variation = combined_variance;
                            best_group = {filtered_blobs[i], filtered_blobs[j], filtered_blobs[k], filtered_blobs[l]};
                            max_distance_best_group=max_distance;
                        }
                    }
                }
            }
        }
        std::cout << "max distance = " << max_distance_best_group << std::endl;
        return best_group;
    }

    //SELECT BLOBS BASED ON CIRCULARITY AND VARIANCE --> NEEDS CLEANING AND OPTIMIZATION
    std::vector<BlobCarolus> selectBlobsMono(const std::vector<BlobCarolus>& blobs, double min_circularity) {
        std::vector<BlobCarolus> filtered_blobs;
        filtered_blobs.reserve(blobs.size());

        if (blobs.size() < 4) {
            ROS_ERROR("Not enough blobs < 4.");
            return {};
        }

        for (const auto& blob : blobs) {
            if (blob.properties.circularity >= min_circularity) {
                filtered_blobs.emplace_back(blob);
            }
        }

        if (filtered_blobs.size() < 4) {
            ROS_ERROR("Not enough blobs with required circularity.");
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

                        
                        if (max_distance > max_distance_lim_) {//500
                            continue;
                        }
                        if (min_distance < 100) {//500
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


                        double combined_variance = distance_variance + 1.5*area_variance;

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

        bool success = SortTargetsUsingTetrahedronGeometry(imagePoints, sortedImagePoints);
        for (int i = 0; i < sortedImagePoints.size(); i++) {
            sortedImagePoints[i](0) = sortedImagePoints[i](0) * fx +cx;
            sortedImagePoints[i](1) = sortedImagePoints[i](1) * fy +cy;
            //GOTTA CREATE THE LANDMARK MESSAGE AS PER ASTORBBE DEFS
            visual_landmarks_vec_[i].u = sortedImagePoints[i](0);
            visual_landmarks_vec_[i].v = sortedImagePoints[i](1);
            visual_landmarks_vec_[i].x = knownPoints_[i](0);
            visual_landmarks_vec_[i].y = knownPoints_[i](1);
            visual_landmarks_vec_[i].z = knownPoints_[i](2);
        }
        if (!success) {
            ROS_ERROR("Failed to sort targets using tetrahedron geometry.");
            return;
        }

        //COBRAS FUMANTES POSE SOLVER
        //THE SNAKE IS GOING TO SMOKE
        CobrasFumantes poseSolver(cameraMatrix_, measType_);
        poseSolver.computeAndValidatePosesWithRefinement(sortedImagePoints, knownPoints_, undistortedPoints, bestPose);
    } else {
        imagePoints.reserve(undistortedPoints.size());
        for (const auto& point : undistortedPoints) {
        imagePoints.emplace_back(Eigen::Vector3d(point.x, point.y, 1.0));
        }

        bool success = SortTargetsUsingTetrahedronGeometry(imagePoints, sortedImagePoints);
        if (!success) {
            ROS_ERROR("Failed to sort targets using tetrahedron geometry.");
            return;
        }
        for (int i = 0; i < sortedImagePoints.size(); i++) {
  
            sortedImagePoints[i](0) = sortedImagePoints[i](0); // * fx;
            sortedImagePoints[i](1) = sortedImagePoints[i](1); // * fy;
        //GOTTA CREATE THE LANDMARK MESSAGE AS PER ASTORBBE DEFS
            visual_landmarks_vec_[i].u = sortedImagePoints[i](0);
            visual_landmarks_vec_[i].v = sortedImagePoints[i](1);
            visual_landmarks_vec_[i].x = knownPoints_[i](0);
            visual_landmarks_vec_[i].y = knownPoints_[i](1);
            visual_landmarks_vec_[i].z = knownPoints_[i](2);


        }
        //COBRAS FUMANTES POSE SOLVER
        //THE SNAKE IS GOING TO SMOKE
        CobrasFumantes poseSolver(camMatrixAstrobee, measType_);
        poseSolver.computeAndValidatePosesWithRefinement(sortedImagePoints, knownPoints_, undistortedPoints, bestPose);
    }
   





    // BUG-087 (2026-08-03) — report non-convergence instead of hiding it.
    // Deliberately a WARNING and not a rejection: the pose is still published
    // exactly as before, because we do not yet know how often this fires. If it
    // turns out to be rare, rejecting becomes the right fix; if it is common,
    // rejecting would silence the pipeline. Throttled so a persistent failure
    // cannot flood the log at frame rate.
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
        // ros::Time::now(). last_valid_timestamp is the acquisition time of the
        // most recent image that contributed to this pose (set where the pose is
        // pushed into pose_queue_); the published pose may be an average over
        // the outlier-filter window, and the most recent contributing
        // measurement is the conventional stamp for such an average.
        // Falls back to now() if no pose has been accepted yet, so the very
        // first publication can never carry a zero timestamp.
        if (stamp_from_acquisition_ && !last_valid_timestamp.isZero()) {
            PoseAstrobee.header.stamp = last_valid_timestamp;
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
    std::string nav_cam_topic_, dock_cam_topic_, sci_cam_topic_, pose_topic_, processed_image_topic_;
    //BENCHTEST
    bool benchtest;
    //FIFO
    bool fifo;
    std::deque<CameraPose> pose_queue_;
    int filter_size_int_;
    size_t filter_size_; 
    double translation_threshold_; 
    double rotation_threshold_; //radians
    double max_time_fifo; //seconds
    int reject_limit;
    int reject_count;
    ros::Time last_valid_timestamp;
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

