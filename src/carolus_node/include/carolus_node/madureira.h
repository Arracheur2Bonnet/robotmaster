#ifndef MADUREIRA_HEADER_INCLUDE
#define MADUREIRA_HEADER_INCLUDE

#pragma once
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <thread>
#include <condition_variable>
#include <queue>
#include <functional>
#include <cstring>
#include <mutex>
#include <memory>
#include <iostream>
#include <cmath>
#include <optional>

struct image_u8_t {
    int32_t width;
    int32_t height;
    int32_t stride;
    std::unique_ptr<uint8_t[]> buf;
};

struct nursia_detector_t {
    struct {
        int min_white_black_diff;
        bool deglitch;
    } qtp;
    std::vector<std::thread> threads;
    std::queue<std::function<void()>> task_queue;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop = false;
};

struct minmax_task {
    image_u8_t* im;
    uint8_t* im_max;
    uint8_t* im_min;
    int ty;
};

struct blur_task {
    image_u8_t* im;
    uint8_t* im_max;
    uint8_t* im_min;
    uint8_t* im_max_tmp;
    uint8_t* im_min_tmp;
    int ty;
};

struct threshold_task {
    image_u8_t* im;
    image_u8_t* threshim;
    uint8_t* im_max;
    uint8_t* im_min;
    int ty;
    nursia_detector_t* td;
};


void do_darkthreshold_task(void* p);
std::unique_ptr<image_u8_t> darkthreshold(nursia_detector_t* td, image_u8_t* im);
void binaryMask(image_u8_t* im, uint8_t threshold);
void applyDilation(image_u8_t* im);
void closingMorphologicalOperations(image_u8_t* im);
void openingMorphologicalOperations(image_u8_t* im);
void image_u8_darken(image_u8_t* im); 
std::unique_ptr<image_u8_t> image_u8_create_alignment(unsigned int width, unsigned int height, unsigned int alignment);
std::unique_ptr<image_u8_t> image_u8_copy(const image_u8_t* in);
std::unique_ptr<image_u8_t> image_u8_decimate(image_u8_t* im, float ffactor);
void convolve(const uint8_t* x, uint8_t* y, int sz, const uint8_t* k, int ksz);
void image_u8_convolve_2D(image_u8_t* im, const uint8_t* k, int ksz);
void image_u8_gaussian_blur(image_u8_t* im, double sigma, int ksz);
void image_u8_median_blur(image_u8_t* im, int ksz);
void worker_thread(nursia_detector_t* td);
void workerpool_add_task(nursia_detector_t* td, void(*task_func)(void*), void* task);
void workerpool_run(nursia_detector_t* td);
void initialize_workerpool(nursia_detector_t* td, int nthreads);
void shutdown_workerpool(nursia_detector_t* td);
void do_minmax_task(void* p);
void do_blur_task(void* p);
void do_threshold_task(void* p);
void equalize_histogram(uint8_t* hist, int num_bins, uint32_t clip_limit);
void compute_histogram(const uint8_t* image, int width, int height, int stride, uint8_t* histogram, int num_bins);
void apply_clahe(image_u8_t* image, int clip_limit);
std::unique_ptr<image_u8_t> dynamic_threshold(nursia_detector_t* td, image_u8_t* im);
std::unique_ptr<image_u8_t> threshold(nursia_detector_t* td, image_u8_t* im);

#endif // MADUREIRA_H
