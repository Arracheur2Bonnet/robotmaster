#include "carolus_node/madureira.h"


std::unique_ptr<image_u8_t> image_u8_create_alignment(unsigned int width, unsigned int height, unsigned int alignment) {
    int stride = width;
    if ((stride % alignment) != 0)
        stride += alignment - (stride % alignment);
    auto img = std::make_unique<image_u8_t>();
    img->width = width;
    img->height = height;
    img->stride = stride;
    img->buf = std::make_unique<uint8_t[]>(height * stride);
    return img;
}

std::unique_ptr<image_u8_t> image_u8_copy(const image_u8_t* in) {
    auto buf = std::make_unique<uint8_t[]>(in->height * in->stride);
    std::memcpy(buf.get(), in->buf.get(), in->height * in->stride);
    auto copy = std::make_unique<image_u8_t>();
    copy->width = in->width;
    copy->height = in->height;
    copy->stride = in->stride;
    copy->buf = std::move(buf);
    return copy;
}

std::unique_ptr<image_u8_t> image_u8_decimate(image_u8_t* im, float ffactor) {
    int width = im->width, height = im->height;

    if (ffactor == 1.5) {
        int swidth = width / 3 * 2, sheight = height / 3 * 2;
        auto decim = image_u8_create_alignment(swidth, sheight, 1);
        int y = 0, sy = 0;
        while (sy < sheight) {
            int x = 0, sx = 0;
            while (sx < swidth) {
                uint8_t a = im->buf[(y + 0) * im->stride + (x + 0)];
                uint8_t b = im->buf[(y + 0) * im->stride + (x + 1)];
                uint8_t c = im->buf[(y + 0) * im->stride + (x + 2)];
                uint8_t d = im->buf[(y + 1) * im->stride + (x + 0)];
                uint8_t e = im->buf[(y + 1) * im->stride + (x + 1)];
                uint8_t f = im->buf[(y + 1) * im->stride + (x + 2)];
                uint8_t g = im->buf[(y + 2) * im->stride + (x + 0)];
                uint8_t h = im->buf[(y + 2) * im->stride + (x + 1)];
                uint8_t i = im->buf[(y + 2) * im->stride + (x + 2)];

                decim->buf[(sy + 0) * decim->stride + (sx + 0)] = (4 * a + 2 * b + 2 * d + e) / 9;
                decim->buf[(sy + 0) * decim->stride + (sx + 1)] = (4 * c + 2 * b + 2 * f + e) / 9;
                decim->buf[(sy + 1) * decim->stride + (sx + 0)] = (4 * g + 2 * d + 2 * h + e) / 9;
                decim->buf[(sy + 1) * decim->stride + (sx + 1)] = (4 * i + 2 * f + 2 * h + e) / 9;

                x += 3;
                sx += 2;
            }
            y += 3;
            sy += 2;
        }
        return decim;
    }

    int factor = static_cast<int>(ffactor);
    int swidth = 1 + (width - 1) / factor;
    int sheight = 1 + (height - 1) / factor;
    auto decim = image_u8_create_alignment(swidth, sheight, 1);
    int sy = 0;
    for (int y = 0; y < height; y += factor) {
        int sx = 0;
        for (int x = 0; x < width; x += factor) {
            decim->buf[sy * decim->stride + sx] = im->buf[y * im->stride + x];
            sx++;
        }
        sy++;
    }
    return decim;
}

void convolve(const uint8_t* x, uint8_t* y, int sz, const uint8_t* k, int ksz) {
    assert((ksz & 1) == 1);
    for (int i = 0; i < ksz / 2 && i < sz; i++)
        y[i] = x[i];
    for (int i = 0; i < sz - ksz; i++) {
        uint32_t acc = 0;
        for (int j = 0; j < ksz; j++)
            acc += k[j] * x[i + j];
        y[ksz / 2 + i] = acc >> 8;
    }
    for (int i = sz - ksz + ksz / 2; i < sz; i++)
        y[i] = x[i];
}

void image_u8_convolve_2D(image_u8_t* im, const uint8_t* k, int ksz) {
    assert((ksz & 1) == 1); // ksz must be odd.
    for (int y = 0; y < im->height; y++) {
        auto x = std::make_unique<uint8_t[]>(im->stride);
        std::memcpy(x.get(), &im->buf[y * im->stride], im->stride);
        convolve(x.get(), &im->buf[y * im->stride], im->width, k, ksz);
    }
    for (int x = 0; x < im->width; x++) {
        auto xb = std::make_unique<uint8_t[]>(im->height);
        auto yb = std::make_unique<uint8_t[]>(im->height);
        for (int y = 0; y < im->height; y++)
            xb[y] = im->buf[y * im->stride + x];
        convolve(xb.get(), yb.get(), im->height, k, ksz);
        for (int y = 0; y < im->height; y++)
            im->buf[y * im->stride + x] = yb[y];
    }
}

void image_u8_gaussian_blur(image_u8_t* im, double sigma, int ksz) {
    if (sigma == 0)
        return;
    assert((ksz & 1) == 1); // ksz must be odd.
    auto dk = std::make_unique<double[]>(ksz);
    for (int i = 0; i < ksz; i++) {
        int x = -ksz / 2 + i;
        double v = std::exp(-.5 * std::pow(x / sigma, 2));
        dk[i] = v;
    }
    double acc = 0;
    for (int i = 0; i < ksz; i++)
        acc += dk[i];
    for (int i = 0; i < ksz; i++)
        dk[i] /= acc;
    auto k = std::make_unique<uint8_t[]>(ksz);
    for (int i = 0; i < ksz; i++)
        k[i] = static_cast<uint8_t>(dk[i] * 255);
    image_u8_convolve_2D(im, k.get(), ksz);
}

void image_u8_median_blur(image_u8_t* im, int ksz) {
    assert((ksz & 1) == 1); // ksz must be odd.
    auto copy = image_u8_copy(im);
    int radius = ksz / 2;
    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            std::vector<uint8_t> neighborhood;
            for (int ky = -radius; ky <= radius; ky++) {
                for (int kx = -radius; kx <= radius; kx++) {
                    int ny = y + ky;
                    int nx = x + kx;
                    if (ny >= 0 && ny < im->height && nx >= 0 && nx < im->width) {
                        neighborhood.push_back(copy->buf[ny * copy->stride + nx]);
                    }
                }
            }
            std::sort(neighborhood.begin(), neighborhood.end());
            im->buf[y * im->stride + x] = neighborhood[neighborhood.size() / 2];
        }
    }
}

void worker_thread(nursia_detector_t* td) {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(td->queue_mutex);
            td->condition.wait(lock, [td] { return td->stop || !td->task_queue.empty(); });
            if (td->stop && td->task_queue.empty()) {
                // std::cout << "Worker thread exiting." << std::endl;
                return;
            }
            task = std::move(td->task_queue.front());
            td->task_queue.pop();
        }
        // std::cout << "Worker thread executing task." << std::endl;
        task();
    }
}

void workerpool_add_task(nursia_detector_t* td, void(*task_func)(void*), void* task) {
    {
        std::unique_lock<std::mutex> lock(td->queue_mutex);
        td->task_queue.emplace([task_func, task] { 
            // std::cout << "Task added to workerpool." << std::endl;
            task_func(task); 
        });
    }
    td->condition.notify_one();
}

void workerpool_run(nursia_detector_t* td) {
    // std::cout << "Running workerpool tasks..." << std::endl;
    // Wait for all tasks to complete
    {
        std::unique_lock<std::mutex> lock(td->queue_mutex);
        td->condition.wait(lock, [td] { return td->task_queue.empty(); });
    }
    // Join all threads
    for (std::thread& worker : td->threads) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    td->threads.clear();
    // std::cout << "Workerpool tasks completed." << std::endl;
}
void initialize_workerpool(nursia_detector_t* td, int nthreads) {
    for (int i = 0; i < nthreads; ++i) {
        td->threads.emplace_back(worker_thread, td);
        // std::cout << "Started worker thread " << i << std::endl;
    }
}

void shutdown_workerpool(nursia_detector_t* td) {
    {
        std::unique_lock<std::mutex> lock(td->queue_mutex);
        td->stop = true;
    }
    td->condition.notify_all();
    for (std::thread& worker : td->threads) {
        worker.join();
    }
    td->threads.clear();
    td->stop = false;  // Reset stop flag to allow pool to be restarted
    // std::cout << "Worker pool shutdown completed." << std::endl;
}

void do_minmax_task(void* p) {
    const int tilesz = 4;
    auto* task = static_cast<minmax_task*>(p);
    int s = task->im->stride;
    int ty = task->ty;
    int tw = task->im->width / tilesz;
    auto* im = task->im;
    // std::cout << "Minmax task started for ty=" << ty << std::endl;
    for (int tx = 0; tx < tw; tx++) {
        uint8_t max = 0, min = 255;
        for (int dy = 0; dy < tilesz; dy++) {
            for (int dx = 0; dx < tilesz; dx++) {
                uint8_t v = im->buf[(ty * tilesz + dy) * s + tx * tilesz + dx];
                min = std::min(min, v);
                max = std::max(max, v);
            }
        }
        task->im_max[ty * tw + tx] = max;
        task->im_min[ty * tw + tx] = min;
    }
    // std::cout << "Minmax task completed for ty=" << ty << std::endl;
}

void do_blur_task(void* p) {
    const int tilesz = 4;
    auto* task = static_cast<blur_task*>(p);
    int ty = task->ty;
    int tw = task->im->width / tilesz;
    int th = task->im->height / tilesz;
    auto* im_max = task->im_max;
    auto* im_min = task->im_min;
    // std::cout << "Blur task started for ty=" << ty << std::endl;
    for (int tx = 0; tx < tw; tx++) {
        uint8_t max = 0, min = 255;
        for (int dy = -1; dy <= 1; dy++) {
            if (ty + dy < 0 || ty + dy >= th) continue;
            for (int dx = -1; dx <= 1; dx++) {
                if (tx + dx < 0 || tx + dx >= tw) continue;
                uint8_t m = im_max[(ty + dy) * tw + tx + dx];
                max = std::max(max, m);
                m = im_min[(ty + dy) * tw + tx + dx];
                min = std::min(min, m);
            }
        }
        task->im_max_tmp[ty * tw + tx] = max;
        task->im_min_tmp[ty * tw + tx] = min;
    }
    // std::cout << "Blur task completed for ty=" << ty << std::endl;
}

void image_u8_darken(image_u8_t* im) {
    auto buf = im->buf.get();
    std::for_each(buf, buf + im->height * im->stride, [](uint8_t& pixel) {
        pixel /= 2;
    });
}

void do_threshold_task(void* p) {
    const int tilesz = 4;
    auto* task = static_cast<threshold_task*>(p);
    int ty = task->ty;
    int tw = task->im->width / tilesz;
    int s = task->im->stride;
    auto* im_max = task->im_max;
    auto* im_min = task->im_min;
    auto* im = task->im;
    auto* threshim = task->threshim;
    int min_white_black_diff = task->td->qtp.min_white_black_diff;
    // std::cout << "Threshold task started for ty=" << ty << std::endl;
    for (int tx = 0; tx < tw; tx++) {
        int min = im_min[ty * tw + tx];
        int max = im_max[ty * tw + tx];
        if (max - min < min_white_black_diff) {
            for (int dy = 0; dy < tilesz; dy++) {
                int y = ty * tilesz + dy;
                for (int dx = 0; dx < tilesz; dx++) {
                    int x = tx * tilesz + dx;
                    threshim->buf[y * s + x] = 0;//127;
                }
            }
            continue;
        }
        uint8_t thresh = max - (max - min) / 2; //CHANGED THIS TO FOCUS ON BRIGHT SPOTS
        for (int dy = 0; dy < tilesz; dy++) {
            int y = ty * tilesz + dy;
            for (int dx = 0; dx < tilesz; dx++) {
                int x = tx * tilesz + dx;
                uint8_t v = im->buf[y * s + x];
                threshim->buf[y * s + x] = (v > thresh) ? 255 : 0;
            }
        }
    }
    // std::cout << "Threshold task completed for ty=" << ty << std::endl;
}
//working good
std::unique_ptr<image_u8_t> threshold(nursia_detector_t* td, image_u8_t* im) {
    // std::cout << "Starting threshold function..." << std::endl;

    int w = im->width, h = im->height, s = im->stride;
    assert(w < 32768);
    assert(h < 32768);
    auto threshim = image_u8_create_alignment(w, h, s);
    assert(threshim->stride == s);
    const int tilesz = 4;
    int tw = w / tilesz;
    int th = h / tilesz;
    auto im_max = std::make_unique<uint8_t[]>(tw * th);
    auto im_min = std::make_unique<uint8_t[]>(tw * th);

    // std::cout << "Processing minmax tasks single-threaded..." << std::endl;
    for (int ty = 0; ty < th; ty++) {
        minmax_task task;
        task.im = im;
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.ty = ty;
        do_minmax_task(&task);
    }
    // std::cout << "Minmax tasks completed." << std::endl;

    auto im_max_tmp = std::make_unique<uint8_t[]>(tw * th);
    auto im_min_tmp = std::make_unique<uint8_t[]>(tw * th);

    // std::cout << "Processing blur tasks single-threaded..." << std::endl;
    for (int ty = 0; ty < th; ty++) {
        blur_task task;
        task.im = im;
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.im_max_tmp = im_max_tmp.get();
        task.im_min_tmp = im_min_tmp.get();
        task.ty = ty;
        do_blur_task(&task);
    }
    // std::cout << "Blur tasks completed." << std::endl;

    im_max.swap(im_max_tmp);
    im_min.swap(im_min_tmp);

    // std::cout << "Processing threshold tasks single-threaded..." << std::endl;
    for (int ty = 0; ty < th; ty++) {
        threshold_task task;
        task.im = im;
        task.threshim = threshim.get();
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.ty = ty;
        task.td = td;
        do_threshold_task(&task);
    }
    // std::cout << "Threshold tasks completed." << std::endl;

    // std::cout << "Starting final thresholding..." << std::endl;
    for (int y = 0; y < h; y++) {
        int x0 = (y >= th * tilesz) ? 0 : tw * tilesz;
        int ty = y / tilesz;
        if (ty >= th) ty = th - 1;
        for (int x = x0; x < w; x++) {
            int tx = x / tilesz;
            if (tx >= tw) tx = tw - 1;
            int max = im_max[ty * tw + tx];
            int min = im_min[ty * tw + tx];
            int thresh = max - (max - min) / 2; //CORRECTED THIS TO FOCUS ON BRIGHT SPOTS, DID IT WORK?
            uint8_t v = im->buf[y * s + x];
            threshim->buf[y * s + x] = (v > thresh) ? 255 : 0;
        }
    }
    // std::cout << "Final thresholding completed." << std::endl;

    if (td->qtp.deglitch) {
        auto tmp = image_u8_create_alignment(w, h, s);
        for (int y = 1; y + 1 < h; y++) {
            for (int x = 1; x + 1 < w; x++) {
                uint8_t max = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        uint8_t v = threshim->buf[(y + dy) * s + x + dx];
                        max = std::max(max, v);
                    }
                }
                tmp->buf[y * s + x] = max;
            }
        }
        for (int y = 1; y + 1 < h; y++) {
            for (int x = 1; x + 1 < w; x++) {
                uint8_t min = 255;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        uint8_t v = tmp->buf[(y + dy) * s + x + dx];
                        min = std::min(min, v);
                    }
                }
                threshim->buf[y * s + x] = min;
            }
        }
    }
    // std::cout << "Deglitching completed." << std::endl;
    return threshim;
}


void equalize_histogram(uint8_t* hist, int num_bins, uint32_t clip_limit) {
    uint32_t total_excess = 0;
    for (int i = 0; i < num_bins; ++i) {
        if (hist[i] > clip_limit) {
            total_excess += hist[i] - clip_limit;
            hist[i] = clip_limit;
        }
    }

    int increment = total_excess / num_bins;
    int residual = total_excess % num_bins;

    for (int i = 0; i < num_bins; ++i) {
        hist[i] += increment;
        if (i < residual) ++hist[i];
    }
}

void compute_histogram(const uint8_t* image, int width, int height, int stride, uint8_t* histogram, int num_bins) {
    std::fill_n(histogram, num_bins, 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint8_t pixel = image[y * stride + x];
            histogram[pixel]++;
        }
    }
}

void apply_clahe(image_u8_t* image, int clip_limit) {
    int num_bins = 256;
    int tile_size = 8;
    int tiles_x = (image->width + tile_size - 1) / tile_size;
    int tiles_y = (image->height + tile_size - 1) / tile_size;

    auto histograms = std::make_unique<uint8_t[]>(tiles_x * tiles_y * num_bins);
    auto lut = std::make_unique<uint8_t[]>(tiles_x * tiles_y * num_bins);

    for (int ty = 0; ty < tiles_y; ++ty) {
        for (int tx = 0; tx < tiles_x; ++tx) {
            int x0 = tx * tile_size;
            int y0 = ty * tile_size;
            int x1 = std::min(x0 + tile_size, image->width);
            int y1 = std::min(y0 + tile_size, image->height);
            uint8_t* hist = &histograms[(ty * tiles_x + tx) * num_bins];

            compute_histogram(&image->buf[y0 * image->stride + x0], x1 - x0, y1 - y0, image->stride, hist, num_bins);
            equalize_histogram(hist, num_bins, clip_limit);

            int sum = 0;
            for (int i = 0; i < num_bins; ++i) {
                sum += hist[i];
                lut[(ty * tiles_x + tx) * num_bins + i] = static_cast<uint8_t>(sum * 255 / ((x1 - x0) * (y1 - y0)));
            }
        }
    }

    for (int y = 0; y < image->height; ++y) {
        for (int x = 0; x < image->width; ++x) {
            int tx = x / tile_size;
            int ty = y / tile_size;
            uint8_t* tile_lut = &lut[(ty * tiles_x + tx) * num_bins];
            image->buf[y * image->stride + x] = tile_lut[image->buf[y * image->stride + x]];
        }
    }
}

std::unique_ptr<image_u8_t> dynamic_threshold(nursia_detector_t* td, image_u8_t* im) {
    int w = im->width, h = im->height, s = im->stride;
    assert(w < 32768);
    assert(h < 32768);
    auto threshim = image_u8_create_alignment(w, h, s);
    assert(threshim->stride == s);
    const int tilesz = 4;
    int tw = w / tilesz;
    int th = h / tilesz;
    auto im_max = std::make_unique<uint8_t[]>(tw * th);
    auto im_min = std::make_unique<uint8_t[]>(tw * th);

    for (int ty = 0; ty < th; ty++) {
        minmax_task task;
        task.im = im;
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.ty = ty;
        do_minmax_task(&task);
    }

    auto im_max_tmp = std::make_unique<uint8_t[]>(tw * th);
    auto im_min_tmp = std::make_unique<uint8_t[]>(tw * th);

    for (int ty = 0; ty < th; ty++) {
        blur_task task;
        task.im = im;
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.im_max_tmp = im_max_tmp.get();
        task.im_min_tmp = im_min_tmp.get();
        task.ty = ty;
        do_blur_task(&task);
    }

    im_max.swap(im_max_tmp);
    im_min.swap(im_min_tmp);

    for (int ty = 0; ty < th; ty++) {
        threshold_task task;
        task.im = im;
        task.threshim = threshim.get();
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.ty = ty;
        task.td = td;
        do_threshold_task(&task);
    }

    for (int y = 0; y < h; y++) {
        int x0 = (y >= th * tilesz) ? 0 : tw * tilesz;
        int ty = y / tilesz;
        if (ty >= th) ty = th - 1;
        for (int x = x0; x < w; x++) {
            int tx = x / tilesz;
            if (tx >= tw) tx = tw - 1;
            int max = im_max[ty * tw + tx];
            int min = im_min[ty * tw + tx];

            //DYNAMIC THRESHOLDING STILL DARKER SPOTS
            int mean = (max + min) / 2;
            int thresh = mean - 10;  //CHANGE THIS VALUE TO ADJUST THE THRESHOLD
            uint8_t v = im->buf[y * s + x];
            threshim->buf[y * s + x] = (v > thresh) ? 255 : 0;
        }
    }

    if (td->qtp.deglitch) {
        auto tmp = image_u8_create_alignment(w, h, s);
        for (int y = 1; y + 1 < h; y++) {
            for (int x = 1; x + 1 < w; x++) {
                uint8_t max = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        uint8_t v = threshim->buf[(y + dy) * s + x + dx];
                        max = std::max(max, v);
                    }
                }
                tmp->buf[y * s + x] = max;
            }
        }
        for (int y = 1; y + 1 < h; y++) {
            for (int x = 1; x + 1 < w; x++) {
                uint8_t min = 255;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        uint8_t v = tmp->buf[(y + dy) * s + x + dx];
                        min = std::min(min, v);
                    }
                }
                threshim->buf[y * s + x] = min;
            }
        }
    }
    return threshim;
}


//DARK SPOTS FUNCTIONS

void do_darkthreshold_task(void* p) {
    const int tilesz = 4;
    auto* task = static_cast<threshold_task*>(p);
    int ty = task->ty;
    int tw = task->im->width / tilesz;
    int s = task->im->stride;
    auto* im_max = task->im_max;
    auto* im_min = task->im_min;
    auto* im = task->im;
    auto* threshim = task->threshim;
    int min_white_black_diff = task->td->qtp.min_white_black_diff;
    // std::cout << "Threshold task started for ty=" << ty << std::endl;
    for (int tx = 0; tx < tw; tx++) {
        int min = im_min[ty * tw + tx];
        int max = im_max[ty * tw + tx];
        if (max - min < min_white_black_diff) {
            for (int dy = 0; dy < tilesz; dy++) {
                int y = ty * tilesz + dy;
                for (int dx = 0; dx < tilesz; dx++) {
                    int x = tx * tilesz + dx;
                    threshim->buf[y * s + x] = 127;
                }
            }
            continue;
        }
        uint8_t thresh = min + (max - min) / 2;
        for (int dy = 0; dy < tilesz; dy++) {
            int y = ty * tilesz + dy;
            for (int dx = 0; dx < tilesz; dx++) {
                int x = tx * tilesz + dx;
                uint8_t v = im->buf[y * s + x];
                threshim->buf[y * s + x] = (v > thresh) ? 255 : 0;
            }
        }
    }
    // std::cout << "Threshold task completed for ty=" << ty << std::endl;
}

std::unique_ptr<image_u8_t> darkthreshold(nursia_detector_t* td, image_u8_t* im) {
    // std::cout << "Starting threshold function..." << std::endl;

    int w = im->width, h = im->height, s = im->stride;
    assert(w < 32768);
    assert(h < 32768);
    auto threshim = image_u8_create_alignment(w, h, s);
    assert(threshim->stride == s);
    const int tilesz = 4;
    int tw = w / tilesz;
    int th = h / tilesz;
    auto im_max = std::make_unique<uint8_t[]>(tw * th);
    auto im_min = std::make_unique<uint8_t[]>(tw * th);

    // std::cout << "Processing minmax tasks single-threaded..." << std::endl;
    for (int ty = 0; ty < th; ty++) {
        minmax_task task;
        task.im = im;
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.ty = ty;
        do_minmax_task(&task);
    }
    // std::cout << "Minmax tasks completed." << std::endl;

    auto im_max_tmp = std::make_unique<uint8_t[]>(tw * th);
    auto im_min_tmp = std::make_unique<uint8_t[]>(tw * th);

    // std::cout << "Processing blur tasks single-threaded..." << std::endl;
    for (int ty = 0; ty < th; ty++) {
        blur_task task;
        task.im = im;
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.im_max_tmp = im_max_tmp.get();
        task.im_min_tmp = im_min_tmp.get();
        task.ty = ty;
        do_blur_task(&task);
    }
    // std::cout << "Blur tasks completed." << std::endl;

    im_max.swap(im_max_tmp);
    im_min.swap(im_min_tmp);

    // std::cout << "Processing threshold tasks single-threaded..." << std::endl;
    for (int ty = 0; ty < th; ty++) {
        threshold_task task;
        task.im = im;
        task.threshim = threshim.get();
        task.im_max = im_max.get();
        task.im_min = im_min.get();
        task.ty = ty;
        task.td = td;
        do_darkthreshold_task(&task);
    }
    // std::cout << "Threshold tasks completed." << std::endl;

    // std::cout << "Starting final thresholding..." << std::endl;
    for (int y = 0; y < h; y++) {
        int x0 = (y >= th * tilesz) ? 0 : tw * tilesz;
        int ty = y / tilesz;
        if (ty >= th) ty = th - 1;
        for (int x = x0; x < w; x++) {
            int tx = x / tilesz;
            if (tx >= tw) tx = tw - 1;
            int max = im_max[ty * tw + tx];
            int min = im_min[ty * tw + tx];
            int thresh = min + (max - min) / 2;
            uint8_t v = im->buf[y * s + x];
            threshim->buf[y * s + x] = (v > thresh) ? 255 : 0;
        }
    }
    // std::cout << "Final thresholding completed." << std::endl;

    if (td->qtp.deglitch) {
        auto tmp = image_u8_create_alignment(w, h, s);
        for (int y = 1; y + 1 < h; y++) {
            for (int x = 1; x + 1 < w; x++) {
                uint8_t max = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        uint8_t v = threshim->buf[(y + dy) * s + x + dx];
                        max = std::max(max, v);
                    }
                }
                tmp->buf[y * s + x] = max;
            }
        }
        for (int y = 1; y + 1 < h; y++) {
            for (int x = 1; x + 1 < w; x++) {
                uint8_t min = 255;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        uint8_t v = tmp->buf[(y + dy) * s + x + dx];
                        min = std::min(min, v);
                    }
                }
                threshim->buf[y * s + x] = min;
            }
        }
    }
    // std::cout << "Deglitching completed." << std::endl;
    return threshim;
}

// std::unique_ptr<image_u8_t> threshold(nursia_detector_t* td, image_u8_t* im) {
//     std::cout << "Starting threshold function..." << std::endl;

//     int w = im->width, h = im->height, s = im->stride;
//     std::cout << "Image dimensions: " << w << "x" << h << std::endl;

//     assert(w < 32768);
//     assert(h < 32768);
//     auto threshim = image_u8_create_alignment(w, h, s);
//     assert(threshim->stride == s);

//     const int tilesz = 4;
//     int tw = w / tilesz;
//     int th = h / tilesz;
//     std::cout << "Tile dimensions: " << tw << "x" << th << std::endl;

//     auto im_max = std::make_unique<uint8_t[]>(tw * th);
//     auto im_min = std::make_unique<uint8_t[]>(tw * th);

//     std::cout << "Processing minmax tasks single-threaded..." << std::endl;
//     for (int ty = 0; ty < th; ty++) {
//         minmax_task task;
//         task.im = im;
//         task.im_max = im_max.get();
//         task.im_min = im_min.get();
//         task.ty = ty;
//         do_minmax_task(&task);
//     }
//     std::cout << "Minmax tasks completed." << std::endl;

//     auto im_max_tmp = std::make_unique<uint8_t[]>(tw * th);
//     auto im_min_tmp = std::make_unique<uint8_t[]>(tw * th);

//     std::cout << "Processing blur tasks single-threaded..." << std::endl;
//     for (int ty = 0; ty < th; ty++) {
//         blur_task task;
//         task.im = im;
//         task.im_max = im_max.get();
//         task.im_min = im_min.get();
//         task.im_max_tmp = im_max_tmp.get();
//         task.im_min_tmp = im_min_tmp.get();
//         task.ty = ty;
//         do_blur_task(&task);
//     }
//     std::cout << "Blur tasks completed." << std::endl;

//     im_max.swap(im_max_tmp);
//     im_min.swap(im_min_tmp);

//     std::cout << "Processing threshold tasks single-threaded..." << std::endl;
//     for (int ty = 0; ty < th; ty++) {
//         threshold_task task;
//         task.im = im;
//         task.threshim = threshim.get();
//         task.im_max = im_max.get();
//         task.im_min = im_min.get();
//         task.ty = ty;
//         task.td = td;
//         do_threshold_task(&task);
//     }
//     std::cout << "Threshold tasks completed." << std::endl;

//     std::cout << "Starting final thresholding..." << std::endl;
//     for (int y = 0; y < h; y++) {
//         int x0 = (y >= th * tilesz) ? 0 : tw * tilesz;
//         int ty = y / tilesz;
//         if (ty >= th) ty = th - 1;
//         for (int x = x0; x < w; x++) {
//             int tx = x / tilesz;
//             if (tx >= tw) tx = tw - 1;
//             int max = im_max[ty * tw + tx];
//             int min = im_min[ty * tw + tx];
//             int thresh = min + (max - min) / 2;
//             uint8_t v = im->buf[y * s + x];
//             threshim->buf[y * s + x] = (v > thresh) ? 255 : 0;
//         }
//     }
//     std::cout << "Final thresholding completed." << std::endl;

//     if (td->qtp.deglitch) {
//         std::cout << "Starting deglitching..." << std::endl;
//         auto tmp = image_u8_create_alignment(w, h, s);
//         for (int y = 1; y + 1 < h; y++) {
//             for (int x = 1; x + 1 < w; x++) {
//                 uint8_t max = 0;
//                 for (int dy = -1; dy <= 1; dy++) {
//                     for (int dx = -1; dx <= 1; dx++) {
//                         uint8_t v = threshim->buf[(y + dy) * s + x + dx];
//                         max = std::max(max, v);
//                     }
//                 }
//                 tmp->buf[y * s + x] = max;
//             }
//         }
//         for (int y = 1; y + 1 < h; y++) {
//             for (int x = 1; x + 1 < w; x++) {
//                 uint8_t min = 255;
//                 for (int dy = -1; dy <= 1; dy++) {
//                     for (int dx = -1; dx <= 1; dx++) {
//                         uint8_t v = tmp->buf[(y + dy) * s + x + dx];
//                         min = std::min(min, v);
//                     }
//                 }
//                 threshim->buf[y * s + x] = min;
//             }
//         }
//         std::cout << "Deglitching completed." << std::endl;
//     }

//     std::cout << "Threshold function completed." << std::endl;
//     return threshim;
// }

void applyDilation(image_u8_t* im) { //PURE DILATION
    constexpr int kernel_size = 3;
    constexpr int kernel_radius = kernel_size / 2;
    auto dilated = image_u8_create_alignment(im->width, im->height, im->stride);

    auto get_pixel = [im](int y, int x) -> std::optional<uint8_t> {
        if (y >= 0 && y < im->height && x >= 0 && x < im->width) {
            return im->buf[y * im->stride + x];
        }
        return std::nullopt;
    };

 
    for (int y = kernel_radius; y < im->height - kernel_radius; ++y) {
        for (int x = kernel_radius; x < im->width - kernel_radius; ++x) {
            uint8_t max_val = 0;
            for (int ky = -kernel_radius; ky <= kernel_radius; ++ky) {
                for (int kx = -kernel_radius; kx <= kernel_radius; ++kx) {
                    if (auto pixel = get_pixel(y + ky, x + kx); pixel.has_value()) {
                        max_val = std::max(max_val, pixel.value());
                    }
                }
            }
            dilated->buf[y * dilated->stride + x] = max_val;
        }
    }

    std::memcpy(im->buf.get(), dilated->buf.get(), im->height * im->stride);
}


void openingMorphologicalOperations(image_u8_t* im) {//OPENING OPERATION
    constexpr int kernel_size = 3;
    constexpr int kernel_radius = kernel_size / 2;
    auto eroded = image_u8_create_alignment(im->width, im->height, im->stride);
    auto dilated = image_u8_create_alignment(im->width, im->height, im->stride);

    auto get_pixel = [im](int y, int x) -> std::optional<uint8_t> {
        if (y >= 0 && y < im->height && x >= 0 && x < im->width) {
            return im->buf[y * im->stride + x];
        }
        return std::nullopt;
    };

    // Erosion
    for (int y = kernel_radius; y < im->height - kernel_radius; ++y) {
        for (int x = kernel_radius; x < im->width - kernel_radius; ++x) {
            uint8_t min_val = 255;
            for (int ky = -kernel_radius; ky <= kernel_radius; ++ky) {
                for (int kx = -kernel_radius; kx <= kernel_radius; ++kx) {
                    if (auto pixel = get_pixel(y + ky, x + kx); pixel.has_value()) {
                        min_val = std::min(min_val, pixel.value());
                    }
                }
            }
            eroded->buf[y * eroded->stride + x] = min_val;
        }
    }

    // Dilation
    for (int y = kernel_radius; y < eroded->height - kernel_radius; ++y) {
        for (int x = kernel_radius; x < eroded->width - kernel_radius; ++x) {
            uint8_t max_val = 0;
            for (int ky = -kernel_radius; ky <= kernel_radius; ++ky) {
                for (int kx = -kernel_radius; kx <= kernel_radius; ++kx) {
                    if (auto pixel = get_pixel(y + ky, x + kx); pixel.has_value()) {
                        max_val = std::max(max_val, pixel.value());
                    }
                }
            }
            dilated->buf[y * dilated->stride + x] = max_val;
        }
    }

    std::memcpy(im->buf.get(), dilated->buf.get(), im->height * im->stride);
}


void closingMorphologicalOperations(image_u8_t* im) {//CLOSING OPERATION
    constexpr int kernel_size = 3;
    constexpr int kernel_radius = kernel_size / 2;
    auto dilated = image_u8_create_alignment(im->width, im->height, im->stride);
    auto eroded = image_u8_create_alignment(im->width, im->height, im->stride);

    auto get_pixel = [im](int y, int x) -> std::optional<uint8_t> {
        if (y >= 0 && y < im->height && x >= 0 && x < im->width) {
            return im->buf[y * im->stride + x];
        }
        return std::nullopt;
    };

    // Dilation
    for (int y = kernel_radius; y < im->height - kernel_radius; ++y) {
        for (int x = kernel_radius; x < im->width - kernel_radius; ++x) {
            uint8_t max_val = 0;
            for (int ky = -kernel_radius; ky <= kernel_radius; ++ky) {
                for (int kx = -kernel_radius; kx <= kernel_radius; ++kx) {
                    if (auto pixel = get_pixel(y + ky, x + kx); pixel.has_value()) {
                        max_val = std::max(max_val, pixel.value());
                    }
                }
            }
            dilated->buf[y * dilated->stride + x] = max_val;
        }
    }

    // Erosion
    for (int y = kernel_radius; y < dilated->height - kernel_radius; ++y) {
        for (int x = kernel_radius; x < dilated->width - kernel_radius; ++x) {
            uint8_t min_val = 255;
            for (int ky = -kernel_radius; ky <= kernel_radius; ++ky) {
                for (int kx = -kernel_radius; kx <= kernel_radius; ++kx) {
                    if (auto pixel = get_pixel(y + ky, x + kx); pixel.has_value()) {
                        min_val = std::min(min_val, pixel.value());
                    }
                }
            }
            eroded->buf[y * eroded->stride + x] = min_val;
        }
    }

    std::memcpy(im->buf.get(), eroded->buf.get(), im->height * im->stride);
}

void binaryMask(image_u8_t* im, uint8_t threshold) {
    for (int y = 0; y < im->height; ++y) {
        for (int x = 0; x < im->width; ++x) {
            uint8_t& pixel = im->buf[y * im->stride + x];
            pixel = (pixel < threshold) ? 0 : 255;
        }
    }
}
// std::unique_ptr<image_u8_t> threshold(nursia_detector_t* td, image_u8_t* im) {
//     // std::cout << "Starting threshold function..." << std::endl;

//     int w = im->width, h = im->height, s = im->stride;
//     assert(w < 32768);
//     assert(h < 32768);
//     auto threshim = image_u8_create_alignment(w, h, s);
//     assert(threshim->stride == s);
//     const int tilesz = 4;
//     int tw = w / tilesz;
//     int th = h / tilesz;
//     auto im_max = std::make_unique<uint8_t[]>(tw * th);
//     auto im_min = std::make_unique<uint8_t[]>(tw * th);
//     auto minmax_tasks = std::make_unique<minmax_task[]>(th);

//     // std::cout << "Adding minmax tasks..." << std::endl;
//     for (int ty = 0; ty < th; ty++) {
//         minmax_tasks[ty].im = im;
//         minmax_tasks[ty].im_max = im_max.get();
//         minmax_tasks[ty].im_min = im_min.get();
//         minmax_tasks[ty].ty = ty;
//         workerpool_add_task(td, do_minmax_task, &minmax_tasks[ty]);
//     }
//     workerpool_run(td);
//     // std::cout << "Minmax tasks completed." << std::endl;

//     auto im_max_tmp = std::make_unique<uint8_t[]>(tw * th);
//     auto im_min_tmp = std::make_unique<uint8_t[]>(tw * th);
//     auto blur_tasks = std::make_unique<blur_task[]>(th);

//     // std::cout << "Adding blur tasks..." << std::endl;
//     for (int ty = 0; ty < th; ty++) {
//         blur_tasks[ty].im = im;
//         blur_tasks[ty].im_max = im_max.get();
//         blur_tasks[ty].im_min = im_min.get();
//         blur_tasks[ty].im_max_tmp = im_max_tmp.get();
//         blur_tasks[ty].im_min_tmp = im_min_tmp.get();
//         blur_tasks[ty].ty = ty;
//         workerpool_add_task(td, do_blur_task, &blur_tasks[ty]);
//     }
//     workerpool_run(td);
//     // std::cout << "Blur tasks completed." << std::endl;

//     im_max.swap(im_max_tmp);
//     im_min.swap(im_min_tmp);
//     auto threshold_tasks = std::make_unique<threshold_task[]>(th);

//     // std::cout << "Adding threshold tasks..." << std::endl;
//     for (int ty = 0; ty < th; ty++) {
//         threshold_tasks[ty].im = im;
//         threshold_tasks[ty].threshim = threshim.get();
//         threshold_tasks[ty].im_max = im_max.get();
//         threshold_tasks[ty].im_min = im_min.get();
//         threshold_tasks[ty].ty = ty;
//         threshold_tasks[ty].td = td;
//         workerpool_add_task(td, do_threshold_task, &threshold_tasks[ty]);
//     }
//     workerpool_run(td);
//     // std::cout << "Threshold tasks completed." << std::endl;

//     // std::cout << "Starting final thresholding..." << std::endl;
//     for (int y = 0; y < h; y++) {
//         int x0 = (y >= th * tilesz) ? 0 : tw * tilesz;
//         int ty = y / tilesz;
//         if (ty >= th) ty = th - 1;
//         for (int x = x0; x < w; x++) {
//             int tx = x / tilesz;
//             if (tx >= tw) tx = tw - 1;
//             int max = im_max[ty * tw + tx];
//             int min = im_min[ty * tw + tx];
//             int thresh = min + (max - min) / 2;
//             uint8_t v = im->buf[y * s + x];
//             threshim->buf[y * s + x] = (v > thresh) ? 255 : 0;
//         }
//     }
//     // std::cout << "Final thresholding completed." << std::endl;

//     if (td->qtp.deglitch) {
//         auto tmp = image_u8_create_alignment(w, h, s);
//         for (int y = 1; y + 1 < h; y++) {
//             for (int x = 1; x + 1 < w; x++) {
//                 uint8_t max = 0;
//                 for (int dy = -1; dy <= 1; dy++) {
//                     for (int dx = -1; dx <= 1; dx++) {
//                         uint8_t v = threshim->buf[(y + dy) * s + x + dx];
//                         max = std::max(max, v);
//                     }
//                 }
//                 tmp->buf[y * s + x] = max;
//             }
//         }
//         for (int y = 1; y + 1 < h; y++) {
//             for (int x = 1; x + 1 < w; x++) {
//                 uint8_t min = 255;
//                 for (int dy = -1; dy <= 1; dy++) {
//                     for (int dx = -1; dx <= 1; dx++) {
//                         uint8_t v = tmp->buf[(y + dy) * s + x + dx];
//                         min = std::min(min, v);
//                     }
//                 }
//                 threshim->buf[y * s + x] = min;
//             }
//         }
//     }
//     // std::cout << "Deglitching completed." << std::endl;
//     return threshim;
// }