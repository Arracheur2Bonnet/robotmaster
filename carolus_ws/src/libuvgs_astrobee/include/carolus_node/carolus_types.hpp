/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-06-28 01:13:16
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-08-10 09:05:50
 * @ Description:
 * STRUCTS AND STRUCTS
 */

#ifndef CAROLUS_TYPES_HPP
#define CAROLUS_TYPES_HPP

#pragma once

#include <opencv2/opencv.hpp>


struct Blob {
    double x, y;
};

struct BlobProperties {
    double perimeter;
    double m00;
    double circularity;
    // std::string color;
    double hue;
    cv::Rect boundingContour;
    
};

struct BlobCarolus { 
    Blob blob;
    BlobProperties properties;
};

#endif // CAROLUS_TYPES_HPP