#pragma once

#include <atomic>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

namespace ros_camera
{

struct WebcamConfig
{
    int device_index{0};
    int width{0};
    int height{0};
    double fps{10.0};
};

class WebcamCapture
{
public:
    bool open(const WebcamConfig & config);
    bool read(cv::Mat & frame);
    void close();
    bool is_open() const;

    int actual_width() const;
    int actual_height() const;
    double actual_fps() const;

private:
    cv::VideoCapture capture_;
    std::atomic<int> actual_width_{0};
    std::atomic<int> actual_height_{0};
};

}  // namespace ros_camera
