#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

namespace ros_camera
{

struct WebcamConfig
{
    int device_index{0};
    int width{640};
    int height{480};
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
};

}  // namespace ros_camera
