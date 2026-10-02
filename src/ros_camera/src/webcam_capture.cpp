#include "ros_camera/webcam_capture.hpp"

namespace ros_camera
{

bool WebcamCapture::open(const WebcamConfig & config)
{
    close();

    if (!capture_.open(config.device_index, cv::CAP_V4L2)) {
        capture_.release();
        if (!capture_.open(config.device_index, cv::CAP_ANY)) {
            return false;
        }
    }

    if (config.width > 0) {
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    }
    if (config.height > 0) {
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    }
    capture_.set(cv::CAP_PROP_FPS, config.fps);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    return capture_.isOpened();
}

bool WebcamCapture::read(cv::Mat & frame)
{
    if (!capture_.isOpened() || !capture_.read(frame) || frame.empty()) {
        return false;
    }

    actual_width_.store(frame.cols);
    actual_height_.store(frame.rows);
    return true;
}

void WebcamCapture::close()
{
    capture_.release();
    actual_width_.store(0);
    actual_height_.store(0);
}

bool WebcamCapture::is_open() const
{
    return capture_.isOpened();
}

int WebcamCapture::actual_width() const
{
    return actual_width_.load();
}

int WebcamCapture::actual_height() const
{
    return actual_height_.load();
}

double WebcamCapture::actual_fps() const
{
    return capture_.get(cv::CAP_PROP_FPS);
}

}  // namespace ros_camera
