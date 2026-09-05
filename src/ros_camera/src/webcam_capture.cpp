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

    capture_.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    capture_.set(cv::CAP_PROP_FPS, config.fps);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    return capture_.isOpened();
}

bool WebcamCapture::read(cv::Mat & frame)
{
    return capture_.isOpened() && capture_.read(frame) && !frame.empty();
}

void WebcamCapture::close()
{
    capture_.release();
}

bool WebcamCapture::is_open() const
{
    return capture_.isOpened();
}

int WebcamCapture::actual_width() const
{
    return static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
}

int WebcamCapture::actual_height() const
{
    return static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
}

double WebcamCapture::actual_fps() const
{
    return capture_.get(cv::CAP_PROP_FPS);
}

}  // namespace ros_camera
