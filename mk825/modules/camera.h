#pragma once

#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstring>
#include <cstdint>
#include <chrono>


namespace mk825
{

struct Image
{
    int width, height;
    int channels;
    std::vector<uint8_t> data;
    int64_t stamp{0};

    Image(int w = 0, int h = 0, int c = 1) 
    : width(w), height(h), channels(c)
    {}
};

struct ColorImage : Image
{
    ColorImage(int w, int h) : Image(w, h, 3)
    {

    }

    ColorImage(int w, int h, std::vector<uint8_t> d) : Image(w, h, 3)
    {
        data = std::move(d);
    }
};

class Camera
{
private:

protected:
    const int width, height;
    const int channels;
    ColorImage _recent_frame;

    std::thread _capture_thread;
    std::mutex _frame_mutex;
    std::atomic<bool> _running{false};
    std::atomic<bool> _started{false};
    
public:
    Camera(const int w, const int h, const int c) 
    : width(w), height(h), channels(c), _recent_frame(w, h)
    {
    }

    void start()
    {
        if (_started) return;
        this->init(); 
        _started = true;
        _running = true;
        _capture_thread = std::thread(&Camera::_capture_loop, this);
    }

    int frameSize() const
    {
        return width * height * channels;
    }

    virtual void init() = 0;
    virtual void _capture_loop() = 0;

    ColorImage getFrame()
    {   
        std::lock_guard<std::mutex> lock(_frame_mutex);
        if(_recent_frame.data.empty())
            return ColorImage(width, height, std::vector<uint8_t>(width*height*channels, 0));

        return _recent_frame;
    }

    void stop()
    {
        _running = false;
        if( _capture_thread.joinable() ) { _capture_thread.join(); }
    }

    virtual ~Camera()
    {
        this->stop();
    }

};


class RS2Camera : public Camera
{
private:
    rs2::pipeline _pipe;
    

public:
    RS2Camera(int w, int h) : Camera(w, h, 3)
    {

    }

    ~RS2Camera()
    {
        this->stop();
        if (_started) _pipe.stop();
    }

    void init() override
    {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
        auto profile = _pipe.start(cfg);

        // Force constant 30 fps: stop the RGB sensor from lowering framerate to
        // lengthen exposure in dim light (this is the usual cause of "max 13 fps").
        try {
            auto color_sensor = profile.get_device().first<rs2::color_sensor>();
            if (color_sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
                color_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.f);
        } catch (const rs2::error & e) {
            std::cerr << "Warning - could not disable auto-exposure \
                        in favor of 30fps " << e.what() << std::endl;
        } 
    }


private:
    rs2::frameset getRSFrames()
    {
        rs2::frameset frames;
        try {
            frames = _pipe.wait_for_frames(1000);
        } catch (const rs2::error &) {
            
        }
        return frames;
    }

    void _capture_loop() override
    {
        while (_running) {

            rs2::frameset frames = this->getRSFrames();
            if(!frames) continue;

            // --- color ---
            auto f = frames.get_color_frame();

            if(!f) continue;

            const int h = f.get_height();
            const size_t rowBytes = f.get_width() * f.get_bytes_per_pixel();
            const size_t stride = f.get_stride_in_bytes();
            
            std::vector<uint8_t> colorFrame_vec = std::vector<uint8_t>(rowBytes * h / sizeof(uint8_t));


            const auto* src = static_cast<const uint8_t*>(f.get_data());
            auto* dst = reinterpret_cast<uint8_t*>(colorFrame_vec.data());
            if (stride == rowBytes)
                std::memcpy(dst, src, rowBytes * h);
            else
                for (int y = 0; y < h; ++y)
                    std::memcpy(dst + y * rowBytes, src + y * stride, rowBytes);

            {
                std::lock_guard<std::mutex> lock(_frame_mutex);
                this->_recent_frame = ColorImage(this->width, this->height, std::move(colorFrame_vec));
                this->_recent_frame.stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
            }

            /*
            cv::Mat color_mat(
                cv::Size(color_frame.get_width(), color_frame.get_height()),
                CV_8UC3,
                const_cast<void *>(color_frame.get_data()),
                cv::Mat::AUTO_STEP);
            */
            // raw for compute_node (ML)
            /*
            auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_mat).toImageMsg();
            color_msg->header.stamp = stamp;
            color_msg->header.frame_id = "camera_color_optical_frame";
            color_pub_->publish(*color_msg);
            */
            




            // --- depth ---
            /*
            auto depth_frame = frames.get_depth_frame();
            cv::Mat depth_mat(
                cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                CV_16UC1,
                const_cast<void *>(depth_frame.get_data()),
                cv::Mat::AUTO_STEP);
            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_mat).toImageMsg();
            depth_msg->header.stamp = stamp;
            depth_msg->header.frame_id = "camera_depth_optical_frame";
            depth_pub_->publish(*depth_msg);
            */
        }
    } //-- _captureLoo()
}; //-- class RS2Camera

}//-- namespace mk825
