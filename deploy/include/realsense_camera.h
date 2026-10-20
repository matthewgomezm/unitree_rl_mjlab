#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <librealsense2/rs.hpp>
#include "depth_camera.h"

struct RealSenseConfig
{
    int stream_width = 848;   // D435 depth native 16:9 mode
    int stream_height = 480;
    int fps = 30;
    std::string serial = "";  // empty = first device found
};

/**
 * Intel RealSense (D435 / D4xx) depth camera via librealsense2.
 * Streams raw Z16 depth, converts it to meters, and reads the vertical FOV
 * from the device intrinsics so the base class crops it to match the sim.
 */
class RealSenseCamera : public DepthCamera
{
public:
    RealSenseCamera(RealSenseConfig rs_cfg, DepthCamera::Config cfg = DepthCamera::Config())
    : DepthCamera(cfg), rs_cfg_(rs_cfg) {}

    ~RealSenseCamera() override { stop(); }

protected:
    bool open() override
    {
        try {
            rs2::config c;
            if(!rs_cfg_.serial.empty()) c.enable_device(rs_cfg_.serial);
            c.enable_stream(RS2_STREAM_DEPTH, rs_cfg_.stream_width, rs_cfg_.stream_height,
                            RS2_FORMAT_Z16, rs_cfg_.fps);
            auto profile = pipe_.start(c);

            depth_scale_ = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
            auto intr = profile.get_stream(RS2_STREAM_DEPTH)
                               .as<rs2::video_stream_profile>().get_intrinsics();
            fovy_deg_ = 2.0f * std::atan(0.5f * intr.height / intr.fy) * 180.0f / static_cast<float>(M_PI);

            spdlog::info("RealSense: {} {}x{} @ {} fps, vertical FOV {:.1f} deg, depth scale {}",
                         profile.get_device().get_info(RS2_CAMERA_INFO_NAME),
                         intr.width, intr.height, rs_cfg_.fps, fovy_deg_, depth_scale_);
            return true;
        } catch(const rs2::error& e) {
            spdlog::error("RealSense: {} ({})", e.what(), e.get_failed_function());
            return false;
        }
    }

    void close() override
    {
        try { pipe_.stop(); } catch(const rs2::error&) {}
    }

    bool read_frame(std::vector<float>& depth_m, int& width, int& height) override
    {
        try {
            rs2::frameset frames;
            if(!pipe_.try_wait_for_frames(&frames, 1000)) {
                spdlog::warn("RealSense: no frame for 1 s.");
                return false;
            }
            rs2::depth_frame depth = frames.get_depth_frame();
            width = depth.get_width();
            height = depth.get_height();

            // Z16: raw units * depth_scale = meters; 0 means no reading.
            const auto* raw = static_cast<const uint16_t*>(depth.get_data());
            depth_m.resize(static_cast<size_t>(width) * height);
            for(size_t i = 0; i < depth_m.size(); ++i) {
                depth_m[i] = raw[i] * depth_scale_;
            }
            return true;
        } catch(const rs2::error& e) {
            spdlog::error("RealSense: {}", e.what());
            return false;
        }
    }

    float native_fovy_deg() const override { return fovy_deg_; }

private:
    RealSenseConfig rs_cfg_;
    rs2::pipeline pipe_;
    float depth_scale_ = 0.001f;
    float fovy_deg_ = 58.0f;  // overwritten from intrinsics in open()
};
