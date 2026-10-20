#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>
#include <spdlog/spdlog.h>


struct DepthCameraConfig
{
    int out_width = 64;     // sim camera resolution
    int out_height = 36;
    float out_fovy_deg = 57.0f;  // sim camera vertical FOV
};

class DepthCamera
{
public:
    using Config = DepthCameraConfig;

    explicit DepthCamera(Config cfg = Config()) : cfg_(cfg) {}

    virtual ~DepthCamera()
    {
        if(running_) {
            spdlog::error("DepthCamera: destroyed while running; the subclass "
                          "destructor must call stop().");
        }
    }

    DepthCamera(const DepthCamera&) = delete;
    DepthCamera& operator=(const DepthCamera&) = delete;

    bool start()
    {
        if(running_) return true;
        if(!open()) {
            spdlog::error("DepthCamera: failed to open device.");
            return false;
        }
        running_ = true;
        thread_ = std::thread([this]{ capture_loop(); });
        return true;
    }

    void stop()
    {
        if(!running_) return;
        running_ = false;
        if(thread_.joinable()) thread_.join();
        close();
    }

    std::vector<float> get_latest_depth() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return latest_;
    }

    double frame_age() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if(latest_.empty()) return 1e9;
        return std::chrono::duration<double>(clock::now() - stamp_).count();
    }

    const Config& config() const { return cfg_; }

protected:
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool read_frame(std::vector<float>& depth_m, int& width, int& height) = 0;
    virtual float native_fovy_deg() const = 0;

private:
    using clock = std::chrono::steady_clock;

    void capture_loop()
    {
        std::vector<float> raw;
        int w = 0, h = 0;
        while(running_)
        {
            if(!read_frame(raw, w, h) || w <= 0 || h <= 0) {
                continue;
            }
            auto out = preprocess(raw, w, h);
            std::lock_guard<std::mutex> lock(mtx_);
            latest_ = std::move(out);
            stamp_ = clock::now();
        }
    }

    std::vector<float> preprocess(const std::vector<float>& raw, int w, int h) const
    {
        const float deg2rad = static_cast<float>(M_PI) / 180.0f;
        const float fov_ratio = std::tan(0.5f * cfg_.out_fovy_deg * deg2rad)
                              / std::tan(0.5f * native_fovy_deg() * deg2rad);
        if(fov_ratio > 1.0f) {
            spdlog::warn("DepthCamera: native FOV is narrower than the sim FOV; "
                         "using the full frame.");
        }
        const float crop_h = h * std::min(fov_ratio, 1.0f);
        const float crop_w = std::min<float>(w, crop_h * cfg_.out_width / cfg_.out_height);
        const float x0 = 0.5f * (w - crop_w);
        const float y0 = 0.5f * (h - crop_h);

        std::vector<float> out(cfg_.out_width * cfg_.out_height, 0.0f);
        const float cell_w = crop_w / cfg_.out_width;
        const float cell_h = crop_h / cfg_.out_height;
        for(int oy = 0; oy < cfg_.out_height; ++oy)
        {
            const int ys = static_cast<int>(y0 + oy * cell_h);
            const int ye = std::max(ys + 1, static_cast<int>(y0 + (oy + 1) * cell_h));
            for(int ox = 0; ox < cfg_.out_width; ++ox)
            {
                const int xs = static_cast<int>(x0 + ox * cell_w);
                const int xe = std::max(xs + 1, static_cast<int>(x0 + (ox + 1) * cell_w));
                float sum = 0.0f;
                int count = 0;
                for(int y = ys; y < std::min(ye, h); ++y) {
                    for(int x = xs; x < std::min(xe, w); ++x) {
                        const float d = raw[y * w + x];
                        if(std::isfinite(d) && d > 0.0f) {
                            sum += d;
                            ++count;
                        }
                    }
                }
                out[oy * cfg_.out_width + ox] = count > 0 ? sum / count : 0.0f;
            }
        }
        return out;
    }

    Config cfg_;
    std::thread thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mtx_;
    std::vector<float> latest_;
    clock::time_point stamp_;
};
