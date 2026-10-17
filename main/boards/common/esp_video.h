#pragma once
#include "sdkconfig.h"

#include <lvgl.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "camera.h"
#include "esp_video_init.h"
#include "jpg/image_to_jpeg.h"

struct JpegChunk {
    uint8_t* data;
    size_t len;
};

class EspVideo : public Camera {
private:
    struct FrameBuffer {
        uint8_t* data = nullptr;
        size_t len = 0;
        uint16_t width = 0;
        uint16_t height = 0;
        v4l2_pix_fmt_t format = 0;
    } frame_;
    v4l2_pix_fmt_t sensor_format_ = 0;
#ifdef CONFIG_XIAOZHI_ENABLE_ROTATE_CAMERA_IMAGE
    uint16_t sensor_width_ = 0;
    uint16_t sensor_height_ = 0;
#endif  // CONFIG_XIAOZHI_ENABLE_ROTATE_CAMERA_IMAGE
    int video_fd_ = -1;
    bool streaming_on_ = false;
    struct MmapBuffer {
        void* start = nullptr;
        size_t length = 0;
    };
    std::vector<MmapBuffer> mmap_buffers_;
    std::string explain_url_;
    std::string explain_token_;
    std::thread encoder_thread_;
    // Serializes Capture()/GetLastJpegFrame(): two concurrent captures would
    // race on the single V4L2 DQBUF/QBUF pair, and frame_.data may be freed
    // and reallocated inside Capture().
    std::mutex capture_mutex_;
    std::atomic<bool> preview_enabled_{true};
    // Capture() 内部丢弃的旧帧数：默认 3 次 DQBUF（拍新帧最稳）。
    // 低延迟场景（人脸跟踪）可设 1，省掉两帧等待，~300ms/次。
    int capture_dqbuf_count_ = 3;

public:
    EspVideo(const esp_video_init_config_t& config);
    ~EspVideo() override;

    virtual void SetExplainUrl(const std::string& url, const std::string& token);
    virtual bool Capture();
    // 翻转控制函数
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual std::expected<std::string, std::string> Explain(const std::string& question) override;

    // Copy of the last captured frame. Only supported when the sensor frame
    // is JPEG (CONFIG_XIAOZHI_CAMERA_ALLOW_JPEG_INPUT boards). The copy is
    // taken under capture_mutex_, so it is safe against concurrent Capture().
    bool GetLastJpegFrame(std::vector<uint8_t>& out, uint16_t* width, uint16_t* height);
    // Suppress the LVGL preview path in Capture() (skips the full-frame
    // decode + alloc); used by low-rate background watchers.
    void SetPreviewEnabled(bool enabled) { preview_enabled_.store(enabled); }
    // 2026-10-16: Capture() 内部 DQBUF 次数（1=直取最浅延迟，3=默认最稳）
    void SetCaptureDqbufCount(int n) { capture_dqbuf_count_ = n < 1 ? 1 : n; }
    // 2026-10-17: 请求更低的相机分辨率档位（在构造前调用）。相机实际档位
    // 有限（本机支持 1280x720/800x480/640x480/480x320），S_FMT 被拒时
    // 自动保持相机默认档。分辨率影响所有照片路径（在座判定阈值按 640x480
    // 标定，改档后需重新验证）。
    static void SetPreferredSize(int width, int height) {
        s_pref_width_ = width;
        s_pref_height_ = height;
    }
    // 2026-10-17: 请求相机帧率档（低光下 UVC 相机常自动降到 7.5fps）。
    // 强制高帧率在暗处会缩短曝光、画面变暗——脸检测变差时改回去。
    static void SetPreferredFps(int fps) { s_pref_fps_ = fps; }

private:
    static int s_pref_width_;
    static int s_pref_height_;
    static int s_pref_fps_;
};
