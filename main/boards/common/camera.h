#ifndef CAMERA_H
#define CAMERA_H

#include <expected>
#include <string>

class Camera {
public:
    virtual ~Camera() = default;

    virtual void SetExplainUrl(const std::string& url, const std::string& token) = 0;
    virtual bool Capture() = 0;
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual bool SetSwapBytes(bool enabled) { return false; }  // Optional, default no-op
    virtual std::expected<std::string, std::string> Explain(const std::string& question) = 0;
    // Capture must be called first. Uploads the current frame as JPEG to an
    // arbitrary URL (e.g. a LAN PC receiver), bypassing the cloud explain URL.
    // Returns the server's response body (e.g. the saved filename) or an error.
    virtual std::expected<std::string, std::string> UploadPhotoTo(const std::string& url) {
        return std::unexpected("UploadPhotoTo is not supported by this camera");
    }
};

#endif  // CAMERA_H
