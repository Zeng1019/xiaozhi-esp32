#ifndef _UAC_AUDIO_CODEC_H_
#define _UAC_AUDIO_CODEC_H_

#include "audio_codec.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "usb/uac_host.h"

/**
 * USB UAC 1.0 host audio codec (e.g. webcam dongle with built-in mic + speaker).
 * Requires the USB host library to be installed beforehand (esp_video usb_uvc
 * with init_usb_host_lib=true, or manual usb_host_install).
 */
class UacAudioCodec : public AudioCodec {
public:
    // Installs the UAC host driver (usb host lib must already be installed).
    static esp_err_t InstallDriver();

    UacAudioCodec(int input_sample_rate, int output_sample_rate);
    virtual ~UacAudioCodec();

    virtual void Start() override;
    virtual void SetOutputVolume(int volume) override;

protected:
    virtual int Read(int16_t* dest, int samples) override;
    virtual int Write(const int16_t* data, int samples) override;

private:
    static void DriverEventCallback(uint8_t addr, uint8_t iface_num,
                                    const uac_host_driver_event_t event, void* arg);
    static void MicDeviceEventCallback(uac_host_device_handle_t handle,
                                       const uac_host_device_event_t event, void* arg);
    static void SpkDeviceEventCallback(uac_host_device_handle_t handle,
                                       const uac_host_device_event_t event, void* arg);

    struct StreamCtx {
        uac_host_device_handle_t handle = nullptr;
        SemaphoreHandle_t connected_sem = nullptr;
        volatile bool ready = false;
        uint8_t addr = 0;
        uint8_t iface_num = 0;
        // Negotiated stream format
        uint8_t alt_setting = 1;
        uint8_t channels = 1;
        uint32_t sample_freq = 16000;
    };

    bool OpenAndStart(StreamCtx& ctx, bool is_input);
    void NegotiateFormat(StreamCtx& ctx, uac_host_dev_info_t& info);

    static StreamCtx mic_;
    static StreamCtx spk_;
    static volatile bool driver_installed_;

    int16_t* scratch_ = nullptr;   // scratch buffer for resampling / channel conversion
    size_t scratch_samples_ = 0;
    float resample_pos_ = 0.0f;    // fractional position for linear resampler (output stream)
    float resample_pos_in_ = 0.0f; // fractional position (input stream)
    int16_t resample_last_out_ = 0;
    int16_t resample_last_in_ = 0;
};

#endif // _UAC_AUDIO_CODEC_H_
