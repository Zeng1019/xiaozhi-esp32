#include "uac_audio_codec.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <climits>
#include <algorithm>

#define TAG "UacAudioCodec"

UacAudioCodec::StreamCtx UacAudioCodec::mic_;
UacAudioCodec::StreamCtx UacAudioCodec::spk_;
volatile bool UacAudioCodec::driver_installed_ = false;

static const uint32_t TARGET_FREQ = 16000;

esp_err_t UacAudioCodec::InstallDriver() {
    if (driver_installed_) {
        return ESP_OK;
    }
    mic_.connected_sem = xSemaphoreCreateBinary();
    spk_.connected_sem = xSemaphoreCreateBinary();
    uac_host_driver_config_t driver_config = {};
    driver_config.create_background_task = true;
    driver_config.task_priority = 5;
    driver_config.stack_size = 4096;
    driver_config.core_id = 0;
    driver_config.callback = &UacAudioCodec::DriverEventCallback;
    driver_config.callback_arg = nullptr;
    esp_err_t err = uac_host_install(&driver_config);
    if (err == ESP_OK) {
        driver_installed_ = true;
        ESP_LOGI(TAG, "UAC host driver installed");
    } else {
        ESP_LOGE(TAG, "UAC host driver install failed: %s", esp_err_to_name(err));
    }
    return err;
}

void UacAudioCodec::DriverEventCallback(uint8_t addr, uint8_t iface_num,
                                        const uac_host_driver_event_t event, void* arg) {
    switch (event) {
        case UAC_HOST_DRIVER_EVENT_RX_CONNECTED:
            mic_.addr = addr;
            mic_.iface_num = iface_num;
            ESP_LOGI(TAG, "UAC microphone found, addr: %u, iface: %u", addr, iface_num);
            xSemaphoreGive(mic_.connected_sem);
            break;
        case UAC_HOST_DRIVER_EVENT_TX_CONNECTED:
            spk_.addr = addr;
            spk_.iface_num = iface_num;
            ESP_LOGI(TAG, "UAC speaker found, addr: %u, iface: %u", addr, iface_num);
            xSemaphoreGive(spk_.connected_sem);
            break;
        default:
            break;
    }
}

void UacAudioCodec::MicDeviceEventCallback(uac_host_device_handle_t handle,
                                           const uac_host_device_event_t event, void* arg) {
    switch (event) {
        case UAC_HOST_DEVICE_EVENT_RX_DONE:
            // Data available in driver buffer; Read() pulls it synchronously.
            break;
        case UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR:
            ESP_LOGE(TAG, "UAC mic transfer error");
            break;
        case UAC_HOST_DRIVER_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "UAC microphone disconnected");
            mic_.ready = false;
            mic_.handle = nullptr;
            break;
        default:
            break;
    }
}

void UacAudioCodec::SpkDeviceEventCallback(uac_host_device_handle_t handle,
                                           const uac_host_device_event_t event, void* arg) {
    switch (event) {
        case UAC_HOST_DEVICE_EVENT_TX_DONE:
            break;
        case UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR:
            ESP_LOGE(TAG, "UAC speaker transfer error");
            break;
        case UAC_HOST_DRIVER_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "UAC speaker disconnected");
            spk_.ready = false;
            spk_.handle = nullptr;
            break;
        default:
            break;
    }
}

void UacAudioCodec::NegotiateFormat(StreamCtx& ctx, uac_host_dev_info_t& info) {
    // Enumerate alternate settings, pick 16-bit PCM closest to TARGET_FREQ,
    // preferring mono.
    int best_alt = -1;
    uint32_t best_freq = 0;
    uint8_t best_channels = 1;
    int best_score = INT32_MAX;

    for (uint8_t alt = 1; alt < info.iface_alt_num; alt++) {
        uac_host_dev_alt_param_t param = {};
        if (uac_host_get_device_alt_param(ctx.handle, alt, &param) != ESP_OK) {
            continue;
        }
        if (param.bit_resolution != 16 || param.format != 1 /* PCM */) {
            continue;
        }
        uint32_t freq = TARGET_FREQ;
        if (param.sample_freq_type == 0) {
            // Continuous range
            if (TARGET_FREQ < param.sample_freq_lower) freq = param.sample_freq_lower;
            else if (TARGET_FREQ > param.sample_freq_upper) freq = param.sample_freq_upper;
        } else {
            // Discrete list
            uint32_t best_disc = 0;
            int32_t best_diff = INT32_MAX;
            for (int i = 0; i < UAC_FREQ_NUM_MAX; i++) {
                uint32_t f = param.sample_freq[i];
                if (f == 0) continue;
                int32_t diff = abs((int32_t)f - (int32_t)TARGET_FREQ);
                if (diff < best_diff) {
                    best_diff = diff;
                    best_disc = f;
                }
            }
            if (best_disc == 0) continue;
            freq = best_disc;
        }
        int score = (int)freq - (int)TARGET_FREQ;
        if (score < 0) score = -score;
        if (param.channels != 1) score += 1000; // prefer mono
        if (score < best_score) {
            best_score = score;
            best_alt = alt;
            best_freq = freq;
            best_channels = param.channels;
        }
    }

    if (best_alt >= 0) {
        ctx.alt_setting = best_alt;
        ctx.sample_freq = best_freq;
        ctx.channels = best_channels;
    }
    ESP_LOGI(TAG, "UAC stream: alt=%u ch=%u freq=%lu", ctx.alt_setting,
             ctx.channels, (unsigned long)ctx.sample_freq);
}

bool UacAudioCodec::OpenAndStart(StreamCtx& ctx, bool is_input) {
    uac_host_dev_info_t info = {};
    uac_host_device_config_t dev_config = {};
    dev_config.addr = ctx.addr;
    dev_config.iface_num = ctx.iface_num;
    dev_config.buffer_size = 16384;
    dev_config.buffer_threshold = is_input ? 1024 : 2048;
    dev_config.callback = is_input ? &UacAudioCodec::MicDeviceEventCallback
                                   : &UacAudioCodec::SpkDeviceEventCallback;
    dev_config.callback_arg = nullptr;

    esp_err_t err = uac_host_device_open(&dev_config, &ctx.handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open UAC %s device: %s", is_input ? "mic" : "speaker",
                 esp_err_to_name(err));
        return false;
    }

    if (uac_host_get_device_info(ctx.handle, &info) == ESP_OK) {
        NegotiateFormat(ctx, info);
    }

    uac_host_stream_config_t stream_config = {};
    stream_config.channels = ctx.channels;
    stream_config.bit_resolution = 16;
    stream_config.sample_freq = ctx.sample_freq;
    stream_config.flags = 0;

    err = uac_host_device_start(ctx.handle, &stream_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start UAC %s: %s", is_input ? "mic" : "speaker",
                 esp_err_to_name(err));
        uac_host_device_close(ctx.handle);
        ctx.handle = nullptr;
        return false;
    }

    // Open uses alt setting 1 by default; if we negotiated a different one,
    // restart with explicit alt setting via stream config is not exposed, the
    // driver selects the closest matching alt setting itself.
    ctx.ready = true;
    ESP_LOGI(TAG, "UAC %s started: %lu Hz", is_input ? "microphone" : "speaker",
             (unsigned long)ctx.sample_freq);
    return true;
}

UacAudioCodec::UacAudioCodec(int input_sample_rate, int output_sample_rate) {
    duplex_ = true;
    input_reference_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;
    // Base Start() will load persisted volume; keep default here.
}

UacAudioCodec::~UacAudioCodec() {
    if (mic_.handle) {
        uac_host_device_stop(mic_.handle);
        uac_host_device_close(mic_.handle);
    }
    if (spk_.handle) {
        uac_host_device_stop(spk_.handle);
        uac_host_device_close(spk_.handle);
    }
    if (scratch_) {
        heap_caps_free(scratch_);
    }
}

void UacAudioCodec::Start() {
    AudioCodec::Start();

    // Wait for the UAC interfaces of the webcam (mic + speaker) to show up.
    if (xSemaphoreTake(mic_.connected_sem, pdMS_TO_TICKS(15000)) == pdTRUE) {
        OpenAndStart(mic_, true);
    } else {
        ESP_LOGW(TAG, "UAC microphone not detected within 15s");
    }
    if (xSemaphoreTake(spk_.connected_sem, pdMS_TO_TICKS(10000)) == pdTRUE) {
        OpenAndStart(spk_, false);
        uac_host_device_set_volume(spk_.handle, output_volume_);
    } else {
        ESP_LOGW(TAG, "UAC speaker not detected within 10s");
    }
}

void UacAudioCodec::SetOutputVolume(int volume) {
    output_volume_ = volume;
    if (spk_.ready && spk_.handle) {
        uac_host_device_set_volume(spk_.handle, volume);
    }
    AudioCodec::SetOutputVolume(volume);
}

int UacAudioCodec::Read(int16_t* dest, int samples) {
    if (!mic_.ready || !mic_.handle) {
        return 0;
    }
    int got = 0;
    uint8_t* p = (uint8_t*)dest;
    uint32_t want = samples * mic_.channels * sizeof(int16_t);
    // Blocking read with retries; give up after ~200ms of silence.
    while (got < (int)want) {
        uint32_t bytes_read = 0;
        esp_err_t err = uac_host_device_read(mic_.handle, p + got, want - got,
                                             &bytes_read, 50);
        if (err != ESP_OK || bytes_read == 0) {
            if (got == 0) {
                return 0;
            }
            break;
        }
        got += bytes_read;
    }
    if (got == 0) {
        return 0;
    }

    int frames = got / (mic_.channels * (int)sizeof(int16_t));

    // Debug: periodically report input level so we can verify real audio
    // is arriving from the USB microphone.
    static int dbg_counter = 0;
    static int64_t dbg_sum = 0;
    static int16_t dbg_max = 0;
    for (int i = 0; i < frames; i++) {
        int16_t v = dest[i];
        int32_t a = v < 0 ? -v : v;
        dbg_sum += a;
        if (v > dbg_max) dbg_max = v;
        else if (-v > dbg_max) dbg_max = -v;
    }
    if (++dbg_counter >= 50) {
        ESP_LOGI(TAG, "Mic level: avg=%lld max=%d (50 frames, %d Hz)",
                 dbg_sum / (50 * frames), dbg_max, (int)mic_.sample_freq);
        dbg_counter = 0;
        dbg_sum = 0;
        dbg_max = 0;
    }

    if (mic_.channels == 2) {
        // Downmix stereo (in-place, interleaved -> mono)
        for (int i = 0; i < frames; i++) {
            dest[i] = (int16_t)(((int32_t)dest[2 * i] + dest[2 * i + 1]) / 2);
        }
    }

    if (mic_.sample_freq == (uint32_t)input_sample_rate_) {
        return frames;
    }

    // Linear resample device freq -> input_sample_rate_ (in-place, step > 1
    // for typical 48k -> 16k downsampling, so out index never passes read index)
    float step = (float)mic_.sample_freq / input_sample_rate_;
    int out = 0;
    float pos = resample_pos_in_;   // position in input frames
    int16_t last = resample_last_in_;
    while (out < samples) {
        int idx = (int)pos;
        if (idx >= frames) break;
        float frac = pos - idx;
        int16_t s0 = (idx == 0) ? last : dest[idx - 1];
        int16_t s1 = dest[idx];
        dest[out++] = (int16_t)(s0 + (s1 - s0) * frac);
        pos += step;
    }
    resample_pos_in_ = pos - frames;
    resample_last_in_ = dest[frames - 1];
    return out;
}

int UacAudioCodec::Write(const int16_t* data, int samples) {
    if (!spk_.ready || !spk_.handle) {
        return samples; // pretend success when no device attached
    }

    const int16_t* out_data = data;
    int out_frames = samples;
    int16_t* buf = nullptr;

    if (spk_.sample_freq != (uint32_t)output_sample_rate_ || spk_.channels == 2) {
        // Convert: resample 16k -> device freq, mono -> N channels
        float ratio = (float)spk_.sample_freq / output_sample_rate_;
        out_frames = (int)(samples * ratio) + 2;
        size_t need = out_frames * spk_.channels * sizeof(int16_t);
        if (scratch_samples_ < out_frames * spk_.channels) {
            if (scratch_) heap_caps_free(scratch_);
            scratch_ = (int16_t*)heap_caps_malloc(need, MALLOC_CAP_8BIT);
            scratch_samples_ = out_frames * spk_.channels;
        }
        buf = scratch_;
        float step = ratio;
        float pos = resample_pos_;   // position in input frames
        int16_t last = resample_last_out_;
        int i = 0;
        for (; i < out_frames; i++) {
            if ((int)pos >= samples) break;
            int idx = (int)pos;
            float frac = pos - idx;
            int16_t s0 = (idx == 0) ? last : data[idx - 1];
            int16_t s1 = data[idx];
            int16_t val = (int16_t)(s0 + (s1 - s0) * frac);
            for (int c = 0; c < spk_.channels; c++) {
                buf[i * spk_.channels + c] = val;
            }
            pos += step;
        }
        resample_pos_ = pos - samples;  // keep phase across buffers
        resample_last_out_ = data[samples - 1];
        out_data = buf;
        out_frames = i;
    }

    uint32_t bytes = out_frames * spk_.channels * sizeof(int16_t);
    const uint8_t* p = (const uint8_t*)out_data;
    uint32_t written = 0;
    while (written < bytes) {
        uint32_t chunk = std::min<uint32_t>(bytes - written, 2048);
        esp_err_t err = uac_host_device_write(spk_.handle, (uint8_t*)(p + written), chunk, 500);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "UAC speaker write error: %s", esp_err_to_name(err));
            break;
        }
        written += chunk;
    }
    return samples;
}
