#include "wifi_board.h"
#include "board.h"
#include "uac_audio_codec.h"
#include "display/lcd_display.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <nvs.h>
#include <time.h>
#include <freertos/semphr.h>
#include <vector>
#include <string>
#include <atomic>
#include <atomic>
#include <cstring>

#include "usb/usb_host.h"
#include "esp_video_init.h"
#include "esp_video.h"
#include "jpg/jpeg_to_image.h"
#include <driver/ledc.h>
#include <led_strip.h>
#include <cstdlib>
#include <cJSON.h>

#define TAG "EspClawBoard"

// Temporary diagnostic: dump every USB device that enumerates, with the class
// of each interface, to verify the UVC+UAC webcam dongle is visible at all.
static void ProbeClientCb(const usb_host_client_event_msg_t*, void*) {}

static void UsbProbeTask(void*) {
    usb_host_client_config_t cfg = {};
    cfg.is_synchronous = false;  // esp-usb 1.5.x supports async clients only
    cfg.max_num_event_msg = 10;
    cfg.async.client_event_callback = ProbeClientCb;
    cfg.async.callback_arg = nullptr;
    usb_host_client_handle_t client = nullptr;
    if (usb_host_client_register(&cfg, &client) != ESP_OK) {
        vTaskDelete(nullptr);
        return;
    }
    bool dumped = false;
    for (int waited = 0; waited < 25000 && !dumped; waited += 100) {
        usb_host_client_handle_events(client, 5);
        uint8_t addr_list[8] = {};
        int num = 0;
        usb_host_device_addr_list_fill(8, addr_list, &num);
        for (int i = 0; i < num; ++i) {
            usb_device_handle_t dev = nullptr;
            if (usb_host_device_open(client, addr_list[i], &dev) != ESP_OK) {
                continue;
            }
            const usb_device_desc_t* dev_desc = nullptr;
            const usb_config_desc_t* cfg_desc = nullptr;
            usb_host_get_device_descriptor(dev, &dev_desc);
            usb_host_get_active_config_descriptor(dev, &cfg_desc);
            if (dev_desc) {
                ESP_LOGI(TAG, "USB dev addr=%u VID=%04x PID=%04x", addr_list[i],
                         dev_desc->idVendor, dev_desc->idProduct);
            }
            if (cfg_desc) {
                int offset = cfg_desc->bLength;
                while (offset + 9 <= cfg_desc->wTotalLength) {
                    const uint8_t* p = ((const uint8_t*)cfg_desc) + offset;
                    if (p[1] == 0x04) { // interface descriptor
                        ESP_LOGI(TAG, "  iface %u alt %u class=0x%02x subclass=0x%02x",
                                 p[2], p[3], p[5], p[6]);
                    }
                    if (p[0] == 0) break;
                    offset += p[0];
                }
            }
            dumped = true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!dumped) {
        ESP_LOGE(TAG, "USB probe: NO DEVICE ENUMERATED within 25s");
    } else {
        ESP_LOGI(TAG, "USB probe done");
    }
    vTaskDelete(nullptr);
}


// SPI LCD that boots straight into the dark theme (black bg / white text),
// matching the black-background claw-face emoji set. SetTheme persists the
// choice in NVS, so later boots load "dark" by themselves.
class ClawLcdDisplay : public SpiLcdDisplay {
public:
    using SpiLcdDisplay::SpiLcdDisplay;

    virtual void SetupUI() override {
        SpiLcdDisplay::SetupUI();
        auto* dark_theme = LvglThemeManager::GetInstance().GetTheme("dark");
        if (dark_theme != nullptr) {
            SetTheme(dark_theme);
        }
    }
};

class EspClawBoard : public WifiBoard {
private:
    Button boot_button_;
    Display* display_ = nullptr;
    Camera* camera_ = nullptr;
    EspVideo* esp_video_ = nullptr;  // typed alias of camera_ for frame access
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    led_strip_handle_t led_strip_ = nullptr;
    int led_brightness_ = 128;  // 0-255 master scale for the LED ring
#if CLAW_SERVO_POSITIONAL
    int servo_angle_ = 90;  // 0-180 degrees, SG90 positional servo holds here
    int tilt_angle_ = 90;   // 0-180 degrees, second servo (pan-tilt TILT axis, GPIO48)
    // ---- 人脸跟踪（2026-10-14 舵机云台 v1）----
    // 摄像头装在 SG90 摆臂上：跟踪任务以 ~3fps 抓拍上传 /vision
    // (X-Photo-Type: track)，桥接跑 YuNet 算人脸偏移，在 HTTP 响应里
    // 回 {"servo": 角度}，本端转动——一帧一闭环。开关经 /pending 领单。
    std::atomic<bool> tracking_{false};
    int track_fail_ = 0;  // 连续上传失败计数，>8 次自动停止
#else
    int servo_speed_ = 0;  // -100..100 percent, 0 = stopped
#endif

    void InitializeUsb() {
        // Audio-first bring-up: install the USB host library ourselves and let
        // the UAC driver claim the mic/speaker interfaces. The UVC camera is
        // left for a later pass — its probe currently fails ("no supported
        // pixel format") and its error path interferes with UAC detection.
        // Diagnostic: sample the raw levels of the USB data pins BEFORE the
        // host stack claims them. A powered full-speed device pulls D+ high.
        gpio_config_t gpio_in_cfg = {
            .pin_bit_mask = (1ULL << 19) | (1ULL << 20),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&gpio_in_cfg);
        int dm = gpio_get_level((gpio_num_t)19);
        int dp = gpio_get_level((gpio_num_t)20);
        ESP_LOGI(TAG, "USB pin levels before host install: D-=%d D+=%d (%s)",
                 dm, dp, dp ? "device powered" : "D+ low: device may be unpowered/disconnected");
        usb_host_config_t host_config = {
            .skip_phy_setup = false,
            .intr_flags = ESP_INTR_FLAG_LEVEL1,
        };
        ESP_ERROR_CHECK(usb_host_install(&host_config));
        ESP_LOGI(TAG, "USB host library installed");
        // esp-usb 1.5.x (IDF 6.x) has NO internal background task: the library
        // (and therefore device enumeration) only runs while someone calls
        // usb_host_lib_handle_events(). Drive it from a dedicated task.
        xTaskCreate([](void*) {
            while (true) {
                uint32_t flags = 0;
                usb_host_lib_handle_events(portMAX_DELAY, &flags);
                if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                    usb_host_device_free_all();
                }
            }
        }, "usb_lib", 4096, nullptr, 2, nullptr);
        xTaskCreate(UsbProbeTask, "usb_probe", 4096, nullptr, 4, nullptr);
        // Install the UAC host class driver (claims mic + speaker interfaces)
        ESP_ERROR_CHECK(UacAudioCodec::InstallDriver());
        // Camera: UVC shares the USB host library installed above (do NOT let
        // esp_video install it again).
        esp_video_init_usb_uvc_config_t usb_uvc_config = {
            .uvc = {
                .uvc_dev_num = 1,
                .task_stack = 4096,
                .task_priority = 5,
                .task_affinity = -1,
            },
            .usb = {
                .init_usb_host_lib = false,
                .task_stack = 4096,
                .task_priority = 5,
                .task_affinity = -1,
            },
        };
        esp_video_init_config_t video_config = {
            .usb_uvc = &usb_uvc_config,
        };
        camera_ = new EspVideo(video_config);
        esp_video_ = static_cast<EspVideo*>(camera_);
        ESP_LOGI(TAG, "USB UVC camera initialized");
    }

    // Display pins are derived from the tutorial wiring (GPIO4-9) and not yet
    // verified on hardware — never abort boot on display errors.
    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        if (spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed, falling back to NoDisplay");
            display_ = new NoDisplay();
            return;
        }

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 8 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        if (esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io) != ESP_OK) {
            ESP_LOGE(TAG, "panel IO init failed");
            display_ = new NoDisplay();
            return;
        }

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        if (esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel) != ESP_OK ||
            esp_lcd_panel_reset(panel) != ESP_OK ||
            esp_lcd_panel_init(panel) != ESP_OK) {
            ESP_LOGE(TAG, "ST7789 init failed, falling back to NoDisplay");
            display_ = new NoDisplay();
            return;
        }

        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        // GMT020-02: 2.0" 240x320 ST7789V glass, used landscape 320x240 via
        // the MV bit (swap_xy) — the same standard path as the waveshare
        // 3.5" board. MADCTL = MV|MY was verified correct on hardware.
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_set_gap(panel, 0, 0);
        esp_lcd_panel_disp_on_off(panel, true);

        display_ = new ClawLcdDisplay(panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                     DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                     DISPLAY_SWAP_XY);
    }

    // ---- Claw peripherals exposed to the AI via MCP tools ----
    static int Clamp255(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
    int ScaledColor(int color) const { return Clamp255(color) * led_brightness_ / 255; }

    void InitializeLedStrip() {
        led_strip_config_t strip_config = {
            .strip_gpio_num = CLAW_LED_STRIP_GPIO,
            .max_leds = CLAW_LED_STRIP_NUM,
            .led_model = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .flags = { .invert_out = false },
        };
        led_strip_rmt_config_t rmt_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,
            .mem_block_symbols = 0,
            .flags = { .with_dma = false },
        };
        if (led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_) != ESP_OK) {
            ESP_LOGE(TAG, "LED strip init failed on GPIO%d", CLAW_LED_STRIP_GPIO);
            led_strip_ = nullptr;
            return;
        }
        led_strip_clear(led_strip_);
        ESP_LOGI(TAG, "LED ring ready: %d leds on GPIO%d", CLAW_LED_STRIP_NUM, CLAW_LED_STRIP_GPIO);
    }

#if CLAW_SERVO_POSITIONAL
    // Positional servo (e.g. SG90): pulse width = absolute position.
    // 500 us = 0 deg, 1500 us = 90 deg, 2500 us = 180 deg.
    // 14-bit duty at 50 Hz: 20000 us = 16384 counts, so 1 us = 0.8192 counts.
    static uint32_t ServoDuty(int angle) {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        return 410 + angle * 1638 / 180;  // 90 deg -> 1229 counts = 1500 us
    }

    void InitializeServo() {
        ledc_timer_config_t timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_14_BIT,
            .timer_num = LEDC_TIMER_1,  // TIMER_0/CHANNEL_0 are used by the LCD backlight
            .freq_hz = 50,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false,
        };
        if (ledc_timer_config(&timer) != ESP_OK ||
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ServoDuty(servo_angle_)) != ESP_OK ||
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1) != ESP_OK) {
            ESP_LOGE(TAG, "Servo LEDC init failed");
            return;
        }
        ledc_channel_config_t channel = {
            .gpio_num = CLAW_SERVO_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .timer_sel = LEDC_TIMER_1,
            .duty = ServoDuty(servo_angle_),
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = { .output_invert = 0 },
            .deconfigure = false,
        };
        if (ledc_channel_config(&channel) != ESP_OK) {
            ESP_LOGE(TAG, "Servo channel config failed on GPIO%d", CLAW_SERVO_GPIO);
            return;
        }
        // 第二路舵机（俯仰轴）：共用 TIMER_1，走 CHANNEL_2
        ledc_channel_config_t tilt_ch = {
            .gpio_num = CLAW_SERVO2_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_2,
            .timer_sel = LEDC_TIMER_1,
            .duty = ServoDuty(tilt_angle_),
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = { .output_invert = 0 },
            .deconfigure = false,
        };
        if (ledc_channel_config(&tilt_ch) != ESP_OK) {
            ESP_LOGE(TAG, "Tilt servo channel config failed on GPIO%d", CLAW_SERVO2_GPIO);
            return;
        }
        ESP_LOGI(TAG, "Positional servos ready: pan GPIO%d, tilt GPIO%d, holding 90 deg",
                 CLAW_SERVO_GPIO, CLAW_SERVO2_GPIO);
    }
#else
    // The attached servo is a 360-degree CONTINUOUS-ROTATION type (confirmed
    // on the bench: any pulse other than the neutral one spins the output
    // shaft non-stop). Pulse width therefore sets SPEED and DIRECTION, not
    // position: 1500 us = stop, deviation from center = speed magnitude,
    // sign = direction. 14-bit duty at 50 Hz: 1 ms = 819 counts.
    static uint32_t ServoDuty(int speed) {
        if (speed < -100) speed = -100;
        if (speed > 100) speed = 100;
        return 1229 + speed * 409 / 100;  // 1229 = 1500 us neutral (stop)
    }

    void InitializeServo() {
        ledc_timer_config_t timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_14_BIT,
            .timer_num = LEDC_TIMER_1,  // TIMER_0/CHANNEL_0 are used by the LCD backlight
            .freq_hz = 50,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false,
        };
        if (ledc_timer_config(&timer) != ESP_OK ||
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ServoDuty(servo_speed_)) != ESP_OK ||
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1) != ESP_OK) {
            ESP_LOGE(TAG, "Servo LEDC init failed");
            return;
        }
        ledc_channel_config_t channel = {
            .gpio_num = CLAW_SERVO_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .timer_sel = LEDC_TIMER_1,
            .duty = ServoDuty(servo_speed_),
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = { .output_invert = 0 },
            .deconfigure = false,
        };
        if (ledc_channel_config(&channel) != ESP_OK) {
            ESP_LOGE(TAG, "Servo channel config failed on GPIO%d", CLAW_SERVO_GPIO);
            return;
        }
        ESP_LOGI(TAG, "Continuous-rotation servo ready on GPIO%d (neutral = stopped)", CLAW_SERVO_GPIO);
    }
#endif

    // ---- Persistent reminders (NVS) that make the device speak when due ----
    // Daily reminders fire once per calendar day at HH:MM; interval reminders
    // re-fire every N minutes; one-shot reminders fire once at an absolute
    // time and are then removed. On fire the device opens an audio session and
    // sends the message text as if spoken, so the cloud AI answers by voice.
    static constexpr int kMaxReminders = 4;
    struct ClawReminder {
        uint8_t type;            // 0 = daily HH:MM, 1 = every interval_min, 2 = one-shot at next_due
        uint8_t hour;            // daily: 0-23
        uint8_t minute;          // daily: 0-59
        uint16_t interval_min;   // interval type: period in minutes
        char message[64];        // what the reminder is about (UTF-8)
        uint16_t last_fired_yday;// daily dedup: day-of-year already fired
        time_t next_due;         // interval: earliest fire time; one-shot: absolute fire time
    };
    std::vector<ClawReminder> reminders_;
    SemaphoreHandle_t reminders_mutex_ = nullptr;
    std::atomic<bool> test_fire_{false};  // set by the test tool: fire reminders_[0] once idle
    static constexpr uint16_t kYdayNever = 0xFFFF;

    void LoadReminders() {
        nvs_handle_t h;
        if (nvs_open("claw_rem", NVS_READONLY, &h) != ESP_OK) return;
        size_t len = 0;
        if (nvs_get_blob(h, "entries", nullptr, &len) == ESP_OK && len >= sizeof(ClawReminder)) {
            int count = len / sizeof(ClawReminder);
            if (count > kMaxReminders) count = kMaxReminders;
            auto* buf = new ClawReminder[count];
            if (nvs_get_blob(h, "entries", buf, &len) == ESP_OK) {
                reminders_.assign(buf, buf + count);
            }
            delete[] buf;
        }
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded %d reminder(s) from NVS", (int)reminders_.size());
    }

    void SaveReminders() {
        nvs_handle_t h;
        if (nvs_open("claw_rem", NVS_READWRITE, &h) != ESP_OK) return;
        if (reminders_.empty()) {
            nvs_erase_key(h, "entries");  // missing key is fine on first run
        } else {
            nvs_set_blob(h, "entries", reminders_.data(),
                         reminders_.size() * sizeof(ClawReminder));
        }
        nvs_commit(h);
        nvs_close(h);
    }

    // UTF-8 aware clip: never cut in the middle of a multi-byte character.
    static std::string Utf8Clip(const std::string& s, size_t max_bytes) {
        if (s.size() <= max_bytes) return s;
        size_t cut = max_bytes;
        while (cut > 0 && (static_cast<unsigned char>(s[cut]) & 0xC0) == 0x80) --cut;
        return s.substr(0, cut);
    }

    // The cloud only accepts SHORT text in the wake-word "detect" message
    // (long text is rejected with "Detector is only for wake words. Don't
    // send long ..."), and a short non-wake-word text is forwarded to the
    // LLM as a one-shot question, so the AI answers the reminder by voice.
    void FireReminder(const ClawReminder& r) {
        std::string msg = Utf8Clip(std::string(r.message), 27);  // <= 9 CJK chars
        std::string prompt = msg.empty() ? "喝水时间到了" : "提醒：" + msg;
        if (prompt.size() > 36) prompt = Utf8Clip(prompt, 36);   // <= 12 CJK chars
        ESP_LOGI(TAG, "Reminder due: %s (prompt: %s)", r.message, prompt.c_str());
        // Full message on screen in case the voice flow is interrupted.
        GetDisplay()->ShowNotification(("提醒：" + std::string(r.message)).c_str(), 5000);
        Application::GetInstance().WakeWordInvoke(prompt);
    }

    static void ReminderTask(void* arg) {
        auto* self = static_cast<EspClawBoard*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(2 * 1000));
            time_t now = time(nullptr);
            struct tm lt;
            localtime_r(&now, &lt);
            if (lt.tm_year + 1900 < 2026) {
                continue;  // SNTP not synced yet; never fire on bogus time
            }
            bool dirty = false;
            xSemaphoreTake(self->reminders_mutex_, portMAX_DELAY);
            // Test tool request: fire reminders_[0] once, with the same
            // bookkeeping as a real fire so it does not re-fire right after.
            // Consume the flag only when we actually fire (&& short-circuits),
            // so a request made mid-conversation survives until the device
            // goes idle.
            if (self->GetDeviceStateSafe() == kDeviceStateIdle &&
                !self->reminders_.empty() &&
                self->test_fire_.exchange(false)) {
                auto& tr = self->reminders_.front();
                if (tr.type == 0) tr.last_fired_yday = lt.tm_yday;
                else if (tr.type == 1) tr.next_due = now + (time_t)tr.interval_min * 60;
                // type 2 (one-shot): leave next_due untouched — the scheduler
                // fires it when due (immediately, if already past due).
                dirty = true;
                self->FireReminder(tr);
            }
            std::vector<int> to_erase;  // fired one-shots, erased after the loop
            for (int i = 0; i < (int)self->reminders_.size(); ++i) {
                auto& r = self->reminders_[i];
                if (r.type == 0) {
                    // Daily at HH:MM, once per calendar day
                    if (lt.tm_hour == r.hour && lt.tm_min == r.minute &&
                        r.last_fired_yday != lt.tm_yday &&
                        self->GetDeviceStateSafe() == kDeviceStateIdle) {
                        r.last_fired_yday = lt.tm_yday;
                        self->FireReminder(r);
                    }
                } else if (r.type == 1) {
                    // Every N minutes; skip while busy, catch up once free
                    if (now >= r.next_due &&
                        self->GetDeviceStateSafe() == kDeviceStateIdle) {
                        r.next_due = now + (time_t)r.interval_min * 60;
                        dirty = true;  // persist the advanced next_due
                        self->FireReminder(r);
                    }
                } else if (r.type == 2) {
                    // One-shot at absolute time; removed after firing.
                    // If the moment passes while busy, fire as soon as idle.
                    if (now >= r.next_due &&
                        self->GetDeviceStateSafe() == kDeviceStateIdle) {
                        to_erase.push_back(i);
                        dirty = true;
                        self->FireReminder(r);
                    }
                }
            }
            for (int k = (int)to_erase.size() - 1; k >= 0; --k) {
                ESP_LOGI(TAG, "One-shot reminder fired and removed: %s",
                         self->reminders_[to_erase[k]].message);
                self->reminders_.erase(self->reminders_.begin() + to_erase[k]);
            }
            xSemaphoreGive(self->reminders_mutex_);
            if (dirty) self->SaveReminders();
        }
    }

    // WakeWordInvoke requires the device to be idle; check from this task
    DeviceState GetDeviceStateSafe() {
        return Application::GetInstance().GetDeviceState();
    }

    void InitializeReminders() {
        reminders_mutex_ = xSemaphoreCreateMutex();
        LoadReminders();
        xTaskCreate(ReminderTask, "claw_remind", 4096, this, 1, nullptr);
        ESP_LOGI(TAG, "Reminder engine started");
    }

    // ---- Auto look (NVS persisted): every N minutes (while idle) the device
    // opens a conversation with a SHORT seeing-oriented prompt; the cloud LLM
    // then calls its own take_photo tool and SPEAKS an analysis of what the
    // camera sees. Same wake-text mechanism as reminders (long detect text is
    // rejected by the cloud), no photos are stored anywhere. ----
    std::atomic<bool> auto_look_en_{false};
    std::atomic<uint16_t> auto_look_interval_{0};  // minutes, 0 = off
    std::atomic<bool> auto_look_now_{false};       // one-shot test request
    char auto_look_msg_[64] = "看看周围有什么";
    time_t look_next_due_ = 0;

    void LoadAutoLook() {
        nvs_handle_t h;
        if (nvs_open("claw_look", NVS_READONLY, &h) != ESP_OK) return;
        size_t len = sizeof(auto_look_msg_);
        nvs_get_str(h, "msg", auto_look_msg_, &len);
        uint16_t iv = 0;
        uint8_t en = 0;
        if (nvs_get_u16(h, "interval", &iv) == ESP_OK) auto_look_interval_.store(iv);
        if (nvs_get_u8(h, "en", &en) == ESP_OK) auto_look_en_.store(en != 0);
        nvs_close(h);
        ESP_LOGI(TAG, "Auto look: %s, every %u min, msg=%s",
                 auto_look_en_.load() ? "enabled" : "disabled",
                 auto_look_interval_.load(), auto_look_msg_);
    }

    void SaveAutoLook() {
        nvs_handle_t h;
        if (nvs_open("claw_look", NVS_READWRITE, &h) != ESP_OK) return;
        nvs_set_str(h, "msg", auto_look_msg_);
        nvs_set_u16(h, "interval", auto_look_interval_.load());
        nvs_set_u8(h, "en", auto_look_en_.load() ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }

    void FireAutoLook() {
        // Cloud rejects long wake-detect text; keep the prompt <= 9 CJK chars.
        std::string msg = Utf8Clip(std::string(auto_look_msg_), 27);
        if (msg.empty()) msg = "看看周围有什么";
        ESP_LOGI(TAG, "Auto look due: %s", msg.c_str());
        GetDisplay()->ShowNotification(("看看：" + msg).c_str(), 5000);
        Application::GetInstance().WakeWordInvoke(msg);
    }

    static void AutoLookTask(void* arg) {
        auto* self = static_cast<EspClawBoard*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(2 * 1000));
            if (self->GetDeviceStateSafe() != kDeviceStateIdle) continue;
            bool one_shot = self->auto_look_now_.exchange(false);
            bool scheduled = false;
            if (self->auto_look_en_.load() && self->auto_look_interval_.load() > 0 &&
                time(nullptr) >= self->look_next_due_) {
                scheduled = true;
            }
            if (!one_shot && !scheduled) continue;
            self->FireAutoLook();
            int mins = self->auto_look_interval_.load();
            if (mins < 1) mins = 1;
            self->look_next_due_ = time(nullptr) + (time_t)mins * 60;
        }
    }

    void InitializeAutoLook() {
        LoadAutoLook();
        xTaskCreate(AutoLookTask, "claw_look", 4096, this, 1, nullptr);
        ESP_LOGI(TAG, "Auto look engine started");
    }

    // ---- Desk sentinel (桌面前哨): on-device frame-difference watch. ----
    // Every N seconds (while the device is idle, so the camera is free) grab
    // one JPEG frame, decode it and downsample to a 160x90 grayscale map.
    // Two signals are derived locally (frames never leave the device):
    //   1. GLOBAL motion grid (16x9 cells)  -> person presence / left-seat
    //   2. CUP ROI (configurable permille rect) vs a quiet-scene baseline
    //      -> cup pickup/return/absence events
    // Policy (user-defined):
    //   - cup picked up and put back at a NEW spot while the user is present:
    //     counted as a drink event SILENTLY (no voice, no notification)
    //   - cup absent for >= sent_cup_absent_ minutes: ONE voice reminder
    //     "水杯去哪了" per absence episode
    //   - left-seat detection: logged for now; cloud-confirm wiring is phase 3
    static constexpr int kSentGW = 160;          // grayscale map width
    static constexpr int kSentGH = 90;           // grayscale map height
    // Calibration 2026-10-08 evening: gray~107, sensor noise floor is sparse
    // hot pixels (davg~3, px10~1400, px20~300, px40~40 over 14400 px, spread
    // over ~50 of 144 cells with only 3-6 px each). Real motion produces
    // whole cells at d>=40. Cup spans ~12 ROI cells (1 gray px ~= 0.75 cm).
    static constexpr int kSentPxTh = 20;         // gray delta that counts as a changed pixel
    static constexpr int kSentCellActPx = 15;    // changed px in a 10x10 cell => motion cell
    static constexpr int kSentQuietCells = 2;    // motion_cells <= this => scene quiet
    static constexpr int kSentHotTh = 4;         // consecutive active cycles => hot cell (masked)
    static constexpr int kSentRoiCell = 4;       // ROI cell size (gray px, finer grid)
    static constexpr int kSentRoiActCells = 3;   // changed ROI cells => activity
    static constexpr int kSentRoiGoneCells = 6;  // changed ROI cells => cup likely gone
    static constexpr int kSentMeanShift = 3;     // ROI mean shift => cup left the ROI
    static constexpr int kSentSettleS = 25;      // quiet seconds that end an event
    static constexpr int kSentEventMaxS = 90;    // force-settle watchdog for a stuck event
    static constexpr int kSentBackStreak = 2;    // consecutive quiet cycles to confirm a state
    static constexpr uint8_t kSentRoiVer = 2;    // bump to migrate stored ROI to new defaults

    std::atomic<bool> sent_en_{false};
    std::atomic<uint16_t> sent_interval_{15};    // seconds between samples
    std::atomic<uint16_t> sent_away_{120};       // seconds without motion => away
    std::atomic<uint16_t> sent_cup_absent_{30};  // minutes of absence => one reminder
    std::atomic<bool> sent_probe_{false};        // one-shot cycle for calibration
    uint16_t sent_roi_[4] = {150, 450, 750, 450}; // permille x,y,w,h (full desk band, v2)
    uint32_t sent_drink_count_ = 0;              // persisted, silent counter

    // Runtime state, touched only by the sentinel task
    uint8_t* sent_gray_ = nullptr;       // kSentGW*kSentGH current map
    uint8_t* sent_prev_ = nullptr;       // previous map for global motion
    uint8_t* sent_roi_base_ = nullptr;   // ROI baseline (last stable scene)
    uint8_t* sent_roi_empty_ = nullptr;  // ROI scene without the cup (absent ref)
    bool sent_has_prev_ = false;
    bool sent_has_base_ = false;
    uint8_t sent_hot_[16 * 9] = {0};     // per-cell consecutive-activity counter (hot masking)

    // PC bridge client (esp32-robot-brain), touched by the bridge task only
    std::string bridge_url_;
    std::string bridge_token_;
    std::atomic<int> bridge_play_result_{0};  // 0=waiting 1=started 2=busy/failed
    // 语音任务派单成功后收线计数（秒）：设备此刻多半还在聆听会话里，而
    // PlayNotifyAudio 只在 idle 才能播——不收线的话桥接回执要等会话超时
    // 退出才突然冒出来（2026-10-11 16:57 实测"卡很久→退出→才播回执"）。
    // BridgeTask 每 1s 递减：见 Listening 立即 StopListening 收线回 idle；
    // 已是 idle 或倒数到 0（异常兜底）就放弃。
    std::atomic<int> bridge_close_session_{0};
    // 看门狗进度戳（ms since boot）：BridgeTask 每次循环/每个 HTTP 步骤后更新；
    // 守护任务发现 >90s 未更新即判定挂死并重启自愈。TCP 组件（78__esp-ml307，
    // hash 保护不可改）的 Send()/Connect() 无超时，对端停收时 send() 会永久
    // 阻塞——2026-10-10 11:49 两次挂死均属此类，只能靠看门狗兜底恢复。
    std::atomic<int64_t> bridge_progress_ms_{0};
    int sent_roi_mean_base_ = 0;         // mean gray of ROI at baseline
    int sent_event_run_s_ = 0;           // seconds the current event has run
    bool sent_absent_ = false;
    bool sent_absent_reminded_ = false;
    bool sent_event_ = false;            // ROI event in progress
    int sent_event_quiet_s_ = 0;
    int sent_gone_streak_ = 0;           // consecutive quiet cycles with huge ROI diff
    int sent_baseline_quiet_s_ = 0;
    int sent_still_s_ = 0;               // consecutive seconds without global motion
    bool sent_away_latched_ = false;
    int sent_absent_s_ = 0;
    time_t sent_last_motion_ = 0;

    void LoadSentinel() {
        nvs_handle_t h;
        if (nvs_open("claw_sentry", NVS_READONLY, &h) != ESP_OK) return;
        uint8_t en = 0;
        uint16_t iv = 0, away = 0, cupabs = 0;
        uint32_t drink = 0;
        uint16_t roi[4];
        size_t len = sizeof(roi);
        if (nvs_get_u8(h, "en", &en) == ESP_OK) sent_en_.store(en != 0);
        if (nvs_get_u16(h, "iv", &iv) == ESP_OK && iv >= 5 && iv <= 300)
            sent_interval_.store(iv);
        if (nvs_get_u16(h, "away", &away) == ESP_OK && away >= 30)
            sent_away_.store(away);
        if (nvs_get_u16(h, "cupabs", &cupabs) == ESP_OK && cupabs >= 1)
            sent_cup_absent_.store(cupabs);
        if (nvs_get_u32(h, "drink", &drink) == ESP_OK) sent_drink_count_ = drink;
        uint8_t roiv = 0;
        nvs_get_u8(h, "roiv", &roiv);
        bool roi_migrated = false;
        if (roiv >= kSentRoiVer &&
            nvs_get_blob(h, "roi", roi, &len) == ESP_OK && len == sizeof(roi)) {
            for (int i = 0; i < 4; ++i) {
                if (roi[i] <= 1000) sent_roi_[i] = roi[i];
            }
        } else if (roiv < kSentRoiVer) {
            roi_migrated = true;  // ROI from an older layout: keep new defaults
        }
        nvs_close(h);
        if (roi_migrated) SaveSentinel();  // persist new defaults + version
        ESP_LOGI(TAG, "Desk sentinel: %s, every %us, roi=%u,%u,%u,%u, drinks=%lu",
                 sent_en_.load() ? "enabled" : "disabled", sent_interval_.load(),
                 sent_roi_[0], sent_roi_[1], sent_roi_[2], sent_roi_[3],
                 (unsigned long)sent_drink_count_);
    }

    void SaveSentinel() {
        nvs_handle_t h;
        if (nvs_open("claw_sentry", NVS_READWRITE, &h) != ESP_OK) return;
        nvs_set_u8(h, "en", sent_en_.load() ? 1 : 0);
        nvs_set_u16(h, "iv", sent_interval_.load());
        nvs_set_u16(h, "away", sent_away_.load());
        nvs_set_u16(h, "cupabs", sent_cup_absent_.load());
        nvs_set_u32(h, "drink", sent_drink_count_);
        nvs_set_u8(h, "roiv", kSentRoiVer);
        nvs_set_blob(h, "roi", sent_roi_, sizeof(sent_roi_));
        nvs_commit(h);
        nvs_close(h);
    }

    void ResetSentinelScene() {
        sent_has_prev_ = false;
        sent_has_base_ = false;
        sent_event_ = false;
        sent_absent_ = false;
        sent_absent_reminded_ = false;
        sent_gone_streak_ = 0;
        sent_baseline_quiet_s_ = 0;
        sent_event_quiet_s_ = 0;
        sent_still_s_ = 0;
        sent_away_latched_ = false;
    }

    // Diff of the ROI rect between the baseline buffer and sent_gray_.
    // Returns changed-cell count and the bbox of changed cells (gray px).
    int SentinelRoiDiff(int rx, int ry, int rwp, int rhp, const uint8_t* base,
                        int* min_x, int* min_y, int* max_x, int* max_y) {
        int cells_x = rwp / kSentRoiCell;
        int cells_y = rhp / kSentRoiCell;
        int changed_cells = 0;
        if (min_x) *min_x = -1;
        if (min_y) *min_y = -1;
        if (max_x) *max_x = -1;
        if (max_y) *max_y = -1;
        int cell_act_th = (kSentRoiCell * kSentRoiCell) / 4;
        for (int cy = 0; cy < cells_y; ++cy) {
            for (int cx = 0; cx < cells_x; ++cx) {
                int act = 0;
                for (int y = ry + cy * kSentRoiCell;
                     y < ry + (cy + 1) * kSentRoiCell; ++y) {
                    for (int x = rx + cx * kSentRoiCell;
                         x < rx + (cx + 1) * kSentRoiCell; ++x) {
                        int cur = y * kSentGW + x;
                        int ref = (y - ry) * rwp + (x - rx);  // packed ROI stride
                        if (abs((int)sent_gray_[cur] - (int)base[ref]) >= kSentPxTh) ++act;
                    }
                }
                if (act >= cell_act_th) {
                    ++changed_cells;
                    if (min_x) {
                        int x0 = rx + cx * kSentRoiCell, y0 = ry + cy * kSentRoiCell;
                        if (*min_x < 0 || x0 < *min_x) *min_x = x0;
                        if (*min_y < 0 || y0 < *min_y) *min_y = y0;
                        if (x0 + kSentRoiCell > *max_x) *max_x = x0 + kSentRoiCell;
                        if (y0 + kSentRoiCell > *max_y) *max_y = y0 + kSentRoiCell;
                    }
                }
            }
        }
        return changed_cells;
    }

    void SentinelSnapshotRoi(int rx, int ry, int rwp, int rhp, uint8_t* dst) {
        for (int y = 0; y < rhp; ++y) {
            memcpy(dst + y * rwp, sent_gray_ + (ry + y) * kSentGW + rx, rwp);
        }
    }

    // Mean gray of the current ROI rect (used to tell "cup moved within ROI"
    // (mean ~unchanged) from "cup removed / left the ROI" (mean shifts)).
    int SentinelRoiMean(int rx, int ry, int rwp, int rhp) {
        long sum = 0;
        for (int y = ry; y < ry + rhp; ++y) {
            const uint8_t* row = sent_gray_ + y * kSentGW + rx;
            for (int x = 0; x < rwp; ++x) sum += row[x];
        }
        return (int)(sum / (rwp * rhp));
    }

    void SentinelCycle(bool verbose) {
        if (esp_video_ == nullptr) return;
        if (GetDeviceStateSafe() != kDeviceStateIdle) {
            // Camera may be claimed by an active conversation; scene reference
            // is stale, so re-baseline on the next quiet moment. Rate-limit
            // the log (once a minute) to avoid spamming long conversations.
            static int busy_skip = 0;
            if ((busy_skip++ % 4) == 0) {
                ESP_LOGI(TAG, "Sentinel: device busy, skip (state=%d)",
                         (int)GetDeviceStateSafe());
            }
            if (sent_has_prev_ || sent_has_base_) {
                ESP_LOGI(TAG, "Sentinel: scene state reset");
                ResetSentinelScene();
            }
            return;
        }
        int iv = sent_interval_.load();
        if (iv < 5) iv = 5;

        TickType_t t0 = xTaskGetTickCount();
        esp_video_->SetPreviewEnabled(false);
        bool ok = esp_video_->Capture();
        std::vector<uint8_t> jpeg;
        uint16_t jw = 0, jh = 0;
        if (ok) ok = esp_video_->GetLastJpegFrame(jpeg, &jw, &jh);
        esp_video_->SetPreviewEnabled(true);
        if (!ok) {
            ESP_LOGW(TAG, "Sentinel: capture failed");
            return;
        }

        uint8_t* rgb = nullptr;
        size_t rlen = 0, rw = 0, rh = 0, rstride = 0;
        if (jpeg_to_image(jpeg.data(), jpeg.size(), &rgb, &rlen, &rw, &rh, &rstride) != ESP_OK ||
            rgb == nullptr) {
            ESP_LOGW(TAG, "Sentinel: jpeg decode failed");
            return;
        }
        for (int y = 0; y < kSentGH; ++y) {
            int sy = (int)((int64_t)y * (int)rh / kSentGH);
            const uint8_t* row = rgb + (size_t)sy * rstride;
            for (int x = 0; x < kSentGW; ++x) {
                int sx = (int)((int64_t)x * (int)rw / kSentGW);
                uint16_t v = row[sx * 2] | (row[sx * 2 + 1] << 8);
                int r8 = (int)((v >> 11) & 0x1F) << 3;
                int g8 = (int)((v >> 5) & 0x3F) << 2;
                int b8 = (int)(v & 0x1F) << 3;
                sent_gray_[y * kSentGW + x] =
                    (uint8_t)((r8 * 77 + g8 * 151 + b8 * 28) >> 8);
            }
        }
        heap_caps_free(rgb);
        uint32_t cost_ms = (xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;

        // --- ROI rect in gray coordinates ---
        int rx = (int)((int64_t)sent_roi_[0] * kSentGW / 1000);
        int ry = (int)((int64_t)sent_roi_[1] * kSentGH / 1000);
        int rwp = (int)((int64_t)sent_roi_[2] * kSentGW / 1000);
        int rhp = (int)((int64_t)sent_roi_[3] * kSentGH / 1000);
        if (rx < 0) rx = 0;
        if (ry < 0) ry = 0;
        if (rx + rwp > kSentGW) rwp = kSentGW - rx;
        if (ry + rhp > kSentGH) rhp = kSentGH - ry;
        if (rwp < 8 || rhp < 8) {
            ESP_LOGW(TAG, "Sentinel: ROI too small, skipping cycle");
            return;
        }

        // --- Global motion (with hot-cell self-masking) ---
        // Cells active in kSentHotTh+ consecutive cycles are persistent noise
        // sources (sensor hot pixels / PWM flicker): masked out until they go
        // quiet again. Intermittent real motion never accumulates the streak.
        int motion_cells = 0;
        long sum_diff = 0;
        int px10 = 0, px20 = 0, px40 = 0;
        long sum_gray = 0;
        int hot_cells = 0;
        char act_list[48] = {0};  // counted cell coords for calibration logs
        size_t act_len = 0;
        if (sent_has_prev_) {
            for (int cy = 0; cy < 9; ++cy) {
                for (int cx = 0; cx < 16; ++cx) {
                    int act = 0;
                    for (int y = cy * 10; y < cy * 10 + 10; ++y) {
                        for (int x = cx * 10; x < cx * 10 + 10; ++x) {
                            int idx = y * kSentGW + x;
                            int d = abs((int)sent_gray_[idx] - (int)sent_prev_[idx]);
                            sum_diff += d;
                            if (d >= 10) ++px10;
                            if (d >= 20) ++px20;
                            if (d >= 40) ++px40;
                            if (d >= kSentPxTh) ++act;
                        }
                    }
                    uint8_t& h = sent_hot_[cy * 16 + cx];
                    if (act >= kSentCellActPx) {
                        if (h < 250) ++h;
                        if (h >= kSentHotTh) {
                            ++hot_cells;  // masked, not counted as motion
                        } else {
                            ++motion_cells;
                            if (act_len + 8 < sizeof(act_list)) {
                                act_len += snprintf(act_list + act_len,
                                                    sizeof(act_list) - act_len,
                                                    "%d,%d;", cx, cy);
                            }
                        }
                    } else if (h > 0) {
                        --h;
                    }
                }
            }
        }
        // ROI activity vs the PREVIOUS frame (computed before sent_prev_ is
        // overwritten below). Used for event quiet detection: a settled scene
        // that permanently differs from the baseline must not latch events.
        int roi_activity = 0;
        if (sent_has_prev_) {
            roi_activity = SentinelRoiDiff(rx, ry, rwp, rhp, sent_prev_,
                                           nullptr, nullptr, nullptr, nullptr);
        }
        for (int i = 0; i < kSentGW * kSentGH; ++i) sum_gray += sent_gray_[i];
        memcpy(sent_prev_, sent_gray_, kSentGW * kSentGH);
        sent_has_prev_ = true;

        if (motion_cells > kSentQuietCells) {
            int away_total = sent_still_s_;
            sent_still_s_ = 0;
            sent_last_motion_ = time(nullptr);
            if (sent_away_latched_) {
                sent_away_latched_ = false;
                ESP_LOGI(TAG, "Sentinel EVENT: PERSON BACK after %ds away", away_total);
                BridgeNotify("speak", "主人回到座位啦，刚才离开了大约 " +
                             std::to_string((away_total + 59) / 60) + " 分钟。");
            }
            sent_gone_streak_ = 0;
        } else {
            sent_still_s_ += iv;
            if (!sent_away_latched_ && sent_still_s_ >= (int)sent_away_.load()) {
                sent_away_latched_ = true;
                ESP_LOGI(TAG, "Sentinel EVENT: AWAY (no motion for %ds)", sent_still_s_);
                BridgeNotify("alert", "主人离开了座位（已 " +
                             std::to_string(sent_still_s_) + " 秒没有活动）");
                // Phase 3: WakeWordInvoke cloud confirm goes here once
                // thresholds are calibrated.
            }
        }
        bool person_present = (time(nullptr) - sent_last_motion_) < 90;

        // --- Cup ROI state machine ---
        if (!sent_has_base_) {
            if (motion_cells <= 2) {  // near-quiet; a couple of noise cells are ok
                sent_baseline_quiet_s_ += iv;
                if (sent_baseline_quiet_s_ >= kSentSettleS) {
                    SentinelSnapshotRoi(rx, ry, rwp, rhp, sent_roi_base_);
                    sent_roi_mean_base_ = SentinelRoiMean(rx, ry, rwp, rhp);
                    sent_has_base_ = true;
                    sent_gone_streak_ = 0;
                    sent_event_ = false;
                    ESP_LOGI(TAG, "Sentinel: cup baseline captured (%dx%d px)", rwp, rhp);
                }
            } else {
                sent_baseline_quiet_s_ = 0;
            }
        } else if (sent_absent_) {
            // Cup declared gone; watch for its return against the empty ref
            int bminx, bminy, bmaxx, bmaxy;
            int changed = SentinelRoiDiff(rx, ry, rwp, rhp, sent_roi_empty_,
                                          &bminx, &bminy, &bmaxx, &bmaxy);
            if (motion_cells == 0 && changed >= kSentRoiActCells) {
                ++sent_gone_streak_;
            } else if (motion_cells > kSentQuietCells) {
                sent_gone_streak_ = 0;
            }
            if (sent_gone_streak_ >= kSentBackStreak) {
                sent_gone_streak_ = 0;
                sent_absent_ = false;
                sent_absent_reminded_ = false;
                sent_absent_s_ = 0;
                SentinelSnapshotRoi(rx, ry, rwp, rhp, sent_roi_base_);
                ESP_LOGI(TAG, "Sentinel EVENT: CUP BACK (absent %dmin)",
                         sent_absent_s_ / 60);
            } else {
                sent_absent_s_ += iv;
                int lim = (int)sent_cup_absent_.load() * 60;
                if (!sent_absent_reminded_ && sent_absent_s_ >= lim &&
                    GetDeviceStateSafe() == kDeviceStateIdle) {
                    sent_absent_reminded_ = true;
                    ESP_LOGI(TAG, "Sentinel EVENT: cup absent %dmin -> reminder",
                             sent_absent_s_ / 60);
                    GetDisplay()->ShowNotification("水杯去哪了？", 5000);
                    Application::GetInstance().WakeWordInvoke("水杯去哪了");
                }
            }
        } else if (sent_event_) {
            // Event in progress; wait for the ROI to go quiet. Quietness is
            // measured against the PREVIOUS frame (is anything still moving?),
            // never against the baseline, else a scene that settled into a
            // different look would latch the event forever.
            int bminx, bminy, bmaxx, bmaxy;
            int changed = SentinelRoiDiff(rx, ry, rwp, rhp, sent_roi_base_,
                                          &bminx, &bminy, &bmaxx, &bmaxy);
            if (roi_activity >= kSentRoiActCells) {
                sent_event_quiet_s_ = 0;
            } else {
                sent_event_quiet_s_ += iv;
            }
            sent_event_run_s_ += iv;
            bool force_settle = sent_event_run_s_ >= kSentEventMaxS;
            if (sent_event_quiet_s_ >= kSentSettleS || force_settle) {
                sent_event_ = false;
                sent_event_run_s_ = 0;
                int cells_total = (rwp / kSentRoiCell) * (rhp / kSentRoiCell);
                int mean_now = SentinelRoiMean(rx, ry, rwp, rhp);
                int dmean = mean_now - sent_roi_mean_base_;
                ESP_LOGI(TAG,
                         "Sentinel EVENT: CUP_SETTLED%s cells=%d/%d bbox=[%d,%d..%d,%d] dmean=%d present=%d",
                         force_settle ? "(timeout)" : "",
                         changed, cells_total, bminx, bminy, bmaxx, bmaxy,
                         dmean, (int)person_present);
                if (changed >= kSentRoiGoneCells && abs(dmean) >= kSentMeanShift) {
                    // Cup left the ROI (mean shifted toward the desk-only look)
                    sent_absent_ = true;
                    sent_absent_s_ = 0;
                    sent_absent_reminded_ = false;
                    SentinelSnapshotRoi(rx, ry, rwp, rhp, sent_roi_empty_);
                    ESP_LOGI(TAG, "Sentinel: cup ABSENT (empty reference captured)");
                } else if (changed > 0 && person_present) {
                    // Picked up and put back (same or new spot) while present:
                    // silent drink event (user policy: count, never speak)
                    ++sent_drink_count_;
                    SaveSentinel();
                    ESP_LOGI(TAG, "Sentinel: drink event #%lu (silent)",
                             (unsigned long)sent_drink_count_);
                } else if (changed > 0 && !person_present) {
                    ESP_LOGW(TAG, "Sentinel: ROI changed while user away (tamper candidate)");
                    BridgeNotify("alert", "离座期间桌面前哨发现监测区有变化，已拍照存档。");
                    BridgeUploadPhoto("seat");  // archive evidence to the dashboard
                } else {
                    ESP_LOGI(TAG, "Sentinel: scene returned to baseline (no event)");
                }
                SentinelSnapshotRoi(rx, ry, rwp, rhp, sent_roi_base_);
                sent_roi_mean_base_ = SentinelRoiMean(rx, ry, rwp, rhp);
            }
        } else {
            // Quiet: classify deviations from the baseline
            int bminx, bminy, bmaxx, bmaxy;
            int changed = SentinelRoiDiff(rx, ry, rwp, rhp, sent_roi_base_,
                                          &bminx, &bminy, &bmaxx, &bmaxy);
            if (motion_cells <= kSentQuietCells && changed >= kSentRoiGoneCells) {
                ++sent_gone_streak_;
            } else {
                if (changed < kSentRoiGoneCells) sent_gone_streak_ = 0;
            }
            if (sent_gone_streak_ >= kSentBackStreak) {
                // Persistent huge diff while globally quiet: cup removed from
                // the ROI (mean shift) or relocated far inside it (mean same).
                sent_gone_streak_ = 0;
                int mean_now = SentinelRoiMean(rx, ry, rwp, rhp);
                int dmean = mean_now - sent_roi_mean_base_;
                if (abs(dmean) >= kSentMeanShift) {
                    sent_absent_ = true;
                    sent_absent_s_ = 0;
                    sent_absent_reminded_ = false;
                    SentinelSnapshotRoi(rx, ry, rwp, rhp, sent_roi_empty_);
                    SentinelSnapshotRoi(rx, ry, rwp, rhp, sent_roi_base_);
                    sent_roi_mean_base_ = mean_now;
                    ESP_LOGI(TAG, "Sentinel EVENT: CUP ABSENT (removed while quiet, dmean=%d)",
                             dmean);
                } else {
                    // Cup relocated inside the ROI while quiet: big move
                    if (person_present) {
                        ++sent_drink_count_;
                        SaveSentinel();
                        ESP_LOGI(TAG, "Sentinel: drink event #%lu (silent, far move)",
                                 (unsigned long)sent_drink_count_);
                    } else {
                        ESP_LOGW(TAG, "Sentinel: cup moved while user away (tamper candidate)");
                    }
                    SentinelSnapshotRoi(rx, ry, rwp, rhp, sent_roi_base_);
                    sent_roi_mean_base_ = mean_now;
                }
            } else if (changed >= kSentRoiActCells) {
                sent_event_ = true;
                sent_event_quiet_s_ = 0;
                sent_event_run_s_ = 0;
                ESP_LOGI(TAG, "Sentinel: ROI event started (cells=%d)", changed);
            }
        }

        // Per-cycle heartbeat (also serves threshold calibration)
        int hb_roi = -1;
        if (sent_has_base_ && !sent_absent_) {
            hb_roi = SentinelRoiDiff(rx, ry, rwp, rhp, sent_roi_base_,
                                     nullptr, nullptr, nullptr, nullptr);
        }
        ESP_LOGI(TAG,
                 "Sentinel cyc: cost=%lums motion=%d hot=%d act=[%s] davg=%ld px10=%d px20=%d px40=%d "
                 "gray=%ld roim=%d roi=%d still=%ds base=%d evt=%d absent=%d(%ds) drink=%lu",
                 (unsigned long)cost_ms, motion_cells, hot_cells, act_list,
                 sent_has_prev_ ? sum_diff / (kSentGW * kSentGH) : -1,
                 px10, px20, px40, sum_gray / (kSentGW * kSentGH),
                 sent_has_base_ ? SentinelRoiMean(rx, ry, rwp, rhp) : -1,
                 hb_roi, sent_still_s_,
                 (int)sent_has_base_, (int)sent_event_,
                 (int)sent_absent_, sent_absent_ ? sent_absent_s_ : 0,
                 (unsigned long)sent_drink_count_);
    }

    static void SentinelTask(void* arg) {
        auto* self = static_cast<EspClawBoard*>(arg);
        int disabled_skip = 0;
        while (true) {
            int iv = self->sent_interval_.load();
            if (iv < 5) iv = 5;
            vTaskDelay(pdMS_TO_TICKS(iv * 1000));
            bool probe = self->sent_probe_.exchange(false);
            if (!self->sent_en_.load() && !probe) {
                if ((disabled_skip++ % 4) == 0) {
                    ESP_LOGI(TAG, "Sentinel: disabled (enable via desk_watch)");
                }
                continue;
            }
            self->SentinelCycle(probe);
        }
    }

    void InitializeSentinel() {
        LoadSentinel();
        sent_gray_ = (uint8_t*)heap_caps_malloc(kSentGW * kSentGH,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        sent_prev_ = (uint8_t*)heap_caps_malloc(kSentGW * kSentGH,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        sent_roi_base_ = (uint8_t*)heap_caps_malloc(kSentGW * kSentGH,
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        sent_roi_empty_ = (uint8_t*)heap_caps_malloc(kSentGW * kSentGH,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!sent_gray_ || !sent_prev_ || !sent_roi_base_ || !sent_roi_empty_) {
            ESP_LOGE(TAG, "Sentinel: buffer alloc failed, engine disabled");
            return;
        }
        xTaskCreate(SentinelTask, "claw_sentry", 8192, this, 1, nullptr);
        ESP_LOGI(TAG, "Desk sentinel engine started");
    }

    std::string SentinelStatusString() {
        char buf[224];
        snprintf(buf, sizeof(buf),
                 "{\"enabled\":%s,\"interval_s\":%u,\"drink_count\":%lu,"
                 "\"still_s\":%d,\"away\":%s,\"cup_absent\":%s,\"absent_s\":%d,"
                 "\"roi\":\"%u,%u,%u,%u\"}",
                 sent_en_.load() ? "true" : "false", sent_interval_.load(),
                 (unsigned long)sent_drink_count_, sent_still_s_,
                 sent_away_latched_ ? "true" : "false",
                 sent_absent_ ? "true" : "false",
                 sent_absent_ ? sent_absent_s_ : 0,
                 sent_roi_[0], sent_roi_[1], sent_roi_[2], sent_roi_[3]);
        return std::string(buf);
    }

    // ---------------- PC bridge client (esp32-robot-brain) ----------------
    // Reuses the open-source bridge as the robot's cloud-of-LAN "brain":
    //  - periodic seat photos -> POST /vision (OpenCV presence + posture)
    //  - rule-engine reminders -> GET /pending, played via local TTS URL
    //  - sentinel events -> POST /notify (dashboard + speech queue)

    static std::string BridgeUrlEncode(const std::string& s) {
        static const char* hex = "0123456789ABCDEF";
        std::string out;
        out.reserve(s.size() * 3);
        for (unsigned char c : s) {
            if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '-' || c == '_' ||
                c == '.' || c == '~') {
                out += (char)c;
            } else {
                out += '%';
                out += hex[c >> 4];
                out += hex[c & 0xF];
            }
        }
        return out;
    }

    bool BridgeEnabled() const { return !bridge_url_.empty(); }

    // The ml307 HttpClient has a lifecycle race: when the peer closes the
    // connection, the tcp receive task can be between "recv() returned" and
    // "disconnect callback fired" while our Close() early-returns (connected_
    // was already cleared by the passive path, skipping the wait-for-task).
    // Destroying the client in that window is a use-after-free (mutex lock
    // on freed memory, EXCVADDR 0x31). Give the async disconnect callback a
    // moment to settle before the unique_ptr is released.
    static void BridgeHttpSettle(std::unique_ptr<Http>& http) {
        http->Close();  // idempotent: early-returns when already closed
        vTaskDelay(pdMS_TO_TICKS(200));
        http.reset();   // destroy AFTER the callback has had its chance
    }

    // GET a bridge endpoint; returns the body on HTTP 200, else "".
    std::string BridgeHttpGet(const std::string& url) {
        std::string body;
        auto network = GetNetwork();
        if (network == nullptr) return body;
        auto http = network->CreateHttp(0);
        if (http == nullptr) return body;
        if (!bridge_token_.empty()) http->SetHeader("X-Device-Token", bridge_token_);
        http->SetTimeout(5000);
        if (auto opened = http->Open("GET", url); opened) {
            int status = 0;
            if (auto sc = http->GetStatusCode()) status = sc.value();
            if (status == 200) body = http->ReadAll();
        }
        BridgeHttpSettle(http);
        return body;
    }

    std::string BridgeHttpPostJson(const std::string& path, const std::string& body) {
        std::string resp;
        auto network = GetNetwork();
        if (network == nullptr) return resp;
        auto http = network->CreateHttp(0);
        if (http == nullptr) return resp;
        if (!bridge_token_.empty()) http->SetHeader("X-Device-Token", bridge_token_);
        http->SetHeader("Content-Type", "application/json");
        http->SetTimeout(5000);
        http->SetContent(std::string(body));
        if (auto opened = http->Open("POST", bridge_url_ + path); opened) {
            int status = 0;
            if (auto sc = http->GetStatusCode()) status = sc.value();
            if (status >= 200 && status < 300) resp = http->ReadAll();
        }
        BridgeHttpSettle(http);
        return resp;
    }

    void BridgeAckAll() { BridgeHttpPostJson("/ack", "{}"); }

    // Capture one frame and upload it to /vision (multipart, chunked).
    bool BridgeUploadPhoto(const char* photo_type) {
        if (esp_video_ == nullptr) return false;
        if (GetDeviceStateSafe() != kDeviceStateIdle) return false;

        esp_video_->SetPreviewEnabled(false);
        bool ok = esp_video_->Capture();
        std::vector<uint8_t> jpeg;
        uint16_t jw = 0, jh = 0;
        if (ok) ok = esp_video_->GetLastJpegFrame(jpeg, &jw, &jh);
        esp_video_->SetPreviewEnabled(true);
        if (!ok || jpeg.empty()) {
            ESP_LOGW(TAG, "Bridge: capture failed");
            return false;
        }

        auto network = GetNetwork();
        if (network == nullptr) return false;
        auto http = network->CreateHttp(3);
        if (http == nullptr) return false;
        std::string boundary = "----ESP32_CLAW_BOUNDARY";
        if (!bridge_token_.empty()) http->SetHeader("X-Device-Token", bridge_token_);
        http->SetHeader("X-Photo-Type", photo_type);
        http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
        http->SetHeader("Transfer-Encoding", "chunked");
        http->SetTimeout(10000);
        if (auto opened = http->Open("POST", bridge_url_ + "/vision"); !opened) {
            ESP_LOGW(TAG, "Bridge: /vision connect failed");
            BridgeHttpSettle(http);
            return false;
        }
        std::string head;
        head += "--" + boundary + "\r\n";
        head += "Content-Disposition: form-data; name=\"question\"\r\n\r\n\r\n";
        head += "--" + boundary + "\r\n";
        head += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
        head += "Content-Type: image/jpeg\r\n\r\n";
        std::string tail = "\r\n--" + boundary + "--\r\n";
        bool okw = true;
        if (auto w = http->Write(head.c_str(), head.size()); !w) okw = false;
        if (okw && !jpeg.empty()) {
            if (auto w = http->Write((const char*)jpeg.data(), jpeg.size()); !w) okw = false;
        }
        if (okw) {
            if (auto w = http->Write(tail.c_str(), tail.size()); !w) okw = false;
        }
        if (okw) {
            // Terminating zero-size chunk ("0\r\n\r\n"); without it the
            // server never sees the end of the chunked body.
            if (auto w = http->Write("", 0); !w) okw = false;
        }
        int status = 0;
        if (auto sc = http->GetStatusCode()) status = sc.value();
        http->ReadAll();  // drain the short answer ("好的，我看一眼。")
        BridgeHttpSettle(http);
        ESP_LOGI(TAG, "Bridge: photo(%s) %uB -> status %d",
                 photo_type, (unsigned)jpeg.size(), status);
        return okw && status == 200;
    }

#if CLAW_SERVO_POSITIONAL
    // ---- 人脸跟踪（2026-10-14 舵机云台 v1）----
    void SetServoAngleDirect(int angle) {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        servo_angle_ = angle;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ServoDuty(servo_angle_));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }

    // 俯仰轴（第二路舵机，2026-10-16 二维云台）
    // 逻辑角约定（用户实测装配 2026-10-16）：0 = 最高点（抬头到顶），
    // 180 = 最低点（低头到底），90 = 水平。物理舵机反向，这里统一翻转，
    // 语音 / /servo / 跟踪三条路全部走本函数，约定一致。
    void SetTiltAngleDirect(int angle) {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        tilt_angle_ = angle;
        int physical = 180 - angle;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, ServoDuty(physical));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }

    // 一帧闭环：抓拍 -> 上传 /vision(track) -> 解析 {"servo": N} -> 转动。
    // 与 BridgeUploadPhoto 的区别：预览在整段跟踪期间保持关闭（3fps 逐帧
    // 开关会让屏幕狂闪），失败计数超限自动停止，忙时跳帧不抢音频。
    void TrackingStep() {
        if (esp_video_ == nullptr) { tracking_ = false; return; }
        if (GetDeviceStateSafe() != kDeviceStateIdle) return;

        esp_video_->SetPreviewEnabled(false);
        bool ok = esp_video_->Capture();
        std::vector<uint8_t> jpeg;
        uint16_t jw = 0, jh = 0;
        if (ok) ok = esp_video_->GetLastJpegFrame(jpeg, &jw, &jh);
        if (!ok || jpeg.empty()) return;  // 预览保持关闭，循环结束统一恢复

        auto network = GetNetwork();
        if (network == nullptr) return;
        auto http = network->CreateHttp(3);
        if (http == nullptr) return;
        std::string boundary = "----ESP32_CLAW_BOUNDARY";
        if (!bridge_token_.empty()) http->SetHeader("X-Device-Token", bridge_token_);
        http->SetHeader("X-Photo-Type", "track");
        http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
        http->SetHeader("Transfer-Encoding", "chunked");
        http->SetTimeout(3000);
        if (auto opened = http->Open("POST", bridge_url_ + "/vision"); !opened) {
            BridgeHttpSettle(http);
            if (++track_fail_ > 8) {
                ESP_LOGW(TAG, "Tracking: bridge unreachable, auto stop");
                tracking_ = false;
            }
            return;
        }
        std::string head;
        head += "--" + boundary + "\r\n";
        head += "Content-Disposition: form-data; name=\"question\"\r\n\r\n\r\n";
        head += "--" + boundary + "\r\n";
        head += "Content-Disposition: form-data; name=\"file\"; filename=\"track.jpg\"\r\n";
        head += "Content-Type: image/jpeg\r\n\r\n";
        std::string tail = "\r\n--" + boundary + "--\r\n";
        bool okw = true;
        if (auto w = http->Write(head.c_str(), head.size()); !w) okw = false;
        if (okw && !jpeg.empty()) {
            if (auto w = http->Write((const char*)jpeg.data(), jpeg.size()); !w) okw = false;
        }
        if (okw) {
            if (auto w = http->Write(tail.c_str(), tail.size()); !w) okw = false;
        }
        if (okw) {
            if (auto w = http->Write("", 0); !w) okw = false;  // 结束 chunk
        }
        int status = 0;
        if (auto sc = http->GetStatusCode()) status = sc.value();
        std::string body = (status == 200) ? http->ReadAll() : std::string();
        BridgeHttpSettle(http);
        if (status != 200) {
            if (++track_fail_ > 8) {
                ESP_LOGW(TAG, "Tracking: bad status %d, auto stop", status);
                tracking_ = false;
            }
            return;
        }
        track_fail_ = 0;
        cJSON* root = cJSON_Parse(body.c_str());
        if (root != nullptr) {
            int target_pan = servo_angle_;
            int target_tilt = tilt_angle_;
            const cJSON* js = cJSON_GetObjectItem(root, "servo");
            if (js && cJSON_IsNumber(js)) target_pan = js->valueint;
            const cJSON* jt = cJSON_GetObjectItem(root, "tilt");
            if (jt && cJSON_IsNumber(jt)) target_tilt = jt->valueint;
            cJSON_Delete(root);
            SmoothMoveTo(target_pan, target_tilt);
        }
    }

    // 平滑逼近目标角度（2026-10-16）：每 20ms 各轴最多走 2°，把一帧跳 6°
    // 的顿挫摊成连续滑动。总时长上限 ~260ms 防大角度差时卡住闭环。
    void SmoothMoveTo(int target_pan, int target_tilt) {
        for (int i = 0; i < 13; i++) {
            if (servo_angle_ == target_pan && tilt_angle_ == target_tilt) break;
            if (servo_angle_ < target_pan) {
                SetServoAngleDirect(target_pan - servo_angle_ > 2 ? servo_angle_ + 2 : target_pan);
            } else if (servo_angle_ > target_pan) {
                SetServoAngleDirect(servo_angle_ - target_pan > 2 ? servo_angle_ - 2 : target_pan);
            }
            if (tilt_angle_ < target_tilt) {
                SetTiltAngleDirect(target_tilt - tilt_angle_ > 2 ? tilt_angle_ + 2 : target_tilt);
            } else if (tilt_angle_ > target_tilt) {
                SetTiltAngleDirect(tilt_angle_ - target_tilt > 2 ? tilt_angle_ - 2 : target_tilt);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    static void TrackingTask(void* arg) {
        auto* self = static_cast<EspClawBoard*>(arg);
        ESP_LOGI(TAG, "Face tracking: start, pan/tilt centered");
        self->esp_video_->SetCaptureDqbufCount(1);  // 低延迟：直取帧，省 ~300ms/次
        self->SetServoAngleDirect(90);
        self->SetTiltAngleDirect(90);
        vTaskDelay(pdMS_TO_TICKS(400));
        while (self->tracking_) {
            self->TrackingStep();
            vTaskDelay(pdMS_TO_TICKS(30));  // SmoothMoveTo 自带节拍，这里只留小间隙
        }
        self->esp_video_->SetCaptureDqbufCount(3);  // 恢复默认
        self->SetServoAngleDirect(90);
        self->SetTiltAngleDirect(90);
        if (self->esp_video_ != nullptr) self->esp_video_->SetPreviewEnabled(true);
        ESP_LOGI(TAG, "Face tracking: stopped, pan/tilt centered");
        vTaskDelete(nullptr);
    }
#endif

    // Push a sentinel event to the bridge (dashboard log + speech queue).
    void BridgeNotify(const std::string& type, const std::string& message) {
        if (!BridgeEnabled()) return;
        cJSON* j = cJSON_CreateObject();
        if (j == nullptr) return;
        cJSON_AddStringToObject(j, "type", type.c_str());
        cJSON_AddStringToObject(j, "message", message.c_str());
        cJSON_AddStringToObject(j, "source", "sentinel");
        char* s = cJSON_PrintUnformatted(j);
        std::string body = (s != nullptr) ? s : "";
        cJSON_free(s);
        cJSON_Delete(j);
        BridgeHttpPostJson("/notify", body);
    }

    // Drain one reminder item: speak items are played with the bridge's
    // local TTS via the upstream notify engine; others are shown as alerts.
    void BridgePollPending() {
        std::string body = BridgeHttpGet(bridge_url_ + "/pending");
        if (body.empty()) return;
        cJSON* root = cJSON_Parse(body.c_str());
        if (root == nullptr) return;
        const cJSON* items = cJSON_GetObjectItem(root, "items");
        do {
            if (!cJSON_IsArray(items) || cJSON_GetArraySize(items) == 0) break;
            const cJSON* it = cJSON_GetArrayItem(items, 0);
            const cJSON* jtype = cJSON_GetObjectItem(it, "type");
            const cJSON* jmsg = cJSON_GetObjectItem(it, "message");
            std::string type = (jtype && jtype->valuestring) ? jtype->valuestring : "";
            std::string msg = (jmsg && jmsg->valuestring) ? jmsg->valuestring : "";
            if (msg.empty()) {
                BridgeAckAll();
                break;
            }
            // 桥接派单的"立即拍照复核"（2026-10-10）：哨兵离开/回归事件后
            // 桥接会塞一张拍照单过来，立即上传一帧给视觉对账，取代干等
            // 下一个 5 分钟周期照。设备忙时不 ack，下一个 10s tick 重试。
            if (type == "photo") {
                if (GetDeviceStateSafe() == kDeviceStateIdle) {
                    BridgeUploadPhoto("seat");
                    BridgeAckAll();
                    ESP_LOGI(TAG, "Bridge: on-demand verify photo uploaded");
                } else {
                    ESP_LOGI(TAG, "Bridge: busy, verify photo retry later");
                }
                break;
            }
            // 人脸跟踪开关（2026-10-14）：桥接 /tracking/start|stop 下发
#if CLAW_SERVO_POSITIONAL
            if (type == "tracking") {
                bool want_on = (msg == "on");
                if (want_on && !tracking_) {
                    tracking_ = true;
                    BridgeAckAll();
                    xTaskCreate(TrackingTask, "face_track", 8192, this, 1, nullptr);
                    ESP_LOGI(TAG, "Bridge: face tracking ON");
                } else if (!want_on && tracking_) {
                    tracking_ = false;  // 循环退出后自动回中、恢复预览
                    BridgeAckAll();
                    ESP_LOGI(TAG, "Bridge: face tracking OFF");
                } else {
                    BridgeAckAll();
                }
                break;
            }
            // 直接设舵机角度（2026-10-16）：桥接 /servo 下发。
            // msg = "h"（只动水平）或 "h,v"（二维云台）。
            if (type == "servo") {
                int a = atoi(msg.c_str());
                SetServoAngleDirect(a);
                const char* comma = strchr(msg.c_str(), ',');
                if (comma != nullptr) {
                    SetTiltAngleDirect(atoi(comma + 1));
                }
                BridgeAckAll();
                ESP_LOGI(TAG, "Bridge: servo -> %d deg, tilt -> %d deg",
                         servo_angle_, tilt_angle_);
                break;
            }
#endif
            if (type == "speak" || type == "remind_standup") {
                std::string tts_resp = BridgeHttpGet(
                    bridge_url_ + "/tts/url?text=" + BridgeUrlEncode(msg));
                std::string audio_url;
                cJSON* tj = cJSON_Parse(tts_resp.c_str());
                if (tj != nullptr) {
                    const cJSON* jurl = cJSON_GetObjectItem(tj, "url");
                    if (jurl != nullptr && jurl->valuestring != nullptr) {
                        audio_url = jurl->valuestring;
                    }
                    cJSON_Delete(tj);
                }
                if (audio_url.empty()) {
                    ESP_LOGW(TAG, "Bridge: TTS synth failed, drop message");
                    BridgeAckAll();
                    break;
                }
                bridge_play_result_.store(0);
                Application::GetInstance().Schedule(
                    [this, audio_url, msg]() {
                        bool started = Application::GetInstance().PlayNotifyAudio(audio_url, msg);
                        bridge_play_result_.store(started ? 1 : 2);
                    });
                for (int i = 0; i < 30 && bridge_play_result_.load() == 0; ++i) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                int r = bridge_play_result_.load();
                bridge_play_result_.store(0);
                if (r == 1) {
                    ESP_LOGI(TAG, "Bridge: speaking pending item (%s)", type.c_str());
                    BridgeAckAll();
                } else {
                    ESP_LOGI(TAG, "Bridge: device busy, retry later");
                    // No ack: the item stays queued for the next poll.
                }
            } else {
                // alert / unknown types: show on screen only
                std::string text = msg;
                Application::GetInstance().Schedule([text]() {
                    Application::GetInstance().Alert("Bridge", text.c_str(), "neutral");
                });
                BridgeAckAll();
            }
        } while (false);
        cJSON_Delete(root);
    }

    void BridgeTouch() {
        bridge_progress_ms_.store(esp_timer_get_time() / 1000);
    }

    static void BridgeTask(void* arg) {
        auto* self = static_cast<EspClawBoard*>(arg);
        int ticks = 0;
        // 订阅任务看门狗：本任务每 ≤1s 喂一次（远快于 TWDT 10s 超时）。
        // 一旦卡死在无超时的 send()/connect() 里，TWDT 会在串口打印
        // 该任务的调用栈（仅告警不 panic），同时守护任务 90s 后重启自愈。
        esp_task_wdt_add(nullptr);
        self->BridgeTouch();
        while (true) {
            for (int i = 0; i < 10; ++i) {  // 10s tick，期间每秒喂 TWDT
                esp_task_wdt_reset();
                // 派单后收线：见 bridge_close_session_ 注释。Speaking 状态
                // （云端抢答 TTS 还在被 abort）等下一秒再看；Listening 立即
                // 停止聆听回 idle，让桥接回执能播。
                int close = self->bridge_close_session_.load();
                if (close > 0) {
                    int st = self->GetDeviceStateSafe();
                    if (st == kDeviceStateListening) {
                        Application::GetInstance().StopListening();
                        self->bridge_close_session_.store(0);
                    } else if (st == kDeviceStateIdle) {
                        self->bridge_close_session_.store(0);
                    } else {
                        self->bridge_close_session_.store(close - 1);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            if (!self->BridgeEnabled()) {
                self->BridgeTouch();
                continue;
            }
            self->BridgeTouch();
            self->BridgePollPending();
            self->BridgeTouch();
            if (++ticks >= 30) {  // every ~5 minutes
                // Only clear the counter when we actually attempt the
                // upload: a busy device (conversation / TTS) skips it and
                // should retry on the next 10s tick instead of silently
                // waiting another full 5 minutes.
                if (self->GetDeviceStateSafe() == kDeviceStateIdle) {
                    ticks = 0;
                    self->BridgeUploadPhoto("seat");
                    self->BridgeTouch();
                }
            }
        }
    }

    // 守护任务：BridgeTask 卡死（TWDT 会先打印调用栈）超过 90s 无法自愈时
    // 重启设备。请求失败（如桥接关机、连接被拒）不算挂死——那会立即返回
    // 并照常更新进度戳；只有真正阻塞在 send()/connect()/相机里的死等才触发。
    static void BridgeWatchdogTask(void* arg) {
        auto* self = static_cast<EspClawBoard*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            int64_t last = self->bridge_progress_ms_.load();
            if (last == 0) continue;
            int64_t now = esp_timer_get_time() / 1000;
            if (now - last > 90000) {
                ESP_LOGE(TAG, "Bridge task stuck for %.0fs, rebooting to recover",
                         (now - last) / 1000.0);
                vTaskDelay(pdMS_TO_TICKS(500));  // let the log flush
                esp_restart();
            }
        }
    }

    void InitializeBridge() {
        bridge_url_ = CONFIG_CLAW_BRIDGE_URL;
        bridge_token_ = CONFIG_CLAW_BRIDGE_TOKEN;
        while (!bridge_url_.empty() && bridge_url_.back() == '/') {
            bridge_url_.pop_back();
        }
        if (!BridgeEnabled()) {
            ESP_LOGI(TAG, "Bridge client disabled (CONFIG_CLAW_BRIDGE_URL empty)");
            return;
        }
        xTaskCreate(BridgeTask, "claw_bridge", 8192, this, 1, nullptr);
        xTaskCreate(BridgeWatchdogTask, "claw_bridge_wdt", 3072, this, 1, nullptr);
        ESP_LOGI(TAG, "Bridge client started: %s", bridge_url_.c_str());
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        // --- LED ring (16x WS2812 on GPIO13) ---
        mcp_server.AddTool("self.led.set_all_color",
            "Set all 16 claw ring LEDs to one RGB color.",
            PropertyList({
                Property("red", kPropertyTypeInteger, 0, 255),
                Property("green", kPropertyTypeInteger, 0, 255),
                Property("blue", kPropertyTypeInteger, 0, 255),
            }), [this](const PropertyList& properties) -> ReturnValue {
                if (led_strip_ == nullptr) return false;
                int r = ScaledColor(properties["red"].value<int>());
                int g = ScaledColor(properties["green"].value<int>());
                int b = ScaledColor(properties["blue"].value<int>());
                for (int i = 0; i < CLAW_LED_STRIP_NUM; ++i) {
                    led_strip_set_pixel(led_strip_, i, r, g, b);
                }
                led_strip_refresh(led_strip_);
                return true;
            });

        mcp_server.AddTool("self.led.set_single_color",
            "Set one LED of the 16-led claw ring to an RGB color.",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, CLAW_LED_STRIP_NUM - 1),
                Property("red", kPropertyTypeInteger, 0, 255),
                Property("green", kPropertyTypeInteger, 0, 255),
                Property("blue", kPropertyTypeInteger, 0, 255),
            }), [this](const PropertyList& properties) -> ReturnValue {
                if (led_strip_ == nullptr) return false;
                int index = properties["index"].value<int>();
                if (index < 0 || index >= CLAW_LED_STRIP_NUM) return false;
                led_strip_set_pixel(led_strip_, index,
                                    ScaledColor(properties["red"].value<int>()),
                                    ScaledColor(properties["green"].value<int>()),
                                    ScaledColor(properties["blue"].value<int>()));
                led_strip_refresh(led_strip_);
                return true;
            });

        mcp_server.AddTool("self.led.set_brightness",
            "Set the claw LED ring brightness in percent (0-100).",
            PropertyList({
                Property("percent", kPropertyTypeInteger, 0, 100),
            }), [this](const PropertyList& properties) -> ReturnValue {
                led_brightness_ = Clamp255(properties["percent"].value<int>() * 255 / 100);
                return true;
            });

        mcp_server.AddTool("self.led.turn_off",
            "Turn off all claw ring LEDs.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                if (led_strip_ == nullptr) return false;
                led_strip_clear(led_strip_);
                return true;
            });

#if CLAW_SERVO_POSITIONAL
        // --- Servo (positional SG90 on GPIO47) ---
        // Pulse width = absolute position. The horn STAYS at the commanded
        // angle (PWM keeps flowing) — no auto-return, no timed spin.
        // --- Servo tools (2026-10-16 二维云台：水平 pan / 俯仰 tilt 分轴独立控制) ---
        mcp_server.AddTool("self.servo.set_pan",
            "Set the PAN servo (horizontal axis, left-right, GPIO47) to an "
            "absolute angle. angle: 0-180 degrees (90 = facing straight "
            "ahead). The servo moves there and HOLDS the position.",
            PropertyList({
                Property("angle", kPropertyTypeInteger, 0, 180),
            }), [this](const PropertyList& properties) -> ReturnValue {
                SetServoAngleDirect(properties["angle"].value<int>());
                ESP_LOGI(TAG, "Servo pan %d deg", servo_angle_);
                return true;
            });

        mcp_server.AddTool("self.servo.set_tilt",
            "Set the TILT servo (vertical axis, up-down, GPIO48) to an "
            "absolute angle. tilt: 0-180 degrees. 90 = level. LARGER tilt = "
            "look UP (e.g. 135 = looking up), SMALLER tilt = look DOWN "
            "(e.g. 45 = looking down). The servo moves there and HOLDS the "
            "position.",
            PropertyList({
                Property("tilt", kPropertyTypeInteger, 0, 180),
            }), [this](const PropertyList& properties) -> ReturnValue {
                SetTiltAngleDirect(properties["tilt"].value<int>());
                ESP_LOGI(TAG, "Servo tilt %d deg", tilt_angle_);
                return true;
            });

        mcp_server.AddTool("self.servo.stop",
            "Return both pan and tilt servos to center position (90 degrees).",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                SetServoAngleDirect(90);
                SetTiltAngleDirect(90);
                ESP_LOGI(TAG, "Servos centered (pan 90, tilt 90 deg)");
                return true;
            });
#else
        // --- Servo (continuous-rotation type on GPIO47) ---
        // Pulse width = speed/direction, NOT position: 0% = stop, +/- = spin.
        mcp_server.AddTool("self.servo.set_speed",
            "Spin the continuous-rotation servo. speed: -100..100 percent "
            "(negative = counterclockwise, positive = clockwise, 0 = stop). "
            "duration_ms: if > 0, stop automatically after this time; "
            "if 0, keep spinning until stop is called.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, -100, 100),
                Property("duration_ms", kPropertyTypeInteger, 0, 10000),
            }), [this](const PropertyList& properties) -> ReturnValue {
                servo_speed_ = properties["speed"].value<int>();
                int duration_ms = properties["duration_ms"].value<int>();
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ServoDuty(servo_speed_));
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                if (duration_ms > 0) {
                    // Auto-stop after the requested spin time so the AI can
                    // approximate "turn a bit" on a position-less servo.
                    int speed = servo_speed_;
                    // Use a fresh thread per timed spin; stop_duty applied on wake.
                    xTaskCreate([](void* arg) {
                        TickType_t ticks = pdMS_TO_TICKS((int)(uintptr_t)arg);
                        vTaskDelay(ticks > 0 ? ticks : 1);
                        // 1500 us neutral pulse = motor stop.
                        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1229);
                        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                        vTaskDelete(nullptr);
                    }, "servo_stop", 3072, (void*)(uintptr_t)duration_ms, 2, nullptr);
                    ESP_LOGI(TAG, "Servo speed %d%% for %d ms", servo_speed_, duration_ms);
                } else {
                    ESP_LOGI(TAG, "Servo speed %d%% (continuous)", servo_speed_);
                }
                return true;
            });

        mcp_server.AddTool("self.servo.stop",
            "Stop the continuous-rotation servo immediately.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                servo_speed_ = 0;
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ServoDuty(0));
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ESP_LOGI(TAG, "Servo stopped");
                return true;
            });
#endif

        // --- Persistent reminders (survive reboot, stored in NVS) ---
        mcp_server.AddTool("self.reminder.add_daily",
            "Create a DAILY reminder. At hour:minute every day the device will "
            "speak the message aloud (e.g. drink water, take medicine). "
            "Max 4 reminders total.",
            PropertyList({
                Property("hour", kPropertyTypeInteger, 0, 23),
                Property("minute", kPropertyTypeInteger, 0, 59),
                Property("message", kPropertyTypeString).SetMaxLength(63),
            }), [this](const PropertyList& properties) -> ReturnValue {
                xSemaphoreTake(reminders_mutex_, portMAX_DELAY);
                if (reminders_.size() >= kMaxReminders) {
                    xSemaphoreGive(reminders_mutex_);
                    return std::string("失败：提醒数量已达上限(4个)，请先删除一些");
                }
                ClawReminder r = {};
                r.type = 0;
                r.hour = properties["hour"].value<int>();
                r.minute = properties["minute"].value<int>();
                r.last_fired_yday = kYdayNever;
                r.next_due = 0;
                strncpy(r.message, properties["message"].value<std::string>().c_str(),
                        sizeof(r.message) - 1);
                reminders_.push_back(r);
                SaveReminders();
                int index = (int)reminders_.size() - 1;
                xSemaphoreGive(reminders_mutex_);
                char buf[160];
                snprintf(buf, sizeof(buf), "已设置每天 %02d:%02d 的提醒(编号%d)：%s",
                         r.hour, r.minute, index, r.message);
                ESP_LOGI(TAG, "%s", buf);
                return std::string(buf);
            });

        mcp_server.AddTool("self.reminder.add_interval",
            "Create an INTERVAL reminder that repeats every N minutes "
            "(e.g. drink water every 60 minutes). It keeps repeating until "
            "removed. Max 4 reminders total.",
            PropertyList({
                Property("minutes", kPropertyTypeInteger, 10, 1440),
                Property("message", kPropertyTypeString).SetMaxLength(63),
            }), [this](const PropertyList& properties) -> ReturnValue {
                xSemaphoreTake(reminders_mutex_, portMAX_DELAY);
                if (reminders_.size() >= kMaxReminders) {
                    xSemaphoreGive(reminders_mutex_);
                    return std::string("失败：提醒数量已达上限(4个)，请先删除一些");
                }
                ClawReminder r = {};
                r.type = 1;
                r.interval_min = properties["minutes"].value<int>();
                r.last_fired_yday = kYdayNever;
                r.next_due = time(nullptr) + (time_t)r.interval_min * 60;
                strncpy(r.message, properties["message"].value<std::string>().c_str(),
                        sizeof(r.message) - 1);
                reminders_.push_back(r);
                SaveReminders();
                int index = (int)reminders_.size() - 1;
                xSemaphoreGive(reminders_mutex_);
                char buf[192];
                snprintf(buf, sizeof(buf), "已设置每 %d 分钟的提醒(编号%d)：%s，首次在 %d 分钟后",
                         r.interval_min, index, r.message, r.interval_min);
                ESP_LOGI(TAG, "%s", buf);
                return std::string(buf);
            });

        mcp_server.AddTool("self.reminder.add_once",
            "Create a ONE-TIME reminder: fires ONCE at the given time, then "
            "deletes itself automatically. hour/minute: time of day; "
            "day_offset: 0 = today, 1 = tomorrow, 2 = day after tomorrow... "
            "If that time has already passed, it fires on the next day "
            "instead. Use this for one-off asks like '20:30 提醒我拿快递'. "
            "Max 4 reminders total.",
            PropertyList({
                Property("hour", kPropertyTypeInteger, 0, 23),
                Property("minute", kPropertyTypeInteger, 0, 59),
                Property("day_offset", kPropertyTypeInteger, 0, 30),
                Property("message", kPropertyTypeString).SetMaxLength(63),
            }), [this](const PropertyList& properties) -> ReturnValue {
                xSemaphoreTake(reminders_mutex_, portMAX_DELAY);
                if (reminders_.size() >= kMaxReminders) {
                    xSemaphoreGive(reminders_mutex_);
                    return std::string("失败：提醒数量已达上限(4个)，请先删除一些");
                }
                time_t now = time(nullptr);
                struct tm t;
                localtime_r(&now, &t);
                t.tm_hour = properties["hour"].value<int>();
                t.tm_min = properties["minute"].value<int>();
                t.tm_sec = 0;
                t.tm_mday += properties["day_offset"].value<int>();
                time_t due = mktime(&t);
                int bumped = 0;
                while (due <= now) {  // time already passed -> next day
                    t.tm_mday += 1;
                    due = mktime(&t);
                    ++bumped;
                }
                ClawReminder r = {};
                r.type = 2;
                r.last_fired_yday = kYdayNever;
                r.next_due = due;
                strncpy(r.message, properties["message"].value<std::string>().c_str(),
                        sizeof(r.message) - 1);
                reminders_.push_back(r);
                SaveReminders();
                int index = (int)reminders_.size() - 1;
                xSemaphoreGive(reminders_mutex_);
                struct tm dt;
                localtime_r(&due, &dt);
                char buf[320];
                snprintf(buf, sizeof(buf),
                         "已设置单次提醒(编号%d)：%s，将在 %d月%d日 %02d:%02d 播报一次%s",
                         index, r.message, dt.tm_mon + 1, dt.tm_mday,
                         dt.tm_hour, dt.tm_min,
                         bumped > 0 ? "（该时刻已过，自动顺延到明天）" : "");
                ESP_LOGI(TAG, "%s", buf);
                return std::string(buf);
            });

        mcp_server.AddTool("self.reminder.list",
            "List all active reminders with their index numbers.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                std::string out;
                xSemaphoreTake(reminders_mutex_, portMAX_DELAY);
                if (reminders_.empty()) {
                    out = "当前没有任何提醒";
                } else {
                    out = "共 " + std::to_string(reminders_.size()) + " 个提醒：";
                    for (int i = 0; i < (int)reminders_.size(); ++i) {
                        char buf[128];
                        if (reminders_[i].type == 0) {
                            snprintf(buf, sizeof(buf), "\n%d. 每天 %02d:%02d：%s",
                                     i, reminders_[i].hour, reminders_[i].minute,
                                     reminders_[i].message);
                        } else if (reminders_[i].type == 1) {
                            snprintf(buf, sizeof(buf), "\n%d. 每 %d 分钟：%s",
                                     i, reminders_[i].interval_min, reminders_[i].message);
                        } else {
                            struct tm dt;
                            localtime_r(&reminders_[i].next_due, &dt);
                            snprintf(buf, sizeof(buf),
                                     "\n%d. 单次 %d月%d日 %02d:%02d：%s",
                                     i, dt.tm_mon + 1, dt.tm_mday,
                                     dt.tm_hour, dt.tm_min, reminders_[i].message);
                        }
                        out += buf;
                    }
                }
                xSemaphoreGive(reminders_mutex_);
                return out;
            });

        mcp_server.AddTool("self.reminder.remove",
            "Delete one reminder by its index number (see reminder list).",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, kMaxReminders - 1),
            }), [this](const PropertyList& properties) -> ReturnValue {
                int index = properties["index"].value<int>();
                std::string out;
                xSemaphoreTake(reminders_mutex_, portMAX_DELAY);
                if (index < 0 || index >= (int)reminders_.size()) {
                    out = "失败：编号 " + std::to_string(index) + " 不存在，请先查询提醒列表";
                } else {
                    out = "已删除提醒：" + std::string(reminders_[index].message);
                    reminders_.erase(reminders_.begin() + index);
                    SaveReminders();
                }
                xSemaphoreGive(reminders_mutex_);
                ESP_LOGI(TAG, "%s", out.c_str());
                return out;
            });

        mcp_server.AddTool("self.reminder.test",
            "Fire the FIRST reminder immediately as a test (once the device is "
            "idle). Use this to verify reminders work instead of waiting for "
            "the real schedule. Does not create or delete reminders.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                bool has;
                xSemaphoreTake(reminders_mutex_, portMAX_DELAY);
                has = !reminders_.empty();
                xSemaphoreGive(reminders_mutex_);
                if (!has) {
                    return std::string("失败：当前没有任何提醒可测试");
                }
                test_fire_.store(true);
                ESP_LOGI(TAG, "Test fire requested");
                return std::string("好的，设备空闲时会立即播报一次提醒，请注意听");
            });

        // --- Auto look: periodically describe what the camera sees ---
        mcp_server.AddTool("self.camera.auto_look",
            "Enable or disable PERIODIC LOOKING: every N minutes (while the "
            "device is idle) the device automatically takes a photo with its "
            "camera and SPEAKS a short analysis of what it sees. "
            "message: what to check each time, keep it SHORT (max 9 Chinese "
            "characters), e.g. 看看周围有什么.",
            PropertyList({
                Property("enable", kPropertyTypeBoolean),
                Property("minutes", kPropertyTypeInteger, 10, 1, 1440),
                Property("message", kPropertyTypeString, std::string("看看周围有什么")).SetMaxLength(27),
            }), [this](const PropertyList& properties) -> ReturnValue {
                bool enable = properties["enable"].value<bool>();
                int minutes = properties["minutes"].value<int>();
                std::string msg = properties["message"].value<std::string>();
                if (msg.empty()) msg = "看看周围有什么";
                strncpy(auto_look_msg_, msg.c_str(), sizeof(auto_look_msg_) - 1);
                auto_look_msg_[sizeof(auto_look_msg_) - 1] = '\0';
                auto_look_en_.store(enable);
                auto_look_interval_.store((uint16_t)minutes);
                SaveAutoLook();
                if (enable) {
                    look_next_due_ = time(nullptr);  // first look ASAP when idle
                    char buf[160];
                    snprintf(buf, sizeof(buf),
                             "已开启定时查看，每 %d 分钟看一眼并说说看到的内容，第一张马上看", minutes);
                    ESP_LOGI(TAG, "%s", buf);
                    return std::string(buf);
                }
                ESP_LOGI(TAG, "Auto look disabled");
                return std::string(
                    "已关闭定时查看，之后不会再自动看一眼。请直接告诉用户已关闭，"
                    "不要再拍照或看任何东西。");
            });

        mcp_server.AddTool("self.camera.look_now",
            "Look through the camera ONCE right now (when the device is "
            "idle): take a photo and speak a description of what it sees. "
            "Use this ONLY when the user explicitly asks to look once now "
            "(e.g. for testing). NEVER call it on your own initiative, as a "
            "farewell gesture, or right after disabling periodic looking.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                auto_look_now_.store(true);
                ESP_LOGI(TAG, "One-shot auto look requested");
                return std::string("好的，马上看一眼并说说看到了什么");
            });

        // --- Desk sentinel: local watch for left-seat + water cup ---
        mcp_server.AddTool("self.camera.desk_watch",
            "Enable or disable the on-device desk sentinel. It watches the "
            "camera LOCALLY (frames never leave the device) to detect when "
            "the user leaves the seat, silently count water-drink events, "
            "and give ONE voice reminder if the cup has been gone for many "
            "minutes. Only use this when the user explicitly asks to start "
            "or stop the seat/cup watch. Args: enable; interval_seconds "
            "(5-300, how often to sample, default 15); cup_roi optional "
            "\"x,y,w,h\" in permille of the frame pointing at where the cup "
            "usually sits (default \"430,460,300,400\").",
            PropertyList({
                Property("enable", kPropertyTypeBoolean),
                Property("interval_seconds", kPropertyTypeInteger, 15, 5, 300),
                Property("cup_roi", kPropertyTypeString, std::string("")).SetMaxLength(31),
            }), [this](const PropertyList& properties) -> ReturnValue {
                bool enable = properties["enable"].value<bool>();
                int iv = properties["interval_seconds"].value<int>();
                std::string roi = properties["cup_roi"].value<std::string>();
                if (!roi.empty()) {
                    int v[4] = {0, 0, 0, 0};
                    if (sscanf(roi.c_str(), "%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3]) == 4 &&
                        v[0] >= 0 && v[1] >= 0 && v[2] > 0 && v[3] > 0 &&
                        v[0] + v[2] <= 1000 && v[1] + v[3] <= 1000) {
                        for (int i = 0; i < 4; ++i) sent_roi_[i] = (uint16_t)v[i];
                    } else {
                        return std::string("ROI 格式无效，应为 \"x,y,w,h\" 千分比，例如 430,460,300,400");
                    }
                }
                sent_interval_.store((uint16_t)iv);
                sent_en_.store(enable);
                if (enable) {
                    ResetSentinelScene();  // fresh baseline with the new settings
                }
                SaveSentinel();
                if (enable) {
                    char buf[160];
                    snprintf(buf, sizeof(buf),
                             "已开启桌面前哨：每 %d 秒本地巡检一次，喝水静默计数，"
                             "水杯消失超过 %u 分钟提醒一句，离座检测开启。",
                             iv, (unsigned)sent_cup_absent_.load());
                    ESP_LOGI(TAG, "%s", buf);
                    return std::string(buf);
                }
                ESP_LOGI(TAG, "Desk sentinel disabled");
                return std::string("已关闭桌面前哨巡检。请直接告诉用户已关闭，"
                                   "不要拍照或继续观察。");
            });

        mcp_server.AddTool("self.camera.desk_status",
            "Report the desk sentinel status as JSON: enabled, interval, "
            "drink_count, still_s (seconds without motion), away flag, "
            "cup_absent flag and absent seconds, cup ROI. Use when the user "
            "asks about the seat watch, the cup watch, or their drink count.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                return SentinelStatusString();
            });

        mcp_server.AddTool("self.camera.desk_probe",
            "Run ONE desk sentinel detection cycle right now and log the "
            "motion/ROI scores to the device log (no voice output). Use ONLY "
            "when the user explicitly asks to test or calibrate the sentinel.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                sent_probe_.store(true);
                ESP_LOGI(TAG, "Sentinel probe requested");
                return std::string("好的，下一秒会执行一次本地检测并在日志里输出分数，不会说话");
            });
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    // ---------------- 语音任务路由（2026-10-11） ----------------
    // 规则分类器：命中"复杂任务"关键词 → 转交桥接 /agent_task（WorkBuddy
    // 无头执行），否则放行给小智云端（fail-open：不确定=简单）。整个分类
    // 只是常量字符串子串匹配，几 KB Flash、零堆分配压力，板上跑毫无负担。
    static bool IsComplexTask(const std::string& text) {
        // ASCII 关键词统一小写比较；中文关键词 UTF-8 字节精确匹配
        static const char* const kTriggers[] = {
            "workbuddy", "沃克巴迪",
            "整理", "归纳", "总结", "分析", "调研",
            "报告", "周报", "日报",
            "写一份", "写一篇", "写个", "帮我写", "写一段", "生成一份", "做一份", "做一版",
            "搜索", "搜一下", "查资料", "上网查",
            "代码", "编程", "脚本",
            "文档", "方案", "翻译",
        };
        std::string lower;
        lower.reserve(text.size());
        for (char c : text) lower += static_cast<char>(tolower(static_cast<unsigned char>(c)));
        for (const char* kw : kTriggers) {
            std::string kl;
            for (const char* p = kw; *p; ++p) kl += static_cast<char>(tolower(static_cast<unsigned char>(*p)));
            if (lower.find(kl) != std::string::npos) return true;
        }
        return false;
    }

    // Board 钩子实现：返回 true = 已派单成功，Application 会打断云端抢答；
    // 桥接不可达（电脑关机）/busy/拒单 → 返回 false，小智照常回答。
    bool OnUserUtteranceRoute(const std::string& text) override {
        if (!IsComplexTask(text)) return false;
        if (!BridgeEnabled()) {
            ESP_LOGI(TAG, "Router: complex task but bridge disabled, let cloud answer");
            return false;
        }
        ESP_LOGI(TAG, "Router: complex task (%d bytes), forwarding to bridge", (int)text.size());
        cJSON* j = cJSON_CreateObject();
        if (j == nullptr) return false;
        cJSON_AddStringToObject(j, "text", text.c_str());
        char* s = cJSON_PrintUnformatted(j);
        std::string body = (s != nullptr) ? s : "";
        cJSON_free(s);
        cJSON_Delete(j);
        std::string resp = BridgeHttpPostJson("/agent_task", body);
        bool accepted = resp.find("accepted") != std::string::npos;
        ESP_LOGI(TAG, "Router: bridge %s", accepted ? "accepted the task"
                 : (resp.empty() ? "unreachable, let cloud answer" : resp.substr(0, 60).c_str()));
        if (accepted) {
            // 派单成功：给 BridgeTask 15s 窗口把当前会话收线回 idle，
            // 好让桥接的接单回执（≤10s 轮询）能立刻播出来。
            bridge_close_session_.store(15);
        }
        return accepted;
    }

    EspClawBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeUsb();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeLedStrip();
        InitializeServo();
        RegisterMcpTools();
        InitializeReminders();
        InitializeAutoLook();
        InitializeSentinel();
        InitializeBridge();
        GetBacklight()->RestoreBrightness();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static UacAudioCodec audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }
};

DECLARE_BOARD(EspClawBoard);
