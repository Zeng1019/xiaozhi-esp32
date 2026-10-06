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
#include <cstring>

#include "usb/usb_host.h"
#include "esp_video_init.h"
#include "esp_video.h"
#include <driver/ledc.h>
#include <led_strip.h>

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
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    led_strip_handle_t led_strip_ = nullptr;
    int led_brightness_ = 128;  // 0-255 master scale for the LED ring
    int servo_speed_ = 0;  // -100..100 percent, 0 = stopped

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

    // ---- Persistent reminders (NVS) that make the device speak when due ----
    // Daily reminders fire once per calendar day at HH:MM; interval reminders
    // re-fire every N minutes. On fire the device opens an audio session and
    // sends the message text as if spoken, so the cloud AI answers by voice.
    static constexpr int kMaxReminders = 4;
    struct ClawReminder {
        uint8_t type;            // 0 = daily HH:MM, 1 = every interval_min
        uint8_t hour;            // daily: 0-23
        uint8_t minute;          // daily: 0-59
        uint16_t interval_min;   // interval type: period in minutes
        char message[64];        // what the reminder is about (UTF-8)
        uint16_t last_fired_yday;// daily dedup: day-of-year already fired
        time_t next_due;         // interval type: earliest fire time
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
                else tr.next_due = now + (time_t)tr.interval_min * 60;
                dirty = true;
                self->FireReminder(tr);
            }
            for (auto& r : self->reminders_) {
                if (r.type == 0) {
                    // Daily at HH:MM, once per calendar day
                    if (lt.tm_hour == r.hour && lt.tm_min == r.minute &&
                        r.last_fired_yday != lt.tm_yday &&
                        self->GetDeviceStateSafe() == kDeviceStateIdle) {
                        r.last_fired_yday = lt.tm_yday;
                        self->FireReminder(r);
                    }
                } else {
                    // Every N minutes; skip while busy, catch up once free
                    if (now >= r.next_due &&
                        self->GetDeviceStateSafe() == kDeviceStateIdle) {
                        r.next_due = now + (time_t)r.interval_min * 60;
                        dirty = true;  // persist the advanced next_due
                        self->FireReminder(r);
                    }
                }
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
                        } else {
                            snprintf(buf, sizeof(buf), "\n%d. 每 %d 分钟：%s",
                                     i, reminders_[i].interval_min, reminders_[i].message);
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
    EspClawBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeUsb();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeLedStrip();
        InitializeServo();
        RegisterMcpTools();
        InitializeReminders();
        InitializeAutoLook();
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
