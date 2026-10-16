#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// Audio runs over USB UAC (webcam dongle with built-in mic + speaker).
// Rates are requested targets; the codec negotiates with the device and
// resamples internally if needed.
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define BOOT_BUTTON_GPIO        GPIO_NUM_0

// ---- ST7789 SPI LCD (1.14" 240x135 module) ----
// Tutorial breadboard rows 2-9 map to mainboard header rows, which on the
// ESP32-S3-DevKitC-1 left header are 3V3,3V3,RST,4,5,6,7,15,16 — so the six
// signal pins land on GPIO 4,5,6,7,15,16 (confirmed by ESP-Claw docs:
// SCL=4 SDA=5 RST=6 DC=7 CS=15 BLK=16; LED-ring example GPIO13 = row 19).
#define DISPLAY_MOSI_PIN      GPIO_NUM_5   // SDA
#define DISPLAY_CLK_PIN       GPIO_NUM_4   // SCL
#define DISPLAY_DC_PIN        GPIO_NUM_7
#define DISPLAY_CS_PIN        GPIO_NUM_15
#define DISPLAY_RST_PIN       GPIO_NUM_6
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_16  // BLK
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

// ---- SPI LCD: module is a GMT020-02 (confirmed by the PCB silkscreen
// "2.0 TFTSPI GMT020-02-8P UER 1.21"): a 2.0" 240x320 ST7789V SPI glass —
// NOT the 1.14" 240x135 the tutorial assumed and NOT 320x480. Used landscape
// as 320x240 via the esp_lcd MV bit (swap_xy), like the waveshare 3.5" board.
// MADCTL = MV|MY: direction verified on hardware in flash20 (user confirmed
// the orientation was correct with swap_xy=true + mirror_y=true).
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y true
#define DISPLAY_SWAP_XY true
#define DISPLAY_INVERT_COLOR true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_SPI_MODE 0

// On-board peripherals (from the ESP-CLAW tutorial)
#define CLAW_LED_STRIP_GPIO   GPIO_NUM_13
#define CLAW_LED_STRIP_NUM    16
#define CLAW_SERVO_GPIO       GPIO_NUM_47
// 第二路舵机（2026-10-16 云台俯仰轴）：GPIO48 在右侧排针紧挨 47，固件未占用。
#define CLAW_SERVO2_GPIO      GPIO_NUM_48

// Servo type: 1 = positional servo (SG90, 0-180 degrees, pulse = position,
// holds wherever it is told to go); 0 = 360-degree continuous-rotation servo
// (pulse = speed/direction, 1500 us = stop). Set to 0 if you swap back.
#define CLAW_SERVO_POSITIONAL 1

#endif // _BOARD_CONFIG_H_
