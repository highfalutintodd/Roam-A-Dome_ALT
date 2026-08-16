#ifdef ARDUINO
#include "../pinmap.h"
#include <Arduino.h> // brings in sdkconfig for the IDF target macro below
#endif
#if defined(ARDUINO) && defined(RAD_BOARD_DISPLAY) && defined(CONFIG_IDF_TARGET_ESP32S3)

#include "DisplayS3.h"

#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

namespace rad {
namespace {

constexpr int kW = 320; // landscape
constexpr int kH = 170;
constexpr uint16_t kBlack = 0x0000;
constexpr uint16_t kWhite = 0xFFFF;
constexpr uint16_t kRed = 0xF800;
constexpr uint16_t kAmber = 0xFD20;

// 7-segment geometry
constexpr int kDigitW = 84, kDigitH = 132, kSegT = 16, kGap = 12;
constexpr int kSegLen = (kDigitH - 3 * kSegT) / 2; // vertical segment length
constexpr int kX0 = (kW - (3 * kDigitW + 2 * kGap)) / 2;
constexpr int kY0 = (kH - kDigitH) / 2;

// Segment bits: A top, B top-right, C bottom-right, D bottom, E bottom-left,
// F top-left, G middle.
constexpr uint8_t kDigitSegs[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
constexpr uint8_t kSegG = 0x40;

} // namespace

bool DisplayS3::begin() {
    pinMode(RAD_PIN_LCD_POWER, OUTPUT);
    digitalWrite(RAD_PIN_LCD_POWER, HIGH);
    pinMode(RAD_PIN_LCD_RD, OUTPUT);
    digitalWrite(RAD_PIN_LCD_RD, HIGH); // RD held inactive: write-only bus
    pinMode(RAD_PIN_LCD_BL, OUTPUT);
    digitalWrite(RAD_PIN_LCD_BL, LOW); // backlight off until the first frame

    static const int kData[8] = RAD_PIN_LCD_DATA;

    esp_lcd_i80_bus_config_t bus_config = {};
    bus_config.dc_gpio_num = RAD_PIN_LCD_DC;
    bus_config.wr_gpio_num = RAD_PIN_LCD_WR;
    bus_config.clk_src = LCD_CLK_SRC_DEFAULT;
    for (int i = 0; i < 8; ++i)
        bus_config.data_gpio_nums[i] = kData[i];
    bus_config.bus_width = 8;
    bus_config.max_transfer_bytes = kW * kH * 2 + 32;
    esp_lcd_i80_bus_handle_t bus = nullptr;
    if (esp_lcd_new_i80_bus(&bus_config, &bus) != ESP_OK)
        return false;

    esp_lcd_panel_io_i80_config_t io_config = {};
    io_config.cs_gpio_num = RAD_PIN_LCD_CS;
    io_config.pclk_hz = 10 * 1000 * 1000;
    io_config.trans_queue_depth = 4;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.dc_levels.dc_idle_level = 0;
    io_config.dc_levels.dc_cmd_level = 0;
    io_config.dc_levels.dc_dummy_level = 0;
    io_config.dc_levels.dc_data_level = 1;
    io_config.flags.swap_color_bytes = 1; // host RGB565 -> panel byte order
    esp_lcd_panel_io_handle_t io = nullptr;
    if (esp_lcd_new_panel_io_i80(bus, &io_config, &io) != ESP_OK)
        return false;

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = RAD_PIN_LCD_RES;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    esp_lcd_panel_handle_t panel = nullptr;
    if (esp_lcd_new_panel_st7789(io, &panel_config, &panel) != ESP_OK)
        return false;

    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, true);
    // Landscape with the 170px axis vertical; the ST7789 RAM is 240x320, so the
    // 170-wide panel sits at a 35px offset. If the image comes up mirrored or
    // shifted on a board revision, these three lines are the knobs.
    esp_lcd_panel_swap_xy(panel, true);
    esp_lcd_panel_mirror(panel, false, true);
    esp_lcd_panel_set_gap(panel, 0, 35);
    esp_lcd_panel_disp_on_off(panel, true);
    fPanel = panel;

    fFrame = static_cast<uint16_t*>(
        heap_caps_malloc(kW * kH * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (fFrame == nullptr)
        return false;

    render(false, 0, false); // "---" until the sensor warms up
    digitalWrite(RAD_PIN_LCD_BL, HIGH);
    fReady = true;
    return true;
}

void DisplayS3::update(uint32_t now, bool valid, int16_t deg, bool moving) {
    if (!fReady)
        return;
    if (valid == fLastValid && deg == fLastDeg && moving == fLastMoving)
        return;
    if (now - fLastDrawMs < 80)
        return; // ~12 fps cap; keeps the motion loop responsive
    fLastDrawMs = now;
    fLastValid = valid;
    fLastDeg = deg;
    fLastMoving = moving;
    render(valid, deg, moving);
}

void DisplayS3::fillRect(int x, int y, int w, int h, uint16_t color) {
    for (int row = y; row < y + h; ++row) {
        uint16_t* p = fFrame + row * kW + x;
        for (int col = 0; col < w; ++col)
            *p++ = color;
    }
}

void DisplayS3::drawSegments(int x, int y, uint8_t mask) {
    uint16_t c = (mask == kSegG) ? kRed : kWhite; // bare middle bar = stale dash
    if (mask & 0x01) fillRect(x + kSegT, y, kDigitW - 2 * kSegT, kSegT, c);                 // A
    if (mask & 0x02) fillRect(x + kDigitW - kSegT, y + kSegT, kSegT, kSegLen, c);           // B
    if (mask & 0x04) fillRect(x + kDigitW - kSegT, y + 2 * kSegT + kSegLen, kSegT, kSegLen, c); // C
    if (mask & 0x08) fillRect(x + kSegT, y + 2 * kSegT + 2 * kSegLen, kDigitW - 2 * kSegT, kSegT, c); // D
    if (mask & 0x10) fillRect(x, y + 2 * kSegT + kSegLen, kSegT, kSegLen, c);               // E
    if (mask & 0x20) fillRect(x, y + kSegT, kSegT, kSegLen, c);                             // F
    if (mask & 0x40) fillRect(x + kSegT, y + kSegT + kSegLen, kDigitW - 2 * kSegT, kSegT, c); // G
}

void DisplayS3::render(bool valid, int16_t deg, bool moving) {
    fillRect(0, 0, kW, kH, kBlack);
    if (!valid) {
        for (int i = 0; i < 3; ++i)
            drawSegments(kX0 + i * (kDigitW + kGap), kY0, kSegG); // "---"
    } else {
        int d0 = deg / 100, d1 = (deg / 10) % 10, d2 = deg % 10;
        if (d0 != 0)
            drawSegments(kX0, kY0, kDigitSegs[d0]);
        if (d0 != 0 || d1 != 0)
            drawSegments(kX0 + kDigitW + kGap, kY0, kDigitSegs[d1]);
        drawSegments(kX0 + 2 * (kDigitW + kGap), kY0, kDigitSegs[d2]);
    }
    if (moving)
        fillRect(kW - 20, 4, 16, 16, kAmber);

    esp_lcd_panel_draw_bitmap(static_cast<esp_lcd_panel_handle_t>(fPanel), 0, 0, kW, kH, fFrame);
}

} // namespace rad

#endif // ARDUINO && RAD_BOARD_DISPLAY && ESP32S3
