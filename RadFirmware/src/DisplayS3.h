// Big-number position display for the T-Display-S3 style controller.
// ST7789 (170x320) on the ESP32-S3 8-bit i80 parallel bus, driven with the
// esp_lcd API built into the ESP32 core — no external graphics libraries.
// Renders the dome position as three large 7-segment digits ("---" when the
// sensor is stale), plus a small activity square while automation drives.
#pragma once
#if defined(ARDUINO) && defined(RAD_BOARD_DISPLAY)

#include <Arduino.h>

namespace rad {

class DisplayS3 {
  public:
    bool begin();

    // Call every loop; redraws (throttled) only when something changed.
    // valid=false renders "---"; moving=true lights the activity square.
    void update(uint32_t now, bool valid, int16_t deg, bool moving);

  private:
    void render(bool valid, int16_t deg, bool moving);
    void fillRect(int x, int y, int w, int h, uint16_t color);
    void drawSegments(int x, int y, uint8_t mask);

    void* fPanel = nullptr; // esp_lcd_panel_handle_t (kept void* to keep header light)
    uint16_t* fFrame = nullptr;
    bool fReady = false;
    bool fLastValid = false;
    int16_t fLastDeg = -1;
    bool fLastMoving = false;
    uint32_t fLastDrawMs = 0;
};

} // namespace rad

#endif // ARDUINO && RAD_BOARD_DISPLAY
