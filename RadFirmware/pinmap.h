// Board-variant pin tables for Roam-A-Dome v2.
//
// Variant is selected by build flag; RAD_BOARD_DISPLAY is the default because the
// user's board has the big-number screen. Legacy note: the old firmware used
// espsoftwareserial for the Syren TX and command port because core 2.x hardware
// serial had unacceptable TX latency; core 3.x HardwareSerial maps any UART to any
// pin with no such issue, so v2 is all hardware UARTs on the S3 (UART1 sensor,
// UART2 Syren, UART0 command port — console rides the native USB CDC).
//
// !! Pin values below mirror legacy/pin-map.h and must be confirmed against the
// !! actual PCB during the Phase 0 bench session (BENCH.md §1).

#pragma once

#if !defined(RAD_BOARD_COMPACT) && !defined(RAD_BOARD_DISPLAY)
#define RAD_BOARD_DISPLAY
#endif

#ifdef RAD_BOARD_DISPLAY
// LilyGO T-Display-S3 style controller (ESP32-S3, parallel LCD)

#define RAD_PIN_SENSOR_RX 1   // dome sensor ring input (UART1, RX only)
#define RAD_PIN_SYREN_IN_RX 17  // Syren packet serial from droid controller (UART2 RX)
#define RAD_PIN_SYREN_OUT_TX 16 // Syren packet serial to motor controller (UART2 TX)
#define RAD_PIN_CMD_RX 18       // command serial in (UART0 RX; wired to WCB S2)
#define RAD_PIN_CMD_TX 13       // command serial out (UART0 TX)

#define RAD_PIN_PWM_IN 3   // RC pulse input
#define RAD_PIN_PWM_OUT 2  // RC pulse output
#define RAD_PIN_PPM_IN 21  // PPM mode input

#define RAD_PIN_DOUT                                                                               \
    { 10, 11, 12 } // digital output pins 1..N

#define RAD_HAS_DISPLAY 1
#define RAD_PIN_LCD_BL 38
#define RAD_PIN_LCD_POWER 15
#define RAD_PIN_LCD_RES 5
#define RAD_PIN_LCD_CS 6
#define RAD_PIN_LCD_DC 7
#define RAD_PIN_LCD_WR 8
#define RAD_PIN_LCD_RD 9
#define RAD_PIN_LCD_DATA                                                                           \
    { 39, 40, 41, 42, 45, 46, 47, 48 }

#endif // RAD_BOARD_DISPLAY

#ifdef RAD_BOARD_COMPACT
// Compact controller (classic ESP32, no display)

#define RAD_PIN_SENSOR_RX 34
#define RAD_PIN_SYREN_IN_RX 17
#define RAD_PIN_SYREN_OUT_TX 16
// Reserved only: the classic ESP32 has no free UART for a command port (UART0 =
// console, UART1 = sensor, UART2 = Syren), so the compact build has no command
// serial — commands arrive via the USB console or the mesh, and setup() prints
// a notice if #DPSERIALCMD is on. Pins kept for a future soft-UART option.
#define RAD_PIN_CMD_RX 32
#define RAD_PIN_CMD_TX 4

#define RAD_PIN_PWM_IN 19
#define RAD_PIN_PWM_OUT 18
#define RAD_PIN_PPM_IN 14

#define RAD_PIN_DOUT                                                                               \
    { 33, 25, 26, 27, 23 }

#define RAD_HAS_DISPLAY 0
#define RAD_PIN_STATUS_LED 5

#endif // RAD_BOARD_COMPACT
