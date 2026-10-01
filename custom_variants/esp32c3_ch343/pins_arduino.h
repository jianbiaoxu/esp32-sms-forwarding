#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>

// 该设备无板载 LED；UART0 通过开发板左侧物理针脚 08/09 引出，实际对应
// ESP32-C3 GPIO21/GPIO20，固件不把 UART0 引脚当作 LED 或其他控制脚使用。
static const uint8_t LED_BUILTIN = 255;
#define BUILTIN_LED LED_BUILTIN
#define RGB_BUILTIN LED_BUILTIN

static const uint8_t BOOT_BUILTIN = 9;

// 该设备针脚图：物理针脚 08/09 为 UART0 TX/RX，实际 GPIO 为 21/20；
// UART1 TX/RX = GPIO0/GPIO1。
static const uint8_t TX = 21;
static const uint8_t RX = 20;

static const uint8_t SDA = 4;
static const uint8_t SCL = 5;

static const uint8_t SS   = 7;
static const uint8_t MOSI = 3;
static const uint8_t MISO = 10;
static const uint8_t SCK  = 2;

static const uint8_t A0 = 0;
static const uint8_t A1 = 1;
static const uint8_t A2 = 2;
static const uint8_t A3 = 3;
static const uint8_t A4 = 4;
static const uint8_t A5 = 5;

#endif /* Pins_Arduino_h */
