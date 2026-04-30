#pragma once

// Project-local TFT_eSPI setup for ESP32-S3 + ILI9341 over HSPI (SPI3).
// TFT_eSPI auto-includes this file when present.
#define USER_SETUP_ID 70

#define ILI9341_DRIVER

#define TFT_CS   7
#define TFT_MOSI 21
#define TFT_SCLK 12
#define TFT_MISO 11

#define TFT_DC   6
#define TFT_RST  -1

// Force HSPI on ESP32-S3 (maps to SPI3_HOST in TFT_eSPI internals).
#define USE_HSPI_PORT

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY   6000000
#define SPI_TOUCH_FREQUENCY  2500000
