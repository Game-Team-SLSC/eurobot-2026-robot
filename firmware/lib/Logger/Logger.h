#pragma once

#include <Arduino.h>
#include <config.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define INFO_TAG  "\033[34m"
#define WARN_TAG  "\033[33m"
#define ERROR_TAG "\033[31m"
#define RESET_TAG "\033[0m"

enum class LogLevel: uint8_t {
    INFO,
    WARN,
    ERROR
};

namespace robot::logger {
    void setup();
    void pushToQueue(LogLevel level, const char* sender, const char* fmt, ...);
}

#ifdef INFO_STATE
    #define info(sender, msg, ...) \
    do { \
        Serial.printf(INFO_TAG "[%s] - " msg RESET_TAG "\n", sender, ##__VA_ARGS__); \
        robot::logger::pushToQueue(LogLevel::INFO, sender, msg, ##__VA_ARGS__); \
    } while (0)
#else
    #define info(sender, msg, ...) do { } while (0)
#endif

#ifdef WARN_STATE
    #define warn(sender, msg, ...) \
    do { \
        Serial.printf(WARN_TAG "[%s] - " msg RESET_TAG "\n", sender, ##__VA_ARGS__); \
        robot::logger::pushToQueue(LogLevel::WARN, sender, msg, ##__VA_ARGS__); \
    } while (0)
#else
    #define warn(sender, msg, ...) do { } while (0)
#endif

#ifdef ERROR_STATE
    #define error(sender, msg, ...) \
    do { \
        Serial.printf(ERROR_TAG "[%s] - " msg RESET_TAG "\n", sender, ##__VA_ARGS__); \
        robot::logger::pushToQueue(LogLevel::ERROR, sender, msg, ##__VA_ARGS__); \
    } while (0)
#else
    #define error(sender, msg, ...) do { } while (0)
#endif