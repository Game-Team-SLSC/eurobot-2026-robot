#pragma once

#include <Arduino.h>
#include <cstdio>

namespace robot::logging {
    constexpr bool enabled = true;

    inline bool serialReady() {
        return static_cast<bool>(Serial);
    }

    inline void emit(const char* tag, const char* level, const char* message) {
        if (!enabled || !serialReady()) {
            return;
        }

        char buffer[256];
        if (level != nullptr) {
            snprintf(buffer, sizeof(buffer), "[%s][%s] %s", tag, level, message);
        } else {
            snprintf(buffer, sizeof(buffer), "[%s] %s", tag, message);
        }
        Serial.println(buffer);
    }

    template <typename... Args>
    inline void emitf(const char* tag, const char* level, const char* format, Args... args) {
        if (!enabled || !serialReady()) {
            return;
        }

        char buffer[196];
        snprintf(buffer, sizeof(buffer), format, args...);
        emit(tag, level, buffer);
    }

    inline void info(const char* tag, const char* message) {
        emit(tag, nullptr, message);
    }

    template <typename... Args>
    inline void infof(const char* tag, const char* format, Args... args) {
        emitf(tag, nullptr, format, args...);
    }

    inline void warn(const char* tag, const char* message) {
        emit(tag, "WARN", message);
    }

    template <typename... Args>
    inline void warnf(const char* tag, const char* format, Args... args) {
        emitf(tag, "WARN", format, args...);
    }
}
