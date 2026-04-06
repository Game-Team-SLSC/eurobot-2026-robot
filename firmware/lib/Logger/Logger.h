#pragma once

#include <config.h>
#include <Arduino.h>

#define INFO_TAG "\033[34m"
#define WARN_TAG "\033[33m"
#define ERROR_TAG "\033[31m"
#define RESET_TAG "\033[0m"

#ifdef LOG_STATE

void loggerSetup();

#else

#define loggerSetup()

#endif

#ifdef INFO_STATE
#define info(sender, msg, ...) \
do { \
Serial.printf("%s[%s] - " msg RESET_TAG "\n", INFO_TAG, sender, ##__VA_ARGS__); \
} while (0)

#else

#define info(sender, msg, ...) do { } while (0)

#endif

#ifdef WARN_STATE

#define warn(sender, msg, ...) \
do { \
Serial.printf("%s[%s] - " msg RESET_TAG "\n", WARN_TAG, sender, ##__VA_ARGS__); \
} while (0)

#else

#define warn(sender, msg, ...) do { } while (0)

#endif

#ifdef ERROR_STATE

#define error(sender, msg, ...) \
do { \
Serial.printf("%s[%s] - " msg RESET_TAG "\n", ERROR_TAG, sender, ##__VA_ARGS__); \
} while (0)

#else

#define error(sender, msg, ...) do { } while (0)

#endif