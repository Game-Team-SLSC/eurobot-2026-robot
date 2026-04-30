#include <Logger.h>
#include <printf.h>
#include <queues.h>

namespace robot::logger {
    void setup() {
        Serial.begin(115200);
        printf_begin();
    }

    void pushToQueue(LogLevel level, const char* sender, const char* fmt, ...) {
        char messageBuffer[128];
        va_list args;
        va_start(args, fmt);
        vsnprintf(messageBuffer, sizeof(messageBuffer), fmt, args);
        va_end(args);

        char* finalLog = (char*)malloc(128 * sizeof(char));
        
        if (finalLog) {
            snprintf(finalLog, 128, "%d[%s] %s", static_cast<int>(level), sender, messageBuffer);
            if (xQueueSend(robot::queues::logs_queue, &finalLog, 0) != pdPASS) {
                free(finalLog);
            }
        }
    }
}