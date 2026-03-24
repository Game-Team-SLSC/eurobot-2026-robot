#pragma once

#include "task_context.h"

namespace robot::tasks {

void CommTask(void* parameter);
void SensorTask(void* parameter);
void ControlTask(void* parameter);
void UiTask(void* parameter);
void SafetyTask(void* parameter);

}  // namespace robot::tasks
