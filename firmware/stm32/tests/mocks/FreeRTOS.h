#pragma once

#include <cstdint>

using BaseType_t = int;
using UBaseType_t = unsigned int;
using TickType_t = unsigned long;

static constexpr TickType_t portMAX_DELAY = 0xFFFFFFFFUL;

using SemaphoreHandle_t = void*;

inline SemaphoreHandle_t xSemaphoreCreateMutex() { return reinterpret_cast<SemaphoreHandle_t>(1); }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return 1; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return 1; }
