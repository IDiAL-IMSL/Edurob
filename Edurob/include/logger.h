#pragma once
#include <cstdarg>
#include <cstdio>
#include "esp_log.h"

class Logger {
public:
  explicit Logger(const char* tag) : tag_(tag) {}

  // optional: set runtime level for THIS tag
  void setLevel(esp_log_level_t lvl) { esp_log_level_set(tag_, lvl); }

  // printf-style helpers (no macros involved)
  inline void e(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(ESP_LOG_ERROR,   fmt, ap); va_end(ap);
  }
  inline void w(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(ESP_LOG_WARN,    fmt, ap); va_end(ap);
  }
  inline void i(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(ESP_LOG_INFO,    fmt, ap); va_end(ap);
  }
  inline void d(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(ESP_LOG_DEBUG,   fmt, ap); va_end(ap);
  }
  inline void v(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(ESP_LOG_VERBOSE, fmt, ap); va_end(ap);
  }

  inline const char* tag() const { return tag_; }

private:
  const char* tag_;

  void vlog(esp_log_level_t lvl, const char* fmt, va_list ap) {
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    switch (lvl) {
      case ESP_LOG_ERROR:   ESP_LOGE(tag_, "%s", buf); break;
      case ESP_LOG_WARN:    ESP_LOGW(tag_, "%s", buf); break;
      case ESP_LOG_INFO:    ESP_LOGI(tag_, "%s", buf); break;
      case ESP_LOG_DEBUG:   ESP_LOGD(tag_, "%s", buf); break;
      default:              ESP_LOGV(tag_, "%s", buf); break;
    }
  }
};
