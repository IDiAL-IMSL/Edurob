#pragma once
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
