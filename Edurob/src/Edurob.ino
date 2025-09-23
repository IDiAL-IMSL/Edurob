#include <WiFi.h>
#include "request_utils.h"
#define LOG_TAG "Edurob"
#include "logger.h"
static Logger LOG(LOG_TAG);

extern "C" void* start_webserver(void);
extern "C" void  stop_webserver(void* server);

// ==== WLAN-Zugangsdaten ====
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";



static void waitForWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  LOG.i("Connecting to WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  LOG.i("WiFi connected in %lu ms", millis() - t0);
  LOG.i("IP: %s", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_INFO);  
  LOG.setLevel(ESP_LOG_DEBUG);          

  LOG.i("Setup läuft…");
  delay(200);

  waitForWiFi();

  LOG.i("sqrt(4)    -> %.6f",  evalExpression("sqrt(4)"));   // 2.000000
  LOG.i("SQRT(9)    -> %.6f",  evalExpression("SQRT(9)"));   // 3.000000
  LOG.i("pow(2,3)   -> %.6f",  evalExpression("pow(2,3)"));  // 8.000000
  LOG.i("sin(PI/2)  -> %.6f",  evalExpression("sin(PI/2)")); // 1.000000
  LOG.i("1+2*3      -> %.6f",  evalExpression("1+2*3"));     // 7.000000
  LOG.i("tan(PI/2)  -> %.6f",  evalExpression("tan(PI/2)")); // beware near singularity

  start_webserver();
}
void loop() {

}
