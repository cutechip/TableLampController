#include <Arduino.h>
#include <U8g2lib.h>
#include <ui.h>
#include <humiture.h>
#include "rotate.h"
#include <slave.h>
#include <menu_handle.h>
#include <ESPWiFiManager.h>
#include <wifi.h>
#include "LittleFS.h"
#include <ntp.h>
#include <mqtt.h>
#include <iotClient.h>
#include <radar.h>
#include <sys_config.h>
#include <ControlCenter.h>
#include <NetData.h>

#include <Ticker.h>
Ticker myTicker;

void setup(void) {

  Serial.begin(115200);
  Serial1.begin(115200);
  LittleFS.begin();
  ui_init();
  humiture_init();
  rotate_init();
  ntp_init();
  wifi_init();
  mqtt_config_init();
  radar_init();
  read_config();
  control_default_value();

  myTicker.attach_ms(5, rotate_task);
}



void loop(void) {
  rotate_task();
  ui_show_task();
  slave_task();
  wifi_process_task();
  humidity_task();
  mqtt_task();
  control_center_task();
  net_data_task();
}

