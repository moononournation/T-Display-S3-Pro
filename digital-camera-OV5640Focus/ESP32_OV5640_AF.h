/*
  ESP32_OV5640_AF.h - Library for OV5640 Auto Focus (ESP32 Camera)
  Created by Eric Nam, December 08, 2021.
  Released into the public domain.
*/

#ifndef ESP32_OV5640_AF_h
#define ESP32_OV5640_AF_h

#include <Arduino.h>
#include "ESP32_OV5640_cfg.h"
#include "esp_camera.h"

class OV5640 {
private:
  sensor_t* sensor;
  bool isOV5640;

public:
  OV5640();
  bool start(sensor_t* _sensor);
  int8_t focusInit();
  int8_t singleFocus();
  int8_t autoFocusMode();
  int8_t getFWStatus();
};

#endif