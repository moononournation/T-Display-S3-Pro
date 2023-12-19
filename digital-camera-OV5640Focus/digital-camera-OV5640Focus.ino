/*******************************************************************************
 * T-Display-S3-Pro
 * This is a simple digital camera example
 *
 * Dependent libraries:
 * Arduino_GFX: https://github.com/moononournation/Arduino_GFX.git
 * ESP32_JPEG: https://github.com/esp-arduino-libs/ESP32_JPEG.git
 * XPowersLib: https://github.com/lewisxhe/XPowersLib.git
 *
 * Modified libraries:
 * ESP32-OV5640-AF: https://github.com/0015/ESP32-OV5640-AF.git
 ******************************************************************************/

#include <WiFi.h>

// button pins
#define BTN1_PIN 12
#define BTN2_PIN 16
#define BTN3_PIN 0

// camera
#include <esp_camera.h>
#define CAMERA_MODEL_LILYGO_T_DISPLAY_PRO // check camera_pins.h for other camera model
#include "camera_pins.h"
#define PREVIEW_QUALITY 6 // 1-63, 1 is the best
#define PREVIEW_SIZE FRAMESIZE_HVGA
#define PREVIEW_WIDTH 480
#define PREVIEW_HEIGHT 320
#define SNAP_QUALITY 6 // 1-63, 1 is the best
#define SNAP_SIZE FRAMESIZE_UXGA
// #define SNAP_SIZE FRAMESIZE_QXGA
// #define SNAP_SIZE FRAMESIZE_QSXGA

#include "ESP32_OV5640_AF.h"
OV5640 ov5640 = OV5640();
int8_t rc;

#include <SD.h>
#define SD_SCK 18
#define SD_MOSI 17
#define SD_MISO 8
#define SD_CS 14
#define SPI_LOCK xSemaphoreTake(spiLock, portMAX_DELAY)
#define SPI_UNLOCK xSemaphoreGive(spiLock)

#include <XPowersLib.h>
PowersSY6970 PMU;

#include <ESP32_JPEG_Library.h>
// Generate default configuration
jpeg_dec_config_t config = {
    .output_type = JPEG_RAW_TYPE_RGB565_BE,
    .rotate = JPEG_ROTATE_0D,
};
jpeg_dec_io_t *jpeg_io;
jpeg_dec_header_info_t *out_info;

/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/
#include <Arduino_GFX_Library.h>
#define GFX_BL 48
Arduino_DataBus *bus = new Arduino_HWSPI(9 /* DC */, 39 /* CS */, 18 /* SCK */, 17 /* MOSI */, 8 /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7796(bus, 47 /* RST */, 1 /* rotation */, true /* IPS */, 222 /* width */, 480 /* height */, 49 /* col offset 1 */, 0 /* row offset 1 */, 49 /* col offset 2 */, 0 /* row offset 2 */);
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

char tmpStr[1024];
char nextFilename[31];
uint16_t fileIdx = 0;
sensor_t *s;
camera_fb_t *fb = 0;
uint8_t *output_buf;
uint8_t *file1_buf;
size_t file1_len;
uint8_t *file2_buf;
size_t file2_len;
uint8_t *file3_buf;
size_t file3_len;

void setup()
{
  WiFi.mode(WIFI_OFF);

  Serial.begin(115200);
  // Serial.setDebugOutput(true);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);

  gfx->begin(80000000);
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  bool hasPMU = PMU.init(Wire, SIOD_GPIO_NUM, SIOC_GPIO_NUM, SY6970_SLAVE_ADDRESS);
  if (!hasPMU)
  {
    Serial.println("Failed to find Power management !");
  }
  else
  {
    // Set the charging target voltage, Range:3840 ~ 4608mV ,step:16 mV
    PMU.setChargeTargetVoltage(3856);

    // Set the precharge current , Range: 64mA ~ 1024mA ,step:64mA
    PMU.setPrechargeCurr(64);

    // Set the charging current , Range:0~5056mA ,step:64mA
    PMU.setChargerConstantCurr(320);

    // To obtain voltage data, the ADC must be enabled first
    PMU.enableADCMeasure();

    // Turn off the PMU charging indicator light
    PMU.disableStatLed();

    Serial.println("Find Power management");
  }

  if (!SD.begin(SD_CS))
  {
    Serial.println("SD Init Fail!");
  }
  else
  {
    snprintf(tmpStr, sizeof(tmpStr), "SD Card Type: %d Size: %lu MB", SD.cardType(), (long unsigned int)SD.cardSize() / 1024 / 1024);
    Serial.println(tmpStr);

    init_folder();

    xTaskCreate(
        findNextFileIdxTask,   /* Task function. */
        "findNextFileIdxTask", /* String with name of task. */
        10000,                 /* Stack size in bytes. */
        NULL,                  /* Parameter passed as input of the task */
        1,                     /* Priority of the task. */
        NULL);                 /* Task handle. */
  }

  esp_err_t err = cam_init();
  if (err != ESP_OK)
  {
    snprintf(tmpStr, sizeof(tmpStr), "Camera init failed with error 0x%x", err);
    Serial.println(tmpStr);
  }

  // drop down frame size for higher initial frame rate
  s = esp_camera_sensor_get();
  // s->set_brightness(s, 1);
  // s->set_contrast(s, 1);
  // s->set_saturation(s, 1);
  // s->set_saturation(s, 4);
  s->set_gainceiling(s, (gainceiling_t)511);
  // s->set_sharpness(s, 1);
  // s->set_sharpness(s, 3);
  s->set_aec2(s, true);
  // s->set_denoise(s, true);
  // s->set_lenc(s, true);
  // s->set_hmirror(s, true);
  // s->set_vflip(s, true);
  s->set_quality(s, PREVIEW_QUALITY);
  s->set_framesize(s, PREVIEW_SIZE);

  switch (s->id.PID)
  {
  case OV9650_PID:
    Serial.println("OV9650_PID");
    break;
  case OV7725_PID:
    Serial.println("OV7725_PID");
    break;
  case OV2640_PID:
    Serial.println("OV2640_PID");
    break;
  case OV3660_PID:
    Serial.println("OV3660_PID");
    break;
  case OV5640_PID:
    Serial.println("OV5640_PID");
    s->set_hmirror(s, true);
    ov5640.start(s);
    if (ov5640.focusInit() == 0)
    {
      Serial.println("OV5640_Focus_Init Successful!");
    }
    if (ov5640.singleFocus() == 0)
    {
      Serial.println("OV5640_Auto_Focus Successful!");
    }
    break;
  case OV7670_PID:
    Serial.println("OV7670_PID");
    break;
  case NT99141_PID:
    Serial.println("NT99141_PID");
    break;
  case GC2145_PID:
    Serial.println("GC2145_PID");
    break;
  case GC032A_PID:
    Serial.println("GC032A_PID");
    break;
  case GC0308_PID:
    Serial.println("GC0308_PID");
    break;
  case BF3005_PID:
    Serial.println("BF3005_PID");
    break;
  case BF20A6_PID:
    Serial.println("BF20A6_PID");
    break;
  case SC101IOT_PID:
    Serial.println("SC101IOT_PID");
    break;
  case SC030IOT_PID:
    Serial.println("SC030IOT_PID");
    break;
  case SC031GS_PID:
    Serial.println("SC031GS_PID");
    break;
  default:
    Serial.println("Unknown CAM PID");
  }

  // Create io_callback handle
  jpeg_io = (jpeg_dec_io_t *)calloc(1, sizeof(jpeg_dec_io_t));

  // Create out_info handle
  out_info = (jpeg_dec_header_info_t *)calloc(1, sizeof(jpeg_dec_header_info_t));

  // allocate buffers
  output_buf = (uint8_t *)heap_caps_aligned_alloc(16, 480 * 320 * 2, MALLOC_CAP_DEFAULT);
  if (!output_buf)
  {
    Serial.println("output_buf malloc failed!");
  }
  file1_buf = (uint8_t *)malloc(2560 * 1920 * 2 / 8);
  if (!file1_buf)
  {
    Serial.println("file1_buf malloc failed!");
  }
  file2_buf = (uint8_t *)malloc(2560 * 1920 * 2 / 8);
  if (!file2_buf)
  {
    Serial.println("file2_buf malloc failed!");
  }
  file3_buf = (uint8_t *)malloc(2560 * 1920 * 2 / 8);
  if (!file3_buf)
  {
    Serial.println("file3_buf malloc failed!");
  }
}

esp_err_t cam_init()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  // init with high specs to pre-allocate larger buffers
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = SNAP_SIZE;
  config.jpeg_quality = SNAP_QUALITY;
  config.fb_count = 2;

  // camera init
  return esp_camera_init(&config);
}

void init_folder()
{
  File file = SD.open("/DCIM");
  if (!file)
  {
    Serial.println("Create /DCIM");
    SD.mkdir("/DCIM");
  }
  else
  {
    Serial.println("Found /DCIM");
    file.close();
  }
  file = SD.open("/DCIM/100ESPDC");
  if (!file)
  {
    Serial.println("Create /DCIM/100ESPDC");
    SD.mkdir("/DCIM/100ESPDC");
  }
  else
  {
    Serial.println("Found /DCIM/100ESPDC");
    file.close();
  }
}

void findNextFileIdxTask(void *parameter)
{
  findNextFileIdx();
  vTaskDelete(NULL);
}

void findNextFileIdx()
{ // TODO: revise ugly code
  fileIdx++;
  File file;
  snprintf(nextFilename, sizeof(nextFilename), "/DCIM/100ESPDC/DSC%05d.JPG", fileIdx);
  file = SD.open(nextFilename);
  if (!file)
  {
    Serial.printf("Next file: %s\n", nextFilename);
    return;
  }
  else
  {
    for (int k = 1000; k <= 30000; k += 1000)
    {
      snprintf(nextFilename, sizeof(nextFilename), "/DCIM/100ESPDC/DSC%05d.JPG", fileIdx + k);
      file = SD.open(nextFilename);
      if (file)
      {
        Serial.printf("Found %s\n", nextFilename);
        file.close();
      }
      else
      {
        Serial.printf("Not found %s\n", nextFilename);
        k -= 1000;
        for (int h = 100; h <= 1000; h += 100)
        {
          snprintf(nextFilename, sizeof(nextFilename), "/DCIM/100ESPDC/DSC%05d.JPG", fileIdx + k + h);
          file = SD.open(nextFilename);
          if (file)
          {
            Serial.printf("Found %s\n", nextFilename);
            file.close();
          }
          else
          {
            Serial.printf("Not found %s\n", nextFilename);
            h -= 100;
            for (int t = 10; t <= 100; t += 10)
            {
              snprintf(nextFilename, sizeof(nextFilename), "/DCIM/100ESPDC/DSC%05d.JPG", fileIdx + k + h + t);
              file = SD.open(nextFilename);
              if (file)
              {
                Serial.printf("Found %s\n", nextFilename);
                file.close();
              }
              else
              {
                Serial.printf("Not found %s\n", nextFilename);
                t -= 10;
                for (int d = 1; d <= 10; d++)
                {
                  snprintf(nextFilename, sizeof(nextFilename), "/DCIM/100ESPDC/DSC%05d.JPG", fileIdx + k + h + t + d);
                  file = SD.open(nextFilename);
                  if (file)
                  {
                    Serial.printf("Found %s\n", nextFilename);
                    file.close();
                  }
                  else
                  {
                    Serial.printf("Next file: %s\n", nextFilename);
                    fileIdx += k + h + t + d;
                    return;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

void saveFilesTask(void *parameter)
{
  if ((file1_len > file2_len) && (file1_len > file3_len))
  {
    Serial.printf("selected first snap.\n");
    saveFile(file1_buf, file1_len);
  }
  else if (file2_len > file3_len)
  {
    Serial.printf("selected second snap.\n");
    saveFile(file2_buf, file2_len);
  }
  else
  {
    Serial.printf("selected third snap.\n");
    saveFile(file3_buf, file3_len);
  }
  gfx->fillRect(470, 0, 10, 222, BLACK);
  vTaskDelete(NULL);
}

void saveFile(uint8_t *buf, size_t len)
{
  Serial.printf("File open: %s\n", nextFilename);
  File file = SD.open(nextFilename, FILE_WRITE, true);
  if (!file)
  {
    Serial.println("SD.open() failed!");
  }
  else
  {
    if (file.write(buf, len))
    {
      Serial.printf("Save file %s: %zu\n", nextFilename, len);
    }
    else
    {
      Serial.println("Write failed!");
    }
  }
  file.close();

  findNextFileIdx();
}

void snap()
{
  gfx->fillScreen(BLACK);

  // s->set_hmirror(s, false);
  // s->set_vflip(s, true);
  s->set_quality(s, SNAP_QUALITY);
  s->set_framesize(s, SNAP_SIZE);
  // skipFrame
  // fb = esp_camera_fb_get();
  // esp_camera_fb_return(fb);
  // fb = esp_camera_fb_get();
  // esp_camera_fb_return(fb);
  // fb = esp_camera_fb_get();
  // esp_camera_fb_return(fb);
  // fb = esp_camera_fb_get();
  // esp_camera_fb_return(fb);
  // fb = esp_camera_fb_get();
  // esp_camera_fb_return(fb);

  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  file1_len = fb->len;
  memcpy(file1_buf, fb->buf, file1_len);
  Serial.printf("First snap: %d\n", file1_len);
  gfx->fillRect(470, 1, 10, 72, RED);
  esp_camera_fb_return(fb);

  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  file2_len = fb->len;
  memcpy(file2_buf, fb->buf, file2_len);
  Serial.printf("Second snap: %d\n", file2_len);
  gfx->fillRect(470, 75, 10, 72, RED);
  esp_camera_fb_return(fb);

  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  file3_len = fb->len;
  memcpy(file3_buf, fb->buf, file3_len);
  Serial.printf("Third snap: %d\n", file3_len);
  gfx->fillRect(470, 149, 10, 72, RED);
  esp_camera_fb_return(fb);
  fb = NULL;

  // s->set_hmirror(s, true);
  // s->set_vflip(s, true);
  s->set_quality(s, PREVIEW_QUALITY);
  s->set_framesize(s, PREVIEW_SIZE);

  xTaskCreate(
      saveFilesTask,   /* Task function. */
      "saveFilesTask", /* String with name of task. */
      10000,           /* Stack size in bytes. */
      NULL,            /* Parameter passed as input of the task */
      1,               /* Priority of the task. */
      NULL);           /* Task handle. */
}

void liveView()
{
  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed!");
    esp_camera_fb_return(fb);
    fb = NULL;
  }
  else
  {
    // Create jpeg_dec
    jpeg_dec_handle_t *jpeg_dec = jpeg_dec_open(&config);

    // Set input buffer and buffer len to io_callback
    jpeg_io->inbuf = fb->buf;
    jpeg_io->inbuf_len = fb->len;

    jpeg_dec_parse_header(jpeg_dec, jpeg_io, out_info);

    jpeg_io->outbuf = output_buf;

    jpeg_dec_process(jpeg_dec, jpeg_io);
    jpeg_dec_close(jpeg_dec);

    gfx->draw16bitBeRGBBitmap(-10, (222 - PREVIEW_HEIGHT) / 2, (uint16_t *)output_buf, PREVIEW_WIDTH, PREVIEW_HEIGHT);

    esp_camera_fb_return(fb);
    fb = NULL;
  }
}

void loop()
{
  rc = ov5640.getFWStatus();
  if (rc == -1)
  {
    Serial.println("Check your OV5640");
  }
  else if (rc == FW_STATUS_S_FOCUSED)
  {
    Serial.println("Focused!");
  }
  else if (rc == FW_STATUS_S_FOCUSING)
  {
    Serial.print("#");
  }

  if (digitalRead(BTN1_PIN) == LOW)
  {
    ov5640.singleFocus();
    do
    {
      liveView();
      rc = ov5640.getFWStatus();
    } while (rc != FW_STATUS_S_FOCUSED);

    Serial.println("Start snap!");
    snap();
  }
  else
  {
    liveView();
  }
}
