#include "esp_camera.h"

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

static const char b64tab[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void printBase64(const uint8_t *data, uint32_t len) {
  uint32_t i = 0;
  int lineLen = 0;

  while (i + 2 < len) {
    uint32_t n = ((uint32_t)data[i] << 16) |
                 ((uint32_t)data[i + 1] << 8) |
                  (uint32_t)data[i + 2];
    Serial.write(b64tab[(n >> 18) & 63]);
    Serial.write(b64tab[(n >> 12) & 63]);
    Serial.write(b64tab[(n >>  6) & 63]);
    Serial.write(b64tab[ n        & 63]);
    i += 3;
    lineLen += 4;
    if (lineLen >= 76) {
      Serial.println();
      lineLen = 0;
    }
  }

  uint32_t rem = len - i;
  if (rem == 1) {
    uint32_t n = (uint32_t)data[i] << 16;
    Serial.write(b64tab[(n >> 18) & 63]);
    Serial.write(b64tab[(n >> 12) & 63]);
    Serial.write('=');
    Serial.write('=');
  } else if (rem == 2) {
    uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8);
    Serial.write(b64tab[(n >> 18) & 63]);
    Serial.write(b64tab[(n >> 12) & 63]);
    Serial.write(b64tab[(n >>  6) & 63]);
    Serial.write('=');
  }
  Serial.println();
}

bool cameraInit() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_240X240;
  config.jpeg_quality = 12;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL && s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  return true;
}

void captureAndSend() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("#ERR neuspesno hvatanje slike");
    return;
  }

  Serial.printf("#START %u %u %u\n",
                (unsigned)fb->len, (unsigned)fb->width, (unsigned)fb->height);
  printBase64(fb->buf, fb->len);
  Serial.println("#END");

  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(1500);

  if (!psramFound()) {
    Serial.println("#ERR PSRAM nije nadjen - ukljuci Tools > PSRAM > OPI PSRAM");
    while (1) delay(1000);
  }

  if (!cameraInit()) {
    Serial.println("#ERR kamera se ne inicijalizuje");
    while (1) delay(1000);
  }

  Serial.println("#READY");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "cap") {
      captureAndSend();
    } else if (cmd == "ping") {
      Serial.println("#READY");
    }
  }
  delay(10);
}