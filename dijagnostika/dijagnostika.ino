#include <XIAO-Objekti-2_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include <string.h>

void *ei_malloc(size_t size) {
  void *p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM);
  if (p == NULL) p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_DEFAULT);
  return p;
}

void *ei_calloc(size_t nitems, size_t size) {
  size_t total = nitems * size;
  void *p = ei_malloc(total);
  if (p != NULL) memset(p, 0, total);
  return p;
}

void ei_free(void *ptr) {
  if (ptr != NULL) heap_caps_aligned_free(ptr);
}

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

#define RAW_COLS          240
#define RAW_ROWS          240
#define BYTES_PO_PIKSELU  3

static uint8_t *snapshot_buf = NULL;

static const char b64tab[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void printBase64(const uint8_t *data, uint32_t len) {
  uint32_t i = 0;
  int lineLen = 0;
  while (i + 2 < len) {
    uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i+1] << 8) | (uint32_t)data[i+2];
    Serial.write(b64tab[(n >> 18) & 63]);
    Serial.write(b64tab[(n >> 12) & 63]);
    Serial.write(b64tab[(n >>  6) & 63]);
    Serial.write(b64tab[ n        & 63]);
    i += 3;
    lineLen += 4;
    if (lineLen >= 76) { Serial.println(); lineLen = 0; }
  }
  uint32_t rem = len - i;
  if (rem == 1) {
    uint32_t n = (uint32_t)data[i] << 16;
    Serial.write(b64tab[(n >> 18) & 63]);
    Serial.write(b64tab[(n >> 12) & 63]);
    Serial.write('='); Serial.write('=');
  } else if (rem == 2) {
    uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i+1] << 8);
    Serial.write(b64tab[(n >> 18) & 63]);
    Serial.write(b64tab[(n >> 12) & 63]);
    Serial.write(b64tab[(n >>  6) & 63]);
    Serial.write('=');
  }
  Serial.println();
}

static bool cameraInit() {
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

  if (esp_camera_init(&config) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    if (s->id.PID == OV3660_PID) s->set_vflip(s, 1);
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
    s->set_ae_level(s, -2);
    s->set_gainceiling(s, GAINCEILING_2X);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
  }
  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * BYTES_PO_PIKSELU;
  size_t out_ix = 0;
  for (size_t i = 0; i < length; i++) {
    out_ptr[out_ix++] = (snapshot_buf[pixel_ix] << 16)
                      + (snapshot_buf[pixel_ix + 1] << 8)
                      +  snapshot_buf[pixel_ix + 2];
    pixel_ix += BYTES_PO_PIKSELU;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(1500);

  if (!psramFound()) {
    Serial.println("#ERR PSRAM");
    while (1) delay(1000);
  }

  snapshot_buf = (uint8_t *)ps_malloc(RAW_COLS * RAW_ROWS * BYTES_PO_PIKSELU);
  if (snapshot_buf == NULL) {
    Serial.println("#ERR bafer");
    while (1) delay(1000);
  }

  if (!cameraInit()) {
    Serial.println("#ERR kamera");
    while (1) delay(1000);
  }

  Serial.println("#READY");
}

void loop() {
  if (Serial.available() <= 0) { delay(10); return; }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "ping") { Serial.println("#READY"); return; }
  if (cmd != "cap") return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb) { Serial.println("#ERR kadar"); return; }

  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(fb);
  if (!ok) { Serial.println("#ERR rgb888"); return; }

  ei::image::processing::crop_and_interpolate_rgb888(
      snapshot_buf, RAW_COLS, RAW_ROWS,
      snapshot_buf, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data     = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);

  if (r != EI_IMPULSE_OK) {
    Serial.printf("#ERR klasifikator %d\n", r);
    return;
  }

  Serial.printf("#RES %.3f %.3f %.3f\n",
                result.classification[0].value,
                result.classification[1].value,
                result.classification[2].value);

  uint32_t n = (uint32_t)EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;
  Serial.printf("#START %u %u %u\n", n,
                (unsigned)EI_CLASSIFIER_INPUT_WIDTH,
                (unsigned)EI_CLASSIFIER_INPUT_HEIGHT);
  printBase64(snapshot_buf, n);
  Serial.println("#END");
}