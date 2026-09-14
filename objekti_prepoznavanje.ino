#include <XIAO-Objekti-2_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include <string.h>

// Slike zauzimaju mnogo više memorije od audio zapisa.
// Pošto običan RAM na ESP32 brzo ponestane
// ove funkcije preusmeravaju alokaciju memorije za 
// Edge Impulse biblioteku direktno u PSRAM (spoljašnju memoriju od nekoliko megabajta) 
// uz poravnanje od 16 bajtova radi bržeg izvršavanja.

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

#define LED_PIN           21

#define IDX_INDEKS    0
#define IDX_NOVCANIK  1
#define IDX_PRAZNO    2

#define PRAG          0.55f
#define POTVRDE       2

#define CILJNA_SVETLINA  145
#define MAX_POJACANJE    8.0f

#define RAW_COLS          240
#define RAW_ROWS          240
#define BYTES_PO_PIKSELU  3

static uint8_t *snapshot_buf = NULL;
static bool debug_nn = false;

static int  stabilna_klasa     = -1;
static int  poslednji_kandidat = -1;
static int  brojac_potvrda     = 0;
static bool led_toggle         = false;
static float zadnje_pojacanje  = 1.0f;
static int   zadnja_svetlina   = 0;


// podešava parametre kamere
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
    s->set_ae_level(s, 0);
    s->set_gainceiling(s, GAINCEILING_16X);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_aec2(s, 1);
  }
  return true;
}

// automatska korekcija osvetljenja
static void normalizujSvetlinu(uint8_t *buf, uint32_t n) {
  uint64_t suma = 0;
  for (uint32_t i = 0; i < n; i++) suma += buf[i];
  int prosek = (int)(suma / n);
  zadnja_svetlina = prosek;

  if (prosek < 5) { zadnje_pojacanje = 1.0f; return; }

  float f = (float)CILJNA_SVETLINA / (float)prosek;
  if (f < 1.0f) f = 1.0f;
  if (f > MAX_POJACANJE) f = MAX_POJACANJE;
  zadnje_pojacanje = f;

  if (f <= 1.01f) return;

  for (uint32_t i = 0; i < n; i++) {
    int v = (int)(buf[i] * f);
    buf[i] = (v > 255) ? 255 : (uint8_t)v;
  }
}

// hvata kadar sa kamere
static bool ei_camera_capture(uint32_t out_w, uint32_t out_h, uint8_t *out_buf) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);

  fb = esp_camera_fb_get();
  if (!fb) {
    ei_printf("GRESKA: hvatanje kadra\n");
    return false;
  }

  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(fb);

  if (!ok) {
    ei_printf("GRESKA: konverzija u RGB888\n");
    return false;
  }

  if (out_w != RAW_COLS || out_h != RAW_ROWS) {
    ei::image::processing::crop_and_interpolate_rgb888(
        out_buf, RAW_COLS, RAW_ROWS,
        out_buf, out_w, out_h);
  }

  normalizujSvetlinu(out_buf, out_w * out_h * BYTES_PO_PIKSELU);
  return true;
}

// funkcija koju Edge Impulse poziva da izvuče piksele slike i preoblikuje ih u float format 
// koji neuronska mreža zahteva za inferenciju
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * BYTES_PO_PIKSELU;
  size_t out_ix = 0;

  for (size_t i = 0; i < length; i++) {
    out_ptr[out_ix++] = (snapshot_buf[pixel_ix + 2] << 16)
                      + (snapshot_buf[pixel_ix + 1] << 8)
                      +  snapshot_buf[pixel_ix];
    pixel_ix += BYTES_PO_PIKSELU;
  }
  return 0;
}

// upravljaju paljenjem, gašenjem i treperenjem LED diode
static void ledOff() { digitalWrite(LED_PIN, HIGH); }
static void ledOn()  { digitalWrite(LED_PIN, LOW);  }

static void ledPuls(int puta, int trajanje) {
  for (int i = 0; i < puta; i++) {
    ledOn();  delay(trajanje);
    ledOff(); delay(trajanje);
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  ledOff();

  ei_printf("=== Prepoznavanje objekata ===\n");

  if (!psramFound()) {
    ei_printf("GRESKA: PSRAM nije nadjen. Tools > PSRAM > OPI PSRAM\n");
    while (1) delay(1000);
  }

  snapshot_buf = (uint8_t *)ps_malloc(RAW_COLS * RAW_ROWS * BYTES_PO_PIKSELU);
  if (snapshot_buf == NULL) {
    ei_printf("GRESKA: nema mesta za bafer slike\n");
    while (1) delay(1000);
  }

  if (!cameraInit()) {
    ei_printf("GRESKA: kamera se ne inicijalizuje\n");
    while (1) delay(1000);
  }

  ei_printf("Gledam... pokazi indeks ili novcanik\n\n");
  delay(1500);
}

void loop() {
  if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH,
                         EI_CLASSIFIER_INPUT_HEIGHT,
                         snapshot_buf)) {
    delay(500);
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data     = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) {
    ei_printf("GRESKA: klasifikator (%d)\n", r);
    delay(500);
    return;
  }

  int   best_idx = 0;
  float best_val = 0.0f;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > best_val) {
      best_val = result.classification[ix].value;
      best_idx = ix;
    }
  }

  static uint32_t zadnji_ispis = 0;
  if (millis() - zadnji_ispis > 1000) {
    zadnji_ispis = millis();
    ei_printf("indeks=%.2f  novcanik=%.2f  prazno=%.2f   [svetlina %d, pojacanje %.1fx]\n",
              result.classification[IDX_INDEKS].value,
              result.classification[IDX_NOVCANIK].value,
              result.classification[IDX_PRAZNO].value,
              zadnja_svetlina, zadnje_pojacanje);
  }

  int kandidat = (best_val >= PRAG) ? best_idx : IDX_PRAZNO;

  if (kandidat == poslednji_kandidat) {
    brojac_potvrda++;
  } else {
    poslednji_kandidat = kandidat;
    brojac_potvrda = 1;
  }

  if (brojac_potvrda >= POTVRDE && kandidat != stabilna_klasa) {
    stabilna_klasa = kandidat;

    switch (stabilna_klasa) {
      case IDX_INDEKS:
        ei_printf(">>> INDEKS    (%.2f)  LED stalno\n", best_val);
        ledPuls(1, 400);
        break;
      case IDX_NOVCANIK:
        ei_printf(">>> NOVCANIK  (%.2f)  LED treperi\n", best_val);
        ledPuls(3, 120);
        break;
      default:
        ei_printf("--- prazno    (%.2f)  LED ugasena\n", best_val);
        ledOff();
        break;
    }
  }

  if (stabilna_klasa == IDX_INDEKS) {
    ledOn();
  } else if (stabilna_klasa == IDX_NOVCANIK) {
    led_toggle = !led_toggle;
    digitalWrite(LED_PIN, led_toggle ? LOW : HIGH);
  } else {
    ledOff();
  }

  delay(100);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Model nije za kameru."
#endif