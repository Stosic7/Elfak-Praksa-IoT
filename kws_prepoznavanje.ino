#define EIDSP_QUANTIZE_FILTERBANK 0

#include <XIAO-KWS-Praksa_inferencing.h>
#include <driver/i2s.h>

#define I2S_WS      42
#define I2S_SD      41
#define I2S_PORT    I2S_NUM_0

#define LED_PIN     21

#define GAIN        10        
#define PRAG        0.70f

#define IDX_KAMERA  0
#define IDX_OSTALO  1
#define IDX_STANI   2

typedef struct {
    int16_t      *buffers[2]; // Implementira ping-pong baferovanje
    uint8_t       buf_select;
    uint8_t       buf_ready;
    uint32_t      buf_count;
    uint32_t      n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static int16_t sampleBuffer[sample_buffer_size];
static bool debug_nn = false;
static volatile bool record_status = true;

static float hp_prev_in  = 0.0f;
static float hp_prev_out = 0.0f;
static const float HP_A  = 0.9764f;

static bool led_stanje = false;

// inicijalizuje I2S periferiju na ESP32 za rad sa digitalnim (PDM) mikrofonom
static void i2sInit() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);

  i2s_pin_config_t pins = {
    .mck_io_num   = I2S_PIN_NO_CHANGE,
    .bck_io_num   = I2S_PIN_NO_CHANGE,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pins);
}

// prima uzorke zvuka i puni aktivni ping-pong bafer
static void audio_inference_callback(uint32_t n_samples_in) {
  for (uint32_t i = 0; i < n_samples_in; i++) {
    inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

    if (inference.buf_count >= inference.n_samples) {
      inference.buf_select ^= 1;
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

// Neprekidno čita podatke iz I2S mikrofona preko i2s_read()
static void capture_samples(void *arg) {
  const size_t bytes_to_read = sample_buffer_size * sizeof(int16_t);
  size_t bytes_read = 0;

  while (record_status) {
    esp_err_t r = i2s_read(I2S_PORT, (void *)sampleBuffer,
                           bytes_to_read, &bytes_read, 100);

    if (r != ESP_OK || bytes_read == 0) {
      continue;
    }

    uint32_t n = bytes_read / sizeof(int16_t);

    for (uint32_t i = 0; i < n; i++) {
      float x = (float)sampleBuffer[i];
      float y = HP_A * (hp_prev_out + x - hp_prev_in);
      hp_prev_in  = x;
      hp_prev_out = y;

      int32_t v = (int32_t)(y * GAIN);
      if (v >  32767) v =  32767;
      if (v < -32768) v = -32768;
      sampleBuffer[i] = (int16_t)v;
    }

    audio_inference_callback(n);
  }
  vTaskDelete(NULL);
}

// alocira memoriju za dva ping-pong bafera potrebna za inferenciju
static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffers[0] = (int16_t *)malloc(n_samples * sizeof(int16_t));
  if (inference.buffers[0] == NULL) return false;

  inference.buffers[1] = (int16_t *)malloc(n_samples * sizeof(int16_t));
  if (inference.buffers[1] == NULL) {
    free(inference.buffers[0]);
    return false;
  }

  inference.buf_select = 0;
  inference.buf_count  = 0;
  inference.n_samples  = n_samples;
  inference.buf_ready  = 0;

  i2sInit();
  ei_sleep(100);

  record_status = true;
  xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, NULL, 10, NULL);
  return true;
}

// proverava da li je bafer spreman
static bool microphone_inference_record(void) {
  if (inference.buf_ready == 1) {
    ei_printf("Preskocen slice - obrada je sporija od snimanja\n");
  }
  while (inference.buf_ready == 0) {
    delay(1);
  }
  inference.buf_ready = 0;
  return true;
}

// konvertuje sirove 16-bitne celobrojne audio podatke iz bafera u float format koji Edge Impulse model zahteva
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset],
                        out_ptr, length);
  return 0;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  ei_printf("=== Prepoznavanje glasovnih komandi ===\n");
  ei_printf("Prozor: %d ms, slice: %d ms\n",
            (int)(EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16),
            (int)(EI_CLASSIFIER_SLICE_SIZE / 16));
  ei_printf("Klase: %d\n", EI_CLASSIFIER_LABEL_COUNT);

  run_classifier_init();

  if (!microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE)) {
    ei_printf("GRESKA: nema memorije za audio bafer\n");
    return;
  }

  ei_printf("\nSlusam... reci 'kamera' ili 'stani'\n\n");
}

void loop() {
  if (!microphone_inference_record()) {
    ei_printf("GRESKA: snimanje nije uspelo\n");
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data     = &microphone_audio_signal_get_data;

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) {
    ei_printf("GRESKA: klasifikator (%d)\n", r);
    return;
  }

  // nadji najjacu klasu
  int   best_idx = 0;
  float best_val = 0.0f;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > best_val) {
      best_val = result.classification[ix].value;
      best_idx = ix;
    }
  }

  if (best_val < PRAG || best_idx == IDX_OSTALO) {
    return;
  }

  if (best_idx == IDX_KAMERA && !led_stanje) {
    led_stanje = true;
    digitalWrite(LED_PIN, LOW);
    ei_printf(">>> KAMERA  (%.2f)  LED UPALJENA\n", best_val);
  }
  else if (best_idx == IDX_STANI && led_stanje) {
    led_stanje = false;
    digitalWrite(LED_PIN, HIGH);
    ei_printf(">>> STANI   (%.2f)  LED UGASENA\n", best_val);
  }

  static uint32_t zadnji = 0;
  if (millis() - zadnji > 3000) {
    zadnji = millis();
    ei_printf("radim... kamera=%.2f ostalo=%.2f stani=%.2f\n",
              result.classification[IDX_KAMERA].value,
              result.classification[IDX_OSTALO].value,
              result.classification[IDX_STANI].value);
  }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Model nije za mikrofon."
#endif