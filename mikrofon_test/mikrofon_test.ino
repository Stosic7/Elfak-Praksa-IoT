#include <driver/i2s.h>

#define I2S_WS      42        // PDM clock
#define I2S_SD      41        // PDM data
#define I2S_PORT    I2S_NUM_0
#define SAMPLE_RATE 16000
#define BUF_LEN     512

int16_t sBuffer[BUF_LEN];

void i2sInit() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = SAMPLE_RATE,
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

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Test mikrofona ===");
  i2sInit();
  Serial.println("I2S pokrenut. Pricaj ili tapsi.");
}

void loop() {
  size_t bytesIn = 0;
  esp_err_t res = i2s_read(I2S_PORT, &sBuffer, BUF_LEN * sizeof(int16_t), &bytesIn, portMAX_DELAY);

  if (res == ESP_OK) {
    int samples = bytesIn / sizeof(int16_t);
    long sum = 0;
    for (int i = 0; i < samples; i++) sum += abs(sBuffer[i]);
    int level = sum / samples;

    // vizuelni indikator jacine
    Serial.printf("%5d |", level);
    int bars = level / 100;
    if (bars > 50) bars = 50;
    for (int i = 0; i < bars; i++) Serial.print("#");
    Serial.println();
  }
  delay(50);
}