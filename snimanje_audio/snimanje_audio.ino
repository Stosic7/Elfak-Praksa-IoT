#include <driver/i2s.h>

#define I2S_WS       42        // PDM clock
#define I2S_SD       41        // PDM data
#define I2S_PORT     I2S_NUM_0

#define SAMPLE_RATE     16000
#define SAMPLE_BITS     16
#define RECORD_SECONDS  10
#define VOLUME_GAIN     4

static const uint32_t RECORD_BYTES =
    (uint32_t)SAMPLE_RATE * (SAMPLE_BITS / 8) * RECORD_SECONDS;

static uint8_t *recBuffer = NULL;

static const char b64tab[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

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

void recordAndSend() {
  size_t bytesRead = 0;

  Serial.println("#REC");
  delay(300);

  esp_err_t res = i2s_read(I2S_PORT, recBuffer, RECORD_BYTES,
                           &bytesRead, portMAX_DELAY);

  if (res != ESP_OK || bytesRead == 0) {
    Serial.println("#ERR snimanje neuspesno");
    return;
  }

    int16_t *s = (int16_t *)recBuffer;
  uint32_t nSamples = bytesRead / 2;

  // 1) izracunaj jednosmerni pomak PDM mikrofona
  int64_t suma = 0;
  for (uint32_t i = 0; i < nSamples; i++) suma += s[i];
  int32_t dc = (int32_t)(suma / (int64_t)nSamples);

  // 2) tek onda oduzmi pomak pa pojacaj
  for (uint32_t i = 0; i < nSamples; i++) {
    int32_t v = ((int32_t)s[i] - dc) * (1 << VOLUME_GAIN);
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    s[i] = (int16_t)v;
  }

  Serial.printf("#DC %d\n", dc);

  Serial.printf("#START %u %u\n", (unsigned)bytesRead, (unsigned)SAMPLE_RATE);
  printBase64(recBuffer, bytesRead);
  Serial.println("#END");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(1500);

  if (!psramFound()) {
    Serial.println("#ERR PSRAM nije nadjen - ukljuci Tools > PSRAM > OPI PSRAM");
    while (1) delay(1000);
  }

  recBuffer = (uint8_t *)ps_malloc(RECORD_BYTES);
  if (recBuffer == NULL) {
    Serial.println("#ERR nema dovoljno PSRAM-a");
    while (1) delay(1000);
  }

  i2sInit();
  Serial.println("#READY");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "rec") {
      recordAndSend();
    } else if (cmd == "ping") {
      Serial.println("#READY");
    }
  }
  delay(10);
}