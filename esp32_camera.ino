#include <Indoor-fire_ML_model_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <ArduinoJson.h>

// --- AI Thinker Camera Pin Configuration ---
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

// -------- Sensor Pins ----------

#define MQ5_PIN 4
#define MQ7_ANALOG_PIN 15
#define FLAME_PIN 14
#define I2C_SDA 13
#define I2C_SCL 16
#define LED 2

// -------- Sensor Thresholds ----------
const int MQ2_THRESHOLD = 2000;
const int MQ5_THRESHOLD = 600;
const int MQ7_THRESHOLD = 1800;
const int TEMP_THRESHOLD = 60;
const int HUM_THRESHOLD = 30;
const int RES_THRESHOLD = 15000;

// -------- Heater Cycle Timing for MQ-7 ----------
unsigned long heaterCycleDuration = 120000;
unsigned long lowVoltagePhaseDuration = 60000;
unsigned long cycleStartTime = 0;
unsigned long lastIntegrationTime = 0;
unsigned long integrationInterval = 1000;

unsigned long mq7Sum = 0;
int mq7Samples = 0;
int mq7IntegratedValue = 0;

// Adafruit_MLX90614 mlx = Adafruit_MLX90614();
// Adafruit_BME680 bme;

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,
  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size = FRAMESIZE_QVGA,
  .jpeg_quality = 12,
  .fb_count = 1,
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Edge Impulse + Gas Sensor Fire Detection");

  pinMode(LED, OUTPUT);
  pinMode(MQ5_PIN, INPUT);
  pinMode(MQ7_ANALOG_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);

  cycleStartTime = millis();
  if (!ei_camera_init()) {
    ei_printf("Failed to initialize Camera!\r\n");
  } else {
    ei_printf("Camera initialized\r\n");
  }
  ei_printf("\nStarting continuous inference in 2 seconds...\n");
  ei_sleep(2000);
}

void loop() {
  // ---------- Sensor Readings ----------
  unsigned long now = millis();
  unsigned long cycleElapsed = (now - cycleStartTime) % heaterCycleDuration;
  float fire_prob = 0.0;
  float safe_prob = 0.0;

  int count = 0;
  int mq2Value  ;
  int mq5Value = analogRead(MQ5_PIN);
  int flameValue = digitalRead(FLAME_PIN);  // <-- Read flame sensor

  if (cycleElapsed < lowVoltagePhaseDuration && now - lastIntegrationTime >= integrationInterval) {
    int reading = analogRead(MQ7_ANALOG_PIN);
    mq7Sum += reading;
    mq7Samples++;
    lastIntegrationTime = now;
  } else if (mq7Samples > 0) {
    mq7IntegratedValue = mq7Sum / mq7Samples;
    mq7Sum = 0;
    mq7Samples = 0;
  }

  // ---------- Print Readings ----------
  bool recieved = false;
  bool temp = false, hum = false, res = false;
  Serial.print("Received from ESP32: ");
  StaticJsonDocument<200> doc;
    
    String json;
    if (Serial.available()) {
      json = Serial.readStringUntil('\n');
      Serial.println(json);
      recieved = true;
      temp = false, hum = false, res = false;
    }
    DeserializationError error = deserializeJson(doc, json);
  
  if (error)
    recieved = false;
  
  temp = false;
  hum = false;
  res = false;
  float temperature = doc["temp"];
  float humidity = doc["humidity"];
  long gas = doc["gas"];
  float mlxObj = doc["mlx_obj"];
  float ambientTemp = doc["mlx_amb"];
  mq2Value = doc["mq2"];

  Serial.println("\n--- Gas Sensor Readings ---");
  Serial.print("MQ2 (Smoke): ");
  Serial.println(mq2Value);
  Serial.print("MQ5 (LPG): ");
  Serial.println(mq5Value);
  Serial.print("MQ7 (CO avg): ");
  Serial.println(mq7IntegratedValue);
  Serial.print("Flame Sensor: ");
  Serial.println(flameValue == LOW ? "🔥 Detected" : "No flame");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Gas Level: ");
  Serial.println(gas);

  Serial.print("MLX Object Temperature: ");
  Serial.print(mlxObj);
  Serial.println(" °C");

  Serial.print("Ambient Temperature: ");
  Serial.print(ambientTemp);
  Serial.println(" °C");


  // ---------- Fire Detection ----------
  bool smokeDetected = mq2Value > MQ2_THRESHOLD;
  bool lpgDetected = mq5Value > MQ5_THRESHOLD;
  bool coDetected = mq7IntegratedValue > MQ7_THRESHOLD;
  bool flameDetected = (flameValue == LOW);
  if (recieved) {
    temp = ((temperature > TEMP_THRESHOLD) || (ambientTemp > TEMP_THRESHOLD)) ? true : false;
    hum = (humidity < HUM_THRESHOLD);
    res = gas < RES_THRESHOLD;
    recieved = false;
  } else {
    temp = false;
    hum = false;
    res = false;
  }

  
  if ((smokeDetected || lpgDetected || flameDetected || temp || hum || res)) {
    Serial.println("🔥🔥🔥 Fire Possibly Detected by Sensors! 🔥🔥🔥");
    if (temp || hum || res)
      Serial.println("From the other one");
  }

  // ---------- Run AI Inference ----------
  if (ei_sleep(5) != EI_IMPULSE_OK) return;

  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
  if (!snapshot_buf) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    return;
  }

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  ei_printf("Predictions:\n");
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (i==0) {
      fire_prob = result.classification[i].value;
    } else if (i==1) {
      safe_prob = result.classification[i].value + result.classification[i+1].value;
    }
  }
  ei_printf("🔥 Fire probability: %.5f\n", fire_prob);
  ei_printf("✅ Non-fire probability: %.5f\n", safe_prob);

  if (fire_prob > 0.7) {
    Serial.println("🔥🔥🔥 Camera Detected Fire 🔥🔥🔥");
  } else {
    Serial.println("✅ Environment considered safe");
  }
  if((smokeDetected && flameDetected &&temp) || (fire_prob > 0.7))
    digitalWrite(LED,HIGH);
  else
    digitalWrite(LED,LOW);
  free(snapshot_buf);
}

float getValue(String data, String label) {
  int start = data.indexOf(label);
  if (start == -1) return -1;  // Label not found
  start += label.length();
  int end = data.indexOf(',', start);
  if (end == -1) end = data.length();
  String valueStr = data.substring(start, end);
  return valueStr.toFloat();
}


bool ei_camera_init(void) {
  if (is_initialised) return true;

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, 0);
  }

  is_initialised = true;
  return true;
}

void ei_camera_deinit(void) {
  esp_camera_deinit();
  is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(fb);

  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }

  if (img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS || img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      snapshot_buf,
      img_width,
      img_height);
  }

  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  for (size_t i = 0; i < length; i++) {
    out_ptr[i] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
    pixel_ix += 3;
  }
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif


