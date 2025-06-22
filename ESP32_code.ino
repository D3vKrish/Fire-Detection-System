#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ArduinoJson.h>

// Define custom I2C pins
#define I2C_SDA 21
#define I2C_SCL 22
#define MQ2_PIN 15

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_BME680 bme;  // I2C

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C on D21 (SDA), D22 (SCL)
  pinMode(MQ2_PIN, INPUT);
  // Initialize MLX90614
  if (!mlx.begin()) {
    Serial.println("MLX90614 not found. Check wiring!");
    while (1);
  }

  // Initialize BME680
  if (!bme.begin()) {
    Serial.println("BME680 not found. Check wiring!");
    while (1);
  }

  // BME680 settings
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150); // 320°C for 150ms
}

void loop() {
  // MLX90614 readings
  int mq2Value = analogRead(MQ2_PIN);
  float objectTemp = mlx.readObjectTempC();
  float ambientTemp = mlx.readAmbientTempC();
    // BME680 reading
  if (!bme.performReading()) {
    Serial.println("\nFailed to perform BME680 reading");
    return;
  }

  StaticJsonDocument<200> doc;
  doc["temp"] = bme.temperature;
  doc["humidity"] = bme.humidity;
  doc["gas"] = bme.gas_resistance;
  doc["mlx_obj"] = mlx.readObjectTempC();
  doc["mlx_amb"] = mlx.readAmbientTempC();
  doc["mq2"] = analogRead(MQ2_PIN);
  String output;
  serializeJson(doc, output);
  Serial.println(output);
  delay(2000);
}