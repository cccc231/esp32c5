#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4;  // Number of measurements to average.
byte rates[RATE_SIZE];     // Array of heart rates.
byte rateSpot = 0;
long lastBeat = 0;  // Time at which the last beat occurred.

float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();                     // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   // Turn off Green LED
}

void loop() {
  long irValue = particleSensor.getIR();
  bool fingerOnSensor = irValue >= 50000;

  if (checkForBeat(irValue) == true) {
    // Sensed a beat.
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;                     

      // Calculate average.
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // Serial.print("IR=");
  // Serial.print(irValue);
  // Serial.print(", BPM=");
  // Serial.print(beatsPerMinute);
  // Serial.print(", Avg BPM=");
  // Serial.print(beatAvg);

  // 统一输出结构化原始心率信号，方便上位机作为 CSI 标签采集
  // 字段：HR_RAW,毫秒时间戳,IR原始值,当前BPM,平均BPM,手指是否贴合
  Serial.print("HR_RAW,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(irValue);
  Serial.print(",");
  Serial.print(beatsPerMinute, 2);
  Serial.print(",");
  Serial.print(beatAvg);
  Serial.print(",");
  Serial.println(fingerOnSensor ? 1 : 0);

  delay(40); // Roughly match CSI sample rate.
}
