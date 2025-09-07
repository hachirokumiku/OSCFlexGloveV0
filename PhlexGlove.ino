/*
  OSC Flex Glove for Wemos D1 Mini

  This sketch reads data from multiple sensors on a flex glove and sends it
  as Open Sound Control (OSC) messages over a Wi-Fi network.

  Hardware components:
  - Wemos D1 Mini (ESP8266)
  - 74HC4051 8-channel Analog Multiplexer (for flex sensors and joystick)
  - BMI270 IMU (for motion control)
  - XY Joystick
  - Two push buttons

  Required Libraries:
  - ESP8266WiFi
  - WiFiUdp
  - OSC-for-Arduino (from GitHub: https://github.com/CNMAT/OSC)
  - SparkFun_BMI270_Arduino_Library (from Arduino Library Manager)
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

// === Wi-Fi Settings ===
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// === OSC Settings ===
const IPAddress outIp(192, 168, 1, 100); // The IP address of your OSC receiver
const unsigned int outPort = 8000;      // The port of your OSC receiver
WiFiUDP Udp;

// === Sensor Pin Definitions ===

// 74HC4051 Analog Multiplexer
// The A0 pin on the Wemos is the only analog input.
// This is connected to the Z pin on the multiplexer.
const int MUX_Z_PIN = A0;
const int MUX_S0_PIN = D5;  // Select pin S0
const int MUX_S1_PIN = D6;  // Select pin S1
const int MUX_S2_PIN = D7;  // Select pin S2

const int NUM_FLEX_SENSORS = 5; // Using 5 flex sensors for a single hand
const int JOY_X_CHANNEL = 5;    // Mux channel for Joystick X
const int JOY_Y_CHANNEL = 6;    // Mux channel for Joystick Y

// Buttons
const int BTN1_PIN = D3;
const int BTN2_PIN = D4;

// BMI270 IMU
// Uses the default I2C pins for Wemos D1 Mini
const int IMU_SDA = D2; // GPIO4
const int IMU_SCL = D1; // GPIO5
BMI270 imu;

// === Global Variables ===
int flexValues[NUM_FLEX_SENSORS];
int joyXValue, joyYValue;
int btn1State, btn2State;
int previousBtn1State = 0;
int previousBtn2State = 0;

unsigned long previousMillis = 0;
const long interval = 50; // Update interval in milliseconds

// === MUX Helper Function ===
void setMuxChannel(int channel) {
  // Set the select pins based on the channel number
  digitalWrite(MUX_S0_PIN, bitRead(channel, 0));
  digitalWrite(MUX_S1_PIN, bitRead(channel, 1));
  digitalWrite(MUX_S2_PIN, bitRead(channel, 2));
}

void setup() {
  Serial.begin(115200);

  // Set up MUX pins
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(MUX_S2_PIN, OUTPUT);
  pinMode(MUX_Z_PIN, INPUT);

  // Set up Button pins
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize BMI270
  // Wire.begin(SDA_PIN, SCL_PIN) can be used to override the default I2C pins.
  // We are using the default D1 and D2, so we don't need to specify pins.
  Wire.begin();
  if (imu.beginI2C() == false) {
    Serial.println("BMI270 not detected. Check wiring.");
    while (1);
  }
  Serial.println("BMI270 detected.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Send data at a fixed interval to prevent network flooding
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // === Read Flex Sensors ===
    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      setMuxChannel(i);
      delay(1); // Small delay for the MUX to settle
      flexValues[i] = analogRead(MUX_Z_PIN);

      // Create and send OSC message
      OSCMessage msg("/glove/flex");
      msg.add(i); // Add the sensor index
      msg.add((float)flexValues[i] / 1023.0); // Add the normalized value (0.0 to 1.0)
      Udp.beginPacket(outIp, outPort);
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
    }

    // === Read Joystick ===
    setMuxChannel(JOY_X_CHANNEL);
    delay(1);
    joyXValue = analogRead(MUX_Z_PIN);

    setMuxChannel(JOY_Y_CHANNEL);
    delay(1);
    joyYValue = analogRead(MUX_Z_PIN);

    OSCMessage joyXMsg("/glove/joystick/x");
    joyXMsg.add((float)joyXValue / 1023.0);
    Udp.beginPacket(outIp, outPort);
    joyXMsg.send(Udp);
    Udp.endPacket();
    joyXMsg.empty();

    OSCMessage joyYMsg("/glove/joystick/y");
    joyYMsg.add((float)joyYValue / 1023.0);
    Udp.beginPacket(outIp, outPort);
    joyYMsg.send(Udp);
    Udp.endPacket();
    joyYMsg.empty();

    // === Read BMI270 IMU ===
    if (imu.gyroscopeRead()) {
      OSCMessage gyroXMsg("/glove/imu/gyro/x");
      gyroXMsg.add(imu.gyroX);
      Udp.beginPacket(outIp, outPort);
      gyroXMsg.send(Udp);
      Udp.endPacket();
      gyroXMsg.empty();

      OSCMessage gyroYMsg("/glove/imu/gyro/y");
      gyroYMsg.add(imu.gyroY);
      Udp.beginPacket(outIp, outPort);
      gyroYMsg.send(Udp);
      Udp.endPacket();
      gyroYMsg.empty();

      OSCMessage gyroZMsg("/glove/imu/gyro/z");
      gyroZMsg.add(imu.gyroZ);
      Udp.beginPacket(outIp, outPort);
      gyroZMsg.send(Udp);
      Udp.endPacket();
      gyroZMsg.empty();
    }

    if (imu.accelerometerRead()) {
      OSCMessage accelXMsg("/glove/imu/accel/x");
      accelXMsg.add(imu.accelX);
      Udp.beginPacket(outIp, outPort);
      accelXMsg.send(Udp);
      Udp.endPacket();
      accelXMsg.empty();

      OSCMessage accelYMsg("/glove/imu/accel/y");
      accelYMsg.add(imu.accelY);
      Udp.beginPacket(outIp, outPort);
      accelYMsg.send(Udp);
      Udp.endPacket();
      accelYMsg.empty();

      OSCMessage accelZMsg("/glove/imu/accel/z");
      accelZMsg.add(imu.accelZ);
      Udp.beginPacket(outIp, outPort);
      accelZMsg.send(Udp);
      Udp.endPacket();
      accelZMsg.empty();
    }

    // === Read Buttons ===
    btn1State = digitalRead(BTN1_PIN);
    btn2State = digitalRead(BTN2_PIN);

    // Send OSC message only when a button state changes
    if (btn1State != previousBtn1State) {
      OSCMessage btn1Msg("/glove/button/1");
      btn1Msg.add(btn1State == LOW ? 1 : 0); // Active low
      Udp.beginPacket(outIp, outPort);
      btn1Msg.send(Udp);
      Udp.endPacket();
      btn1Msg.empty();
      previousBtn1State = btn1State;
    }

    if (btn2State != previousBtn2State) {
      OSCMessage btn2Msg("/glove/button/2");
      btn2Msg.add(btn2State == LOW ? 1 : 0); // Active low
      Udp.beginPacket(outIp, outPort);
      btn2Msg.send(Udp);
      Udp.endPacket();
      btn2Msg.empty();
      previousBtn2State = btn2State;
    }
  }
}
