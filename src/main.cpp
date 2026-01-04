#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_MAX31865.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ================= PINOUT =================
#define PIN_MAX_CS     5
#define PIN_MAX_MOSI   23
#define PIN_MAX_MISO   19
#define PIN_MAX_SCK    18

#define PIN_SSR        26
#define PIN_STEAM_SENSE 27
#define PIN_CFG_BTN     0

#define I2C_SDA 21
#define I2C_SCL 22

// ================= PT100 =================
#define RREF 430.0
#define RNOM 100.0

Adafruit_MAX31865 max31865(
  PIN_MAX_CS, PIN_MAX_MOSI, PIN_MAX_MISO, PIN_MAX_SCK
);

// ================= OLED =================
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// ================= PID =================
float Kp_brew = 2.5, Ki_brew = 0.08, Kd_brew = 12.0;
float Kp_steam = 2.0, Ki_steam = 0.10, Kd_steam = 10.0;

float sp_brew  = 93.0;
float sp_steam = 135.0;
float setpoint = 93.0;

float integral = 0;
float prevTemp = 0;

#define PID_DT_MS 250
#define SSR_WINDOW_MS 1000

unsigned long lastPID = 0;
unsigned long windowStart = 0;

// ================= MODE =================
enum Mode { MODE_BREW, MODE_STEAM };
Mode mode = MODE_BREW;

bool steamReady = false;

// ================= FILTER =================
#define AVG_N 8
float buf[AVG_N];
int bi = 0;

float avg(float v) {
  buf[bi++] = v;
  if (bi >= AVG_N) bi = 0;
  float s = 0;
  for (int i=0;i<AVG_N;i++) s += buf[i];
  return s / AVG_N;
}

// ================= WIFI / MQTT =================
Preferences prefs;
WiFiClient wifi;
PubSubClient mqtt(wifi);
AsyncWebServer server(80);

String mqttHost;
int mqttPort;
bool serviceMode = false;

// ================= OLED =================
void oled(float t, float out) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (mode == MODE_STEAM) {
    display.setCursor(0,0);
    display.println("STEAM");
    display.setCursor(0,9);
    display.println(steamReady ? "Habositasra kesz" : "Melegites...");
  } else {
    display.setCursor(0,0);
    display.println("ESPRESSO");
    display.setCursor(0,9);
    display.println("Stabil homerseklet");
  }

  display.setCursor(0,18);
  display.printf("T:%.1fC SP:%.1fC", t, setpoint);
  display.setCursor(0,26);
  display.printf("OUT: %.0f%%", out);

  display.display();
}

// ================= PID =================
float pid(float temp) {
  float Kp = (mode==MODE_STEAM) ? Kp_steam : Kp_brew;
  float Ki = (mode==MODE_STEAM) ? Ki_steam : Ki_brew;
  float Kd = (mode==MODE_STEAM) ? Kd_steam : Kd_brew;

  float dt = PID_DT_MS / 1000.0;
  float err = setpoint - temp;

  float P = Kp * err;
  integral += err * dt;
  integral = constrain(integral, -50, 50);
  float I = Ki * integral;

  float D = -Kd * ((temp - prevTemp) / dt);

  prevTemp = temp;

  if (serviceMode && mqtt.connected()) {
    mqtt.publish("gaggia/pid/p", String(P,2).c_str());
    mqtt.publish("gaggia/pid/i", String(I,2).c_str());
    mqtt.publish("gaggia/pid/d", String(D,2).c_str());
  }

  return constrain(P + I + D, 0, 100);
}

// ================= SSR =================
void ssr(float out) {
  unsigned long now = millis();
  if (now - windowStart >= SSR_WINDOW_MS) {
    windowStart += SSR_WINDOW_MS;
  }
  unsigned long onTime = out / 100.0 * SSR_WINDOW_MS;
  digitalWrite(PIN_SSR, (now - windowStart < onTime));
}

// ================= MQTT =================
void mqttCb(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i=0;i<len;i++) msg += (char)payload[i];
  msg.trim();

  if (String(topic) == "gaggia/service") {
    serviceMode = (msg == "on");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_SSR, OUTPUT);
  pinMode(PIN_STEAM_SENSE, INPUT_PULLUP);
  pinMode(PIN_CFG_BTN, INPUT_PULLUP);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  max31865.begin(MAX31865_3WIRE);

  // ---- WiFi (hardcode vagy később web UI)
  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED) delay(500);

  mqtt.setServer("MQTT_IP", 1883);
  mqtt.setCallback(mqttCb);
  mqtt.connect("gaggia-pid");

  mqtt.subscribe("gaggia/service");
}

void loop() {
  mqtt.loop();

  bool steam = (digitalRead(PIN_STEAM_SENSE) == LOW);
  Mode newMode = steam ? MODE_STEAM : MODE_BREW;

  if (newMode != mode) {
    mode = newMode;
    setpoint = (mode==MODE_STEAM) ? sp_steam : sp_brew;
    integral = 0;
  }

  float t = max31865.temperature(RNOM, RREF);
  float ta = avg(t);

  steamReady = (mode==MODE_STEAM && ta >= setpoint - 1.0);

  static float out = 0;
  if (millis() - lastPID > PID_DT_MS) {
    lastPID = millis();
    out = pid(ta);
    if (mqtt.connected()) {
      mqtt.publish("gaggia/boiler/temp", String(ta,2).c_str());
      mqtt.publish("gaggia/heater/output", String(out,1).c_str());
    }
  }

  ssr(out);
  oled(ta, out);

  delay(20);
}
