#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// Global display object MUST be declared BEFORE including RoboEyes.h
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#include <DHT.h>
#define DHTPIN 2            // DHT22 sensor data pin
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#include <FluxGarage_RoboEyes.h>
#undef S                    // Remove conflicting macro from RoboEyes
roboEyes eyes;

// ----- Sensor Pin Definitions -----
// ESP32 ADC channels (range 0-4095)
#define SOIL_PIN 34         // Capacitive soil moisture sensor (analog)
#define LDR_PIN 35          // LDR sensor via voltage divider (analog)
#define TOUCH1_PIN 4        // Touch sensor 1 (digital)
#define TOUCH2_PIN 5        // Touch sensor 2 (digital)

// ----- WiFi Credentials (hardcoded) -----
const char* ssid = "Wifi";          // Replace with your WiFi SSID
const char* password = "pass";   // Replace with your WiFi password

// ----- Timing & Threshold Settings -----
const int darkThreshold = 2000;          // LDR reading threshold (0-4095); adjust as needed
unsigned long darkStartTime = 0;
bool isSleeping = false;

const int soilThreshold = 1500;          // Soil moisture raw threshold; lower means wetter soil
const float tempThreshold = 30.0;        // Temperature threshold in °C

unsigned long touchTime = 0;
const unsigned long touchDisplayDuration = 2000;  // 2 seconds for HAPPY mood after touch

// Sensor update interval (non-blocking) in ms:
const unsigned long sensorInterval = 2000;
unsigned long lastSensorUpdate = 0;

// Data sending interval:
const unsigned long sendInterval = 30000;
unsigned long lastSendTime = 0;

// ----- Server URL -----  
const char* serverURL = "https://dashboard/update"; // Replace with the URL of your dashboard

void setup() {
  Serial.begin(115200);
  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected. IP: " + WiFi.localIP().toString());
  
  // Initialize I2C (default ESP32 pins: SDA=21, SCL=22)
  Wire.begin(21, 22);
  
  // Initialize OLED display.
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Change to 0x3D if needed
    Serial.println("SSD1306 allocation failed");
    while(1);
  }
  display.clearDisplay();
  display.display();
  
  // Initialize RoboEyes on the OLED.
  eyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 30);  // 30 FPS for smooth animations
  eyes.setIdleMode(true, 5, 2);                 // Enable idle animations
  
  // Initialize DHT22 sensor.
  dht.begin();
  
  // Set sensor pins.
  pinMode(SOIL_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(TOUCH1_PIN, INPUT);
  pinMode(TOUCH2_PIN, INPUT);
}

void loop() {
  // Always update the RoboEyes animations for smooth motion.
  eyes.update();
  
  // Check if it's time to update sensor readings.
  if (millis() - lastSensorUpdate >= sensorInterval) {
    lastSensorUpdate = millis();
    
    // ---- Read Sensors ----
    float temperature = dht.readTemperature(false); // in Celsius
    float humidity = dht.readHumidity();
    int soilRaw = analogRead(SOIL_PIN);
    int ldrRaw = analogRead(LDR_PIN);
    bool touch1Active = (digitalRead(TOUCH1_PIN) == HIGH);
    bool touch2Active = (digitalRead(TOUCH2_PIN) == HIGH);
    bool anyTouchActive = touch1Active || touch2Active;
    
    // Normalize sensor values (ESP32 ADC: 0-4095)
    float soilPct = ((4095 - soilRaw) / 4095.0) * 100.0; // Lower reading => wetter soil
    float ldrPct = (ldrRaw / 4095.0) * 100.0;              // Higher reading => brighter
    
    Serial.print("Temp: "); Serial.print(temperature);
    Serial.print("°C, Hum: "); Serial.print(humidity);
    Serial.print("%, Soil: "); Serial.print(soilPct);
    Serial.print("%, LDR: "); Serial.print(ldrPct);
    Serial.print("%, Touch: "); Serial.println(anyTouchActive ? "Yes" : "No");
    
    // ---- Update RoboEyes Expression ----
    if (ldrRaw < darkThreshold) {
      if (darkStartTime == 0)
        darkStartTime = millis();
      if (millis() - darkStartTime >= 10000) {  // If dark for 10 sec, sleep
        eyes.close();
        isSleeping = true;
      }
    } else {
      darkStartTime = 0;
      if (isSleeping) {
        eyes.open();
        isSleeping = false;
      }
      if (anyTouchActive) {
        touchTime = millis();
        eyes.setMood(HAPPY);
      }
      else if (millis() - touchTime < touchDisplayDuration) {
        eyes.setMood(HAPPY);
      }
      else if (soilRaw < soilThreshold || temperature > tempThreshold) {
        eyes.setMood(TIRED);
      }
      else {
        eyes.setMood(DEFAULT);
      }
    }
    
    // ---- Send Sensor Data to Server Periodically ----
    if (millis() - lastSendTime > sendInterval) {
      sendSensorData(temperature, humidity, soilPct, ldrPct);
      lastSendTime = millis();
    }
  }
  
  // Short delay to avoid overloading the loop (but keep it smooth)
  delay(20);
}

void sendSensorData(float temp, float hum, float soil, float ldr) {
  // Construct URL with query parameters.
  String url = String(serverURL);
  url += "?device=plant1";
  url += "&temp=" + String(temp, 1);
  url += "&hum=" + String(hum, 1);
  url += "&soil=" + String(soil, 1);
  url += "&ldr=" + String(ldr, 1);
  
  Serial.println("Sending data: " + url);
  
  WiFiClientSecure client;
  client.setInsecure();  // For testing only; in production, validate the certificate
  
  HTTPClient http;
  if(http.begin(client, url)) {  // HTTPS connection
    int httpCode = http.GET();
    if(httpCode > 0) {
      Serial.println("HTTP GET code: " + String(httpCode));
      String payload = http.getString();
      Serial.println("Response: " + payload);
    } else {
      Serial.println("GET request failed: " + String(http.errorToString(httpCode).c_str()));
    }
    http.end();
  } else {
    Serial.println("Unable to connect to server");
  }
}

