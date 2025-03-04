#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ----- OLED Display Setup -----
// Define screen dimensions.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// Declare the global display object (must be declared before including RoboEyes.h).
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ----- DHT22 Sensor Setup -----
// DHT22 uses 3 pins: VCC, Data, and GND. Use a pull-up resistor (4.7KΩ to 10KΩ) on the Data pin.
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ----- Include RoboEyes Library -----
// The library uses the global "display" object. After including, we undefine the macro 'S'.
#include <FluxGarage_RoboEyes.h>
#undef S
roboEyes eyes;

// ----- Sensor Pin Definitions -----
// Capacitive soil moisture sensor (analog) on A0.
#define SOIL_PIN A0
// LDR sensor (voltage divider) on A1.
#define LDR_PIN A1
// Two capacitive touch sensors on digital pins D4 and D5.
#define TOUCH1_PIN 4
#define TOUCH2_PIN 5
// Piezo buzzer on digital pin D7.
#define BUZZER_PIN 7

// ----- Thresholds & Timing Settings -----
// LDR: Lower reading means darker; adjust by calibration.
const int darkThreshold = 400;
unsigned long darkStartTime = 0;
bool isSleeping = false;

// Soil moisture: Lower reading indicates wet soil.
const int soilThreshold = 300;
// Temperature above which the robot appears tired.
const float tempThreshold = 30.0;

// For touch: Show a happy expression for a short duration after touch.
unsigned long touchTime = 0;
const unsigned long touchDisplayDuration = 2000; // 2 seconds

// Animation settings.
const int maxFramerate = 30; // For smooth RoboEyes animations.

// For touch tone control: to play the piezo beep only once per new touch event.
bool touchTriggered = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize OLED display.
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Use 0x3D if your display uses that address.
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }
  display.clearDisplay();
  display.display();

  // Initialize DHT22 sensor.
  dht.begin();

  // Initialize RoboEyes.
  eyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, maxFramerate);
  // Enable natural idle animations (blinking/eye rolling).
  eyes.setIdleMode(true, 5, 2);

  // Configure sensor pins.
  pinMode(SOIL_PIN, INPUT);   // Soil moisture sensor.
  pinMode(LDR_PIN, INPUT);    // LDR sensor.
  pinMode(TOUCH1_PIN, INPUT); // Touch sensor 1.
  pinMode(TOUCH2_PIN, INPUT); // Touch sensor 2.
  
  // Set buzzer pin as output.
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  // ---- Read DHT22 Sensor Data ----
  float temperature = dht.readTemperature(false); // Get temperature in Celsius.
  float humidity = dht.readHumidity();              // Get humidity percentage.

  // ---- Read Other Sensors ----
  int soilValue = analogRead(SOIL_PIN);
  int ldrValue = analogRead(LDR_PIN);
  bool touch1Active = (digitalRead(TOUCH1_PIN) == HIGH);
  bool touch2Active = (digitalRead(TOUCH2_PIN) == HIGH);
  bool anyTouchActive = touch1Active || touch2Active;

  // Debug output.
  Serial.print("Temp: "); Serial.print(temperature);
  Serial.print(" °C, Humidity: "); Serial.print(humidity);
  Serial.print(" %, Soil: "); Serial.print(soilValue);
  Serial.print(", LDR: "); Serial.print(ldrValue);
  Serial.print(", Touch1: "); Serial.print(touch1Active ? "ON" : "OFF");
  Serial.print(", Touch2: "); Serial.println(touch2Active ? "ON" : "OFF");

  // ---- Determine and Update Expression ----
  // (1) Check LDR for darkness.
  if (ldrValue < darkThreshold) {
    if (darkStartTime == 0) darkStartTime = millis();
    // If the environment remains dark for 10 seconds, go to sleep.
    if (millis() - darkStartTime >= 10000) {
      eyes.close();  // Sleep animation: eyes closed.
      isSleeping = true;
    }
  } else {
    // Environment is bright; reset dark timer.
    darkStartTime = 0;
    if (isSleeping) {
      eyes.open();  // Wake up.
      isSleeping = false;
    }
    
    // (2) Check touch sensors.
    if (anyTouchActive) {
      touchTime = millis();
      eyes.setMood(HAPPY);  // Set a happy (loving) expression.
      // If this is a new touch event, trigger a sweet beep.
      if (!touchTriggered) {
        tone(BUZZER_PIN, 1000, 200); // Play a 1 kHz tone for 200 ms.
        touchTriggered = true;
      }
    } else {
      touchTriggered = false; // Reset when no touch is detected.
      // (3) If a recent touch occurred, maintain happy expression.
      if (millis() - touchTime < touchDisplayDuration) {
        eyes.setMood(HAPPY);
      }
      // (4) Otherwise, if soil is wet or temperature is high, show tired expression.
      else if (soilValue < soilThreshold || temperature > tempThreshold) {
        eyes.setMood(TIRED);
      }
      // (5) Default mood.
      else {
        eyes.setMood(DEFAULT);
      }
    }
  }
  
  // ---- Update the Animated Display ----
  eyes.update();  // Must be called frequently for smooth animations.
  
  delay(50);  // Short delay for timing.
}