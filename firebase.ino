#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD (0x27 address, 16 columns, 4 rows)
LiquidCrystal_I2C lcd(0x27, 16, 4);  // Changed to 16 columns

// Wi-Fi credentials
#define WIFI_SSID "iphone"
#define WIFI_PASSWORD "yourname"

// Firebase project credentials
#define API_KEY "AIzaSyD085Ov_q1raWevA2tC-wOt5zbPv6vGldc"
#define DATABASE_URL "https://health-assistance-robot-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Firebase user credentials
#define USER_EMAIL "damiankenny007@gmail.com"
#define USER_PASSWORD "coopertrooper123"

// Sensor pins
#define TEMP_PIN A0
#define HEART_PIN 14  // D5 on NodeMCU

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

volatile int pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
unsigned long lastSampleTime = 0;
const unsigned long sampleTime = 10000;  // 10-second sample
const unsigned long DEBOUNCE_DELAY = 150; // Minimum time between pulses (ms)

// LCD update variables
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 500; // Update LCD every 500ms
int currentBPM = 0;
float currentTemp = 0.0;

void IRAM_ATTR detectPulse() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTime > DEBOUNCE_DELAY) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000); // Allow time to open Serial Monitor

  // Initialize LCD - FIXED: Use begin() instead of init()
  lcd.begin();  // Initialize 16x4 LCD
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(1000);

  // Connect to Wi-Fi
  lcd.clear();
  lcd.print("Connecting WiFi");
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
    if (millis() - startTime > 30000) {
      lcd.setCursor(0, 1);
      lcd.print("WiFi Failed!");
      Serial.println("\n❌ Wi-Fi connection failed!");
      return;
    }
  }
  
  lcd.clear();
  lcd.print("WiFi Connected!");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  Serial.println("\n✅ Wi-Fi connected!");
  Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
  delay(2000);

  // Configure Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  lcd.clear();
  lcd.print("Firebase...");
  Serial.print("Connecting to Firebase");
  while (!Firebase.ready()) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
  }
  
  lcd.clear();
  lcd.print("Firebase Ready!");
  Serial.println("\n🔥 Firebase connected successfully as user!");
  delay(2000);

  // Setup sensors
  pinMode(TEMP_PIN, INPUT);
  pinMode(HEART_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HEART_PIN), detectPulse, RISING);
  
  lcd.clear();
}

void updateLCD() {
  lcd.clear();
  
  // Line 1: Temperature
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(currentTemp, 1);  // 1 decimal place
  lcd.print(" C");
  
  // Line 2: Heart Rate
  lcd.setCursor(0, 1);
  lcd.print("Heart: ");
  if (currentBPM > 0) {
    lcd.print(currentBPM);
    lcd.print(" bpm");
  } else {
    lcd.print("-- bpm");
  }
  
  // Line 3: Status
  lcd.setCursor(0, 2);
  if (millis() - lastSampleTime < sampleTime) {
    int elapsed = (millis() - lastSampleTime) / 1000;
    int remaining = (sampleTime / 1000) - elapsed;
    lcd.print("Next: ");
    lcd.print(remaining);
    lcd.print("s");
  } else {
    lcd.print("Calculating...");
  }
  
  // Line 4: Firebase Status
  lcd.setCursor(0, 3);
  lcd.print("FB: ");
  lcd.print(Firebase.ready() ? "Online" : "Offline");
}

void loop() {
  // Read temperature (convert to Celsius)
  int rawValue = analogRead(TEMP_PIN);
  float voltage = rawValue * (3.3 / 1024.0);
  currentTemp = voltage * 100.0;

  // Calculate heart rate every 10 seconds
  if (millis() - lastSampleTime > sampleTime) {
    // Disable interrupts briefly to safely read pulseCount
    noInterrupts();
    int currentPulseCount = pulseCount;
    pulseCount = 0;
    interrupts();
    
    // Calculate heart rate
    currentBPM = currentPulseCount * 6; // Convert to beats per minute
    
    // Reset sample timer
    lastSampleTime = millis();
    
    // Clamp heart rate to realistic values
    if (currentBPM < 0) currentBPM = 0;
    if (currentBPM > 200) currentBPM = 200;

    // Upload data to Firebase
    if (Firebase.ready()) {
      bool tempSuccess = Firebase.RTDB.setFloat(&fbdo, "/heartRate/Temperature", currentTemp);
      bool heartSuccess = Firebase.RTDB.setInt(&fbdo, "/heartRate/HeartRate", currentBPM);

      if (tempSuccess && heartSuccess) {
        Serial.printf("Data sent! Temp: %.2f C, Heart Rate: %d bpm\n", currentTemp, currentBPM);
      } else {
        Serial.printf("Error sending data: %s\n", fbdo.errorReason().c_str());
      }
    }
  }

  // Update LCD display regularly
  if (millis() - lastLCDUpdate > LCD_UPDATE_INTERVAL) {
    updateLCD();
    lastLCDUpdate = millis();
  }

  delay(100);
}