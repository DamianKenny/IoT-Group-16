//RTC + SERVO TRAY + PULSE SENSOR + TEMPERATURE SENSORS + LCD (ESP AND FIREBASE YET TO FINISH)

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RtcDS3231.h>
#include <Servo.h>
#include <SoftwareSerial.h>

float getInitialTemperature();
float getKalmanFilteredTemp();
void interruptSetup();
void activateServo();
void serialOutputWhenBeatHappens();
void printTime(const RtcDateTime& dt);

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ESP Communication
SoftwareSerial espSerial(10, 4);

// RTC and Servo
RtcDS3231<TwoWire> Rtc(Wire);
Servo myservo;
const int SERVO_PIN = 13;
const int TRIGGER_HOUR = 9;
const int TRIGGER_MINUTE = 45;
const int TRAY_HOLD_TIME = 30000;
bool servoActivated = false;
unsigned long lastActivation = 0;

// Temperature Sensor
int tempPin = A1;
float tempEstimate;
const float kalmanGain = 0.05;
float tempSmooth = 0;
const int numReadings = 20;
float tempReadings[numReadings];
int tempIndex = 0;
float total = 0;

// Pulse Sensor
volatile int BPM;
volatile boolean QS = false;
int pulsePin = A0;
volatile unsigned long lastBeatTime = 0;

// Pulse ISR Variables
static int Signal, IBI = 600, lastBeatTimeISR = 0;
static int P = 512, T = 512, thresh = 525, amp = 100;
static boolean Pulse, firstBeat = true, secondBeat = false;
static int rate[10];
static unsigned long sampleCounter = 0;

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
  Wire.begin();
  
  // LCD Setup
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("BPM: --");
  lcd.setCursor(3, 1);
  lcd.print("TEMP: --C");

  // Temperature Initialization
  tempEstimate = getInitialTemperature();
  for (int i = 0; i < numReadings; i++) {
    tempReadings[i] = tempEstimate;
  }
  total = tempEstimate * numReadings;

  // Pulse Sensor Interrupt
  interruptSetup();

  // RTC Setup
  Rtc.Begin();
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true);
  #endif
  if (!Rtc.IsDateTimeValid()) {
    Rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
  }

  // Servo Initialization
  myservo.attach(SERVO_PIN);
  myservo.write(78);
  delay(500);
  myservo.detach();

  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  rotateMotor(0, 0);
}

void loop() {
  static unsigned long lastSensorUpdate = 0;
  if(millis() - lastSensorUpdate >= 200) {
    float temperature = getKalmanFilteredTemp();
    total -= tempReadings[tempIndex];
    tempReadings[tempIndex] = temperature;
    total += tempReadings[tempIndex];
    tempIndex = (tempIndex + 1) % numReadings;
    tempSmooth = total / numReadings;

    const float smoothingFactor = 0.2;
    tempSmooth = smoothingFactor * tempSmooth + (1 - smoothingFactor) * tempSmooth;

    lcd.setCursor(8, 1);
    lcd.print("    ");
    lcd.setCursor(8, 1);
    lcd.print(tempSmooth, 1);
    lcd.write(0xDF);
    lcd.print("C");

    lastSensorUpdate = millis();
  }

  if (QS) {
    serialOutputWhenBeatHappens();
    QS = false;
  }

  static unsigned long lastSend = 0;
  if(millis() - lastSend >= 1000) {
    if (BPM >= 40 && BPM <= 115) {
        espSerial.print("BPM:");
        espSerial.print(BPM);
    } else {
        espSerial.print("BPM: --");
    }
    espSerial.print(",TEMP:");
    espSerial.println(tempSmooth);
    
    RtcDateTime now = Rtc.GetDateTime();
    printTime(now);
    
    if(now.Hour() == TRIGGER_HOUR && 
       now.Minute() == TRIGGER_MINUTE && 
       !servoActivated) {
      activateServo();
      servoActivated = true;
      lastActivation = millis();
    }
    lastSend = millis();
  }

  if(servoActivated && (millis() - lastActivation > TRAY_HOLD_TIME + 5000)) {
    servoActivated = false;
    myservo.detach();
  }

// Temperature Functions
float getInitialTemperature() {
  float sum = 0;
  for(int i = 0; i < 10; i++) {
    sum += analogRead(tempPin) * 0.48828125;
    delay(50);
  }
  return sum / 10;
}

float getKalmanFilteredTemp() {
  float rawTemp = analogRead(tempPin) * 0.48828125;
  tempEstimate += kalmanGain * (rawTemp - tempEstimate);
  return tempEstimate;
}

// Pulse ISR
ISR(TIMER2_COMPA_vect) {
  Signal = analogRead(pulsePin);
  sampleCounter += 2;
  int N = sampleCounter - lastBeatTimeISR;

  if (Signal < thresh && N > (IBI/5)*3) {
    if (Signal < T) T = Signal;
  }

  if (Signal > thresh && Signal > P) P = Signal;

  if (N > 250) {
    if (Signal > thresh && !Pulse && N > (IBI/5)*3) {
      Pulse = true;
      IBI = sampleCounter - lastBeatTimeISR;
      lastBeatTimeISR = sampleCounter;
      lastBeatTime = millis();

      if (secondBeat) {
        secondBeat = false;
        for (int i = 0; i < 10; i++) rate[i] = IBI;
      }

      if (firstBeat) {
        firstBeat = false;
        secondBeat = true;
        return;
      }

      word runningTotal = 0;
      for (int i = 0; i < 9; i++) {
        rate[i] = rate[i + 1];
        runningTotal += rate[i];
      }
      rate[9] = IBI;
      runningTotal += rate[9];
      BPM = 60000/(runningTotal/10);

      if (BPM >= 40 && BPM <= 120) QS = true;
    }
  }

  if (Signal < thresh && Pulse) {
    Pulse = false;
    amp = P - T;
    thresh = amp/2 + T;
    P = T = thresh;
  }

  if (N > 2500) {
    thresh = 512;
    P = 512;
    T = 512;
    lastBeatTimeISR = sampleCounter;
    firstBeat = true;
    secondBeat = false;
  }
}

void interruptSetup() {
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22) | (1 << CS20);
  OCR2A = 249;
  TIMSK2 = (1 << OCIE2A);
  sei();
}

void activateServo() {
  Serial.println("\n=== MEDICATION TIME ===");
  myservo.attach(SERVO_PIN);
  TIMSK2 &= ~(1 << OCIE2A);

  for (int pos = 78; pos <= 170; pos++) {
    myservo.write(pos);
    delay(30);
  }
  delay(TRAY_HOLD_TIME);
  for (int pos = 170; pos >= 78; pos--) {
    myservo.write(pos);
    delay(30);
  }

  TIMSK2 |= (1 << OCIE2A);
  myservo.detach();
}

void serialOutputWhenBeatHappens() {
  if (BPM >= 40 && BPM <= 115) {
    Serial.print("BPM: ");
    Serial.println(BPM);
    lcd.setCursor(8, 0);
    lcd.print("    ");
    lcd.setCursor(8, 0);
    lcd.print(BPM);
  }
}

void printTime(const RtcDateTime& dt) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", dt.Hour(), dt.Minute(), dt.Second());
  Serial.println(buf);
}