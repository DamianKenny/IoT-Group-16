// WHAT IS LEFT: have to add another ULTRASONIC sensor make the robot follow the line back to its original starting point.

#include <Wire.h>
#include <RTClib.h>
#include <Servo.h>

RTC_DS3231 rtc;
Servo trayServo; 
Servo usServo;  

// Medication Tray pin config
const int TRAY_SERVO_PIN = 13;
const int MEDICATION_HOUR = 17;     // Set medication hour
const int MEDICATION_MINUTE = 55;   // Set medication minute
const int TRAY_HOLD_TIME = 15000;   // 15 seconds
bool trayActivated = false;
unsigned long lastTrayActivation = 0;

// Bot Activation Configuration
const int BOT_HOUR = 17;            // Set bot start hour
const int BOT_MINUTE = 50;           // Set bot start minute
bool botActive = false;
bool botTriggered = false;          
int lastDay = -1;                   

// Line Following + Obstacle Avoidance pin config
#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5
#define L_S A0  // Left IR sensor
#define R_S A1  // Right IR sensor
#define echoPin A2
#define trigPin A3
#define US_SERVO_PIN A6  

int Set = 15; 
int distance_L, distance_F, distance_R;

// Motor calibration
#define RIGHT_MOTOR_OFFSET 0
#define LEFT_MOTOR_OFFSET 10
#define MOTOR_SPEED 100

void setup()
 {
  Serial.begin(115200);
  
  // Initialize RTC
  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
  if (rtc.lostPower())
  {
    Serial.println("RTC reset to compile time");
    rtc.adjust(DateTime(F(_DATE), F(TIME_)));
  }

  // Initialize Medication Tray Servo
  trayServo.attach(TRAY_SERVO_PIN);
  trayServo.write(78); // Initial position
  delay(500);
  trayServo.detach();

  // Initialize Bot Components
  pinMode(L_S, INPUT);
  pinMode(R_S, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(US_SERVO_PIN, OUTPUT);
  
  // Calibrate Ultrasonic Servo
  for (int angle = 70; angle <= 140; angle += 5)
  {
    servoPulse(US_SERVO_PIN, angle);
  }
  for (int angle = 140; angle >= 0; angle -= 5) 
  {
    servoPulse(US_SERVO_PIN, angle);
  }
  for (int angle = 0; angle <= 70; angle += 5) 
  {
    servoPulse(US_SERVO_PIN, angle);
  }
  
  distance_F = readUltrasonic();
  Serial.println("System initialized");
}

void loop() {
  DateTime now = rtc.now();
  printTime(now);
  
  // Daily reset at midnight
  if (now.day() != lastDay) 
  {
    lastDay = now.day();
    trayActivated = false;
    botTriggered = false;
    botActive = false;
    Serial.println("Daily reset triggered");
  }

  // Check medication time
  if (now.hour() == MEDICATION_HOUR && 
      now.minute() == MEDICATION_MINUTE &&
      !trayActivated) {
    activateTray();
    trayActivated = true;
    lastTrayActivation = millis();
  }

  // Reset medication flag after operation
  if (trayActivated && (millis() - lastTrayActivation > TRAY_HOLD_TIME + 30000)) 
  {
    trayActivated = false;
    trayServo.detach();
    Serial.println("Tray system reset");
  }

  // Check bot activation time
  if (now.hour() == BOT_HOUR && 
      now.minute() == BOT_MINUTE &&
      !botTriggered) {
    botActive = true;
    botTriggered = true;
    Serial.println("Bot activated!");
  }

  // Run bot functionality when active
  if (botActive) 
  {
    runBot();
  }
}

// Medication Tray Functions
void activateTray() 
{
  Serial.println("\n=== MEDICATION TIME ===");
  trayServo.attach(TRAY_SERVO_PIN);
  
  for (int pos = 78; pos <= 170; pos++) 
  {
    trayServo.write(pos);
    delay(30);
  }
  Serial.println("Tray open");
  
  delay(TRAY_HOLD_TIME);
  
  for (int pos = 170; pos >= 78; pos--) 
  {
    trayServo.write(pos);
    delay(30);
  }
  Serial.println("Tray closed");
  trayServo.detach();
}

void printTime(const DateTime& dt) 
{
  char buf[20];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
           dt.hour(), dt.minute(), dt.second());
  Serial.print("Current time: "); Serial.println(buf);
}

// Bot Functions
void runBot() {
  distance_F = readUltrasonic();
  Serial.print("Dist: "); Serial.println(distance_F);

  int rightIR = digitalRead(R_S);
  int leftIR = digitalRead(L_S);

  if (!rightIR && !leftIR) 
  {      
    if (distance_F > Set) 
    {
      moveMotors(MOTOR_SPEED, MOTOR_SPEED); 
    }
    else 
    {
      avoidObstacle();
    }
  }
  else if (rightIR && !leftIR) 
  {   
    moveMotors(0, MOTOR_SPEED);    
  }
  else if (!rightIR && leftIR) 
  {   
    moveMotors(MOTOR_SPEED, 0);   
  }
  else 
  {
    moveMotors(0, 0);              
  }
  delay(10);
}

void moveMotors(int rightSpeed, int leftSpeed) {
  // Right motor control
  if (rightSpeed > 0) 
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (rightSpeed < 0) 
  {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else 
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  // Left motor control
  if (leftSpeed > 0) 
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (leftSpeed < 0) 
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else 
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(enB, constrain(abs(rightSpeed) + RIGHT_MOTOR_OFFSET, 0, 255));
  analogWrite(enA, constrain(abs(leftSpeed) + LEFT_MOTOR_OFFSET, 0, 255));
}

void servoPulse(int pin, int angle) 
{
  int pulseWidth = (angle * 11) + 500;
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
  delay(50); 
}

long readUltrasonic() 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) / 29 / 2; // Convert to cm
}

void avoidObstacle() 
{
  moveMotors(0, 0); 
  delay(100);
  
  // Look right
  for (int angle = 70; angle <= 140; angle += 5) 
  {
    servoPulse(US_SERVO_PIN, angle);
  }
  distance_R = readUltrasonic();
  Serial.print("Right: "); Serial.println(distance_R);
  delay(100);
  
  // Look left
  for (int angle = 140; angle >= 0; angle -= 5) 
  {
    servoPulse(US_SERVO_PIN, angle);
  }
  distance_L = readUltrasonic();
  Serial.print("Left: "); Serial.println(distance_L);
  delay(100);
  
  // Return to center
  for (int angle = 0; angle <= 70; angle += 5) 
  {
    servoPulse(US_SERVO_PIN, angle);
  }
  
  // Decide avoidance direction
  if (distance_L > distance_R) 
  {
    moveMotors(-MOTOR_SPEED, MOTOR_SPEED);  
    delay(500);
    moveMotors(MOTOR_SPEED, MOTOR_SPEED);   
    delay(600);
    moveMotors(MOTOR_SPEED, -MOTOR_SPEED);  
    delay(500);
    moveMotors(MOTOR_SPEED, MOTOR_SPEED);   
    delay(600);
    moveMotors(MOTOR_SPEED, -MOTOR_SPEED);  
    delay(400);
  } 
  else 
  {
    moveMotors(MOTOR_SPEED, -MOTOR_SPEED);  
    delay(500);
    moveMotors(MOTOR_SPEED, MOTOR_SPEED);    
    delay(600);
    moveMotors(-MOTOR_SPEED, MOTOR_SPEED);   
    delay(500);
    moveMotors(MOTOR_SPEED, MOTOR_SPEED);    
    delay(600);
    moveMotors(-MOTOR_SPEED, MOTOR_SPEED);   
    delay(400);
  }
}
