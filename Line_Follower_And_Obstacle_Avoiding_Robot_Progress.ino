//LINE FOLLOWING + OBSTACLE AVOIDANCE (YET TO ADD THE RTC MODULE TO START THE BOT)

#define enA 10  
#define in1 9   
#define in2 8   
#define in3 7    
#define in4 6  
#define enB 5   

#define L_S A0  //ir sensor Left
#define R_S A1  //ir sensor Right

#define echo A2     
#define trigger A3  

#define servo A6

int Set = 15;
int distance_L, distance_F, distance_R;

// Motor calibration offsets
#define RIGHT_MOTOR_OFFSET 0
#define LEFT_MOTOR_OFFSET 10
#define MOTOR_SPEED 100

void setup() {
  Serial.begin(9600);

  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);

  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  analogWrite(enA, MOTOR_SPEED);
  analogWrite(enB, MOTOR_SPEED);

  pinMode(servo, OUTPUT);

  for (int angle = 70; angle <= 140; angle += 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 140; angle >= 0; angle -= 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 0; angle <= 70; angle += 5) {
    servoPulse(servo, angle);
  }

  distance_F = Ultrasonic_read();
  delay(500);
}

void loop() {
  distance_F = Ultrasonic_read();
  Serial.print("D F="); Serial.println(distance_F);

  int rightIRSensorValue = digitalRead(R_S);
  int leftIRSensorValue = digitalRead(L_S);

  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
    if (distance_F > Set) {
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);  
    }
    else {
      Check_side();
    }
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    rotateMotor(0, MOTOR_SPEED);  
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    rotateMotor(MOTOR_SPEED, 0);  
  }
  else {
    rotateMotor(0, 0);  // Stop
  }

  delay(10);
}

//motor control
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {

  if (rightMotorSpeed > 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (rightMotorSpeed < 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  // Left motor control (REVERSED)
  if (leftMotorSpeed > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (leftMotorSpeed < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(enB, constrain(abs(rightMotorSpeed) + RIGHT_MOTOR_OFFSET, 0, 255));
  analogWrite(enA, constrain(abs(leftMotorSpeed) + LEFT_MOTOR_OFFSET, 0, 255));
}

void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500;
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50);
}

long Ultrasonic_read() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn(echo, HIGH);
  return time / 29 / 2;
}

void compareDistance() {
  if (distance_L > distance_R) {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);  // Turn left
    delay(500);
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);   // Forward
    delay(600);
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);  // Turn right
    delay(500);
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);   // Forward
    delay(600);
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);  // Turn right
    delay(400);
  }
  else {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);  // Turn right
    delay(500);
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);    // Forward
    delay(600);
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);   // Turn left
    delay(500);
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);    // Forward
    delay(600);
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);   // Turn left
    delay(400);
  }
}

void Check_side() {
  rotateMotor(0, 0);  // Stop
  delay(100);
  
  for (int angle = 70; angle <= 140; angle += 5) {
    servoPulse(servo, angle);
  }
  delay(300);
  distance_R = Ultrasonic_read();
  Serial.print("D R="); Serial.println(distance_R);
  delay(100);
  
  for (int angle = 140; angle >= 0; angle -= 5) {
    servoPulse(servo, angle);
  }
  delay(500);
  distance_L = Ultrasonic_read();
  Serial.print("D L="); Serial.println(distance_L);
  delay(100);
  
  for (int angle = 0; angle <= 70; angle += 5) {
    servoPulse(servo, angle);
  }
  delay(300);
  compareDistance();
}