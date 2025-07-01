//WHATS LEFT: We have yet to only add the esp 8266 to this code and integrate the values of the heart rate and temperature sensors with the mobile app using firebase.

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define BUZZER_PIN 8 
#define PULSE_TIMEOUT 3000  

LiquidCrystal_I2C lcd(0x27, 20, 4);

int tempPin = A1; 

int pulsePin = 0; 
volatile int BPM;
volatile boolean QS = false;

// Kalman Filter variables
float tempEstimate; 
float kalmanGain = 0.07;    
float tempSmooth = 0;   

const int numReadings = 10;
float tempReadings[numReadings];  
int tempIndex = 0;
float total = 0;

volatile unsigned long lastBeatTime = 0;  

void setup() 
{
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(4, 0);
  lcd.print("BPM: --");

  lcd.setCursor(3, 1);
  lcd.print("TEMP: --C");

  tempEstimate = getInitialTemperature();

  for (int i = 0; i < numReadings; i++) 
  {
    tempReadings[i] = tempEstimate; 
  }

  total = tempEstimate * numReadings;  

  interruptSetup();
}

void loop() 
{
  float temperature = getKalmanFilteredTemp();

  // Apply moving average smoothing
  total -= tempReadings[tempIndex]; 
  tempReadings[tempIndex] = temperature; 
  total += tempReadings[tempIndex]; 
  tempIndex = (tempIndex + 1) % numReadings;  
  tempSmooth = total / numReadings;

  lcd.setCursor(8, 1);
  lcd.print("    "); 
  lcd.setCursor(8, 1);
  lcd.print(tempSmooth, 1);
  lcd.print((char)223);
  lcd.print("C");

  if (QS == true) 
  { 
    serialOutputWhenBeatHappens();
    QS = false;
  }

  boolean validPulse = (lastBeatTime != 0) && ((millis() - lastBeatTime) < PULSE_TIMEOUT);

  bool tempAlert = (tempSmooth > 38 || tempSmooth < 25);
  bool bpmAlert = (BPM > 150 || (BPM < 40 && BPM != 0)) && validPulse;
  
  if (tempAlert || bpmAlert) 
  {
    tone(BUZZER_PIN, 1000, 500);
    delay(600);
    tone(BUZZER_PIN, 1000, 500);
  } 
  else 
  {
    noTone(BUZZER_PIN); 
  }

  delay(1000);
}

float getInitialTemperature() 
{
  float sum = 0;
  int numReadings = 10;
  
  for (int i = 0; i < numReadings; i++) 
  {
    sum += readRawTemperature();
    delay(50); 
  }
  
  return sum / numReadings; 
}

float getKalmanFilteredTemp() 
{
  float rawTemp = readRawTemperature();
  tempEstimate = tempEstimate + kalmanGain * (rawTemp - tempEstimate);
  return tempEstimate;
}

float readRawTemperature() 
{
  int rawValue = analogRead(tempPin);
  float voltage = rawValue * (5.0 / 1023.0);

  float temperature = (voltage * 100); 
  return temperature;
}

void serialOutputWhenBeatHappens() 
{  
  Serial.print("BPM: ");
  Serial.println(BPM);

  lcd.setCursor(8, 0);
  lcd.print("    ");
  lcd.setCursor(8, 0);
  lcd.print(BPM);
}

void interruptSetup() 
{     
  TCCR1A = 0;               
  TCCR1B = (1 << WGM12);       
  TCCR1B |= (1 << CS12);       
  OCR1A = 0x7C;                
  TIMSK1 = (1 << OCIE1A);      
  sei();
}

ISR(TIMER1_COMPA_vect) 
{  
  static int P = 512, T = 512, thresh = 525, amp = 100;
  static boolean Pulse = false, firstBeat = true, secondBeat = false;
  static int rate[10];
  static unsigned long sampleCounter = 0;

  Signal = analogRead(pulsePin);
  sampleCounter += 2;
  int N = sampleCounter - lastBeatTimeISR;

  if (Signal < thresh && N > (IBI/5)*3) 
  {      
    if (Signal < T) { T = Signal; }
  }

  if (Signal > thresh && Signal > P)
  { 
    P = Signal; 
  }                                        

  if (N > 250) 
  {                                   
    if ((Signal > thresh) && (!Pulse) && (N > (IBI/5)*3)) 
    {        
      Pulse = true;
      IBI = sampleCounter - lastBeatTimeISR;
      lastBeatTimeISR = sampleCounter;

      lastBeatTime = millis(); 
      
      if (secondBeat) 
      {                        
        secondBeat = false;                  
        for (int i = 0; i <= 9; i++) { rate[i] = IBI; }
      }
  
      if (firstBeat) 
      {                         
        firstBeat = false;                   
        secondBeat = true;                   
        sei();                               
        return;                              
      }   
    
      word runningTotal = 0;                    
      for (int i = 0; i <= 8; i++) 
      {                
        rate[i] = rate[i+1];                   
        runningTotal += rate[i];              
      }
      rate[9] = IBI;                          
      runningTotal += rate[9];                
      runningTotal /= 10;                     
      BPM = 60000 / runningTotal;               

      if (BPM >= 50 && BPM <= 150) 
      { 
        QS = true;
      }
    }
  }
  
  if (Signal < thresh && Pulse) 
  {           
    Pulse = false;                         
    amp = P - T;                           
    thresh = amp / 2 + T;                  
    P = thresh;                            
    T = thresh;                            
  }

  if (N > 2500) 
  {                            
    thresh = 512;                            
    P = 512;                                
    T = 512;                                
    lastBeatTimeISR = sampleCounter;           
    firstBeat = true;                       
    secondBeat = false;                    
  }
}
