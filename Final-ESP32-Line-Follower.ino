#include <Wire.h>
#include "Adafruit_TCS34725.h"

// --- Motor Driver Pins (L298N) ---
const int ENA = 13; // Left Motor Speed (PWM)
const int IN1 = 12; // Left Motor Direction 1
const int IN2 = 14; // Left Motor Direction 2
const int IN3 = 27; // Right Motor Direction 1
const int IN4 = 26; // Right Motor Direction 2
const int ENB = 25; // Right Motor Speed (PWM)

// --- Sensor Suite & Switches ---
const int SENSOR_LT = 16; 
const int SENSOR_L1 = 35; 
const int SENSOR_M  = 34; 
const int SENSOR_R1 = 39; 
const int SENSOR_RT = 17; 
const int SLIDE_SWITCH = 5; // Switch for Mode Selection

// --- Speed & Turn Settings ---
int base_speed = 130;      //CAN BE CHANGED
int turn_speed = 200;      //CAN BE CHANGED

// --- Tuning Variables ---
int overtravelDelay = 70;  //CAN BE CHANGED
int blindSpinDelay = 200;  //CAN BE CHANGED
int brakeDelay = 20;       //CAN BE CHANGED
int cooldownDelay = 250;   //CAN BE CHANGED
unsigned long lastTurnTime = 0; //CAN BE CHANGED

// --- PID Constants & Variables ---
float Kp = 0.018;  //CAN BE CHANGED
float Ki = 0.00;  //SHOULD NOT BE CHANGED
float Kd = 0.40;  //CAN BE CHANGED
float error = 0, previous_error = 0; //SHOULD NOT BE CHANGED
float P = 0, I = 0, D = 0, PID_value = 0; //SHOULD NOT BE CHANGED
int deadEndCounter = 0;  //SHOULD NOT BE CHANGED

// ====================================================================
// AI MEMORY & COLOR SENSOR OBJECTS
// ====================================================================

// Initialize the Color Sensor (FAST INTEGRATION to protect PID)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

// The Robot's "State" Locks
bool hasStarted = false; 
bool mazeCompleted = false; 

// Memory Array
char path[200];      
int pathLength = 0;  
int pathIndex = 0;   

// --- CALIBRATED COLOR DOMINANCE LOGIC ---
//RED IS START AND BLUE IS END
bool isRedLine() {
  float r, g, b;
  tcs.getRGB(&r, &g, &b);
  return (r >= 200 && r > g * 1.5 && r > b * 1.5); //CAN BE CHANGED (Use a serial monitor to know the values of the color of your start and end)
}

bool isFinishLine() { 
  float r, g, b;
  tcs.getRGB(&r, &g, &b);
  return (b >= 115 && b > r * 1.25 && b > g * 1.25); //CAN BE CHANGED (Use a serial monitor to know the values of the color of your start and end)
}

// --- PATH PRUNING (THE LEARNING MATH) ---
void optimizePath() {
  if (pathLength < 3) return; 
  if (path[pathLength - 2] != 'U') return; 

  char move1 = path[pathLength - 3];
  char move2 = path[pathLength - 1];
  char optimizedMove = ' ';

  if      (move1 == 'L' && move2 == 'L') optimizedMove = 'S';
  else if (move1 == 'L' && move2 == 'S') optimizedMove = 'R';
  else if (move1 == 'R' && move2 == 'L') optimizedMove = 'U';
  else if (move1 == 'S' && move2 == 'L') optimizedMove = 'R';
  else if (move1 == 'S' && move2 == 'S') optimizedMove = 'U';
  else if (move1 == 'L' && move2 == 'R') optimizedMove = 'U';

  if (optimizedMove != ' ') {
    path[pathLength - 3] = optimizedMove; 
    pathLength -= 2;                      
    path[pathLength] = '\0';              
  }
}

void recordMove(char move) {
  if (pathLength < 199) { 
    path[pathLength] = move;
    pathLength++;
    path[pathLength] = '\0'; 
    optimizePath(); 

    Serial.print("Current Memory: ");
    Serial.println(path);
  }
}

// ====================================================================
// HELPER FUNCTIONS (CLOSED-LOOP TURNS WITH ACTIVE BRAKES)
// ====================================================================

int formatMotorSpeed(int raw_speed) {
  if (raw_speed > -50 && raw_speed < 50) return 0; 
  return constrain(raw_speed, -255, 255);
}

void executeHardLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  
  analogWrite(ENA, turn_speed); 
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, turn_speed); 
  delay(blindSpinDelay); 
  
  while (analogRead(SENSOR_M) < 2000) { delay(1); } 
  
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, 255); 
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  
  analogWrite(ENB, 255); 
  delay(brakeDelay); 
  
  analogWrite(ENA, 0); analogWrite(ENB, 0); 
  delay(50);
  lastTurnTime = millis();
}

void executeHardRight() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, turn_speed); 
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  
  analogWrite(ENB, turn_speed); 
  delay(blindSpinDelay); 
  
  while (analogRead(SENSOR_M) < 2000) { delay(1); } 
  
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  
  analogWrite(ENA, 255); 
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, 255); 
  delay(brakeDelay); 
  
  analogWrite(ENA, 0); analogWrite(ENB, 0); 
  delay(50);
  lastTurnTime = millis();
}

void executeUTurn() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  
  analogWrite(ENA, turn_speed); 
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, turn_speed); 
  delay(blindSpinDelay); 
  
  while (analogRead(SENSOR_M) < 2000) { delay(1); } 
  
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, 255); 
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  
  analogWrite(ENB, 255); 
  delay(brakeDelay); 
  
  analogWrite(ENA, 0); analogWrite(ENB, 0); 
  delay(20);
  lastTurnTime = millis();
}

// ====================================================================
// MAIN SETUP
// ====================================================================

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(SENSOR_LT, INPUT); pinMode(SENSOR_RT, INPUT);
  
  pinMode(SLIDE_SWITCH, INPUT_PULLUP);

  if (tcs.begin()) {
    Serial.println("Color Sensor Initialized!");
  } else {
    Serial.println("Color Sensor Not Found! Check I2C Wiring.");
  }
}

// ====================================================================
// MAIN LOOP
// ====================================================================

void loop() {
  bool isSpeedRun = (digitalRead(SLIDE_SWITCH) == LOW);

  static bool previousMode = false;
  if (isSpeedRun != previousMode) { 
    pathIndex = 0; 
    hasStarted = false; 
    mazeCompleted = false; 
    previousMode = isSpeedRun;
    Serial.println("Mode Switched! Ready for next run.");
  }

  // --- THE MASTER START LOCK ---
  if (!hasStarted) {
    analogWrite(ENA, 0); analogWrite(ENB, 0);
    
    if (mazeCompleted) return; 

    if (isSpeedRun && pathLength == 0) {
      Serial.println("SAFETY LOCK: Memory Empty! Switch to Exploration Mode.");
      delay(1000);
      return;
    }

    if (isRedLine()) {
      Serial.println("START LINE DETECTED! Deploying...");
      hasStarted = true;
      
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, LOW); 
      analogWrite(ENA, base_speed);

      digitalWrite(IN3, HIGH); 
      digitalWrite(IN4, LOW); 
      analogWrite(ENB, base_speed);
      delay(500);
    }
    return; 
  }

  // --- THE FINISH LINE LOCK ---
  static unsigned long lastColorCheck = 0;
  if (millis() - lastColorCheck > 50) { 
    lastColorCheck = millis();
    if (isFinishLine()) {
      analogWrite(ENA, 0); analogWrite(ENB, 0);
      Serial.println("FINISH LINE REACHED! Maze Solved. Waiting for Switch Toggle...");
      hasStarted = false; 
      mazeCompleted = true; 
      return; 
    }
  }

  int valLT = digitalRead(SENSOR_LT);
  int valRT = digitalRead(SENSOR_RT);
  int valL1 = analogRead(SENSOR_L1);
  int valM  = analogRead(SENSOR_M);
  int valR1 = analogRead(SENSOR_R1);

  // =========================================================
  // THE BLINDFOLD
  // =========================================================
  if (millis() - lastTurnTime > cooldownDelay) {

    // 1. DEAD END DETECTION 
    if (valLT == 0 && valRT == 0 && valL1 < 2000 && valM < 2000 && valR1 < 2000) {
      deadEndCounter++;

      if (deadEndCounter > 15) {
        if (isSpeedRun) {
           analogWrite(ENA, 0); analogWrite(ENB, 0);
           return; 
        } else {
           executeUTurn();
           recordMove('U'); 
           previous_error = 0; I = 0;
           deadEndCounter = 0; 
           return; 
        }
      }
      analogWrite(ENA, 0); analogWrite(ENB, 0); 
      delay(1);
      return; 
    } else {
      deadEndCounter = 0; 
    }

    // ---------------------------------------------------------
    // 2. INTERSECTION HANDLING (ANTI-BLINDFOLD UPGRADE)
    // ---------------------------------------------------------
    // Validates a real junction: The middle MUST also be on the line
    bool validLeftJunction = (valLT == 1 && (valM > 2000 || valL1 > 2000));
    bool validRightJunction = (valRT == 1 && (valM > 2000 || valR1 > 2000));

    if (validLeftJunction || validRightJunction) {
      
      // === PHASE 3: SPEED RUN MODE ===
      if (isSpeedRun) {
        digitalWrite(IN1, HIGH); 
        digitalWrite(IN2, LOW); 
        analogWrite(ENA, base_speed);

        digitalWrite(IN3, HIGH); 
        digitalWrite(IN4, LOW); 
        analogWrite(ENB, base_speed);

        delay(overtravelDelay); 

        analogWrite(ENA, 0); analogWrite(ENB, 0); delay(50);

        if (pathIndex < pathLength) {
          char nextTurn = path[pathIndex];
          pathIndex++; 
          
          if (nextTurn == 'L') executeHardLeft();
          else if (nextTurn == 'R') executeHardRight();
          else if (nextTurn == 'S') { lastTurnTime = millis(); }
        } 
        previous_error = 0; I = 0;
        return;
      }
      
      // === PHASE 3: EXPLORATION MODE ===
      else {
        // SCENARIO A: LEFT IS FOUND FIRST
        if (validLeftJunction) {
          digitalWrite(IN1, HIGH); 
          digitalWrite(IN2, LOW); 
          analogWrite(ENA, base_speed);

          digitalWrite(IN3, HIGH); 
          digitalWrite(IN4, LOW); 
          analogWrite(ENB, base_speed);

          delay(overtravelDelay); 

          analogWrite(ENA, 0); 
          analogWrite(ENB, 0); 
          delay(50);
          
          executeHardLeft();     
          recordMove('L'); 
          
          previous_error = 0; I = 0;
          return; 
        }
        
        // SCENARIO B: RIGHT IS FOUND FIRST (WITH CONTINUOUS STRAIGHT MEMORY)
        else if (validRightJunction) {
          bool foundStaggeredLeft = false;
          bool canGoStraight = false; // Starts false

          digitalWrite(IN1, HIGH); 
          digitalWrite(IN2, LOW); 
          analogWrite(ENA, base_speed);

          digitalWrite(IN3, HIGH); 
          digitalWrite(IN4, LOW); 
          analogWrite(ENB, base_speed);
          
          // ACTIVELY SWEEP FOR THE STRAIGHT LINE WHILE DRIVING FORWARD!
          unsigned long sweepStart = millis();
          while (millis() - sweepStart < overtravelDelay) {
            if (digitalRead(SENSOR_LT) == 1 && analogRead(SENSOR_M) > 2000) { 
              foundStaggeredLeft = true; 
            }
            if (analogRead(SENSOR_M) > 2000 || analogRead(SENSOR_L1) > 2000 || analogRead(SENSOR_R1) > 2000) {
              canGoStraight = true; // Locks in true if it ever sees the line!
            }
            delay(1); 
          }

          analogWrite(ENA, 0); 
          analogWrite(ENB, 0); 
          delay(50);

          if (foundStaggeredLeft) {
            executeHardLeft();     
            recordMove('L'); 
          } 
          else if (canGoStraight) {
            lastTurnTime = millis(); 
            recordMove('S'); 
          } 
          else {
            executeHardRight();    
            recordMove('R'); 
          }

          previous_error = 0; I = 0;
          return; 
        }
      }
    }
  } // END OF BLINDFOLD CHECK

  // ---------------------------------------------------------
  // MICRO-NAVIGATION: STANDARD PID LINE FOLLOWING
  // ---------------------------------------------------------
  error = valL1 - valR1;

  // --- EXTREME REALIGNMENT INJECTION ---
  // If an outer wing hits the line but it wasn't a valid junction, 
  // inject a massive PID error to dynamically snap it back without stopping!
  if (valLT == 1 && valM < 2000) {
      error = 4000; 
  } else if (valRT == 1 && valM < 2000) {
      error = -4000;
  }

  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;

  int raw_left_speed = base_speed + PID_value;
  int raw_right_speed = base_speed - PID_value;

  int final_left_speed = formatMotorSpeed(raw_left_speed);
  int final_right_speed = formatMotorSpeed(raw_right_speed);

  // Drive Left Motor (ENA)
  if (final_left_speed >= 0) { 
    digitalWrite(IN1, HIGH);  
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, final_left_speed);
  } else { 
    digitalWrite(IN1, LOW);   
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, abs(final_left_speed)); 
  }

  // Drive Right Motor (ENB)
  if (final_right_speed >= 0) { 
    digitalWrite(IN3, HIGH);  
    digitalWrite(IN4, LOW); 
    analogWrite(ENB, final_right_speed);
  } else { 
    digitalWrite(IN3, LOW);   
    digitalWrite(IN4, HIGH); 
    analogWrite(ENB, abs(final_right_speed)); 
  }

  delay(1); 
}