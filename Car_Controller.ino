// ==========================================================
// --- HARDWARE PIN CONFIGURATION ---
// ==========================================================
#define PWMA 9
#define AIN2 8
#define AIN1 7
#define PWMB 6
#define BIN1 5
#define BIN2 4
#define BUTTON_PIN 12
#define BUZZER_PIN 11
#define LED_PIN 13

// Sensor Array Configuration (Far Left -> Far Right)
const int sensorPins[8] = {A7, A6, A5, A4, A3, A2, A1, A0};
int sensorMin[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int sensorMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// ==========================================================
// --- PID PARAMETERS & KINEMATIC LIMITS ---
// ==========================================================
float Kp = 0.10; 
float Ki = 0.00; 
float Kd = 2.50;

// Speed Control Variables
int targetBaseSpeed = 200;   // Maximum straight-line speed
int maxSpeed = 255;          // Absolute PWM limit
float currentBaseSpeed = 0;  // Dynamic speed variable for Soft-Start
float accelerationRate = 0.3;// Rate of speed increase (Lower = slower ramp-up)

// State Variables
int lastError = 0;
long integral = 0;

void setup() {
  // 1. Initialize output pins using a standard array structure
  const int outPins[] = {PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, BUZZER_PIN, LED_PIN};
  for (int i = 0; i < 8; i++) pinMode(outPins[i], OUTPUT);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  // 2. Auto-Calibration Phase (5 Seconds)
  waitButtonPress(); beep(2);
  digitalWrite(LED_PIN, HIGH);
  unsigned long startTime = millis();
  
  while (millis() - startTime < 5000) {
    for (int i = 0; i < 8; i++) {
      int val = analogRead(sensorPins[i]);
      sensorMin[i] = min(sensorMin[i], val);
      sensorMax[i] = max(sensorMax[i], val);
    }
    // Sync LED and Buzzer for calibration scanning
    if ((millis() / 100) % 2 == 0) {
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
    }
  }
  digitalWrite(BUZZER_PIN, LOW); digitalWrite(LED_PIN, LOW); beep(1); delay(500);

  // 3. Enter the pre-race holding mode
  enterStandbyMode();
}

void loop() {
  // ========================================================
  // FEATURE 1: EMERGENCY STOP & RETURN TO STANDBY
  // Pressing the button mid-race instantly cuts power and 
  // returns the robot to the Active Alignment holding state.
  // ========================================================
  if (digitalRead(BUTTON_PIN) == LOW) {
    controlMotors(0, 0); // Instant brake
    while (digitalRead(BUTTON_PIN) == LOW); // Wait for button release
    delay(70); // Debounce
    beep(1);
    enterStandbyMode(); // Go back to idle/alignment
    return;
  }

  // ========================================================
  // FEATURE 2: SOFT-START (ANTI-WHEELIE LOGIC)
  // Gradually ramps up the base speed to prevent the high-torque
  // N20 motors from lifting the front sensors off the ground.
  // ========================================================
  if (currentBaseSpeed < targetBaseSpeed) {
    currentBaseSpeed += accelerationRate; 
  }
  int activeBaseSpeed = (int)currentBaseSpeed;

  // ========================================================
  // MAIN PID EXECUTION
  // ========================================================
  int pos = getLinePosition();
  
  if (pos == -1) { // Track Loss Recovery
    if (lastError < 0) controlMotors(-160, 160); // Spin Left
    else controlMotors(160, -160);               // Spin Right
    return;
  }

  int motorSpeed = computePID(pos);
  
  // Superimpose PID compensation onto the dynamic base speed
  int leftSpeed = constrain(activeBaseSpeed + motorSpeed, -maxSpeed, maxSpeed);
  int rightSpeed = constrain(activeBaseSpeed - motorSpeed, -maxSpeed, maxSpeed);
  
  controlMotors(leftSpeed, rightSpeed);
}

// ==========================================================
// --- CORE ALGORITHMS & SUBROUTINES ---
// ==========================================================

/* * Standby Mode: Suspends forward kinematics and enables 
 * Active Alignment. The robot will aggressively hold the 
 * center line using a limited-torque PID loop until triggered.
 */
void enterStandbyMode() {
  currentBaseSpeed = 0; // Reset soft-start accumulator
  lastError = 0;        // Clear error history
  
  while (digitalRead(BUTTON_PIN) == HIGH) {
    int pos = getLinePosition();
    if (pos != -1) {
      int error = pos - 3500;
      if (abs(error) < 150) {
        controlMotors(0, 0); // Dead-zone: Relax motors if perfectly centered
      } else {
        int targetSpeed = computePID(pos);
        targetSpeed = constrain(targetSpeed, -80, 80); // Torque limitation
        controlMotors(targetSpeed, -targetSpeed);      // Pivot in place
      }
    } else {
      controlMotors(0, 0); // Stop if lifted mid-air
    }
  }
  
  waitButtonPress(); // Block until button is released
  beep(3);           // Countdown beeps
  delay(500);        // Launch delay
}

int computePID(int position) {
  int error = position - 3500;
  
  // Calculate PID without any cross-reading filters
  integral += error;
  int motorSpeed = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));
  lastError = error;
  
  return motorSpeed;
}

int getLinePosition() {
  unsigned long num = 0, den = 0;
  bool onLine = false;
  
  for (int i = 0; i < 8; i++) {
    int rawVal = constrain(analogRead(sensorPins[i]), sensorMin[i], sensorMax[i]);
    int calVal = map(rawVal, sensorMin[i], sensorMax[i], 1000, 0); // Inverted logic
    
    if (calVal > 300) onLine = true; // Noise floor rejection
    
    num += (unsigned long)calVal * (i * 1000);
    den += calVal;
  }
  return onLine ? (num / den) : -1;
}

// ==========================================================
// --- HARDWARE INTERFACES ---
// ==========================================================

void setHardwareMotor(int pwm, int in1, int in2, int speed) {
  digitalWrite(in1, speed >= 0);
  digitalWrite(in2, speed < 0);
  analogWrite(pwm, abs(speed));
}

void controlMotors(int left, int right) {
  setHardwareMotor(PWMB, BIN1, BIN2, left);  // Left Motor (Motor B)
  setHardwareMotor(PWMA, AIN1, AIN2, right); // Right Motor (Motor A)
}

void waitButtonPress() {
  while (digitalRead(BUTTON_PIN) == HIGH); delay(70); // Wait for press
  while (digitalRead(BUTTON_PIN) == LOW);  delay(70); // Wait for release
}

// Acoustic and visual notification generator
void beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH); 
    digitalWrite(LED_PIN, HIGH);    // Sync LED ON with buzzer
    delay(30);
    digitalWrite(BUZZER_PIN, LOW);  
    digitalWrite(LED_PIN, LOW);     // Sync LED OFF with buzzer
    delay(100);
  }
}