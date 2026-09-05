#include <Arduino.h>

// --- Pin Definitions ---
// Motor Driver Shared
#define PIN_STBY 4

// Motor Driver 1 (Left Side)
#define PIN_L_IN1 26 // AIN1 & BIN1
#define PIN_L_IN2 33 // AIN2 & BIN2
#define PIN_LF_PWM 25 // PWMA
#define PIN_LR_PWM 27 // PWMB

// Motor Driver 2 (Right Side)
#define PIN_R_IN1 21 // AIN1 & BIN1
#define PIN_R_IN2 23 // AIN2 & BIN2
#define PIN_RF_PWM 22 // PWMA
#define PIN_RR_PWM 16 // PWMB

// Encoders
#define PIN_ENC_LF_A 19
#define PIN_ENC_LF_B 17
#define PIN_ENC_LR_A 32
#define PIN_ENC_LR_B 18
#define PIN_ENC_RF_A 34
#define PIN_ENC_RF_B 35
#define PIN_ENC_RR_A 39
#define PIN_ENC_RR_B 36

// UART
#define PIN_TXD 14
#define PIN_RXD 13

// --- PWM Configuration ---
const int freq = 5000;
const int resolution = 8;
const int ch_lf = 0;
const int ch_lr = 1;
const int ch_rf = 2;
const int ch_rr = 3;

// --- Encoder Counters ---
volatile int32_t ticks_lf = 0;
volatile int32_t ticks_lr = 0;
volatile int32_t ticks_rf = 0;
volatile int32_t ticks_rr = 0;

// --- Safety Watchdog ---
unsigned long last_cmd_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;

// --- Encoder ISRs ---
void IRAM_ATTR isr_lf() {
  if (digitalRead(PIN_ENC_LF_A) == digitalRead(PIN_ENC_LF_B)) { ticks_lf++; } else { ticks_lf--; }
}
void IRAM_ATTR isr_lr() {
  if (digitalRead(PIN_ENC_LR_A) == digitalRead(PIN_ENC_LR_B)) { ticks_lr++; } else { ticks_lr--; }
}
void IRAM_ATTR isr_rf() {
  if (digitalRead(PIN_ENC_RF_A) == digitalRead(PIN_ENC_RF_B)) { ticks_rf++; } else { ticks_rf--; }
}
void IRAM_ATTR isr_rr() {
  if (digitalRead(PIN_ENC_RR_A) == digitalRead(PIN_ENC_RR_B)) { ticks_rr++; } else { ticks_rr--; }
}

void setup() {
  // UART Init
  Serial1.begin(115200, SERIAL_8N1, PIN_RXD, PIN_TXD);

  // Motor Driver Init
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  
  digitalWrite(PIN_STBY, HIGH);
  digitalWrite(PIN_L_IN1, LOW);
  digitalWrite(PIN_L_IN2, LOW);
  digitalWrite(PIN_R_IN1, LOW);
  digitalWrite(PIN_R_IN2, LOW);

  // PWM Init
  ledcSetup(ch_lf, freq, resolution);
  ledcSetup(ch_lr, freq, resolution);
  ledcSetup(ch_rf, freq, resolution);
  ledcSetup(ch_rr, freq, resolution);
  
  ledcAttachPin(PIN_LF_PWM, ch_lf);
  ledcAttachPin(PIN_LR_PWM, ch_lr);
  ledcAttachPin(PIN_RF_PWM, ch_rf);
  ledcAttachPin(PIN_RR_PWM, ch_rr);

  // Encoder Init
  pinMode(PIN_ENC_LF_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LF_B, INPUT_PULLUP);
  pinMode(PIN_ENC_LR_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LR_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RF_A, INPUT); // 34, 35, 36, 39 are input only, no internal pullups
  pinMode(PIN_ENC_RF_B, INPUT);
  pinMode(PIN_ENC_RR_A, INPUT);
  pinMode(PIN_ENC_RR_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LF_A), isr_lf, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LR_A), isr_lr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RF_A), isr_rf, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RR_A), isr_rr, CHANGE);
}

void setMotors(int left_pwm, int right_pwm) {
  // Constrain limits
  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);

  digitalWrite(PIN_STBY, HIGH);

  // Left Motors Direction
  if (left_pwm > 0) {
    digitalWrite(PIN_L_IN1, HIGH);
    digitalWrite(PIN_L_IN2, LOW);
  } else if (left_pwm < 0) {
    digitalWrite(PIN_L_IN1, LOW);
    digitalWrite(PIN_L_IN2, HIGH);
  } else {
    digitalWrite(PIN_L_IN1, LOW);
    digitalWrite(PIN_L_IN2, LOW);
  }

  // Right Motors Direction
  if (right_pwm > 0) {
    digitalWrite(PIN_R_IN1, HIGH);
    digitalWrite(PIN_R_IN2, LOW);
  } else if (right_pwm < 0) {
    digitalWrite(PIN_R_IN1, LOW);
    digitalWrite(PIN_R_IN2, HIGH);
  } else {
    digitalWrite(PIN_R_IN1, LOW);
    digitalWrite(PIN_R_IN2, LOW);
  }

  // Apply PWM
  ledcWrite(ch_lf, abs(left_pwm));
  ledcWrite(ch_lr, abs(left_pwm));
  ledcWrite(ch_rf, abs(right_pwm));
  ledcWrite(ch_rr, abs(right_pwm));
}

void stopMotors() {
  setMotors(0, 0);
  digitalWrite(PIN_STBY, LOW);
}

unsigned long last_telemetry_time = 0;
String inputString = "";

void loop() {
  unsigned long now = millis();

  // --- Telemetry (50Hz = 20ms) ---
  if (now - last_telemetry_time >= 20) {
    last_telemetry_time = now;
    
    // Read atomic copies of ticks
    noInterrupts();
    int32_t t_lf = ticks_lf;
    int32_t t_lr = ticks_lr;
    int32_t t_rf = ticks_rf;
    int32_t t_rr = ticks_rr;
    interrupts();

    // Format: e,<lf>,<lr>,<rf>,<rr>\n
    Serial1.print("e,");
    Serial1.print(t_lf); Serial1.print(",");
    Serial1.print(t_lr); Serial1.print(",");
    Serial1.print(t_rf); Serial1.print(",");
    Serial1.println(t_rr);
  }

  // --- Command Parsing ---
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      if (inputString.startsWith("m,")) {
        int firstComma = inputString.indexOf(',');
        int secondComma = inputString.indexOf(',', firstComma + 1);
        
        if (firstComma != -1 && secondComma != -1) {
          String leftStr = inputString.substring(firstComma + 1, secondComma);
          String rightStr = inputString.substring(secondComma + 1);
          
          int left_pwm = leftStr.toInt();
          int right_pwm = rightStr.toInt();
          
          setMotors(left_pwm, right_pwm);
          last_cmd_time = now;
        }
      }
      inputString = "";
    } else {
      inputString += c;
    }
  }

  // --- Watchdog ---
  if (now - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
    stopMotors();
  }
}
