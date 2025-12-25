/*
OLED Line Follower Code by Paradox
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// ========== CONFIGURATION ==========
const uint8_t NUM_SENSORS = 7;  // 5 or 7
const bool IS_BLACK_LINE = true;
const bool IS_TESTING = false;

// EEPROM addresses
#define EEPROM_MAGIC 0xA5
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MIN_START 1
#define EEPROM_MAX_START 15

// Motor pins (optimized for port manipulation)
#define AIN1 4  // PD4
#define AIN2 7  // PD7
#define BIN1 6  // PD6
#define BIN2 8  // PB0
#define PWMA 9  // PB1
#define PWMB 10 // PB2
#define STBY 5  // PD5

// Motor direction offsets
const int8_t OFFSET_A = -1;
const int8_t OFFSET_B = -1;

// Multiplexer pins
#define S0 A0
#define S1 A1
#define S2 A2
#define SIG A3

// Button and LED pins
#define BTN_CALIBRATE 11 // PB3
#define BTN_START 12     // PB4
#define LED_STATUS 13    // PB5

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ========== PID VARIABLES ==========
float Kp = 0.35;  // Increased from 0.2
float Kd = 0.8;   // Increased from 0.2
float Ki = 0.001; // Small integral term
float previousError = 0;
float integral = 0;
const float INTEGRAL_LIMIT = 1000.0;

// ========== SPEED SETTINGS ==========
const uint8_t MAX_SPEED = IS_TESTING ? 30 : 120;  // Increased max speed
const uint8_t MIN_SPEED = 40;
const uint8_t TURN_SPEED = 60;
const uint8_t SEARCH_SPEED = 80;
uint8_t baseSpeed = MIN_SPEED;

// ========== SENSOR VARIABLES ==========
int minValues[7];
int maxValues[7];
int sensorValue[7];
bool sensorArray[7];
const int8_t sensorWeight7[7] = {4, 2, 1, 0, -1, -2, -4};
const int8_t sensorWeight5[7] = {0, 4, 2, 0, -2, -4, 0};
const int LINE_THRESHOLD = 500;

// ========== STATE VARIABLES ==========
enum State { STOPPED, RUNNING, CALIBRATING };
volatile State robotState = STOPPED;
volatile bool onLine = true;
volatile bool isIntersection = false;
bool prevButtonState = HIGH;
uint8_t displayUpdateCounter = 0;

// Lost line recovery
unsigned long lineStartTime = 0;
unsigned long lineLostTime = 0;
const unsigned long LINE_LOST_TIMEOUT = 500; // ms

// Multiplexer channel mapping
const uint8_t muxChannel[8][3] PROGMEM = {
  {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
  {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}
};

// ========== EEPROM FUNCTIONS ==========
void saveCalibration() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  
  for (uint8_t i = 0; i < 7; i++) {
    EEPROM.put(EEPROM_MIN_START + i * 2, minValues[i]);
    EEPROM.put(EEPROM_MAX_START + i * 2, maxValues[i]);
  }
}

bool loadCalibration() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) {
    return false;
  }
  
  for (uint8_t i = 0; i < 7; i++) {
    EEPROM.get(EEPROM_MIN_START + i * 2, minValues[i]);
    EEPROM.get(EEPROM_MAX_START + i * 2, maxValues[i]);
  }
  return true;
}

// ========== FAST PORT MANIPULATION ==========
// Fast digital write using port manipulation
inline void fastDigitalWrite(uint8_t pin, uint8_t val) {
  if (pin == 4) {       // AIN1 - PD4
    if (val) PORTD |= (1 << 4); else PORTD &= ~(1 << 4);
  } else if (pin == 7) { // AIN2 - PD7
    if (val) PORTD |= (1 << 7); else PORTD &= ~(1 << 7);
  } else if (pin == 6) { // BIN1 - PD6
    if (val) PORTD |= (1 << 6); else PORTD &= ~(1 << 6);
  } else if (pin == 8) { // BIN2 - PB0
    if (val) PORTB |= (1 << 0); else PORTB &= ~(1 << 0);
  } else if (pin == 13) { // LED - PB5
    if (val) PORTB |= (1 << 5); else PORTB &= ~(1 << 5);
  } else {
    digitalWrite(pin, val);
  }
}

// ========== SENSOR READING ==========
// Fast batch sensor read
inline void readAllSensors() {
  const uint8_t startIdx = (NUM_SENSORS == 5) ? 1 : 0;
  const uint8_t endIdx = (NUM_SENSORS == 5) ? 6 : 7;

  for (uint8_t i = startIdx; i < endIdx; i++) {
    // Set mux channel using PROGMEM
    uint8_t s0 = pgm_read_byte(&muxChannel[i][0]);
    uint8_t s1 = pgm_read_byte(&muxChannel[i][1]);
    uint8_t s2 = pgm_read_byte(&muxChannel[i][2]);
    
    digitalWrite(S0, s0);
    digitalWrite(S1, s1);
    digitalWrite(S2, s2);
    
    delayMicroseconds(5);
    int raw = analogRead(SIG);
    
    // Map and constrain in one operation
    if (IS_BLACK_LINE) {
      sensorValue[i] = constrain(map(raw, minValues[i], maxValues[i], 0, 1000), 0, 1000);
    } else {
      sensorValue[i] = constrain(map(raw, minValues[i], maxValues[i], 1000, 0), 0, 1000);
    }
    
    sensorArray[i] = (sensorValue[i] > LINE_THRESHOLD);
  }
}

// Single sensor read for calibration
int sensorRead(uint8_t channel) {
  uint8_t s0 = pgm_read_byte(&muxChannel[channel][0]);
  uint8_t s1 = pgm_read_byte(&muxChannel[channel][1]);
  uint8_t s2 = pgm_read_byte(&muxChannel[channel][2]);
  
  digitalWrite(S0, s0);
  digitalWrite(S1, s1);
  digitalWrite(S2, s2);
  delayMicroseconds(10);
  return analogRead(SIG);
}

// ========== MOTOR CONTROL ==========
void motor1run(int speed) {
  speed = OFFSET_A * constrain(speed, -255, 255);
  
  if (speed > 0) {
    fastDigitalWrite(AIN1, HIGH);
    fastDigitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    fastDigitalWrite(AIN1, LOW);
    fastDigitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  } else {
    fastDigitalWrite(AIN1, HIGH);
    fastDigitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int speed) {
  speed = OFFSET_B * constrain(speed, -255, 255);
  
  if (speed > 0) {
    fastDigitalWrite(BIN1, HIGH);
    fastDigitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else if (speed < 0) {
    fastDigitalWrite(BIN1, LOW);
    fastDigitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  } else {
    fastDigitalWrite(BIN1, HIGH);
    fastDigitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}

// ========== LINE DETECTION ==========
void analyzeLine() {
  const uint8_t startIdx = (NUM_SENSORS == 5) ? 1 : 0;
  const uint8_t endIdx = (NUM_SENSORS == 5) ? 6 : 7;
  
  uint8_t activeSensors = 0;
  onLine = false;
  
  for (uint8_t i = startIdx; i < endIdx; i++) {
    if (sensorArray[i]) {
      activeSensors++;
      onLine = true;
    }
  }
  
  // Intersection detection (4+ sensors see line)
  isIntersection = (activeSensors >= 4);
  
  // Track line timing
  if (onLine) {
    lineStartTime = millis();
  } else if (lineLostTime == 0) {
    lineLostTime = millis();
  }
}

// ========== PID CONTROL WITH ADAPTIVE SPEED ==========
void linefollow() {
  float error = 0;
  int activeSensors = 0;
  const int8_t* weights = (NUM_SENSORS == 7) ? sensorWeight7 : sensorWeight5;
  const uint8_t startIdx = (NUM_SENSORS == 5) ? 1 : 0;
  const uint8_t endIdx = (NUM_SENSORS == 5) ? 6 : 7;

  // Calculate weighted error
  for (uint8_t i = startIdx; i < endIdx; i++) {
    if (sensorArray[i]) {
      error += weights[i] * sensorValue[i];
      activeSensors++;
    }
  }

  // Normalize error
  if (activeSensors > 0) {
    error = -error / activeSensors;
  } else {
    error = previousError;
  }

  // PID calculation
  float P = error;
  integral += error;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float D = error - previousError;

  float pidValue = Kp * P + Ki * integral + Kd * D;
  previousError = error;

  // Adaptive speed - slow down on sharp turns
  float errorMagnitude = abs(error);
  if (errorMagnitude > 2.0) {
    baseSpeed = constrain(baseSpeed - 2, MIN_SPEED, MAX_SPEED);
  } else {
    baseSpeed = constrain(baseSpeed + 1, MIN_SPEED, MAX_SPEED);
  }
  
  // Reduce speed on sharp turns
  uint8_t turnReduction = constrain(abs(pidValue) / 3, 0, 40);
  uint8_t adjustedSpeed = baseSpeed - turnReduction;

  // Calculate motor speeds
  int leftSpeed = constrain(adjustedSpeed - pidValue, 0, MAX_SPEED);
  int rightSpeed = constrain(adjustedSpeed + pidValue, 0, MAX_SPEED);

  motor1run(leftSpeed);
  motor2run(rightSpeed);
}

// ========== TIMER1 INTERRUPT - 100Hz CONTROL LOOP ==========
volatile bool controlTick = false;

ISR(TIMER1_COMPA_vect) {
  controlTick = true;
}

void setupTimer1() {
  // Stop interrupts
  cli();
  
  // Clear timer registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  // Set compare match register for 100Hz (10ms period)
  // 16MHz / (prescaler * desired_frequency) - 1
  // 16000000 / (8 * 100) - 1 = 19999
  OCR1A = 19999;
  
  // CTC mode
  TCCR1B |= (1 << WGM12);
  
  // Prescaler 8
  TCCR1B |= (1 << CS11);
  
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  // Enable interrupts
  sei();
}

// ========== MAIN CONTROL LOGIC ==========
void runLF() {
  if (robotState == RUNNING) {
    readAllSensors();
    analyzeLine();
    
    if (onLine) {
      lineLostTime = 0;
      linefollow();
      fastDigitalWrite(LED_STATUS, HIGH);
    } else {
      // Lost line recovery
      unsigned long timeLost = millis() - lineLostTime;
      
      if (timeLost < LINE_LOST_TIMEOUT) {
        // Quick recovery - turn based on last error
        fastDigitalWrite(LED_STATUS, LOW);
        if (previousError > 0) {
          motor1run(-TURN_SPEED);
          motor2run(SEARCH_SPEED);
        } else {
          motor1run(SEARCH_SPEED);
          motor2run(-TURN_SPEED);
        }
      } else {
        // Line completely lost - stop
        motor1run(0);
        motor2run(0);
        robotState = STOPPED;
      }
    }
  } else {
    motor1run(0);
    motor2run(0);
    fastDigitalWrite(LED_STATUS, LOW);
  }
}

// ========== DISPLAY FUNCTIONS ==========
void drawPercentbar(int x, int y, int width, int height, uint8_t progress) {
  int bar = ((width - 4) * progress) / 100;
  display.drawRect(x, y, width, height, SSD1306_WHITE);
  display.fillRect(x + 2, y + 2, bar, height - 4, SSD1306_WHITE);
  
  if (height >= 15) {
    display.setCursor((width / 2) - 9, y + 5);
    display.setTextSize(1);
    display.setTextColor(progress >= 50 ? SSD1306_BLACK : SSD1306_WHITE);
    display.print(progress);
    display.print(F("%"));
  }
}

void displayName() {
  const char name[] = "By Paradox";
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 8);
  display.println(name);
  display.display();
  delay(1500);
}

void displayMessage(const char* line1, const char* line2 = nullptr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8);
  display.println(line1);
  if (line2) {
    display.println(line2);
  }
  display.display();
}

// ========== CALIBRATION ==========
void calibrate() {
  displayMessage("Calibrating...");
  delay(500);

  // Initialize min/max
  for (uint8_t i = 0; i < 7; i++) {
    int reading = sensorRead(i);
    minValues[i] = reading;
    maxValues[i] = reading;
  }

  // Calibration sweep
  for (int i = 0; i <= 200; i++) {
    if (i % 10 == 0) {
      display.clearDisplay();
      drawPercentbar(0, SCREEN_HEIGHT / 2 - 8, SCREEN_WIDTH, 15, (i * 100) / 200);
      display.display();
    }
    
    motor1run(60);
    motor2run(-60);

    for (uint8_t j = 0; j < 7; j++) {
      int reading = sensorRead(j);
      if (reading < minValues[j]) minValues[j] = reading;
      if (reading > maxValues[j]) maxValues[j] = reading;
    }
    delay(5);
  }

  motor1run(0);
  motor2run(0);

  // Save to EEPROM
  saveCalibration();
  displayMessage("Calibration Done", "Saved to EEPROM");
  delay(1500);
}

// ========== BUTTON HANDLING ==========
void checkStartButton() {
  bool currentButton = digitalRead(BTN_START);
  
  if (currentButton == LOW && prevButtonState == HIGH) {
    delay(50);  // Debounce
    if (digitalRead(BTN_START) == LOW) {
      if (robotState == STOPPED) {
        robotState = RUNNING;
        displayMessage("Running...");
        baseSpeed = MIN_SPEED;
        previousError = 0;
        integral = 0;
        lineLostTime = 0;
      } else {
        robotState = STOPPED;
        displayMessage("Stopped");
      }
    }
  }
  prevButtonState = currentButton;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

  // Initialize display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Display init failed"));
    while (1);
  }

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Multiplexer pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  // Button and LED pins
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(LED_STATUS, OUTPUT);

  displayName();

  // Try to load calibration from EEPROM
  if (loadCalibration()) {
    displayMessage("Calibration", "loaded from EEPROM");
    delay(1500);
    displayMessage("Press to skip", "or recalibrate");
    delay(2000);
    
    // Check if user wants to recalibrate
    if (digitalRead(BTN_CALIBRATE) == LOW) {
      delay(1000);
      calibrate();
    }
  } else {
    // No saved calibration
    displayMessage("Press button to", "calibrate...");
    while (digitalRead(BTN_CALIBRATE) == HIGH);
    delay(1000);
    calibrate();
  }

  // Setup timer interrupt for 100Hz control loop
  setupTimer1();

  // Wait for start
  displayMessage("Press button to", "start...");
  while (digitalRead(BTN_START) == HIGH);
  delay(500);
  
  robotState = RUNNING;
  displayMessage("Running...");
  delay(500);
  display.clearDisplay();
  display.display();
}

// ========== MAIN LOOP ==========
void loop() {
  // Control loop runs at 100Hz via timer interrupt
  if (controlTick) {
    controlTick = false;
    runLF();
  }
  
  // Non-time-critical tasks in main loop
  checkStartButton();
  
  // Optional: Update display occasionally (not in control loop)
  displayUpdateCounter++;
  if (displayUpdateCounter >= 200 && robotState == RUNNING) {  // Every 2 seconds
    displayUpdateCounter = 0;
    // Could show speed, error, etc. here if desired
  }
}
