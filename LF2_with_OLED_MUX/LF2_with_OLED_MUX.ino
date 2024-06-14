/*
OLED Line Follower Code by Paradox
*/

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//Include all the necessary libraries...
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//Sensor Details
uint16_t numSensors = 7;  // Enter number of sensors as 5 or 7

//Flags
bool isBlackLine = true;  //keep 1 in case of black line. In case of white line change this to 0
bool isTesting = false;
bool onLine = true;

//Motor Driver Pin Assignments
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

//Motor Direction Offsets
// -1 -> Reverse the direction
const int offsetA = -1;
const int offsetB = -1;


/*
  CD74HC4067 16-Channel Analog/Digital Multiplexer
  Pin Assignment

  Control Pins
  S0 -> 8;
  S1 -> 9;
  S2 -> 10;
  S3 -> 11;

  Signal Pin
  SIG -> 0;

  En -> GND
*/
#define S0 A0
#define S1 A1
#define S2 A2
// #define S3 30
#define SIG 3

//Display Details
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

//Display Details Declaration
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//PID Assignments
float P;
float I;
float D;
float Pvalue;
float Ivalue;
float Dvalue;
float previousError;
float error;
float PIDvalue;
uint8_t multiP = 0;
uint8_t multiI = 1;
uint8_t multiD = 1;
float Kp = 0.2;
float Kd = 0.2;
float Ki = 0.00;

#define RUNNING 1
#define STOPPED 0
bool state = RUNNING;

//Speed Declaration
int lsp, rsp;
int lfSpeed = isTesting ? 30 : 100;  // Decreased the speed for testing
int currentSpeed = 30;

//QTR Values Assignment
int minValues[7];
int maxValues[7];
int threshold[7];
int sensorValue[7];
int sensorArray[7];
int sensorWeight[7] = { 4, 2, 1, 0, -1, -2, -4 };
int activeSensors;
int lineThreshold = 500;

//Constants for the Multiplexer
int controlPin[] = {
  S0,
  S1,
  S2,
};
int muxChannel[16][4] = {
  { 0, 0, 0, 0 },  //channel 0
  { 1, 0, 0, 0 },  //channel 1
  { 0, 1, 0, 0 },  //channel 2
  { 1, 1, 0, 0 },  //channel 3
  { 0, 0, 1, 0 },  //channel 4
  { 1, 0, 1, 0 },  //channel 5
  { 0, 1, 1, 0 },  //channel 6
  { 1, 1, 1, 0 },  //channel 7
  // { 0, 0, 0, 1 },  //channel 8
  // { 1, 0, 0, 1 },  //channel 9
  // { 0, 1, 0, 1 },  //channel 10
  // { 1, 1, 0, 1 },  //channel 11
  // { 0, 0, 1, 1 },  //channel 12
  // { 1, 0, 1, 1 },  //channel 13
  // { 0, 1, 1, 1 },  //channel 14
  // { 1, 1, 1, 1 }   //channel 15
};

void displayInit() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1) {}
  }
}

void displayName() {
  char name[] = "Developed by Paradox";
  int nameLength = strlen(name);

  display.clearDisplay();
  for (int i = 0; i < nameLength; i++) {
    char* namePart = "";
    for (int j = 0; j <= i; j++) {
      namePart += name[j];
    }
    display.setTextSize(1);                                  // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);                     // Draw white text
    display.setCursor(5 + i * 6, display.height() / 2 - 4);  // Start at top-left corner
    display.println(name[i]);
    display.display();
    delay(100);  // Pause for 2 seconds
  }
  delay(1000);
  display.clearDisplay();
}

void drawPercentbar(int x, int y, int width, int height, int progress) {
  float bar = ((float)(width - 4) / 100) * progress;
  display.drawRect(x, y, width, height, SSD1306_WHITE);
  display.fillRect(x + 2, y + 2, bar, height - 4, SSD1306_WHITE);

  // Display progress text
  if (height >= 15) {
    display.setCursor((width / 2) - 3, y + 5);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    if (progress >= 50)
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  // 'inverted' text

    display.print(progress);
    display.print("%");
  }
}

//Function to run Motor 1
void motor1run(int motorSpeed) {
  motorSpeed = offsetA * constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, abs(motorSpeed));
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//Function to run Motor 2
void motor2run(int motorSpeed) {
  motorSpeed = offsetB * constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, abs(motorSpeed));
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}

bool prevState = STOPPED;
bool currState;
void readState() {
  currState = digitalRead(12);
  if (currState != prevState && currState == LOW) {
    if (state == STOPPED) {
      state = RUNNING;
      display.clearDisplay();
      display.setTextSize(1);               // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);  // Draw white text
      display.setCursor(0, 0);              // Start at top-left corner
      display.println(F("Running..."));
      display.display();
    } else {
      state = STOPPED;

      display.clearDisplay();
      display.setTextSize(1);                          // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);             // Draw white text
      display.setCursor(0, display.height() / 2 - 4);  // Start at top-left corner
      display.println(F("Stopped... \nCalibration Saved..."));
      display.display();
    }
  }
  prevState = currState;
}


//Line Follower main function
void runLF() {
  readState();
  if (state == RUNNING) {

    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine) {  //PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      if (error > 0) {
        motor1run(-50);
        motor2run(lfSpeed);
      } else {
        motor1run(lfSpeed);
        motor2run(-50);
      }
    }
  } else {
    motor1run(0);
    motor2run(0);
  }
}

// Follow the line using PID control
void linefollow() {
  error = 0;
  activeSensors = 0;

  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }

  error *= -1;

  P = error;
  I = I + error;
  D = error - previousError;

  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;


  PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = constrain(lfSpeed - PIDvalue, 0, lfSpeed);
  rsp = constrain(lfSpeed + PIDvalue, 0, lfSpeed);

  motor1run(lsp);
  motor2run(rsp);
}

//Calibrate the sensor
void calibrate() {

  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("Calibrating..."));
  display.display();

  delay(500);
  display.clearDisplay();

  for (int i = 0; i < 7; i++) {
    minValues[i] = sensorRead(i);
    maxValues[i] = sensorRead(i);
  }

  for (int i = 0; i <= 200; i++) {

    display.clearDisplay();
    drawPercentbar(0, display.height() / 2 - 8, display.width(), 15, map(i, 0, 200, 0, 100));
    display.display();
    motor1run(50);
    motor2run(-50);

    for (int i = 0; i < 7; i++) {
      if (sensorRead(i) < minValues[i]) {
        minValues[i] = sensorRead(i);
      }
      if (sensorRead(i) > maxValues[i]) {
        maxValues[i] = sensorRead(i);
      }
    }
  }


  display.clearDisplay();
  display.setTextSize(1);                          // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);             // Draw white text
  display.setCursor(0, display.height() / 2 - 4);  // Start at top-left corner
  display.println(F("Calibration \nDone."));
  display.display();

  motor1run(0);
  motor2run(0);
  delay(1000);
}

// Read the Track Line
void readLine() {
  onLine = 0;
  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > lineThreshold;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine && sensorArray[i]) onLine = 1;
      if (!isBlackLine && !sensorValue[i]) onLine = 1;
    }
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > lineThreshold;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine == 1 && sensorArray[i]) onLine = 1;
      if (isBlackLine == 0 && !sensorValue[i]) onLine = 1;
    }
  }
}


//Sensor Reading using the Demultiplexer
int sensorRead(int channel) {

  //Set the Control Pin values
  for (int i = 0; i < 3; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  return analogRead(SIG);
}


void setup() {

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  //Display initialization
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  displayInit();

  //Motor Driver Pin Initializaion
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  //Demultiplexer Pin Initialization
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  // pinMode(S3, OUTPUT);

  //Pin Initialization(Misc)
  //11 - Calibration
  //12 - Start Run
  //13 - LED for Line Following
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);


  if (numSensors == 5) {
    sensorWeight[1] = 4;
    sensorWeight[5] = -4;
  }


  displayName();


  // Show Calibration Text
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("Waiting for \nCalibration..."));
  display.display();

  // Waiting for Button Press
  while (digitalRead(11)) {
  }
  display.clearDisplay();
  delay(1000);
  calibrate();

  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("Ready to Start..."));
  display.display();

  // Waiting for Button Press
  while (digitalRead(12)) {
  }
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("Running..."));
  display.display();
}


void loop() {
  runLF();
}
