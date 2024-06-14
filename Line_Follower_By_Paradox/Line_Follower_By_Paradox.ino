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
#include <SparkFun_TB6612.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//Sesnor Details
uint16_t numSensors = 7;  // Enter number of sensors as 5 or 7

//Flags
bool isBlackLine = true;  //keep 1 in case of black line. In case of white line change this to 0
bool brakeEnabled = false;
bool isTesting = false;
bool onoff = false;
bool brakeFlag = false;
bool onLine = true;
bool bluetoothEnabled = false;

//Motor Driver Pin Assignments
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5
#define CenterSensorPin 8

//Display Details
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

//Display Details Declaration
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// these constants are used to allow you to make your motor configuration
// line up with function names like forward. Value can be 1 or -1
const int offsetA = -1;
const int offsetB = -1;


//Motor Declarations
// Initializing motors. The library will allow you to initialize as many
// motors as you have memory for. If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motorLeft = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


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

//Speed Declaration
int lsp, rsp;
int lfSpeed = isTesting ? 30 : 150;  // Decreased the speed for testing

//QTR Values Assignment
int minValues[7];
int maxValues[7];
int sensorValue[7];


int lineThreshold = 700;

int val, cnt = 0, v[3];

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

//Line Follower main function
void runLF() {
  readLine();
  if (onLine) {  //PID LINE FOLLOW
    linefollow();
    digitalWrite(13, HIGH);
    brakeFlag = false;
  } else {
    digitalWrite(13, LOW);
    if (error > 0) {
      if (brakeEnabled && !brakeFlag) {
        motorLeft.drive(0);
        motorRight.drive(0);
        delay(30);
      }
      motorLeft.drive(-100);
      motorRight.drive(150);
      brakeFlag = true;
    } else if (error < 0) {
      if (brakeEnabled && !brakeFlag) {
        motorLeft.drive(0);
        motorRight.drive(0);
        delay(30);
      }
      motorLeft.drive(150);
      motorRight.drive(-100);
      brakeFlag = true;
    }
  }
}

// Follow the line using PID control
void linefollow() {
  if (numSensors == 7) {
    error = (3 * sensorValue[0] + 2 * sensorValue[1] + 1 * sensorValue[2] - 1 * sensorValue[4] - 2 * sensorValue[5] - 3 * sensorValue[6]);
  }
  if (numSensors == 5) {
    error = (3 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 3 * sensorValue[5]);
  }

  if (!isBlackLine) {
    error = error * -1;
  }

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

  motorLeft.drive(lsp);
  motorRight.drive(rsp);
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

  minValues[0] = analogRead(0);
  maxValues[0] = analogRead(0);

  minValues[1] = analogRead(1);
  maxValues[1] = analogRead(1);

  minValues[2] = analogRead(2);
  maxValues[2] = analogRead(2);

  minValues[3] = 0;
  maxValues[3] = 1;

  minValues[4] = analogRead(3);
  maxValues[4] = analogRead(3);

  minValues[5] = analogRead(6);
  maxValues[5] = analogRead(6);

  minValues[6] = analogRead(7);
  maxValues[6] = analogRead(7);

  for (int i = 0; i <= 200; i++) {

    display.clearDisplay();
    drawPercentbar(0, display.height() / 2 - 8, display.width(), 15, i / 2);
    display.display();
    motorLeft.drive(50);
    motorRight.drive(-50);

    if (analogRead(0) < minValues[0]) {
      minValues[0] = analogRead(0);
    }
    if (analogRead(0) > maxValues[0]) {
      maxValues[0] = analogRead(0);
    }

    if (analogRead(1) < minValues[1]) {
      minValues[1] = analogRead(1);
    }
    if (analogRead(1) > maxValues[1]) {
      maxValues[1] = analogRead(1);
    }

    if (analogRead(2) < minValues[2]) {
      minValues[2] = analogRead(2);
    }
    if (analogRead(2) > maxValues[2]) {
      maxValues[2] = analogRead(2);
    }

    if (analogRead(3) < minValues[4]) {
      minValues[4] = analogRead(3);
    }
    if (analogRead(3) > maxValues[4]) {
      maxValues[4] = analogRead(3);
    }

    if (analogRead(6) < minValues[5]) {
      minValues[5] = analogRead(6);
    }
    if (analogRead(6) > maxValues[5]) {
      maxValues[5] = analogRead(6);
    }
    if (analogRead(7) < minValues[6]) {
      minValues[6] = analogRead(7);
    }
    if (analogRead(7) > maxValues[6]) {
      maxValues[6] = analogRead(7);
    }
  }


  display.clearDisplay();
  display.setTextSize(1);                          // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);             // Draw white text
  display.setCursor(0, display.height() / 2 - 4);  // Start at top-left corner
  display.println(F("Calibration \nDone."));
  display.display();

  motorLeft.drive(0);
  motorRight.drive(0);
  delay(1000);
}

// Read the Track Line
void readLine() {
  onLine = false;
  if (numSensors == 7) {

    sensorValue[0] = constrain(map(analogRead(0), minValues[0], maxValues[0], 0, 1000), 0, 1000);
    if ((isBlackLine && sensorValue[0] > lineThreshold) || (!isBlackLine && sensorValue[0] < lineThreshold))
      onLine = true;

    sensorValue[1] = constrain(map(analogRead(1), minValues[1], maxValues[1], 0, 1000), 0, 1000);
    if ((isBlackLine && sensorValue[1] > lineThreshold) || (!isBlackLine && sensorValue[1] < lineThreshold))
      onLine = true;

    sensorValue[2] = constrain(map(analogRead(2), minValues[2], maxValues[2], 0, 1000), 0, 1000);
    if ((isBlackLine && sensorValue[2] > lineThreshold) || (!isBlackLine && sensorValue[2] < lineThreshold))
      onLine = true;
    
    sensorValue[3] = constrain(map(digitalRead(CenterSensorPin), minValues[3], maxValues[3], 0, 1000), 0, 1000);
    if ((isBlackLine && sensorValue[3] > lineThreshold) || (!isBlackLine && sensorValue[3] < lineThreshold))
      onLine = true;

    sensorValue[4] = constrain(map(analogRead(3), minValues[4], maxValues[4], 0, 1000), 0, 1000);
    if ((isBlackLine && sensorValue[4] > lineThreshold) || (!isBlackLine && sensorValue[4] < lineThreshold))
      onLine = true;

    sensorValue[5] = constrain(map(analogRead(6), minValues[5], maxValues[5], 0, 1000), 0, 1000);
    if ((isBlackLine && sensorValue[5] > lineThreshold) || (!isBlackLine && sensorValue[5] < lineThreshold))
      onLine = true;

    sensorValue[6] = constrain(map(analogRead(7), minValues[6], maxValues[6], 0, 1000), 0, 1000);
    if ((isBlackLine && sensorValue[6] > lineThreshold) || (!isBlackLine && sensorValue[6] < lineThreshold))
      onLine = true;
  }

  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      sensorValue[i] = constrain(map(analogRead(i), minValues[i], maxValues[i], 0, 1000), 0, 1000);
      if ((isBlackLine && sensorValue[i] > lineThreshold) || (!isBlackLine && sensorValue[i] < lineThreshold))
        onLine = true;
    }
  }
}

//Read PID Values from Sensor
void bluetoothReadAndProcess() {
  val = Serial.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)
    cnt = 0;
  int a = v[1];
  if (a == 1)
    Kp = v[2];

  if (a == 2)
    multiP = v[2];

  if (a == 3)
    Ki = v[2];

  if (a == 4)
    multiI = v[2];

  if (a == 5)
    Kd = v[2];

  if (a == 6)
    multiD = v[2];

  if (a == 7)
    onoff = v[2];
}


void setup() {

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  //Initialize the baud rate for serial communication
  Serial.begin(115200);
  Serial.println("Bluetooth Started! Ready to pair...");

  //Display initialization
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  displayInit();

  displayName();

  //Pin Initialization
  //11 - Calibration
  //12 - Start Run
  //13 - LED for Line Following
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  pinMode(CenterSensorPin,INPUT);

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
  if (bluetoothEnabled) {
    bluetoothReadAndProcess();
    if (onoff) {
      runLF();
    } else {
      motorLeft.drive(0);
      motorRight.drive(0);
    }
  } else {
    runLF();
  }
}
