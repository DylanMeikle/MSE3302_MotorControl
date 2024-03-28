#include <Arduino.h>
#include <ESP32Servo.h>
#include <PID_v1.h> //Use Matlab to tune PID values for motor if used
//#include <AccelStepper.h> //Copilot is telling me to add this for stepper
#include <Adafruit_NeoPixel.h> //For LED control

//GPIO Pins/Constants:
static const int servoPin = 35;
static const int potPin = 1;

static const int LED = 21;
static const int dirPin = 5;
static const int stepPin = 6;

//Varialbes:
int test; //Sets the Case to run (i.e. test servo or run)
int val;  //Value read from potentiometer for servo
int speed = 3000; //Speed of stepper motor
unsigned long stepRate; //Rate of stepper motor steps
volatile int32_t stepCount = 0;
boolean stepDir = true;
hw_timer_t * pTimer = NULL;
boolean runState = false;


// put function declarations here:
Servo servo;  //Controls Cranial Rotation (May need to switch to motor for greater range of motion)
// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(1, LED, NEO_RGB + NEO_KHZ800);

void ARDUINO_ISR_ATTR timerISR();


void setup() {
  // put your setup code here, to run once:
  //int result = myFunction(2, 3);
  Serial.begin(115200);
  servo.attach(servoPin);
  pinMode(potPin, INPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  pTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(pTimer, &timerISR, true);
  timerAlarmWrite(pTimer, 500, true);
  timerAlarmEnable(pTimer);

  //Setup SmartLED
  SmartLEDs.begin();
  SmartLEDs.clear();
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(255, 0, 0));
  SmartLEDs.setBrightness(150);
  SmartLEDs.show();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  

  test = 2;
  switch (test)
  {
    case 0: //Moving servo via 10k potentiometer 
      val = analogRead(potPin);
      val = map(val, 0, 4095, 0, 180);
      
      for(int i = 0; i < val; i++)
      {
        servo.write(i);
        delay(15);
      }
      //servo.write(val);
      Serial.println(val);
      //delay(15);
      break;

    case 1: //Testing servo movement
      for(int i = 0; i < 180; i++)
      {
        servo.write(i);
        delay(20);
      }
      for(int i = 180; i > 0; i--)
      {
        servo.write(i);
        delay(20);
      }
      break;

    case 2: //Stepper motor control using A4988 stepper motor driver
      runState = true;
      digitalWrite(dirPin, HIGH);
      stepRate = map(speed, 0, 4095, 500, 60000);
      timerAlarmWrite(pTimer, stepRate, true);
      runState = false;
      break;
    default:
      break;

  }
}

void ARDUINO_ISR_ATTR timerISR() {
  if (runState) {                                      // Only send pulse if motor should be running
    digitalWrite(stepPin, !digitalRead(stepPin));    // toggle state of step pin
    if (stepDir) {
      stepCount++;                                     // add to count in forward direction
    }
    else {
      stepCount--;                                     // subtract from count in reverse direction
    }
  }
}

