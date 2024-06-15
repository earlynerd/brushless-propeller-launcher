#include <Arduino.h>
#include "dshot/esc.h"
#include "BlHeli-Passthrough/esc_passthrough.h"

#define triggerPin 9
#define throttlePin 5

const DShot::Type escType = DShot::Type::Normal;
const DShot::Speed speed = DShot::Speed::DS600;
const unsigned int poles = 14;

DShot::ESC dshot(throttlePin, pio0, escType, speed, poles);

void setup()
{
  Serial.begin(115200);
  // while(!Serial);
  //delay(500);
  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(throttlePin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (!digitalRead(triggerPin)) // battery plugged in with trigger pulled
  {                             // enter esc passthrough mode
    beginPassthrough(throttlePin);
    while (true)
    {
      bool breakout = processPassthrough(); // the function returns true when you hit the "disconnect" button in BLHeliSuite32. You can ignore that if you want to be able to reconnect
      if (breakout)
        break;
    }
    endPassthrough(); // this restores the previous state of the pins, i.e. which peripheral they were assigned to. It will also free the sm and delete the pio program from this passthrough.
  }
  if (dshot.init())
    Serial.println("DSHOT init success!");
  else
    Serial.println("DSHOT init fail");
  // delay(500);
  // delay(2);
  uint32_t startTime = millis();
  while (millis() - startTime < 1000)
    dshot.setCommand(13); // 1046 is the example command
  startTime = millis();
  while (millis() - startTime < 1000)
    dshot.setCommand(13); // extended telemetry enable
  dshot.setCommand(0);
}

DShot::Telemetry telemetry = {0};
bool lastPressed = false;
uint32_t pressStartTime;
const double spinUpRampTime = 1.50;
uint64_t raw_telemetry;
const double maximumThrottle = 0.55;

void loop()
{
  bool pressed = !digitalRead(triggerPin);
  if (!pressed)
  {
    dshot.setThrottle(0.0);
    digitalWrite(LED_BUILTIN, LOW);
    // Serial.println("throttle: 0.000");
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if (lastPressed == false)
    {
      pressStartTime = millis();
    }
    uint32_t pressDuration = millis() - pressStartTime;
    double duration = pressDuration / 1000.0;\
    if (duration > spinUpRampTime)
      duration = spinUpRampTime;

    double throttle = duration / spinUpRampTime;
    if(throttle > maximumThrottle) dshot.setThrottle(0.0);
    else dshot.setThrottle(throttle);
  }
  lastPressed = pressed;
  if (escType == DShot::Type::Bidir)
  {
    delay(2);
    // uint64_t raw_telemetry;

    if (dshot.getRawTelemetry(raw_telemetry))
    {
      // Serial.print("Tel: ");
      // print_bin(raw_telemetry);
      dshot.decodeTelemetry(raw_telemetry, telemetry);
      if (Serial)
      {
        Serial.print("rpm:");
        Serial.print(telemetry.rpm);
        Serial.print(", temperature_C:");
        Serial.print(telemetry.temperature_C);
        Serial.print(", volts_cV:");
        Serial.print(telemetry.volts_cV / 100);
        Serial.print(".");
        Serial.print(telemetry.volts_cV % 100);
        Serial.print(", amps_A:");
        Serial.print(telemetry.amps_A);
        Serial.print(", errors:");
        Serial.print(telemetry.errors);
        Serial.println("");
      }
    }
  }
  // else
  // {
  //  Serial.println("No telemetry :(");
  //}
}
