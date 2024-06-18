#include <Arduino.h>
#include "dshot/esc.h"
#include "BlHeli-Passthrough/esc_passthrough.h"
#include <Wire.h>
#include "AS5600.h"
#include <Adafruit_NeoPixel.h>

#define triggerPin 9
#define throttlePin 6
#define LED_PIN 21

#define LED_COUNT 6

const DShot::Type escType = DShot::Type::Normal;
const DShot::Speed speed = DShot::Speed::DS600;
const unsigned int poles = 14;

DShot::ESC dshot(throttlePin, pio0, escType, speed, poles);

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
AS5600 as5600;

DShot::Telemetry telemetry = {0};

bool lastPressed = false;
uint32_t pressStartTime;
const double spinUpRampTime = 1.50;
uint64_t raw_telemetry;
// const double maximumThrottle = 0.55;
double knobAnglePercent = 0.0;
int32_t zeroPosition;
const int32_t span = 4096;

void bargraph(float throttle);
void bootAnimation();

void setup()
{
  Serial.begin(115200);
  Wire.setSCL(5);
  Wire.setSDA(4);
  Wire.setClock(400000);
  Wire.setTimeout(3);
  Wire.begin();
  strip.begin();
  strip.setBrightness(255);
  // strip.fill(strip.Color(0, 10, 128), 0, 6);
  strip.show();
  as5600.begin();
  // as5600.setAddress(0x40);
  // as5600.setDirection(AS5600_CLOCK_WISE);
  as5600.resetCumulativePosition(0);
  zeroPosition = as5600.getCumulativePosition();

  // if(as5600.detectMagnet()) strip.fill(strip.Color(0, 255, 0), 0, 6);
  // else strip.fill(strip.Color(128, 0, 0), 0, 6);
  strip.show();

  // while(!Serial);
  // delay(500);
  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(throttlePin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (!digitalRead(triggerPin)) // battery plugged in with trigger pulled
  {                             // enter esc passthrough mode
    uint32_t pressStart = millis();
    while (!digitalRead(triggerPin) && (millis() - pressStart < 4000))    //must be pulled for four seconds
      ;
    if (millis() - pressStart >= 4000)
    {
      strip.fill(strip.Color(128, 0, 0), 0, 6);
      strip.show();
      beginPassthrough(throttlePin);
      while (true)
      {
        bool breakout = processPassthrough(); // the function returns true when you hit the "disconnect" button in BLHeliSuite32. You can ignore that if you want to be able to reconnect

        if (breakout)
          break;
      }
      endPassthrough(); // this restores the previous state of the pins, i.e. which peripheral they were assigned to. It will also free the sm and delete the pio program from this passthrough.
      strip.fill(0, 0, 6);
      strip.show();
    }
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
  // while (millis() - startTime < 1000)
  dshot.setCommand(13); // extended telemetry enable
  bootAnimation();
  dshot.setCommand(0);
}

// const uint16_t knobDeadband = 200;

void loop()
{
  bool pressed = !digitalRead(triggerPin);
  // int knobAngle = as5600.readAngle();
  // Serial.println(as5600.getCumulativePosition());
  // if(knobAngle < knobDeadband) knobAngle = knobDeadband;
  // if(knobAngle > (4095 - knobDeadband)) knobAngle = (4095 - knobDeadband);
  // knobAngle = map(knobAngle, knobDeadband, 4095 - knobDeadband, 0, 10000);
  // knobAnglePercent = (double)knobAngle/10000.0;
  int32_t knobPosition = as5600.getCumulativePosition();
  if (knobPosition < zeroPosition)
    zeroPosition = knobPosition; // if we are turning the knob past the limits, just drag the "window" along too
  if (knobPosition > zeroPosition + span)
    zeroPosition = knobPosition - span;
  knobAnglePercent = (double)map(knobPosition, zeroPosition, zeroPosition + span, 0, 10000) / 10000.0;

  // Serial.println(knobAnglePercent, 2);
  if (!pressed)
  {
    dshot.setThrottle(0.0);
    digitalWrite(LED_BUILTIN, LOW);
    bargraph(knobAnglePercent);
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
    double duration = pressDuration / 1000.0;
    if (duration > spinUpRampTime)
      duration = spinUpRampTime;

    double throttle = duration / spinUpRampTime;
    if (throttle > knobAnglePercent)
    {
      dshot.setThrottle(0.0);
      bargraph(0.0);
    }
    else
    {
      dshot.setThrottle(throttle);
      bargraph(throttle);
    }
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

void bargraph(float throttle)
{
  if (throttle < 0.0)
    throttle = 0.0;
  if (throttle > 1.0)
    throttle = 1.0;
  uint16_t hue = 30000.0 - (throttle * 30000.00);
  uint32_t color = strip.ColorHSV(hue, 255, 128);
  int numPixels = throttle * 6;
  double fractional = throttle * 6.0 - (double)numPixels;
  for (int i = 0; i < numPixels; i++)
  {
    strip.setPixelColor(i, color);
  }
  strip.setPixelColor(numPixels, strip.ColorHSV(hue, 255, 128.0 * fractional));
  for (int i = numPixels + 1; i < 6; i++)
  {
    strip.setPixelColor(i, 0);
  }
  strip.show();
}

// brrzap kerbang
void bootAnimation()
{
  const uint32_t duration = 1500;
  uint32_t entryTime = millis();
  while (millis() - entryTime < duration)
  {
    uint32_t now = millis() - entryTime;
    float bar = 0.0;
    if (now < duration / 2)
    {
      bar = (2.0 * (float)now) / (float)duration;
    }
    else
    {
      bar = (2.0 * (float)duration - (2.0 * (float)now)) / (float)duration;
    }
    bargraph(bar);
    dshot.setCommand(13);
  }
}