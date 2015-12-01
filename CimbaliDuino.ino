
#include <SoftwareSerial.h>

SoftwareSerial mySerial(1, 11); //  pin 0 = TX, pin 1 = RX (unused)
int pin = 11;
const int boilerFill = A0;    // pin that the sensor is attached to
const int fillSol = 2;       // attached to pin2
const int threshold = 10;   // threshold level that's in the range of the analog input
int pullshot = 9;    // right button
int pullsol = 3;    // opens grouphead solenoid
int ledLeft = 6;     // Left Light
int ledRight = 7;    // Right Light
int LCD = 10;        // LCD Power
//int LCDpower = 10;      // Left Button
int pump = 4;        // Left Button
int element = 13;    // Element Relay
int state = LOW;    // the current state of the output pin
int reading;         // the current reading from the input pin
int previous = HIGH;  // the previous reading from the input pin
long time = 0;       // the last time the output pin was toggled
long debounce = 200; // the debounce time, increase if the output flickers
int brightness = 0;  // how bright the LED is
int fadeAmount = 5;  // how many points to fade the LED by


void setup()
{
  pinMode(boilerFill, OUTPUT);
  pinMode(LCD, OUTPUT);
  pinMode(fillSol, OUTPUT);
  pinMode(pullshot, INPUT);
  //  pinMode(LCDpower, INPUT);
  pinMode(pullsol, OUTPUT);
  pinMode(ledLeft, OUTPUT);
  pinMode(ledRight, OUTPUT);
  pinMode(pump, OUTPUT);
  pinMode(element, OUTPUT);
  digitalWrite(element, LOW);
  digitalWrite(LCD, HIGH);



  // initialize serial communications:
  Serial.begin(9600);
  int inByte = 0;         // incoming serial byte
  boolean status_unlock;
  mySerial.begin(9600); // set up serial port for 9600 baud
  delay(500); // wait for display to boot up
  {
    mySerial.write(254); // cursor to beginning of first line
    mySerial.write(128);

    mySerial.write("------1994------"); // clear display + legends
    mySerial.write("-Cimbali Junior-");
    delay(1000);
    mySerial.write(254); // cursor to beginning of first line
    mySerial.write(128);

    mySerial.write("Boiler      Time"); // clear display + legends
    mySerial.write("   C           s");

  }
}
int buttonState = 0;         // variable for reading the pushbutton status
boolean shotIsPulling = false;
unsigned long timeShotStarted = 0;
int lastBoiler = 0;
unsigned long lastTime = 0;
void loop()
{
  // read the value of the probe:
  int analogValue = analogRead(boilerFill);

  // if the analog value is lower than threshold, turn on the fillSol:
  if (analogValue < threshold)
  {
    digitalWrite(fillSol, HIGH);
  }
  else
  {
    digitalWrite(fillSol, LOW);
  }

  // print the analog value:
  Serial.println(analogValue);
  delay(1);        // delay in between reads for stability
  buttonState = digitalRead(pullshot);

  reading = digitalRead(pullshot);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (reading == HIGH && previous == LOW && millis() - time > debounce)
  {
    if (state == HIGH)
      state = LOW;
    else
      state = HIGH;
    time = millis();
  }

  digitalWrite(pullsol, state);
  digitalWrite(ledRight, state);
  digitalWrite(pump, state);

  if (state == HIGH)
  {
    shotIsPulling = true;
    if (timeShotStarted == 0)
    {
      timeShotStarted = millis();
    }
  }
  else
  {
    shotIsPulling = false;
    timeShotStarted = 0;
  }
  mySerial.write(254); // cursor to 14th position on first line
  mySerial.write(205);

  if (shotIsPulling)
  {
    int time = (millis() - timeShotStarted) / 1000;
    if (time != lastTime)
    {
      if (time < 10)
      {
        mySerial.print(" ");
      }
      if (time > 99)
      {
        mySerial.print("");
      }
      else if (time > 0)
      {
        mySerial.print(time);
      }
      else
      {
        mySerial.print("*");
      }

      lastTime = time;
    }
  }
  else
  {
    mySerial.print("  ");
  }



  previous = reading;

  // set the brightness of pin 9:
  analogWrite(ledLeft, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness == 0 || brightness == 255) {
    fadeAmount = -fadeAmount ;
  }

  // wait for 30 milliseconds to see the dimming effect
  delay(30);
}

