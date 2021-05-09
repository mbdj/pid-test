#include <Arduino.h>

void setup()
{
// put your setup code here, to run once:
#define IN 13
  pinMode(IN, INPUT);
  Serial.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly:
  digitalRead(IN) == HIGH ? Serial.write("HAUT\n") : Serial.write("BAS\n");
}