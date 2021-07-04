#include <Arduino.h>

// variables globales
const uint8_t pinIN{13};             // pin de lecture du capteur : HIGH si capte un objet réfléchissant à proximité sinon LOW
unsigned long changeStateCounter{0}; // compte le nombre de changements d'état du capteur afin de calculer la vitesse du moteur

unsigned long oldTime;                        // instant du dernier changement de valeur
unsigned long currentTime;                    // instant de le mesure courante
const unsigned long delayMs{1000};            // délai de mesure en milliseconde
const unsigned long delaySec{delayMs / 1000}; // delayMs précédent ramené en sec pour optimiser le calcul de vitesse

int newValue;
int oldValue;

void setup()
{
  pinMode(pinIN, INPUT);
  Serial.begin(9600);

  oldTime = micros();
  oldValue = LOW;
}

void loop()
{
  //digitalRead(pinIN) == HIGH ? Serial.write("HAUT\n") : Serial.write("BAS\n");
  currentTime = millis();

  newValue = digitalRead(pinIN);
  if (newValue != oldValue)
  {
    changeStateCounter++;
    oldValue = newValue;
  }

  unsigned long d = currentTime - oldTime;
  if (d > delayMs)
  {
    Serial.println(changeStateCounter / delaySec);
    oldTime = currentTime;
    changeStateCounter = 0;
  }
}