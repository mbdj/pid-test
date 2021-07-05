#include <Arduino.h>

// variables globales
const uint8_t pinIN{13};             // pin de lecture du capteur : HIGH si capte un objet réfléchissant à proximité sinon LOW
unsigned long changeStateCounter{0}; // compte le nombre de changements d'état du capteur afin de calculer la vitesse du moteur

const int numberOfBlades{4}; // nombre de pales du moteur
                             // pour chaque pale on a 2 changements du capteur

unsigned long beginTime;                                  // instant du dernier changement de valeur
unsigned long currentTime;                                // instant de le mesure courante
const unsigned long delayMs{1000};                        // délai de mesure en milliseconde
const unsigned long k{delayMs / (numberOfBlades * 1000)}; // coefficient par lequel il faut multiplier le nombre de changements pour avoir la vitesse en tr/sec
int newValue;
int oldValue;

void setup()
{
  pinMode(pinIN, INPUT);
  Serial.begin(9600);

  beginTime = micros(); // initialisation
  oldValue = LOW;       // initialisation. L'état est supposé LOW au départ
}

void loop()
{
  // affichage de l'état du capteur : HAUT = détection ; BAS = pas de détection
  //digitalRead(pinIN) == HIGH ? Serial.write("HAUT\n") : Serial.write("BAS\n");

  // On incrémente un compteur durant le délai delayMs

  // Si on détecte un changement d'état du capteur on incrémente le compteur
  // et la nouvelle valeur devient la valeur "ancienne" à comparer
  currentTime = millis();
  newValue = digitalRead(pinIN);
  if (newValue != oldValue)
  {
    changeStateCounter++;
    oldValue = newValue;
  }

  // quand le delai delayMs est atteint on affiche le résultat et on réinitialise le compteur
  unsigned long d = currentTime - beginTime; // délai depuis le début du comptage
  if (d > delayMs)
  {
    Serial.println(changeStateCounter * k);
    beginTime = currentTime;
    changeStateCounter = 0;
  }
}