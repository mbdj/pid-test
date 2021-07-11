#include <Arduino.h>

// Pour l'écran lcd
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// variables globales pour le capteur réflechissant
const uint8_t pinIN_IRsensor{12};    // pin de lecture du capteur : HIGH si capte un objet réfléchissant à proximité sinon LOW
unsigned long changeStateCounter{0}; // compte le nombre de changements d'état du capteur afin de calculer la vitesse du moteur

const int numberOfBlades{4}; // nombre de pales du moteur sur lesquelles se réflechit la lumière du capteur
                             // pour chaque pale on a 2 changements du capteur

unsigned long beginTime;                                          // instant du dernier changement de valeur
unsigned long currentTime;                                        // instant de le mesure courante
const unsigned long delayMs{1000};                                // délai de mesure en milliseconde
const float k{(float)delayMs / ((float)numberOfBlades * 1000.0)}; // coefficient par lequel il faut multiplier le nombre de changements pour avoir la vitesse en tr/sec

// valeurs lues sur le capteur old et new pour identifier les changements d'état
int newValue;
int oldValue;

// vitesse mesurée du moteur
float measuredSpeed; // vitesse du moteur en tr/min
int motorDC{1};      // tension DC appliquée sur le moteur

//=====================
// Contrôles du moteur
//=====================
const uint8_t pinPWM_Motor{3}; // pin contrôlant la vitesse du moteur par PWM (connecté à la gate du mosfet alimentant le moteur)

inline void motorRun(int rate)
{
  analogWrite(pinPWM_Motor, rate);
}

// Potentionmètre qui fixe la vitesse du moteur
const uint8_t pinIN_Pot{4}; // pin pour la lecture du potentiomètre ; sa valeur fixe la vitesse du moteur 0 à 255

void setup()
{
  //Serial.begin(9600);

  // initialisation du lcd
  lcd.init();
  lcd.backlight();

  // initialisation de la pin du potentiomètre
  pinMode(pinIN_Pot, INPUT);

  // initialisation du capteur IR
  pinMode(pinIN_IRsensor, INPUT);
  beginTime = micros(); // initialisation
  oldValue = LOW;       // initialisation. L'état est supposé LOW au départ

  // initialisation du moteur
  pinMode(pinPWM_Motor, OUTPUT);
}

void loop()
{
  // affichage de l'état du capteur : HAUT = détection ; BAS = pas de détection
  //digitalRead(pinIN_IRsensor) == HIGH ? Serial.write("HAUT\n") : Serial.write("BAS\n");

  // lecture du potentiomètre qui fixe la consigne de vitesse du moteur
  int orderSpeed = map(analogRead(pinIN_Pot), 0, 1023, 0, 255);

  // On incrémente un compteur durant le délai delayMs
  // Si on détecte un changement d'état du capteur on incrémente le compteur
  currentTime = millis();
  newValue = digitalRead(pinIN_IRsensor);
  if (newValue != oldValue)
  {
    changeStateCounter++;
    oldValue = newValue;
  }

  // quand le delai delayMs est atteint on affiche le résultat et on réinitialise le compteur
  unsigned long d = currentTime - beginTime; // délai depuis le début du comptage
  if (d > delayMs)
  {
    measuredSpeed = changeStateCounter * k;

    // asservissement basique (mais perso !)
    // on rattrape la consigne en injectant le 1/3 de la différence entre consigne et mesure
    motorDC += (orderSpeed - measuredSpeed) / 2;

    motorRun(motorDC);

    beginTime = currentTime;
    changeStateCounter = 0;

    // Affichage de la consigne et de la vitesse
    // tous les delayMs également
    lcd.clear();
    lcd.setCursor(0, 0); // ligne 0
    lcd.print("consigne :");
    lcd.print(orderSpeed);
    lcd.setCursor(0, 1); // ligne 1
    lcd.print("mesure   :");
    lcd.print(measuredSpeed);
  }
}