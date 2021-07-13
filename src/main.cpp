#include <Arduino.h>

// Asservissement d'un moteur CC avec un ATTiny85
// Mehdi 13/07/2021

// pin out ATTINY85
//                      -----
//               RESET-|1   8|-5V
//  Pot >       A3-PB3-|2   7|-PB2-A1-SCL   > LCD SCL
//  IRsensor >  A2-PB4-|3   6|-PB1-PWM      > Motor (via gate du mosfet)
//                 GND-|4   5|-PB0-PWM-SDA  > LCD SDA
//                      -----

#define PIN_pinIN_IRsensor PB4
#define PIN_pinIN_Pot PB3
#define PIN_pinPWM_Motor PB1

// Ecran lcd pour afficher la consigne et la mesure de vitesse
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// Capteur réflechissant TCRT5000
const uint8_t pinIN_IRsensor{PIN_pinIN_IRsensor}; // pin de lecture du capteur : HIGH si capte un objet réfléchissant à proximité sinon LOW
unsigned long changeStateCounter{0};              // compteur du nombre de changements d'état du capteur afin de calculer la vitesse du moteur

// hélice du moteur permettant de calculer sa vitesse
const int numberOfBlades{4}; // nombre de pales du moteur sur lesquelles se réflechit la lumière du capteur
                             // pour chaque pale on a 2 changements d'état du capteur

unsigned long lastTime;                                                    // instant de la dernière mesure du capteur
unsigned long currentTime;                                                 // instant de le mesure courante du capteur
const unsigned long delayMs{500};                                          // délai de mesure en milliseconde
const float rpm{60000.0 / ((float)numberOfBlades * 2.0 * (float)delayMs)}; // coefficient par lequel il faut multiplier le nombre de changements pour avoir la vitesse en tr/min

// valeurs lues sur le capteur (HIGH ou LOW) ; old et new pour identifier les changements d'état
int newValue;
int oldValue;

// vitesse mesurée du moteur
const int maxSpeed{6000}; // vitesse maximum du moteur
float measuredSpeed;      // vitesse du moteur en tr/min
float motorDC{0};         // tension DC appliquée sur le moteur pour obtenir une certaine vitesse

// asservissement
// coefficient proportionnel appliqué à la différence entre la consigne et la mesure et qui est rajoutée/enlevée à la tension du moteur
const float fraction{1.0 / 2.0};                          // fraction de la différence entre consigne et mesure appliquée pour rattrapper la consigne
const float factor{fraction * (254.0 / (float)maxSpeed)}; // cette fraction de vitesse 0 à maxSpeed est ramenée en fraction de commande moteur 0 à 254

//=====================
// Contrôles du moteur
//=====================
const uint8_t pinPWM_Motor{PIN_pinPWM_Motor}; // pin contrôlant la vitesse du moteur par PWM (connecté à la gate du mosfet alimentant le moteur)

inline void motorRun(int rate)
{
  analogWrite(pinPWM_Motor, rate);
}

// Potentionmètre qui fixe la vitesse du moteur
const uint8_t pinIN_Pot{PIN_pinIN_Pot}; // pin pour la lecture du potentiomètre ; sa valeur fixe la vitesse du moteur 0 à maxSpeed tr/min (0 à 1024)

void setup()
{
  // initialisation du lcd
  lcd.init();
  lcd.backlight();

  // initialisation de la pin du potentiomètre
  pinMode(pinIN_Pot, INPUT);

  // initialisation du capteur IR
  pinMode(pinIN_IRsensor, INPUT);
  lastTime = micros(); // initialisation
  oldValue = LOW;      // initialisation. L'état est supposé LOW au départ

  // initialisation du moteur
  pinMode(pinPWM_Motor, OUTPUT);
}

void loop()
{
  // lecture du potentiomètre qui fixe la consigne de vitesse du moteur en tr/min
  int orderSpeed = map(analogRead(pinIN_Pot), 0, 1023, 0, maxSpeed);

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
  unsigned long d = currentTime - lastTime; // délai depuis le début du comptage
  if (d > delayMs)
  {
    measuredSpeed = changeStateCounter * rpm;

    // asservissement basique (mais perso !)
    // on rattrape la consigne en injectant une fraction de la différence entre consigne et mesure

    if (orderSpeed > measuredSpeed)
    {
      motorDC += (orderSpeed - measuredSpeed) * factor;
      if (motorDC > 254)
        motorDC = 254;
    }
    else
    {
      motorDC -= (measuredSpeed - orderSpeed) * factor;
      if (motorDC < 0)
        motorDC = 0;
    }

    motorRun(motorDC);

    lastTime = currentTime;
    changeStateCounter = 0;

    // Affichage de la consigne et de la vitesse sur le lcd
    // tous les delayMs également
    lcd.setCursor(0, 0); // ligne 0
    lcd.print("consigne :");
    lcd.print(orderSpeed);
    lcd.print("      ");
    lcd.setCursor(0, 1); // ligne 1
    lcd.print("mesure   :");
    lcd.print(measuredSpeed);
    lcd.print("      ");
  }
}