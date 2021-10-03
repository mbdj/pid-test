#include <Arduino.h>

//#include <Wire.h>
#include <U8x8lib.h> // bibliothèque pour l'écran OLED

//#include <LiquidCrystal_I2C.h>

// Asservissement d'un moteur CC
// Mehdi 05/09/2021

// choix du microcontrôleur cible
//#define ATMEGA2560
#define ATTINY85

//====================
// ATTINY85
//====================
//
// pin out ATTINY85
//                      -----
//               RESET-|1   8|-5V
//  Pot >       A3-PB3-|2   7|-PB2-A1-SCL   > LCD SCL
//  IRsensor >  A2-PB4-|3   6|-PB1-PWM      > Motor (via gate du mosfet)
//                 GND-|4   5|-PB0-PWM-SDA  > LCD SDA
//                      -----
#ifdef ATTINY85
#define PIN_pinIN_IRsensor PB4
#define PIN_pinIN_Pot PB3
#define PIN_pinPWM_Motor PB1

const long cpu_frequency{1000000}; // fréquence de l'ATTiny85
const unsigned int ms{1000000 / cpu_frequency};
#endif

//====================
// ATMEGA2560
//====================
#ifdef ATMEGA2560
#define PIN_pinIN_IRsensor A0
#define PIN_pinIN_Pot A1
#define PIN_pinPWM_Motor 2
const unsigned int ms{1};
#endif

unsigned long currentTime; // instant de le mesure courante du capteur

// Ecran lcd pour afficher la consigne et la mesure de vitesse

//LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// Ecran OLED
U8X8_SH1106_128X64_NONAME_HW_I2C OLED(U8X8_PIN_NONE); //, SCL, SDA); // reset, clock, data

const int displayDelay{2000 * ms}; // délai d'affichage. nb : on affiche peu souvent car l'affichage dure environ 1/3 s !
unsigned long lastTimeDisplay;

// Capteur réflechissant TCRT5000
const uint8_t pinIN_IRsensor{PIN_pinIN_IRsensor}; // pin de lecture du capteur : HIGH si capte un objet réfléchissant à proximité sinon LOW
unsigned long changeStateCounter{0};              // compteur du nombre de changements d'état du capteur afin de calculer la vitesse du moteur

// hélice du moteur permettant de calculer sa vitesse
const int numberOfBlades{4}; // nombre de pales du moteur sur lesquelles se réflechit la lumière du capteur
                             // pour chaque pale on a 2 changements d'état du capteur

unsigned long lastTimeStateCounter;              // instant de la dernière mesure du capteur
const unsigned long stateCounterDelay{100 * ms}; // délai de mesure en milliseconde
// nb : une boucle loop() fait 160us pour le comptage, 2 ms pour l'asservissement et environ 276 ms pour l'affichage lcd !

// valeurs lues sur le capteur (HIGH ou LOW) ; old et new pour identifier les changements d'état
int sensorNewValue;
int sensorOldValue;

// vitesses du moteur
const float maxSpeed{8000}; // vitesse maximum du moteur en tr/min
int orderSpeed{0};          // vitesse de consigne pour le moteur (déterminée par la valeur du potentiomètre)
int measuredSpeed;          // vitesse mesurée du moteur en tr/min
int motorDC{0};             // tension DC appliquée sur le moteur pour se rapprocher de la  vitesse de consigne

unsigned long currentStateCounterDelay; // délai depuis le début du comptage ; sert à déterminer la vitesse du moteur
unsigned long currentDisplayDelay;      // délai depuis le dernier affichage

// asservissement
// coefficient proportionnel appliqué à la différence entre la consigne et la mesure et qui est rajoutée/enlevée à la tension du moteur
const float fraction{1.0 / 2.0};                   // fraction de la différence entre consigne et mesure appliquée pour rattrapper la consigne
const float factor{fraction * (255.0 / maxSpeed)}; // cette fraction de vitesse 0 à maxSpeed est ramenée en fraction de commande moteur 0 à 254

// pré-calcul de 2 coefficients pour des raisons de performance
const float coeffCalculVitesse{60000.0 / ((float)numberOfBlades * 2.0)}; // pour le calcul de la vitesse mesurée
const float maxSpeedDivisePar1023 = maxSpeed / 1023.0;                   // pour le calcul de la vitesse consigne

//=====================
// Contrôle du moteur
//=====================
const uint8_t pinPWM_Motor{PIN_pinPWM_Motor}; // pin contrôlant la vitesse du moteur par PWM (connecté à la gate du mosfet alimentant le moteur)

inline void motorRun(uint8_t rate)
{
  analogWrite(pinPWM_Motor, rate);
}

// Potentiomètre qui fixe la vitesse du moteur
const uint8_t pinIN_Pot{PIN_pinIN_Pot}; // pin pour la lecture du potentiomètre ; sa valeur fixe la vitesse du moteur 0 à maxSpeed tr/min (0 à 1024)

unsigned long beginLoop;

//=========
// setup()
//=========
void setup()
{
  // initialisation du lcd
  //lcd.init();
  //lcd.backlight();

  // initialisation de l'écran OLED
  OLED.begin();
  OLED.setPowerSave(0);

  // initialisation du potentiomètre
  pinMode(pinIN_Pot, INPUT);

  // initialisation du capteur IR
  pinMode(pinIN_IRsensor, INPUT);
  sensorOldValue = LOW; // initialisation. L'état est LOW au départ

  // initialisation du moteur
  pinMode(pinPWM_Motor, OUTPUT);

  // initialisation des instants de calcul
  lastTimeDisplay = millis();
  lastTimeStateCounter = millis(); // initialisation des instants de mesure
}

//========
// loop()
//========
void loop()
{
  OLED.setFont(u8x8_font_chroma48medium8_r);
  OLED.drawString(0, 1, "Hello World!");
  OLED.refreshDisplay();

  // Si on détecte un changement d'état du capteur on incrémente le compteur
  sensorNewValue = digitalRead(pinIN_IRsensor);
  if (sensorNewValue != sensorOldValue)
  {
    changeStateCounter++;
    sensorOldValue = sensorNewValue;
  }

  // quand le delai de mesure est atteint on recalcule la tension à appliquer sur le moteur et on réinitialise le compteur
  currentTime = millis();
  currentStateCounterDelay = currentTime - lastTimeStateCounter; // délai depuis le début du comptage

  if (currentStateCounterDelay >= stateCounterDelay)
  {
    measuredSpeed = coeffCalculVitesse * ((float)changeStateCounter / ((float)currentStateCounterDelay));
    // asservissement proportionnel
    // on rattrape la consigne en injectant une fraction de la différence entre consigne et mesure

    // lecture du potentiomètre qui fixe la consigne de vitesse du moteur en tr/min
    orderSpeed = analogRead(pinIN_Pot) * maxSpeed / 1023;

    if (orderSpeed > measuredSpeed)
    {
      motorDC += (orderSpeed - measuredSpeed) * factor;
      if (motorDC > 255)
        motorDC = 255;
    }
    else if (orderSpeed < measuredSpeed)
    {
      motorDC -= (measuredSpeed - orderSpeed) * factor;
      if (motorDC < 0)
        motorDC = 0;
    }

    motorRun(motorDC);

    // on recommence le comptage
    changeStateCounter = 0;
    lastTimeStateCounter = millis();
  }

  // affichage si délai atteint
  currentDisplayDelay = currentTime - lastTimeDisplay;
  if (currentDisplayDelay >= displayDelay)
  {
    // Affichage de la consigne et de la vitesse sur le lcd
    //lcd.setCursor(0, 0); // ligne 0
    //lcd.print("consigne :");
    //lcd.print(orderSpeed);
    //lcd.print("      ");
    //lcd.setCursor(0, 1); // ligne 1
    //lcd.print("mesure   :");
    //lcd.print(measuredSpeed);
    //lcd.print("      ");

    lastTimeDisplay = currentTime;

    // après un affichage qui est très long (276 ms) on remet le compteur à zéro pour recommencer le comptage après l'affichage et ne pas fausser la mesure
    // sinon on va rater le comptage des changements d'états pendant près de 1/3 sec
    changeStateCounter = 0;
    lastTimeStateCounter = millis();
  }
}