#include <Arduino.h>

//#include <Wire.h>
#include <U8x8lib.h> // bibliothèque pour l'écran OLED

// Asservissement d'un moteur CC
// Mehdi 09/01/2022
//
// Contrôle par ATMega2560 qui permet la sortie série
// Les résultats de la sortie série peuvent être analysés dans XLS (courbe)
// Cette version by-passe le potentiomètre pour imposer une consigne déterminée (en particulier pour analyser la réponse d'une consigne "carrée")
// l'asservissement perso est remplacé par un asservissement PID de type standard :
// commande = Kp * (erreur + Kd * dérivée(erreur) + Ki Intégrale(erreur))

// choix du microcontrôleur cible
#define ATMEGA2560
//#define ATTINY85

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
#endif

//====================
// ATMEGA2560
//====================
#ifdef ATMEGA2560
#define PIN_pinIN_IRsensor A0
#define PIN_pinIN_Pot A1
#define PIN_pinPWM_Motor 2
#endif

unsigned long currentTime; // instant de le mesure courante du capteur

// Ecran OLED
// U8X8_SSD1306_128X64_NONAME_SW_I2C oled(SCL, SDA, /* reset=*/U8X8_PIN_NONE);

const int displayDelay{100}; // délai d'affichage. nb : on affiche peu souvent car l'affichage est long
unsigned long lastTimeDisplay;
unsigned long currentDisplayDelay; // délai depuis le dernier affichage

// spécification du signal carré qui sert de consigne
const int SquareHalfPeriod{3000}; // Demi période en ms du signal carré donné en consigne ; à chaque demi-période on bascule du niveau haut au niveua bas
unsigned long lastTimeSquareHalfPeriod;
unsigned long currentTimeSquareHalfPeriod;

// Capteur réflechissant TCRT5000
const uint8_t pinIN_IRsensor{PIN_pinIN_IRsensor}; // pin de lecture du capteur : HIGH si capte un objet réfléchissant à proximité sinon LOW
unsigned long changeStateCounter{0};              // compteur du nombre de changements d'état du capteur afin de calculer la vitesse du moteur

// hélice du moteur permettant de calculer sa vitesse
const int numberOfBlades{4}; // nombre de pales du moteur sur lesquelles se réflechit la lumière du capteur
                             // pour chaque pale on a 2 changements d'état du capteur

unsigned long lastTimeStateCounter;        // instant de la dernière mesure du capteur
const unsigned long stateCounterDelay{20}; // délai de mesure en milliseconde
unsigned long currentStateCounterDelay;    // délai depuis le début du comptage ; sert à déterminer la vitesse du moteur
// nb : une boucle loop() fait 160us pour le comptage, 2 ms pour l'asservissement et environ 276 ms pour l'affichage lcd !

// valeurs lues sur le capteur (HIGH ou LOW) ; old et new pour identifier les changements d'état
int sensorNewValue;
int sensorOldValue;

// vitesses du moteur
const float maxSpeed{8000}; // vitesse maximum du moteur en tr/min
int orderSpeed{0};          // vitesse de consigne pour le moteur (déterminée par la valeur du potentiomètre)
int measuredSpeed;          // vitesse mesurée du moteur en tr/min
float motorDC{0};           // tension DC appliquée sur le moteur pour se rapprocher de la  vitesse de consigne

// asservissement
// coefficient proportionnel appliqué à la différence entre la consigne et la mesure et qui est rajoutée/enlevée à la tension du moteur
// const float fraction{200.0 / 1.0};                 // fraction de la différence entre consigne et mesure appliquée pour rattrapper la consigne
// const float factor{fraction * (255.0 / maxSpeed)}; // cette fraction de vitesse 0 à maxSpeed est ramenée en fraction de commande moteur 0 à 254
//
// coefficients de l'asservissement PID
const float Kp{2};
const float Kd{0.0};
const float Ki{0.0};
long error;
float lastError{0.0};
float sumError{0.0};

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

//=========
// setup()
//=========
void setup()
{ /*
   // initialisation de l'écran OLED
   oled.begin();
   // oled.setPowerSave(0);
   oled.setFont(u8x8_font_chroma48medium8_r);

   oled.drawString(0, 0, "ASSERVISSEMENT");
   oled.drawString(0, 2, "consigne");
   oled.drawString(0, 3, "mesure");
   oled.drawString(0, 4, "\% erreur");
   oled.drawString(0, 5, "\% commande");
 */
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
  lastTimeSquareHalfPeriod = millis();

  // initialisation du moniteur série
  Serial.begin(115200);
}

//========
// loop()
//========
void loop()
{
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
  currentTimeSquareHalfPeriod = currentTime - lastTimeSquareHalfPeriod;

  if (currentStateCounterDelay >= stateCounterDelay)
  {
    measuredSpeed = coeffCalculVitesse * ((float)changeStateCounter / ((float)currentStateCounterDelay));
    // asservissement
    // on rattrape la consigne en injectant une fraction de la différence entre consigne et mesure (erreur)

    // lecture du potentiomètre qui fixe la consigne de vitesse du moteur en tr/min
    // orderSpeed = analogRead(pinIN_Pot) * maxSpeed / 1023;

    // Dans cette version la consigne est calculée. C'est un signal carré
    if (currentTimeSquareHalfPeriod >= SquareHalfPeriod)
    {
      if (orderSpeed == 0)
        orderSpeed = 4000;
      else
        orderSpeed = 0;

      lastTimeSquareHalfPeriod = currentTime;
    }

    // Régulation PID mixte
    error = orderSpeed - measuredSpeed;
    sumError += error;
    motorDC = Kp * (error + Kd * (error - lastError) + Ki * sumError);
    lastError = error;

    if (motorDC > 255)
      motorDC = 255;
    else if (motorDC < 0)
      motorDC = 0;

    /*
    // Régulation "perso"
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
    */

    motorRun(motorDC);

    // on recommence le comptage
    changeStateCounter = 0;
    lastTimeStateCounter = millis();
  }

  // affichage si délai atteint
  currentDisplayDelay = currentTime - lastTimeDisplay;
  if (currentDisplayDelay >= displayDelay)
  { /*
     // affichage de la vitesse de consigne du potentiomètre
     char str[5]; // 4 car + 0 de fin de string
     sprintf(str, "%4d", orderSpeed);
     oled.drawString(12, 2, str);

     // affichage de la vitesse mesurée
     sprintf(str, "%4d", measuredSpeed);
     oled.drawString(12, 3, str);

     // affichage de l'erreur mesurée
     double erreur = 100.0 * ((double)orderSpeed - (double)measuredSpeed) / (double)orderSpeed;
     sprintf(str, "%6d", (int)erreur); // pas de %f dans la librairie Arduino !
     oled.drawString(10, 4, str);

     // affichage du % de tension appliqué sur le moteur
     sprintf(str, "%3d", 100 * motorDC / 255);
     oled.drawString(13, 5, str);
 */
    // affichage sur le moniteur série
    Serial.print(orderSpeed);
    Serial.print(";");
    Serial.print(measuredSpeed);
    Serial.print(";");
    double erreur = 100.0 * ((double)orderSpeed - (double)measuredSpeed) / (double)orderSpeed;
    Serial.print(erreur);
    Serial.print(";");
    Serial.print(motorDC);
    Serial.println(";");

    lastTimeDisplay = currentTime;

    // après un affichage qui est long on remet le compteur à zéro pour recommencer le comptage après l'affichage et ne pas fausser la mesure
    // sinon on va rater le comptage des changements d'états pendant l'affichage
    changeStateCounter = 0;
    lastTimeStateCounter = millis();
  }
}
