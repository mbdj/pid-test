#include <Arduino.h>

// variables globales pour le capteur réflechissant
const uint8_t pinIN_IRsensor{13};    // pin de lecture du capteur : HIGH si capte un objet réfléchissant à proximité sinon LOW
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

//=====================
// Contrôles du moteur
//=====================
// Commande du moteur
//const uint8_t pinPWM_EN{3};    // pin 1 du L293D : la valeur pwm détermine la vitesse du moteur
//const uint8_t pinOUTPUT_1A{4}; // pin 2 du L293D : 1A et 2A sont les switchs qui contrôlent le sens ou l'arrêt du moteur
//const uint8_t pinOUTPUT_2A{7}; // pin 7 du L293D : 1A et 2A sont les switchs qui contrôlent le sens ou l'arrêt du moteur
const uint8_t pinPWM_Motor{3}; // pin contrôlant la vitesse du moteur par PWM (connecté à la gate du mosfet alimentant le moteur)

//
inline void run(int rate)
{
  analogWrite(pinPWM_Motor, rate);
}

/*
// tourne dans un sens
void forward(int rate)
{
  //analogWrite(pinPWM_EN, LOW);
  //digitalWrite(pinOUTPUT_1A, HIGH);
  //digitalWrite(pinOUTPUT_2A, LOW);
  analogWrite(pinPWM_EN, rate);
}

// tourne dans l'autre sens
void reverse(int rate)
{
  analogWrite(pinPWM_EN, LOW);
  digitalWrite(pinOUTPUT_1A, LOW);
  digitalWrite(pinOUTPUT_2A, HIGH);
  analogWrite(pinPWM_EN, rate);
}

// arrêt du moteur
void brake(int rate)
{
  analogWrite(pinPWM_EN, LOW);
  digitalWrite(pinOUTPUT_1A, LOW);
  digitalWrite(pinOUTPUT_2A, LOW);
  analogWrite(pinPWM_EN, HIGH);
}
*/

const uint8_t pinIN_Pot{4}; // pin pour la lecture du potentiomètre ; sa valeur fixe la vitesse du moteur 0 à 255

void setup()
{
  Serial.begin(9600);

  // initialisation de la pin du potentiomètre
  pinMode(pinIN_Pot, INPUT);

  // initialisation du capteur IR
  pinMode(pinIN_IRsensor, INPUT);
  beginTime = micros(); // initialisation
  oldValue = LOW;       // initialisation. L'état est supposé LOW au départ

  // initialisation du moteur
  pinMode(pinPWM_Motor, OUTPUT);
  /*
  pinMode(pinPWM_EN, OUTPUT);
  pinMode(pinOUTPUT_1A, OUTPUT);
  pinMode(pinOUTPUT_2A, OUTPUT);
  */
}

void loop()
{
  // affichage de l'état du capteur : HAUT = détection ; BAS = pas de détection
  //digitalRead(pinINSensor) == HIGH ? Serial.write("HAUT\n") : Serial.write("BAS\n");

// lecture du potentiomètre qui fixe la vitesse du moteur
// et activation du moteur
  int v = map(analogRead(pinIN_Pot),0,1023,0,255);
  Serial.println(v);
  run(v);

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
    float speedTrMin = changeStateCounter * k;
    //Serial.println(speedTrMin);
    beginTime = currentTime;
    changeStateCounter = 0;
  }
}