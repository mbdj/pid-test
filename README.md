Asservissement de la vitesse d'un moteur cc
Asservissement proportionnel (on injecte une commande proportionnelle au delta entre mesure et consigne)
La consigne est donnée par un potentiomètre
vitesse de consigne et vitesse mesurée affichées sur un lcd
commande du moteur par pwm

microcontroleur ATTINY85 :
                      -----
               RESET-|1   8|-5V
  Pot >       A3-PB3-|2   7|-PB2-A1-SCL   > LCD SCL
  IRsensor >  A2-PB4-|3   6|-PB1-PWM      > Motor (via gate du mosfet)
                 GND-|4   5|-PB0-PWM-SDA  > LCD SDA
                      -----

