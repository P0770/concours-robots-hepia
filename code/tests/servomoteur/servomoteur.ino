#include <Servo.h>

Servo monServo;

const int boutonPin = 8;     // Broche du bouton
int etatBouton = 0;          // État lu sur la broche du bouton

void setup() {
  monServo.attach(9);        // Attache le servomoteur à la broche 9
  monServo.write(0);         // Initialise la position à 0°

  pinMode(boutonPin, INPUT_PULLUP);  // Active la résistance de pull-up interne
}

void loop() {
  etatBouton = digitalRead(boutonPin);   // Lit l'état du bouton

  if (etatBouton == HIGH) {               // Si le bouton est appuyé (état bas)
    monServo.write(100);                  // Tourne à 90°
    delay(1500);                         // Pause 10 secondes
  }
}
