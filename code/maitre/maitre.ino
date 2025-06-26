
#include <Servo.h>

Servo monServo;

// ===================== CONSTANTES CALIBRABLES =====================
const int vitesseMax = 250;
const int vitesseMoyenne = 150;
const int vitesseRalentie = 60;
const int vitesseInitiale = 100;

const unsigned long dureePalier1 = 1500;
const unsigned long dureePalier2 = 6300;

const int vitessePont = 80;
const unsigned long dureePont = 3600;

const unsigned long dureePompe = 8000;
const unsigned long prechargeDelai = 10000;
const unsigned long prechargeDuree = 7000;

const float KP = 6.0;
const float KI = 0.05;
const float KD = 0.6;
const float zoneMorte = 0.02;
const float integralMax = 50;

// ===================== PINS =====================
#define ENA 3
#define ENB 5
#define IN1 9
#define IN2 10
#define IN3 12
#define IN4 13
#define ENA2 44
#define ENB2 53
#define IN12 46
#define IN22 48
#define IN32 50
#define IN42 52
#define G1 6
#define G2 7
#define CAPTEUR_VIDE 4
#define BT_STATE 24
#define BOUTONUP 28
#define BOUTONDOWN 29
#define BOUTON_STOP 22
#define BOUTON_START 17

// ===================== VARIABLES =====================
float angleSlave = 0;
float tauxRotationSlave = 0;
float angleReconstruit = 0;
float integral = 0;
float lastError = 0;
unsigned long previousTime = 0;

unsigned long roueStartTime = 0;
bool roueActive = false;

unsigned long pontStartTime = 0;
bool pontActive = false;

unsigned long pompeStartTime = 0;
bool pompeActive = false;

bool vide = 0;

int etat = 0;
int etatmem = -1;
bool actionEtatFait = false;
bool attenteAck = false;
bool pretPourSuivant = false;

unsigned long prechargeStart = 0;
bool prechargeActive = false;
bool prechargeDemarree = false;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  monServo.attach(31);
  monServo.write(0);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(G1, OUTPUT);
  pinMode(G2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA2, OUTPUT);
  pinMode(ENB2, OUTPUT);
  pinMode(IN12, OUTPUT);
  pinMode(IN22, OUTPUT);
  pinMode(IN32, OUTPUT);
  pinMode(IN42, OUTPUT);

  pinMode(BOUTON_STOP, INPUT_PULLUP);
  pinMode(BOUTON_START, INPUT_PULLUP);
  pinMode(CAPTEUR_VIDE, INPUT_PULLUP);
  pinMode(BT_STATE, INPUT);

  digitalWrite(G2, LOW);
  analogWrite(G1, 0);
  MoteursMise0();
  Serial.println("[MAITRE] Initialisé.");
}

void loop() {
  unsigned long now = millis();
  verifierMessagesSlave();
  BTCheckARRET();
  vide = digitalRead(CAPTEUR_VIDE);

  if (etat != etatmem) {
    Serial.print("==> Passage à l'état ");
    Serial.println(etat);
    etatmem = etat;
    actionEtatFait = false;
    MoteursMise0();
    digitalWrite(G2, LOW);
  }

  switch (etat) {
    case 0:
      if (!actionEtatFait) {
        Serial.println("État 0 : Système à zéro. Appuyez sur START.");
        monServo.write(0);
        actionEtatFait = true;
      } else if (digitalRead(BOUTON_START) == LOW) {
        Serial.println("→ BOUTON_START appuyé. Passage à l'état 1");
        monServo.write(100);
        etat++;
        actionEtatFait = false;
      }
      break;

    case 1:
      if (!actionEtatFait) {
        Serial.println("Etat 1 : Avancée avec PID (angle reconstruit)");
        Serial1.println("ETAT1");
        previousTime = now;
        integral = 0;
        lastError = 0;
        roueStartTime = now;
        roueActive = true;
        angleReconstruit = 0;
        actionEtatFait = true;
      } else if (roueActive) {
        float dt = (now - previousTime) / 1000.0;
        previousTime = now;
        if (dt > 0) {
          angleReconstruit += tauxRotationSlave * dt;
          unsigned long tEcoule = now - roueStartTime;

          if (tEcoule <= dureePalier1) {
            AvancerAvecPID(vitesseInitiale, angleReconstruit, dt);
          } else if (tEcoule <= (dureePalier1 + dureePalier2)) {
            AvancerAvecPID(vitesseMax, angleReconstruit, dt);
          } else {
            AvancerSansPID(vitesseRalentie);
            if (digitalRead(CAPTEUR_VIDE) == HIGH) {
              Serial.println("→ Vide détecté, arrêt moteurs");
              MoteursMise0();
              roueActive = false;
              Serial1.println("STOP_ANGLE");
              etat++;
              actionEtatFait = false;
            }
          }
        }
      }
      break;

    case 2:
      if (!actionEtatFait) {
        Serial.println("État 2 : Déploiement du pont");
        pontStartTime = now;
        pontActive = true;
        digitalWrite(IN12, HIGH);
        digitalWrite(IN22, LOW);
        digitalWrite(ENA2, HIGH);        // Active l'alim
        analogWrite(ENA2, vitessePont);  // Démarre le pont
        actionEtatFait = true;
      } else if (pontActive && now - pontStartTime >= dureePont) {
        Serial.println("→ Déploiement terminé. Passage à l'état 3.");
        digitalWrite(IN12, LOW);
        digitalWrite(IN22, LOW);
        analogWrite(ENA2, 0);
        digitalWrite(ENA2, LOW);
        digitalWrite(G2, LOW);
        pontActive = false;
        etat++;
        actionEtatFait = false;
      }
      break;

    case 3:
      if (!actionEtatFait) {
        Serial.println("État 3 : Mouvement du robot esclave");
        Serial1.println("ETAT3");
        Serial.println("→ Ordre envoyé au SLAVE : ETAT3");
        attenteAck = true;
        actionEtatFait = true;
      }
      break;

    case 4:
      if (!actionEtatFait) {
        Serial.println("État 4 : Activation pompe finale");
        pompeStartTime = now;
        pompeActive = true;
        analogWrite(G1, 200);
        digitalWrite(G2, HIGH);
        actionEtatFait = true;
      } else if (pompeActive && now - pompeStartTime >= dureePompe) {
        analogWrite(G1, 0);
        digitalWrite(G2, LOW);
        pompeActive = false;
        etat++;
        actionEtatFait = false;
      }
      break;

    case 5:
      if (!actionEtatFait) {
        Serial.println("État 5 : Recul du robot esclave");
        Serial1.println("ETAT5");
        attenteAck = true;
        actionEtatFait = true;
      }
      break;
  }

  static unsigned long dernierAffichage = 0;
  if (etat == 1 && millis() - dernierAffichage >= 100) {
    Serial.print("Gyro: ");
    Serial.print(tauxRotationSlave);
    Serial.print(" | Angle: ");
    Serial.println(angleReconstruit);
    dernierAffichage = millis();
  }
}

// ===================== PID ET MOUVEMENT =====================
void AvancerAvecPID(int vitesseCible, float angle, float dt) {
  if (abs(angle) < zoneMorte) angle = 0;
  float error = -angle;

  integral += error * dt;
  integral = constrain(integral, -integralMax, integralMax);
  float derivative = (error - lastError) / dt;
  lastError = error;

  float correction = KP * error + KI * integral + KD * derivative;
  correction = constrain(correction, -60, 60);
  if (abs(correction) < 20) correction = (correction > 0) ? 20 : -20;

  float vitesseG = constrain(vitesseCible - correction, 0, 255);
  float vitesseD = constrain(vitesseCible + correction, 0, 255);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, vitesseG);
  analogWrite(ENB, vitesseD);
}

void AvancerSansPID(int vitesseFixe) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, vitesseFixe);
  analogWrite(ENB, vitesseFixe);
}

void MoteursMise0() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(G1, LOW);
}

// ===================== COMMUNICATION =====================
void BTCheckARRET() {
  if (digitalRead(BOUTON_STOP) == LOW || digitalRead(BT_STATE) == LOW) {
    Serial.println("==> ARRET D'URGENCE / BT PERDU");
    resetSystem();
  }
}

void verifierMessagesSlave() {
  if (Serial1.available()) {
    String msg = Serial1.readStringUntil('\n');
    msg.trim();
    if (msg.startsWith("G:")) {
      tauxRotationSlave = msg.substring(2).toFloat();
    } else if (msg == "FIN_ETAT") {
      Serial.println("→ FIN_ETAT reçu");
      attenteAck = false;
      etat++;
      actionEtatFait = false;
    }
  }
}

void resetSystem() {
  etat = 0;
  etatmem = -1;
  actionEtatFait = false;
  attenteAck = false;
  roueActive = false;
  pontActive = false;
  pompeActive = false;
  integral = 0;
  lastError = 0;
  previousTime = 0;
  angleReconstruit = 0;
  tauxRotationSlave = 0;
  MoteursMise0();
  digitalWrite(G2, LOW);
  digitalWrite(G1, LOW);
}
