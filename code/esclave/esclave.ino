// ===================== LIBRAIRIES & OBJETS =====================
#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
MPU6050 mpu;
SoftwareSerial BT(7, 8);  // RX, TX

// ===================== PARAMÈTRES CALIBRABLES =====================
// PID
const float KP_SLAVE = 4.5;
const float KI_SLAVE = 0.0;
const float KD_SLAVE = 1.0;
const float alphaGyro = 0.9;  // pour filtrage exponentiel

// Vitesse et phases de l'état 3
const int vitessePont = 120;             // Phase 1 : passage du pont
const int vitesseRapide = 250;           // Phase 2 : vitesse max
const int vitesseFinale = 60;            // Phase 3 : approche lente avec détection bord
const unsigned long dureePont = 4000;    // Durée sans détection (sur le pont)
const unsigned long dureeRapide = 15000;  // Durée après le pont (vitesse rapide)

// Recul (ETAT5)
const unsigned long DUREE_RECUL = 3000;

// Ralentissement lors du bord
const int PAS_RALENTI_SLAVE = 10;
const unsigned long DELAI_RALENTI_SLAVE = 100;

// Fréquence d’envoi des angles (ETAT1 uniquement)
const unsigned long DELAI_ENVOI_ANGLE = 50;

// ===================== PINS =====================
#define ENA 3
#define ENB 5
#define IN1 9
#define IN2 10
#define IN3 12
#define IN4 13
#define SWITCH_BORD 4

// ===================== VARIABLES =====================
float angle = 0;
float integral = 0;
float lastError = 0;
unsigned long previousTime = 0;
unsigned long dernierEnvoi = 0;
unsigned long debutMouvement = 0;

bool envoyerAngles = false;
bool enMouvement = false;
int etatMouvement = 0;

// ===================== SETUP =====================
void setup() {
  BT.begin(9600);
  Wire.begin();
  mpu.initialize();
  while (!mpu.testConnection()) delay(500);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SWITCH_BORD, INPUT_PULLUP);

  arretMoteurs();
}

// ===================== LOOP =====================
void loop() {
  unsigned long now = millis();

  // Réception des commandes Bluetooth
  if (BT.available()) {
    String commande = BT.readStringUntil('\n');
    commande.trim();

    if (commande == "ETAT1") {
      envoyerAngles = true;

    } else if (commande == "STOP_ANGLE") {
      envoyerAngles = false;

    } else if (commande == "ETAT3") {
      etatMouvement = 3;
      enMouvement = true;
      envoyerAngles = false;
      debutMouvement = now;

    } else if (commande == "ETAT5") {
      etatMouvement = 5;
      enMouvement = true;
      envoyerAngles = false;
      debutMouvement = now;
      reculer();
    }
  }

  // Envoi régulier de l’angle (ETAT1)
  if (envoyerAngles && now - dernierEnvoi >= DELAI_ENVOI_ANGLE) {
    float taux = getTauxRotation();
    BT.print("G:");
    BT.println(taux);
    dernierEnvoi = now;
  }

  // Gestion des mouvements
  if (enMouvement && etatMouvement == 3) {
    gererAvanceEtat3(now);
  }

  if (enMouvement && etatMouvement == 5 && now - debutMouvement >= DUREE_RECUL) {
    arretMoteurs();
    BT.println("FIN_ETAT");
    enMouvement = false;
    etatMouvement = 0;
  }
}

// ===================== MOUVEMENT ÉTAT 3 =====================
void gererAvanceEtat3(unsigned long now) {
  unsigned long t = now - debutMouvement;
  unsigned long t1 = dureePont;
  unsigned long t2 = t1 + dureeRapide;

  if (t < t1) {
    avancerConstamment(vitessePont);

  } else if (t < t2) {
    avancerConstamment(vitesseRapide);

  } else {
    avancerConstamment(vitesseFinale);

    if (digitalRead(SWITCH_BORD) == LOW) {
      arretMoteurs();  // 🔧 arrêt net
      BT.println("FIN_ETAT");
      enMouvement = false;
      etatMouvement = 0;
    }
  }
}

// ===================== MOTEURS =====================
void avancerConstamment(int vit) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, vit);
  analogWrite(ENB, vit);
}

void reculer() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void ralentirEtArreter() {
  for (int v = vitesseRapide; v >= 0; v -= PAS_RALENTI_SLAVE) {
    avancerConstamment(v);
    delay(DELAI_RALENTI_SLAVE);
  }
  arretMoteurs();
}

void arretMoteurs() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

// ===================== MPU6050 =====================
float getTauxRotation() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  return gy / 131.0;  // taux de rotation en °/s
}
