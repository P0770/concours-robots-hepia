// ===================== CALIBRAGE CENTRALISÉ SLAVE =====================

// --- PID gyroscope ---
const float KP_SLAVE = 4.5;
const float KI_SLAVE = 0.0;
const float KD_SLAVE = 1.0;

// --- Mouvement & PID ---
const int VITESSE_SLAVE = 80;
const unsigned long DELAI_SANS_PID = 2000;
const unsigned long DELAI_ENVOI_ANGLE = 50;
const unsigned long DUREE_RECUL = 3000;
const int PAS_RALENTI_SLAVE = 10;
const unsigned long DELAI_RALENTI_SLAVE = 100;

// ===================== INCLUSIONS & OBJETS =====================
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
#include <SoftwareSerial.h>
SoftwareSerial BT(7, 8);  // RX, TX

// ===================== PINS =====================
#define ENA 3
#define ENB 5
#define IN1 9
#define IN2 10
#define IN3 12
#define IN4 13
#define SWITCH_BORD 4

// ===================== VARIABLES =====================
unsigned long debutMouvement = 0;
unsigned long previousTime = 0;
unsigned long dernierEnvoi = 0;

bool enPID = false;
bool enMouvement = false;
int etatMouvement = 0;
bool envoyerAngles = false;

float integral = 0;
float lastError = 0;
float angle0 = 0;

// ===================== SETUP =====================
void setup() {
  BT.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  while (!mpu.testConnection()) {
    Serial.println("Erreur MPU...");
    delay(500);
  }

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SWITCH_BORD, INPUT_PULLUP);

  arretMoteurs();
  Serial.println("SLAVE prêt !");
}

// ===================== LOOP =====================
void loop() {
  unsigned long now = millis();

  // Envoi des angles (seulement en ETAT1)
  if (envoyerAngles && now - dernierEnvoi >= DELAI_ENVOI_ANGLE) {
    float angle = getAngle();
    BT.print("A:");
    BT.println(angle);
    dernierEnvoi = now;
  }

  // Traitement des commandes maître
  if (BT.available()) {
    String commande = BT.readStringUntil('\n');
    commande.trim();
    Serial.print("Reçu : ");
    Serial.println(commande);

    if (commande == "ETAT1") {
      etatMouvement = 1;
      enMouvement = true;
      envoyerAngles = true;
      debutMouvement = millis();
    } else if (commande == "STOP_ANGLE") {
      envoyerAngles = false;
      Serial.println("→ Arrêt des envois d'angle");
    } else if (commande == "ETAT3") {
      float angle = getAngle();
      Serial.print("Angle initial : ");
      Serial.println(angle);
      etatMouvement = 3;
      enMouvement = true;
      envoyerAngles = false;
      enPID = false;
      debutMouvement = millis();
      previousTime = millis();
      integral = 0;
      lastError = 0;
      angle0 = angle;
    } else if (commande == "ETAT5") {
      etatMouvement = 5;
      enMouvement = true;
      envoyerAngles = false;
      debutMouvement = millis();
      reculer();
    }
  }

  // Mouvement autonome avec PID (ETAT3)
  if (enMouvement && etatMouvement == 3) {
    if (!enPID && now - debutMouvement < DELAI_SANS_PID) {
      avancerConstamment(VITESSE_SLAVE);
    } else {
      enPID = true;
      float angle = getAngle() - angle0;
      float dt = (now - previousTime) / 1000.0;
      if (dt > 0) {
        previousTime = now;
        float error = -angle;
        integral += error * dt;
        float derivative = (error - lastError) / dt;
        float correction = KP_SLAVE * error + KI_SLAVE * integral + KD_SLAVE * derivative;
        lastError = error;

        float vitesseG = constrain(VITESSE_SLAVE - correction, 0, 255);
        float vitesseD = constrain(VITESSE_SLAVE + correction, 0, 255);

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, vitesseG);
        analogWrite(ENB, vitesseD);
      }
    }

    if (digitalRead(SWITCH_BORD) == LOW) {
      ralentirEtArreter();
      BT.println("FIN_ETAT");
      Serial.println("→ FIN_ETAT envoyé (bord atteint)");
      enMouvement = false;
      etatMouvement = 0;
    }
  }

  // Recul (ETAT5)
  if (enMouvement && etatMouvement == 5 && now - debutMouvement >= DUREE_RECUL) {
    arretMoteurs();
    BT.println("FIN_ETAT");
    Serial.println("→ FIN_ETAT envoyé (fin de recul)");
    enMouvement = false;
    etatMouvement = 0;
  }
}

// ===================== FONCTIONS MOTEURS =====================

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
  for (int v = VITESSE_SLAVE; v >= 0; v -= PAS_RALENTI_SLAVE) {
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
}

// ===================== MPU6050 - Angle =====================
float getAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  return gy / 131.0;  // Angle approximatif en degrés/s
}
