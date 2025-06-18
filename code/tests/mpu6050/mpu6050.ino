// === Paramètres moteurs ===
#define ENA 3
#define ENB 5
#define IN1 9
#define IN2 10
#define IN3 12
#define IN4 13

// === MPU6050 ===
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;

float angle0 = 0;
float angle = 0;
float previousAngle = 0;
unsigned long previousTime = 0;
float vitesse = 0;
float accelAng = 0;

// === PID (modéré) ===
float Kp = 4.5;
float Ki = 0.0;
float Kd = 1.0;
float integral = 0;
float lastError = 0;

// === État ===
bool robotActif = false;
bool ralentir = false;
unsigned long t0 = 0;
unsigned long tempsActuel = 0;
float vitesseCible = 200;

// === Setup ===
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  while (!mpu.testConnection()) {
    Serial.println("MPU non connecté...");
    delay(500);
  }

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Prêt. Entrez '1' pour démarrer ou arrêter.");
}

// === Boucle principale ===
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '1') {
      robotActif = !robotActif;
      if (robotActif) {
        angle0 = getAngle();
        t0 = millis();
        Serial.println("Robot activé");
      } else {
        arreterRobot();
        Serial.println("Robot arrêté");
      }
    }
  }

  if (!robotActif) return;

  tempsActuel = millis();
  float t = (tempsActuel - t0) / 1000.0;

  // Lecture MPU
  float rawAngle = getAngle();
  angle = rawAngle - angle0;
  float dt = (tempsActuel - previousTime) / 1000.0;
  if (dt == 0) return;

  accelAng = (angle - previousAngle) / dt;
  vitesse = calculerVitesse(t);
  previousAngle = angle;
  previousTime = tempsActuel;

  // PID
  float error = -angle;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  correction = constrain(correction, -100, 100);

  // Moteurs
  float vitesseG = constrain(vitesse - correction, 0, 255);
  float vitesseD = constrain(vitesse + correction, 0, 255);
  setMoteurs(vitesseG, vitesseD);

  Serial.print("t="); Serial.print(tempsActuel - t0);
  Serial.print(" | V="); Serial.print(vitesse);
  Serial.print(" | A="); Serial.print(angle, 2);
  Serial.print(" | Corr="); Serial.println((int)correction);
}

// === Fonctions ===
float getAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  return gy / 131.0;
}

float calculerVitesse(float temps) {
  if (temps < 1.0) {
    return map(temps * 1000, 0, 1000, 0, vitesseCible);  // accélération douce
  }
  return vitesseCible;
}

void setMoteurs(float vitesseG, float vitesseD) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, vitesseG);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, vitesseD);
}

void arreterRobot() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  robotActif = false;
  integral = 0;
  lastError = 0;
}
