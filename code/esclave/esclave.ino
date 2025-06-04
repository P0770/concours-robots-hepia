// ===================== CODE SLAVE =====================
#define ENA 3
#define ENB 5
#define IN1 9
#define IN2 10
#define IN3 12
#define IN4 13

unsigned long debutMouvement = 0;
unsigned long dureeMouvement = 3000; // à ajuster selon vitesse réelle pour 3m
int etatMouvement = 0; // 0 = inactif, 1 = avancer, 2 = reculer
bool enMouvement = false;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  arretMoteurs();
}

void loop() {
  if (Serial.available()) {
    String commande = Serial.readStringUntil('\n');

    if (commande == "ETAT1") {
      debutMouvement = millis();
      etatMouvement = 1; // avancer
      enMouvement = true;
      avancer();
    } else if (commande == "ETAT2") {
      debutMouvement = millis();
      etatMouvement = 2; // reculer
      enMouvement = true;
      reculer();
    }
  }

  if (enMouvement && (millis() - debutMouvement >= dureeMouvement)) {
    arretMoteurs();
    enMouvement = false;
    Serial.println("FIN_ETAT"); // accusé de fin pour le maître
  }
}

void avancer() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200); // vitesse ajustable
  analogWrite(ENB, 200);
}

void reculer() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void arretMoteurs() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
