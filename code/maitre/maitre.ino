// ===================== CODE MAÎTRE =====================
#define ENA 3
#define ENB 5
#define IN1 9
#define IN2 10
#define IN3 12
#define IN4 13
#define G1 6
#define G2 7
#define BOUTON_STOP 22
#define CAPTEUR_VIDE 23
#define BT_STATE 24

#define ENA2 44
#define ENB2 53
#define IN12 46
#define IN22 48
#define IN32 50
#define IN42 52


const unsigned long dureeBipSlave = 2000;
const unsigned long dureeRoueMax = 3000;
const unsigned long dureePont = 3600;
const unsigned long dureePompe = 3000;

int etat = 0;
int etatmem = -1;
bool actionEtatFait = false;
bool attenteAck = false;
bool pretPourSuivant = false;
bool attenteDeuxBips = false;
bool deuxBipsFait = false;

unsigned long roueStartTime = 0;
bool roueActive = false;
bool phaseRalentie = false;

unsigned long pontStartTime = 0;
bool pontActive = false;

unsigned long pompeStartTime = 0;
bool pompeActive = false;

bool capteurVideEtatPrecedent = HIGH;


void resetSystem() {
  etat = 0;
  actionEtatFait = false;
  pretPourSuivant = false;
  attenteAck = false;
  attenteDeuxBips = false;
  deuxBipsFait = false;
  etatmem = -1;
  MoteursMise0();
}




void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

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
  pinMode(CAPTEUR_VIDE, INPUT_PULLUP);
  pinMode(BT_STATE, INPUT);

  digitalWrite(G2, LOW);
  analogWrite(G1, 0);
  MoteursMise0();
}

void loop() {
  unsigned long now = millis();
  BTCheckARRET();
  verifierMessagesSlave();

  if (etat != etatmem) {
    Serial.print("==> Passage à l'état ");
    Serial.println(etat);
    etatmem = etat;
    actionEtatFait = false;
    pretPourSuivant = false;
    attenteAck = false;
    attenteDeuxBips = false;
    deuxBipsFait = false;
    MoteursMise0();
  }

  switch (etat) {

    case 0:  // STOP et Attente
      if (!actionEtatFait) Etat0();
      if (digitalRead(BT_STATE) == HIGH) {
        Serial.println("Bluetooth connecté. Tapez 1 pour démarrer.");
        pretPourSuivant = true;
      } else pretPourSuivant = false;
      break;

    case 1:  // moteur Maitre +
      if (!actionEtatFait) Etat1(now);
      else if (roueActive && !phaseRalentie && now - roueStartTime >= dureeRoueMax) {
        analogWrite(ENA, 128);
        analogWrite(ENB, 128);
        Serial.println("Phase ralentie...");
        phaseRalentie = true;
      } else if (phaseRalentie) {
        bool capteurActuel = digitalRead(CAPTEUR_VIDE);
        if (capteurVideEtatPrecedent == HIGH && capteurActuel == LOW) {
          analogWrite(ENA, 0);
          analogWrite(ENB, 0);
          Serial.println("Vide détecté. Tapez 1 pour continuer.");
          roueActive = false;
          pretPourSuivant = true;
        }
        capteurVideEtatPrecedent = capteurActuel;
      }
      break;

    case 2:  // Pont
      if (!actionEtatFait) Etat2(now);
      else if (pontActive && now - pontStartTime >= dureePont) {
        digitalWrite(IN12, LOW);
        digitalWrite(IN22, HIGH);
        analogWrite(ENA2, 0);
        Serial.println("Pont terminé. Tapez 1 pour continuer.");
        pontActive = false;
        pretPourSuivant = true;
      }
      break;

    case 3:  // Slave Moteur +
      if (!actionEtatFait) Etat3();
      if (attenteAck && Serial1.available()) {
        String ack = Serial1.readStringUntil('\n');
        ack.trim();
        if (ack == "OK") {
          Serial.println("Bip terminé. Tapez 1 pour passer à l'état pompe.");
          attenteAck = false;
          pretPourSuivant = true;
        }
      }
      break;

    case 4:  // Pompe
      if (!actionEtatFait) Etat4(now);
      else if (pompeActive && now - pompeStartTime >= dureePompe) {
        digitalWrite(G2, LOW);
        Serial.println("Pompe OFF. Tapez 1 pour passer à l'état bip final.");
        pompeActive = false;
        pretPourSuivant = true;
      }
      break;

    case 5:  // Slave Moteur -
      if (!actionEtatFait) Etat5();
      if (attenteDeuxBips && Serial1.available()) {
        String ack = Serial1.readStringUntil('\n');
        ack.trim();
        if (ack == "OK") {
          Serial.println("2 bips finaux terminés. Tapez 0 pour revenir à l'état 0.");
          pretPourSuivant = true;
          attenteDeuxBips = false;
        }
      }
      break;
  }

  InputUtilisateur();
}
