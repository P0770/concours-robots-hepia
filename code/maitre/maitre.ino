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
#define BT_CONNECTED 24

#define ENA2 44
#define ENB2 53
#define IN12 46
#define IN22 48
#define IN32 50
#define IN42 52

const unsigned long dureeBipSlave = 2000;
const unsigned long dureeRoueMax = 3000;
const unsigned long dureePont = 1500;
const unsigned long dureePompe = 3000;
const unsigned long bluetoothTimeout = 5000;

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

unsigned long bluetoothPerteTime = 0;
bool capteurVideEtatPrecedent = HIGH;

void eteindreTousMoteurs() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENA2, 0);
  analogWrite(ENB2, 0);
  analogWrite(G1, 0);
  digitalWrite(G2, LOW);
}

void resetSystem() {
  etat = 0;
  actionEtatFait = false;
  pretPourSuivant = false;
  attenteAck = false;
  attenteDeuxBips = false;
  deuxBipsFait = false;
  etatmem = -1;
  bluetoothPerteTime = 0;
  eteindreTousMoteurs();
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
  pinMode(BT_CONNECTED, INPUT);

  digitalWrite(G2, LOW);
  analogWrite(G1, 0);
  eteindreTousMoteurs();
}

void loop() {
  unsigned long now = millis();
  bool btOk = (digitalRead(BT_CONNECTED) == HIGH);

  if (digitalRead(BOUTON_STOP) == LOW) {
    Serial.println("==> ARRÊT D'URGENCE manuel");
    resetSystem();
    return;
  }

  if (!btOk) {
    if (bluetoothPerteTime == 0) bluetoothPerteTime = now;
    else if (now - bluetoothPerteTime >= bluetoothTimeout && etat != 0) {
      Serial.println("==> PERTE BLUETOOTH > 5 sec : retour à l'état 0");
      resetSystem();
      return;
    }
  } else bluetoothPerteTime = 0;

  if (etat != etatmem) {
    Serial.print("==> Passage à l'état ");
    Serial.println(etat);
    etatmem = etat;
    actionEtatFait = false;
    pretPourSuivant = false;
    attenteAck = false;
    attenteDeuxBips = false;
    deuxBipsFait = false;
    eteindreTousMoteurs();
  }

  switch (etat) {
    case 0:
      if (!actionEtatFait) {
        eteindreTousMoteurs();
        Serial.println("État 0 : Système à zéro.");
        actionEtatFait = true;
      }
      if (btOk) {
        Serial.println("Bluetooth connecté. Tapez 1 pour démarrer.");
        pretPourSuivant = true;
      } else pretPourSuivant = false;
      break;

    case 1:
      if (!actionEtatFait) {
        Serial.println("État 1 : Roues AV");
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 255);
        analogWrite(ENB, 255);
        roueStartTime = now;
        roueActive = true;
        actionEtatFait = true;
      } else if (roueActive && !phaseRalentie && now - roueStartTime >= dureeRoueMax) {
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

    case 2:
      if (!actionEtatFait) {
        Serial.println("État 2 : Pont");
        digitalWrite(IN12, LOW);
        digitalWrite(IN22, HIGH);
        analogWrite(ENA2, 100);
        pontStartTime = now;
        pontActive = true;
        actionEtatFait = true;
      } else if (pontActive && now - pontStartTime >= dureePont) {
        digitalWrite(IN12, LOW);
        digitalWrite(IN22, HIGH);
        analogWrite(ENA2, 0);
        Serial.println("Pont terminé. Tapez 1 pour continuer.");
        pontActive = false;
        pretPourSuivant = true;
      }
      break;

    case 3:
      if (!actionEtatFait) {
        Serial.println("État 3 : Long bip slave");
        Serial1.print("BIP2S:");
        Serial1.println(dureeBipSlave);
        attenteAck = true;
        actionEtatFait = true;
      }
      if (attenteAck && Serial1.available()) {
        String ack = Serial1.readStringUntil('\n');
        ack.trim();
        if (ack == "OK") {
          Serial.println("Bip terminé. Tapez 1 pour 2 bips supplémentaires.");
          attenteAck = false;
          attenteDeuxBips = true;
        }
      }
      if (attenteDeuxBips && deuxBipsFait && Serial1.available()) {
        String ack = Serial1.readStringUntil('\n');
        ack.trim();
        if (ack == "OK") {
          Serial.println("2 bips terminés. Passage à l'état 4.");
          attenteDeuxBips = false;
          deuxBipsFait = false;
          etat = 4;
          actionEtatFait = false;
        }
      }
      break;

    case 4:
      if (!actionEtatFait) {
        Serial.println("État 4 : Pompe");
        digitalWrite(G2, HIGH);
        pompeStartTime = now;
        pompeActive = true;
        actionEtatFait = true;
      } else if (pompeActive && now - pompeStartTime >= dureePompe) {
        digitalWrite(G2, LOW);
        Serial.println("Pompe OFF. Tapez 0 pour revenir à l'état 0.");
        pompeActive = false;
        pretPourSuivant = true;
      }
      break;
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "1") {
      if (attenteDeuxBips && !deuxBipsFait) {
        Serial1.println("BIP2X");
        Serial.println("2 bips envoyés au slave...");
        deuxBipsFait = true;
      } else if (pretPourSuivant) {
        etat++;
        actionEtatFait = false;
        pretPourSuivant = false;
      }
    } else if (cmd == "0" && pretPourSuivant && etat == 4) {
      Serial.println("Retour état 0.");
      resetSystem();
    }
  }
}
