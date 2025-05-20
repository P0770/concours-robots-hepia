// ===================== CODE SLAVE =====================

#define BUZZER_PIN 8
#define PIN_STOP 3

unsigned long startTime = 0;
unsigned long dureeBip = 0;
int etatBip = 0;
bool buzzerActif = false;
 
void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIN_STOP, INPUT_PULLUP);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {

  verifierBoutonStop();


  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    if (message.startsWith("BIP2S:")) {
      unsigned long duree = message.substring(6).toInt();
      BipLong(duree);
    } else if (message == "BIP2X") {
      BipDouble(200);
    }
  }

  if (buzzerActif) Bip();
}
