

// ===================== CODE SLAVE =====================

#define BUZZER_PIN A5
unsigned long startTime = 0;
unsigned long dureeBip = 0;
int etatBip = 0;
bool buzzerActif = false;

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    if (message.startsWith("BIP2S:")) {
      dureeBip = message.substring(6).toInt();
      digitalWrite(BUZZER_PIN, HIGH);
      startTime = millis();
      etatBip = 1;
      buzzerActif = true;
    } else if (message == "BIP2X") {
      dureeBip = 200;
      digitalWrite(BUZZER_PIN, HIGH);
      startTime = millis();
      etatBip = 2;
      buzzerActif = true;
    }
  }

  if (buzzerActif) {
    unsigned long now = millis();
    if (etatBip == 1 && now - startTime >= dureeBip) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerActif = false;
      etatBip = 0;
      Serial.println("OK");
    } else if (etatBip == 2 && now - startTime >= dureeBip) {
      digitalWrite(BUZZER_PIN, LOW);
      startTime = now;
      etatBip = 3;
    } else if (etatBip == 3 && now - startTime >= 150) {
      digitalWrite(BUZZER_PIN, HIGH);
      startTime = now;
      etatBip = 4;
    } else if (etatBip == 4 && now - startTime >= dureeBip) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerActif = false;
      etatBip = 0;
      Serial.println("OK");
    }
  }
}
