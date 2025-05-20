
void Bip() {
  unsigned long now = millis();
  switch (etatBip) {
    case 1: // Bip long
      if (now - startTime >= dureeBip) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerActif = false;
        etatBip = 0;
        Serial.println("OK");
      }
      break;

    case 2: // Premier bip court
      if (now - startTime >= dureeBip) {
        digitalWrite(BUZZER_PIN, LOW);
        startTime = now;
        etatBip = 3;
        Serial.println("Pause");
      }
      break;

    case 3: // Pause
      if (now - startTime >= 150) {
        digitalWrite(BUZZER_PIN, HIGH);
        startTime = now;
        etatBip = 4;
        Serial.println("Deuxième bip");
      }
      break;

    case 4: // Deuxième bip
      if (now - startTime >= dureeBip) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerActif = false;
        etatBip = 0;
        Serial.println("OK");
      }
      break;
  }
}
