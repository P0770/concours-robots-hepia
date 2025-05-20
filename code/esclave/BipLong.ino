
void BipLong(unsigned long duree) {
  digitalWrite(BUZZER_PIN, HIGH);
  startTime = millis();
  dureeBip = duree;
  etatBip = 1;
  buzzerActif = true;
  Serial.println("Début bip long");
}