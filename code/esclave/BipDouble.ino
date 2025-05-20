
void BipDouble(unsigned long duree) {
  digitalWrite(BUZZER_PIN, HIGH);
  startTime = millis();
  dureeBip = duree;
  etatBip = 2;
  buzzerActif = true;
  Serial.println("Début bip double");
}