void verifierBoutonStop() {
  if (digitalRead(PIN_STOP) == LOW) {
    Serial.println("STOP");
    delay(200); // anti-rebond
  }
}
