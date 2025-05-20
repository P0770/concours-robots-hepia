
void BTCheckARRET() {
  if (digitalRead(BOUTON_STOP) == LOW || digitalRead(BT_STATE) == LOW) {
    Serial.println("==> ARRÊT D'URGENCE ou PERTE BLUETOOTH détectée via pin STATE");
    resetSystem();
  }
}