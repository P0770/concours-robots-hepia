
void verifierMessagesSlave() {
  if (Serial1.available()) {
    String msg = Serial1.readStringUntil('\n');
    msg.trim();
    if (msg == "STOP") {
      Serial.println("==> Signal d'arrêt reçu du slave");
      resetSystem();
    } else if (attenteAck || attenteDeuxBips) {
      if (msg == "OK") {
        if (etat == 3) {
          Serial.println("Bip terminé. Tapez 1 pour passer à l'état pompe.");
          attenteAck = false;
          pretPourSuivant = true;
        } else if (etat == 5) {
          Serial.println("2 bips finaux terminés. Tapez 0 pour revenir à l'état 0.");
          attenteDeuxBips = false;
          pretPourSuivant = true;
        }
      }
    }
  }
}