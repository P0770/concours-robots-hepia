void GererPhaseAccelerationEtConstante(unsigned long tEcoule) {
  if (tEcoule <= 3000) {
    // Accélération progressive
    vitesse = map(tEcoule, 0, 3000, 0, 250);
  } 
  else if (tEcoule <= dureeRoueMax) {
    // Vitesse constante
    vitesse = 250;
  } 
  else {
    // Passage à la phase ralentie
    phaseRalentie = true;
    Serial.println("Phase ralentie...");
  }
}