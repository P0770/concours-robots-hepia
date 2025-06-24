void GererPhaseRalentieEtDetection(unsigned long tEcoule) {
  if (tEcoule <= dureeRoueMax + 1500) {
    // Décélération progressive
    vitesse = map(tEcoule, dureeRoueMax, dureeRoueMax + 1500, 250, 60);
  } else {
    vitesse = 60;
  }

  Serial.println("ralentit...");

  // Lire l’état du capteur
  bool capteurActuel = digitalRead(CAPTEUR_VIDE);

  // Détection du vide (ajuste selon la logique de ton capteur, LOW ou HIGH)
  if (capteurActuel == HIGH) {  // ou LOW selon ton capteur
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    Serial.println("Vide détecté. Tapez 1 pour continuer.");
    roueActive = false;
    pretPourSuivant = true;
    etat++;
  }

  // Historique éventuellement utile pour un front
  capteurVideEtatPrecedent = capteurActuel;
}