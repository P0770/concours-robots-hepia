void InputUtilisateur() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "1" && pretPourSuivant) {
      Serial.println("➡️ État avancé à : " + String(etat + 1));
      etat++;
      actionEtatFait = false;
      pretPourSuivant = false;
    } 
    else if (cmd == "0" && pretPourSuivant && etat == 5) {
      Serial.println("Retour état 0.");
      resetSystem();
    } 
    else {
      Serial.print("Touche reçue : ");
      Serial.println(cmd);
      Serial.println(" (ignoré, pas pretPourSuivant)");
    }
  }
}
