
void InputUtilisateur() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "1" && pretPourSuivant) {
      etat++;
      actionEtatFait = false;
      pretPourSuivant = false;
    } else if (cmd == "0" && pretPourSuivant && etat == 5) {
      Serial.println("Retour état 0.");
      resetSystem();
    }
  }
}