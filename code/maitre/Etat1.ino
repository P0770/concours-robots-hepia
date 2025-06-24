
/*void Etat1(unsigned long now) {
  Serial.println("État 1 : Roues AV");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  roueStartTime = now;
  roueActive = true;
  actionEtatFait = true;
}*/
void Etat1(unsigned long now) {
  Serial.println("État 1 : Roues AV");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 0);  // Démarrage à 0 pour l'accélération
  analogWrite(ENB, 0);
  roueStartTime = now;
  roueActive = true;
  actionEtatFait = true;
}