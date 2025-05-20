
void Etat1(unsigned long now) {
  Serial.println("État 1 : Roues AV");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  roueStartTime = now;
  roueActive = true;
  actionEtatFait = true;
}