
void Etat2(unsigned long now) {
  Serial.println("État 2 : Pont");
  digitalWrite(IN12, LOW);
  digitalWrite(IN22, HIGH);
  analogWrite(ENA2, 50);
  pontStartTime = now;
  pontActive = true;
  actionEtatFait = true;
}