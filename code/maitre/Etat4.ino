
void Etat4(unsigned long now) {
  Serial.println("État 4 : Pompe");
  digitalWrite(G2, HIGH);
  pompeStartTime = now;
  pompeActive = true;
  actionEtatFait = true;
}