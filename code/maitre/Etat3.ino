
void Etat3() {
  Serial.println("État 3 : Long bip slave");
  Serial1.print("BIP2S:"); Serial1.println(dureeBipSlave);
  attenteAck = true;
  actionEtatFait = true;
}