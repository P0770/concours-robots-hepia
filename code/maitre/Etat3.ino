void Etat3() {
  Serial.println("État 3 : Mouvement en avant du SLAVE sur 3 m");
  Serial1.println("ETAT1");  // Nouvelle commande pour faire avancer le SLAVE
  attenteAck = true;
  actionEtatFait = true;
}
