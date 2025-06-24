void controlepont() {
  boutonUP = digitalRead(BOUTONUP);
  boutonDOWN = digitalRead(BOUTONDOWN);
  if (etat != 4) {
    if (boutonUP == 1) {
      digitalWrite(IN12, LOW);
      digitalWrite(IN22, HIGH);
      analogWrite(ENA2, 30);
    } else if (boutonDOWN == 1) {
      digitalWrite(IN12, HIGH);
      digitalWrite(IN22, LOW);
      analogWrite(ENA2, 30);
    } else {
      digitalWrite(IN12, LOW);
      digitalWrite(IN22, HIGH);
      analogWrite(ENA2, 0);
    }
  }
}