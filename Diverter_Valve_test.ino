int intPin = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(intPin,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(intPin,HIGH);
  delay(100);

  digitalWrite(intPin,LOW);
  delay(100);
}

