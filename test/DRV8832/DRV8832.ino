// Arduiuno - DRV8832
// PIN 10   - IN1
// PIN 11   - IN2
// PIN 12   - FAULTn

void setup() {
  Serial.begin(9600);
  pinMode(12, INPUT_PULLUP);
}

void loop() {

  //約2秒かけて+最大に
  for (int i = 0; i < 255; i++) {
    DRV8832_update(i);
    delay(20);
  }

  //最大を約2秒維持
  for (int i = 0; i < 255; i++) {
    DRV8832_update(255);
    delay(8);
  }

  //約4秒かけて-最大に
  for (int i = 255; i > -255; i--) {
    DRV8832_update(i);
    delay(20);
  }

  //-最大を約2秒維持
  for (int i = 0; i < 255; i++) {
    DRV8832_update(-255);
    delay(8);
  }

  //約2秒かけて0に
  for (int i = -255; i < 0; i++) {
    DRV8832_update(i);
    delay(20);
  }

  //0を約2秒維持
  for (int i = 0; i < 100; i++) {
    DRV8832_update(0);
    delay(8);
  }
  
}

void DRV8832_update(int power) {
  int error = digitalRead(12);
  int in1 = power > 0 ? +1 * power : 0;
  int in2 = power < 0 ? -1 * power : 0;

  analogWrite( 10, in1 );
  analogWrite( 11, in2 );

  Serial.print(in1);
  Serial.print(",");
  Serial.print(in2);
  Serial.print(",");
  Serial.print(error);
  Serial.println("00");

}
