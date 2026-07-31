#define IR_SENSOR_PIN 9   // 장애물감지 IR센서 OUT 핀

void setup() {
  Serial.begin(9600);
  pinMode(IR_SENSOR_PIN, INPUT);
  Serial.println("IR센서 단독 테스트 시작");
}

void loop() {
  int irValue = digitalRead(IR_SENSOR_PIN); // 0(LOW) 또는 1(HIGH)

  Serial.print("IR센서 값: ");
  Serial.println(irValue);

  delay(500);
}
