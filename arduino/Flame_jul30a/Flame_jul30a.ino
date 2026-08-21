#define FLAME_SENSOR_PIN 8   // 불꽃센서 DO(디지털 출력) 핀

void setup() {
  Serial.begin(9600);
  pinMode(FLAME_SENSOR_PIN, INPUT);
  Serial.println("불꽃센서 단독 테스트 시작");
}

void loop() {
  int flameValue = digitalRead(FLAME_SENSOR_PIN); // 0(LOW) 또는 1(HIGH)

  Serial.print("불꽃센서 값: ");
  Serial.println(flameValue);

  delay(500); // 0.5초마다 확인
}
