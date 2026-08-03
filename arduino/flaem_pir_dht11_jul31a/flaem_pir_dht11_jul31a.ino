/*
  ===== sensor_bridge_main.ino =====
  DHT11 + 불꽃센서 + IR센서 값을 읽어서 보조 아두이노(눈 보드)로 전송
*/

#include <DHT.h>
#include <SoftwareSerial.h>

#define DHT_PIN 7
#define DHT_TYPE DHT11
#define FLAME_SENSOR_PIN 8
#define IR_SENSOR_PIN 9
#define TX_PIN 10
#define RX_PIN 11

DHT dht(DHT_PIN, DHT_TYPE);
SoftwareSerial toEyesBoard(RX_PIN, TX_PIN);

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 2000; // DHT11은 2초 이상 간격 필요

void setup() {
  Serial.begin(9600);
  toEyesBoard.begin(9600);
  dht.begin();
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  Serial.println("[센서보드] 시작됨");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("[센서보드] DHT11 읽기 실패, 배선/센서 확인 필요");
      return;
    }

    // 불꽃/IR센서: 감지 시 LOW(0)인 모듈이 많음 -> 감지되면 1로 변환해서 보냄
    // 테스트해보고 반대면 == LOW 를 == HIGH 로 바꾸면 됨
    int flame = (digitalRead(FLAME_SENSOR_PIN) == LOW) ? 1 : 0;
    int irDetected = (digitalRead(IR_SENSOR_PIN) == LOW) ? 1 : 0;

    String packet = "T:" + String(temperature, 1) +
                     ",H:" + String(humidity, 1) +
                     ",F:" + String(flame) +
                     ",I:" + String(irDetected) + "\n";

    toEyesBoard.print(packet);
    Serial.print("[전송] ");
    Serial.print(packet);
  }
}
