/*
  ===== sensor_bridge_main.ino =====   [2026-08-19 이전 버전 - 롤백용 보관]
  이 파일은 2026-08-19에 R: 단독 라인(100ms)을 추가하기 "전" 버전이다.
  새 펌웨어에 문제가 생기면 이 내용을 flaem_pir_dht11_jul31a.ino 로 되돌려
  다시 업로드하면 된다. (업로드 시점에 보드에 올라가 있던 펌웨어)

  DHT11 + 불꽃센서 + 후방 초음파(HC-SR04류) + 조이스틱(X축+버튼) 값을 읽어서
  ROS2 쪽(com_sensors/serial_bridge_node.py)으로 시리얼 전송한다.

  ROS 쪽 라인 프로토콜:
    텔레메트리(2초 주기, DHT11 제약) : "T:25.0,H:60.0,F:0,R:15.3,O:1"
    조이스틱(약 50ms 주기)         : "J:154,B:1"
*/

#include <DHT.h>

#define DHT_PIN 5
#define DHT_TYPE DHT11
#define FLAME_SENSOR_PIN 8
#define TRIG_PIN 10
#define ECHO_PIN 11
#define JOY_X   A1
#define JOY_BTN 13

#define OBSTACLE_THRESHOLD_CM 20.0

DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 2000;

unsigned long lastJoyTime = 0;
const unsigned long JOY_INTERVAL = 50;

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(JOY_BTN, INPUT_PULLUP);
  Serial.println("[센서보드] 시작됨");
}

void loop() {
  unsigned long now = millis();

  if (now - lastJoyTime >= JOY_INTERVAL) {
    lastJoyTime = now;
    sendJoystick();
  }

  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = now;
    sendTelemetry();
  }
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0;
  return duration / 58.0;
}

void sendJoystick() {
  int x = analogRead(JOY_X);
  int btn = (digitalRead(JOY_BTN) == LOW) ? 1 : 0;
  Serial.print("J:");
  Serial.print(x);
  Serial.print(",B:");
  Serial.println(btn);
}

void sendTelemetry() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  bool dhtOk = !(isnan(humidity) || isnan(temperature));

  if (!dhtOk) {
    Serial.println("[센서보드] DHT11 읽기 실패, 배선/센서 확인 필요");
  }

  int flame = (digitalRead(FLAME_SENSOR_PIN) == LOW) ? 1 : 0;

  float distance = readDistanceCm();
  int obstacle = (distance >= 0 && distance <= OBSTACLE_THRESHOLD_CM) ? 1 : 0;

  String packet = "T:" + String(dhtOk ? temperature : -1.0, 1) +
                   ",H:" + String(dhtOk ? humidity : -1.0, 1) +
                   ",F:" + String(flame) +
                   ",R:" + String(distance, 1) +
                   ",O:" + String(obstacle) + "\n";

  Serial.print(packet);
}
