/*
  ===== sensor_bridge_main.ino =====
  DHT11 + 불꽃센서 + 후방 초음파(HC-SR04류) + 조이스틱(X축+버튼) 값을 읽어서
  ROS2 쪽(com_sensors/serial_bridge_node.py, /dev/ttyACM0)으로 시리얼 전송한다.

  2026-08-04 재구성: 후방 IR 디지털 센서 -> 초음파 거리 센서로 교체, 조이스틱을
  이 보드로 옮김(TFT 보드는 화면만 그림). 조이스틱 메뉴 이동/선택 판정(디바운스,
  임계값)은 더 이상 아두이노가 아니라 파이 쪽(hmi_bridge_node.py)에서 함 - 여기서는
  raw 값만 자주 보내면 됨.

  IR/PIR/긴급정지버튼/배터리전압/조도센서는 프로젝트 범위 제외(2026-08-04 확정) -
  이 보드에서 아예 안 읽음.

  ROS 쪽 라인 프로토콜 (com_sensors/serial_bridge_node.py 와 반드시 일치해야 함):
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

#define OBSTACLE_THRESHOLD_CM 20.0  // service_manager obstacle_stop_distance(기본 0.20m)와 일치

DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 2000; // DHT11은 2초 이상 간격 필요

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

  // 30ms 타임아웃 -> 약 5m 이상은 미검출로 처리 (pulseIn 0 반환)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0;  // 미검출
  return duration / 58.0;          // 왕복시간(us) -> cm
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

  // 불꽃센서: 감지 시 LOW(0)인 모듈이 많음 -> 감지되면 1로 변환해서 보냄
  int flame = (digitalRead(FLAME_SENSOR_PIN) == LOW) ? 1 : 0;

  float distance = readDistanceCm();
  int obstacle = (distance >= 0 && distance <= OBSTACLE_THRESHOLD_CM) ? 1 : 0;

  // DHT 실패해도 불꽃/초음파는 계속 보낸다 (한 센서 문제로 나머지 데이터까지
  // 안 보내면 진단이 더 어려워짐) - T/H는 실패 시 마지막 유효값 대신 -1로 표시.
  String packet = "T:" + String(dhtOk ? temperature : -1.0, 1) +
                   ",H:" + String(dhtOk ? humidity : -1.0, 1) +
                   ",F:" + String(flame) +
                   ",R:" + String(distance, 1) +
                   ",O:" + String(obstacle) + "\n";

  Serial.print(packet);
}
