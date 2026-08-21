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

  2026-08-19: 후방 초음파(R/O)를 별도 라인으로 분리해 100ms 주기로 보낸다.
  [이유] 2초 제약은 DHT11(온습도)에만 해당하는데 초음파까지 같은 패킷에 묶여 있어서
  후방 거리 정보가 0.5Hz로만 갱신됐다. 그 결과
    - service_manager 의 장애물 확정이 몇 초씩 걸려 정지가 늦었고(실측 8초, 1.7m)
    - auto_dock_node 의 초음파 폐루프 후진은 제어 주기가 2초라 성립 자체가 안 됐다
      (갱신 사이에 8cm 이동 - 목표 간격은 3cm).
  [설계] 파이 쪽 serial_bridge_node.py 는 2026-08-18에 이미 "R:<cm>,O:<0/1>" 단독
  라인을 받도록 되어 있었다(parse_and_publish_rear). 펌웨어만 안 올라가 있었을 뿐이라
  그 프로토콜에 맞춘다. 기존 T/H/F/R/O 통합 텔레메트리는 2초 주기로 그대로 둔다
  (호환성 유지 - 브릿지가 둘 다 처리한다).
  [대역폭] 9600 baud = 약 960 B/s.
    조이스틱 약 12B x 20Hz = 240 B/s
    R 라인   약 12B x 10Hz = 120 B/s
    텔레메트리 약 40B x 0.5Hz = 20 B/s   -> 합계 약 380 B/s (40%) 로 여유 있음.

  ROS 쪽 라인 프로토콜 (com_sensors/serial_bridge_node.py 와 반드시 일치해야 함):
    텔레메트리(2초 주기)    : "T:25.0,H:60.0,F:0,R:15.3,O:1"
    후방거리(100ms 주기)    : "R:15.3,O:1"
    조이스틱(약 50ms 주기)  : "J:154,B:1"
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

// 2026-08-19: 후방 거리만 따로 자주 보낸다 (파일 상단 주석 참고).
unsigned long lastRearTime = 0;
const unsigned long REAR_INTERVAL = 100;

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

  if (now - lastRearTime >= REAR_INTERVAL) {
    lastRearTime = now;
    sendRear();
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

  // 2026-08-19: 100ms 주기로 부르므로 타임아웃을 30ms -> 15ms 로 줄였다.
  // 15ms면 약 2.5m 까지 측정 가능한데, 이 프로젝트가 쓰는 범위(장애물 20cm,
  // 도킹 후진 3cm, auto_dock ultrasonic_max_valid 2.0m)엔 충분하다.
  // 30ms를 그대로 두면 미검출 시 매 주기 30%를 블로킹해 조이스틱까지 밀린다.
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 15000UL);
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

// 2026-08-19: 후방 거리 단독 라인. serial_bridge_node.parse_and_publish_rear 가 받는다.
void sendRear() {
  float distance = readDistanceCm();
  int obstacle = (distance >= 0 && distance <= OBSTACLE_THRESHOLD_CM) ? 1 : 0;
  Serial.print("R:");
  Serial.print(distance, 1);
  Serial.print(",O:");
  Serial.println(obstacle);
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
