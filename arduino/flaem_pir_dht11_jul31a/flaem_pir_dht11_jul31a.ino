/*
  ===== sensor_bridge_main.ino =====
  DHT11 + 불꽃센서 + IR센서 + 조도센서(LDR) + 배터리전압 값을 읽어서
  ROS2 쪽(serial_bridge_node, /dev/ttyACM0)으로 시리얼 전송하고,
  긴급정지 버튼 + 감지값은 보조 아두이노(눈 보드)로도 전송한다.

  ROS 쪽 라인 프로토콜 (com_sensors/serial_bridge_node.py 와 반드시 일치해야 함):
    텔레메트리(2초 주기, DHT11 제약) : "T:25.0,H:60.0,F:0,I:1,L:512,V:11.8"
    버튼 이벤트(즉시)               : "EVT:B:1" (짧게) / "EVT:B:2" (길게=긴급정지)

  ※ BATTERY_DIVIDER_RATIO, LCD 배선 등은 실제 하드웨어에 맞춰 재확인/캘리브레이션 필요.
*/

#include <DHT.h>
#include <SoftwareSerial.h>

#define DHT_PIN 7
#define DHT_TYPE DHT11
#define FLAME_SENSOR_PIN 8
#define IR_SENSOR_PIN 9
#define TX_PIN 10
#define RX_PIN 11
#define BUTTON_PIN 4      // 긴급정지 푸시버튼 (INPUT_PULLUP, 눌리면 LOW)
#define LIGHT_SENSOR_PIN A0  // 조도센서(LDR) 아날로그 출력
#define BATTERY_PIN A2       // 배터리 전압 분배 회로 아날로그 입력

// 분배저항 비율: V_actual = analogRead * (5.0 / 1023.0) * BATTERY_DIVIDER_RATIO
// 예: 10k/2.2k 분배 시 실배터리전압 12.6V -> 아날로그핀 약 2.27V 로 낮춤.
// 실측 후 반드시 보정할 것.
#define BATTERY_DIVIDER_RATIO 5.545

DHT dht(DHT_PIN, DHT_TYPE);
SoftwareSerial toEyesBoard(RX_PIN, TX_PIN);

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 2000; // DHT11은 2초 이상 간격 필요

// --- 버튼 디바운스/짧게-길게 판정 (joystick_menu_input.ino 의 패턴과 동일) ---
const unsigned long DEBOUNCE_DELAY = 30;
const unsigned long LONG_PRESS_MS = 800;
unsigned long lastDebounceTime = 0;
unsigned long pressStartTime = 0;
int lastButtonReading = HIGH;
int buttonState = HIGH;
bool longPressFired = false;

void setup() {
  Serial.begin(9600);
  toEyesBoard.begin(9600);
  dht.begin();
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[센서보드] 시작됨");
}

void loop() {
  handleButton();

  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    sendTelemetry();
  }
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        // 눌리기 시작
        pressStartTime = millis();
        longPressFired = false;
      } else {
        // 뗌: 길게눌림이 이미 발생하지 않았다면 짧게눌림으로 처리
        unsigned long heldMs = millis() - pressStartTime;
        if (!longPressFired && heldMs < LONG_PRESS_MS) {
          Serial.println("EVT:B:1");
        }
      }
    }
  }

  // 누르고 있는 도중 LONG_PRESS_MS 넘으면 뗄 때까지 기다리지 않고 즉시 발화 (긴급정지 지연 최소화)
  if (buttonState == LOW && !longPressFired
      && (millis() - pressStartTime) >= LONG_PRESS_MS) {
    longPressFired = true;
    Serial.println("EVT:B:2");
    toEyesBoard.println("EVT:B:2");
  }

  lastButtonReading = reading;
}

void sendTelemetry() {
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
  int light = analogRead(LIGHT_SENSOR_PIN);

  float batteryRaw = analogRead(BATTERY_PIN) * (5.0 / 1023.0);
  float batteryVoltage = batteryRaw * BATTERY_DIVIDER_RATIO;

  String packet = "T:" + String(temperature, 1) +
                   ",H:" + String(humidity, 1) +
                   ",F:" + String(flame) +
                   ",I:" + String(irDetected) +
                   ",L:" + String(light) +
                   ",V:" + String(batteryVoltage, 2) + "\n";

  Serial.print(packet);
  toEyesBoard.print(packet);
}
