/*
 * HW-504 조이스틱 SW(버튼) - sensor_bridge용 최종 코드
 * 시리얼 프로토콜: BTN,<pressed 1 / released 0>
 * 짧게/길게 판정은 ROS2 쪽에서 (DDS-001 설계)
 */

const int BUTTON_PIN = 7;   // HW-504 SW 핀

const unsigned long DEBOUNCE_DELAY = 30;   // 디바운싱 30ms

unsigned long lastDebounceTime = 0;
int lastButtonReading = HIGH;
int buttonState = HIGH;

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println("READY");
}

void loop() {
    readAndSendButton();
}

void readAndSendButton() {
    int reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonReading) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
            buttonState = reading;
            bool pressed = (buttonState == LOW);   // 풀업이라 누르면 LOW

            Serial.print("BTN,");
            Serial.println(pressed ? 1 : 0);
        }
    }

    lastButtonReading = reading;
}
