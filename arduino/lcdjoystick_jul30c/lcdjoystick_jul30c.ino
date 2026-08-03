/*
 * joystick_menu_input.ino
 * HW-504 조이스틱으로 메뉴 커서 이동/선택 신호 전송
 * 프로토콜: UP / DOWN / SELECT  (한 줄씩)
 */

const int VRY_PIN = A1;   // 위아래 기울임
const int SW_PIN  = 7;    // 누르기(선택)

const int CENTER   = 512;
const int THRESH_HI = 700;
const int THRESH_LO = 300;

bool armed = true;

const unsigned long DEBOUNCE_DELAY = 30;
unsigned long lastDebounceTime = 0;
int lastButtonReading = HIGH;
int buttonState = HIGH;

void setup() {
    Serial.begin(115200);
    pinMode(SW_PIN, INPUT_PULLUP);
    Serial.println("READY");
}

void loop() {
    readJoystickTilt();
    readSelectButton();
}

void readJoystickTilt() {
    int y = analogRead(VRY_PIN);

    if (armed) {
        if (y > THRESH_HI) {
            Serial.println("DOWN");
            armed = false;
        } else if (y < THRESH_LO) {
            Serial.println("UP");
            armed = false;
        }
    } else {
        if (y > CENTER - 100 && y < CENTER + 100) {
            armed = true;
        }
    }
}

void readSelectButton() {
    int reading = digitalRead(SW_PIN);

    if (reading != lastButtonReading) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
            buttonState = reading;
            if (buttonState == LOW) {
                Serial.println("SELECT");
            }
        }
    }

    lastButtonReading = reading;
}
