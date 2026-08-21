#include <DHT.h>

#define DHT_PIN 7
#define DHT_TYPE DHT11

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("DHT11 테스트 시작");
  dht.begin();
}

void loop() {
  delay(2000); // DHT11은 최소 2초 간격 필요, 이것보다 빠르면 원래도 실패함

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  Serial.print("습도: ");
  Serial.print(h);
  Serial.print(" %  온도: ");
  Serial.print(t);
  Serial.println(" C");

  if (isnan(h) || isnan(t)) {
    Serial.println("-> 읽기 실패");
  }
}
