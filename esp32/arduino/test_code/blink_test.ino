// Simple ESP32 blink test
// Built-in LED is on GPIO2

#define LED 2

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED, OUTPUT);
  Serial.println("ESP32 is working!");
  Serial.println("LED will blink now...");
}

void loop() {
  digitalWrite(LED, HIGH);
  Serial.println("LED ON");
  delay(500);
  
  digitalWrite(LED, LOW);
  Serial.println("LED OFF");
  delay(500);
}