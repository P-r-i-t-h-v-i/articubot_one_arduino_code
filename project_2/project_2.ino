#include <WiFi.h>
#include <WiFiUdp.h>

// -------- WIFI --------
const char* ssid = "";
const char* password = "";
const char* pc_ip = "";   // example: 192.168.1.10
const int pc_port = 8888;
const int esp_port = 9999;

// -------- MOTOR PINS --------
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

// -------- ULTRASONIC --------
#define TRIG 12
#define ECHO 13

WiFiUDP udp;

// -------- MOTOR CONTROL (NO PWM) --------
void stop_motors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turn_left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turn_right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// -------- ULTRASONIC --------
float read_ultrasonic() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 4.0;
  return (duration * 0.034 / 2.0) / 100.0;
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

// FORCE STOP AT BOOT
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  stop_motors();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  udp.begin(esp_port);
}

void loop() {
  // ---- RECEIVE CMD_VEL ----
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buffer[64];
    int len = udp.read(buffer, 64);
    buffer[len] = 0;

    float linear, angular;
    sscanf(buffer, "%f,%f", &linear, &angular);

    if (linear > 0.1) forward();
    else if (linear < -0.1) backward();
    else if (angular > 0.1) turn_left();
    else if (angular < -0.1) turn_right();
    else stop_motors();
    if (packetSize) {
  Serial.println("UDP RECEIVED");
  Serial.println(buffer);
  }
  }

  // ---- SEND ULTRASONIC ----
  float dist = read_ultrasonic();
  char msg[16];
  sprintf(msg, "%.2f", dist);

  udp.beginPacket(pc_ip, pc_port);
  udp.print(msg);
  udp.endPacket();

  

  delay(100);
}
