/*
 * KS0567 Cloud Smart Farm — ESP32 + MQTT (ใช้คู่กับ index.html)
 * ไลบรารีที่ต้องลง: PubSubClient (Nick O'Leary), ESP32Servo, Dht11, LiquidCrystal_I2C
 *
 * Topic:  <BASE>/status   บอร์ด -> เว็บ  (JSON ทุก 2 วินาที)
 *         <BASE>/control  เว็บ -> บอร์ด  (คำสั่งข้อความ)
 *
 * คำสั่งที่รองรับ:
 *   led | led:on | led:off            fan | fan:on | fan:off
 *   pump | pump:on | pump:off         water:<วินาที>   (รดน้ำตามเวลาแล้วหยุดเอง)
 *   servo:<0-180>  feed:open  feed:close
 *   buzzer  alarm  music  buzzer:off
 *   guard:on | guard:off              (PIR กันขโมย -> ไซเรน + กระพริบไฟ)
 *   auto | auto:on | auto:off
 *   config:{"soil":30,"temp":32,"light":20,"water":15,"rain":40,"dist":7}
 */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <dht11.h>
#include <LiquidCrystal_I2C.h>

// ---------- WiFi / MQTT ----------
const char* WIFI_SSID = "your_SSID";
const char* WIFI_PASS = "your_PASSWORD";
const char* MQTT_HOST = "broker.emqx.io";
const int   MQTT_PORT = 1883;
const char* TOPIC_BASE = "sbt_smartfarm_01";   // ต้องตรงกับหน้าเว็บ

// ---------- Pins (ตามชุด KS0567) ----------
#define DHT11PIN     17
#define RAINPIN      35
#define LIGHTPIN     34
#define WATERPIN     33
#define SOILPIN      32
#define PIRPIN       23
#define TRIGPIN      12
#define ECHOPIN      13
#define LEDPIN       27
#define RELAYPIN     25
#define SERVOPIN     26
#define FANPIN1      19
#define FANPIN2      18
#define BUZZERPIN    16

#define SERVO_OPEN   80
#define SERVO_CLOSE  160

WiFiClient net;
PubSubClient mqtt(net);
dht11 DHT11;
Servo feeder;
LiquidCrystal_I2C lcd(0x27, 16, 2);

String tStatus, tControl;

// ---------- state ----------
bool ledOn = false, fanOn = false, pumpOn = false, autoMode = false, guard = false;
int  servoAngle = SERVO_CLOSE;
unsigned long pumpUntil = 0;      // 0 = ไม่มีตัวจับเวลา
unsigned long buzzUntil = 0; int buzzMode = 0;   // 0 off,1 beep,2 alarm,3 music
unsigned long lastPub = 0, lastAuto = 0, lastLcd = 0;

// thresholds
int thSoil = 30, thTemp = 32, thLight = 20, thWater = 15, thRain = 40, thDist = 7;

// sensor values
int temp = 0, hum = 0, soil = 0, light = 0, water = 0, rain = 0, dist = 0, pir = 0;

// ---------- actuators ----------
void setLed(bool on)  { ledOn = on;  digitalWrite(LEDPIN, on); }
void setFan(bool on)  { fanOn = on;  digitalWrite(FANPIN1, on); digitalWrite(FANPIN2, LOW); }
void setPump(bool on) { pumpOn = on; digitalWrite(RELAYPIN, on); if (!on) pumpUntil = 0; }
void setServo(int a)  { servoAngle = constrain(a, 0, 180); feeder.write(servoAngle); }
void buzz(int mode, unsigned long ms) { buzzMode = mode; buzzUntil = millis() + ms; if (!mode) ledcWriteTone(BUZZERPIN, 0); }

int pct(int raw, float gain) { int p = raw / 4095.0 * 100 * gain; return p > 100 ? 100 : p; }

int readDistance() {
  digitalWrite(TRIGPIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH); delayMicroseconds(10); digitalWrite(TRIGPIN, LOW);
  long d = pulseIn(ECHOPIN, HIGH, 25000);   // timeout ~4m กันค้าง
  return d ? d / 58 : 0;
}

void readSensors() {
  if (DHT11.read(DHT11PIN) == 0) { temp = DHT11.temperature; hum = DHT11.humidity; }
  soil  = pct(analogRead(SOILPIN), 2.3);
  water = pct(analogRead(WATERPIN), 2.5);
  rain  = pct(analogRead(RAINPIN), 1.0);
  light = pct(analogRead(LIGHTPIN), 1.0);
  dist  = readDistance();
  pir   = digitalRead(PIRPIN);
}

void publishStatus() {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"temp\":%d,\"hum\":%d,\"soil\":%d,\"light\":%d,\"water\":%d,\"rain\":%d,\"dist\":%d,\"pir\":%d,"
    "\"led\":%d,\"fan\":%d,\"pump\":%d,\"servo\":%d,\"auto\":%d,\"guard\":%d}",
    temp, hum, soil, light, water, rain, dist, pir, ledOn, fanOn, pumpOn, servoAngle, autoMode, guard);
  mqtt.publish(tStatus.c_str(), buf);
}

int jsonInt(const String& s, const char* key, int def) {
  int i = s.indexOf(String("\"") + key + "\"");
  if (i < 0) return def;
  i = s.indexOf(':', i);
  return i < 0 ? def : s.substring(i + 1).toInt();
}

bool parseOnOff(const String& arg, bool cur) { return arg == "on" ? true : arg == "off" ? false : !cur; }

void handleCommand(String cmd) {
  cmd.trim();
  String key = cmd, arg = "";
  int c = cmd.indexOf(':');
  if (c >= 0) { key = cmd.substring(0, c); arg = cmd.substring(c + 1); }
  Serial.println("CMD: " + cmd);

  if      (key == "led")   setLed(parseOnOff(arg, ledOn));
  else if (key == "fan")   setFan(parseOnOff(arg, fanOn));
  else if (key == "pump")  setPump(parseOnOff(arg, pumpOn));
  else if (key == "water") { int s = constrain(arg.toInt(), 1, 60); setPump(true); pumpUntil = millis() + s * 1000UL; }
  else if (key == "servo") setServo(arg.toInt());
  else if (key == "feed")  setServo(arg == "open" ? SERVO_OPEN : SERVO_CLOSE);
  else if (key == "buzzer") buzz(arg == "off" ? 0 : 1, 600);
  else if (key == "alarm") buzz(2, 4000);
  else if (key == "music") buzz(3, 4000);
  else if (key == "guard") guard = parseOnOff(arg, guard);
  else if (key == "auto")  autoMode = parseOnOff(arg, autoMode);
  else if (key == "config") {
    thSoil = jsonInt(arg, "soil", thSoil);   thTemp  = jsonInt(arg, "temp", thTemp);
    thLight = jsonInt(arg, "light", thLight); thWater = jsonInt(arg, "water", thWater);
    thRain = jsonInt(arg, "rain", thRain);   thDist  = jsonInt(arg, "dist", thDist);
  }
  publishStatus();   // ตอบสถานะทันทีให้เว็บอัปเดตไว
}

void onMessage(char* topic, byte* payload, unsigned int len) {
  String s; s.reserve(len);
  for (unsigned int i = 0; i < len; i++) s += (char)payload[i];
  handleCommand(s);
}

void runAuto() {
  if (!autoMode) return;
  if (pumpUntil == 0) {                       // ไม่ยุ่งกับการรดน้ำแบบตั้งเวลา
    if (soil < thSoil && water > thWater) setPump(true);
    else if (soil >= thSoil + 5 || water <= thWater) setPump(false);
  }
  if (temp > thTemp) setFan(true); else if (temp < thTemp - 1) setFan(false);
  setLed(light < thLight);
  if (dist > 0 && dist <= thDist) setServo(SERVO_OPEN); else setServo(SERVO_CLOSE);
}

void runBuzzer() {
  static const int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
  unsigned long now = millis();
  if (buzzMode && now > buzzUntil) { buzzMode = 0; ledcWriteTone(BUZZERPIN, 0); }
  switch (buzzMode) {
    case 1: ledcWriteTone(BUZZERPIN, 1000); break;
    case 2: ledcWriteTone(BUZZERPIN, 200 + (now / 5) % 800); break;           // ไซเรนกวาดความถี่
    case 3: ledcWriteTone(BUZZERPIN, melody[(now / 300) % 8]); break;
  }
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lcd.clear(); lcd.print("WiFi...");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nIP: " + WiFi.localIP().toString());
}

void connectMqtt() {
  while (!mqtt.connected()) {
    String id = "ks0567_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(id.c_str())) { mqtt.subscribe(tControl.c_str()); Serial.println("MQTT connected"); }
    else { Serial.printf("MQTT fail rc=%d\n", mqtt.state()); delay(2000); }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT); pinMode(RELAYPIN, OUTPUT);
  pinMode(FANPIN1, OUTPUT); pinMode(FANPIN2, OUTPUT);
  pinMode(PIRPIN, INPUT); pinMode(TRIGPIN, OUTPUT); pinMode(ECHOPIN, INPUT);
  pinMode(RAINPIN, INPUT); pinMode(LIGHTPIN, INPUT); pinMode(SOILPIN, INPUT); pinMode(WATERPIN, INPUT);
  setLed(false); setFan(false); setPump(false);
  feeder.attach(SERVOPIN); setServo(SERVO_CLOSE);
  ledcAttachChannel(BUZZERPIN, 1000, 8, 4);
  lcd.init(); lcd.backlight();

  tStatus = String(TOPIC_BASE) + "/status";
  tControl = String(TOPIC_BASE) + "/control";
  connectWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(512);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMqtt();
  mqtt.loop();

  unsigned long now = millis();
  if (pumpUntil && now > pumpUntil) setPump(false);

  if (now - lastAuto > 1000) {
    lastAuto = now;
    readSensors();
    runAuto();
    if (guard && pir) { buzz(2, 3000); setLed((now / 250) % 2); }
  }
  runBuzzer();

  if (now - lastPub > 2000) { lastPub = now; publishStatus(); }

  if (now - lastLcd > 2000) {
    lastLcd = now;
    lcd.setCursor(0, 0); lcd.printf("T:%2dC H:%2d%% S:%2d", temp, hum, soil);
    lcd.setCursor(0, 1); lcd.printf("W:%2d%% L:%2d %s   ", water, light, autoMode ? "AUTO" : "MAN ");
  }
}
