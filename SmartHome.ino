#include <DHT.h>

#define B 5
#define R 18
#define G 19

#define DHT_TYPE DHT22
#define DHT_PIN 12

#define FAN 15
#define SERVO_PIN 13
#define TRIG_PIN 27
#define ECHO_PIN 26
#define MOTION_PIN 25
#define LDR_DO_PIN 33
#define SMOKE_AO_PIN 32
#define IR_FLAME_PIN 35

DHT dht(DHT_PIN, DHT_TYPE);
// --- متغیرها ---
bool doorOpened = false;
unsigned long lastDoorActionTime = 0;

bool alarmTouchLastState = false;
unsigned long lastAlarmTouchTime = 0;
const unsigned long alarmTouchDebounce = 250;

bool ledTouchLastState = false;
unsigned long lastLedTouchTime = 0;
const unsigned long ledTouchDebounce = 250;

bool isArmed = false;
bool buzzerOn = false;
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 20000;  // ۲۰ ثانیه

bool touchLedState = false;

bool motionLedOn = false;
unsigned long motionLedStartTime = 0;

unsigned long lastUltrasonicRead = 0;
const unsigned long ultrasonicInterval = 2000;

unsigned long lastSensorCheck = 0;

// مقادیر هیسترزیس برای پارکینگ
const float openDistanceMin = 5;
const float openDistanceMax = 10;
const float closeDistance = 20;

void setup() {
  Serial.begin(115200);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(IR_FLAME_PIN, INPUT);

  // pinMode(ALARM_TOUCH_PIN, INPUT);
  // pinMode(LED_TOUCH_PIN, INPUT);
  // pinMode(TOUCH_LED_PIN, OUTPUT);
  // digitalWrite(TOUCH_LED_PIN, LOW);

  // pinMode(BUZZER_PIN, OUTPUT);
  // digitalWrite(BUZZER_PIN, LOW);

  pinMode(MOTION_PIN, INPUT);

  pinMode(LDR_DO_PIN, INPUT);

  // digitalWrite(LDR_LED_PIN, LOW);
  if(ledcSetClockSource(LEDC_USE_APB_CLK))
  Serial.println("CLOCK SOURCE SET");
  ledcAttach(SERVO_PIN, 20, 20); 
  ledcWrite(SERVO_PIN,10);
  dht.begin();

  Serial.println("🚀 سیستم راه‌اندازی شد");
  analogWrite(R, 255);
  analogWrite(G, 255);
  analogWrite(B, 255);
}

void loop() {
  unsigned long now = millis();
  // --- تاچ برای فعال/غیرفعال کردن دزدگیر ---
  // bool currentAlarmTouch = (digitalRead(ALARM_TOUCH_PIN) == LOW);
  // if (currentAlarmTouch && !alarmTouchLastState && (now - lastAlarmTouchTime > alarmTouchDebounce)) {
  //   isArmed = !isArmed;
  //   Serial.println(isArmed ? "🔒 دزدگیر فعال شد" : "🔓 دزدگیر غیرفعال شد");

  // if (!isArmed) {
  //   digitalWrite(BUZZER_PIN, LOW);
  //   buzzerOn = false;
  // }
  // lastAlarmTouchTime = now;
  // }
  // alarmTouchLastState = currentAlarmTouch;

  // --- تاچ برای کنترل LED ---
  // bool currentLedTouch = (digitalRead(LED_TOUCH_PIN) == LOW);
  // if (currentLedTouch && !ledTouchLastState && (now - lastLedTouchTime > ledTouchDebounce)) {
  //   touchLedState = !touchLedState;
  //   digitalWrite(TOUCH_LED_PIN, touchLedState ? HIGH : LOW);
  //   Serial.println(touchLedState ? "💡 LED روشن شد" : "💡 LED خاموش شد");
  //   lastLedTouchTime = now;
  // }
  // ledTouchLastState = currentLedTouch;

  // --- دزدگیر و موشن ---
  if (isArmed && digitalRead(MOTION_PIN) == HIGH) {
    if (!buzzerOn) {
      // digitalWrite(BUZZER_PIN, HIGH);
      // buzzerOn = true;
      // buzzerStartTime = now;
      Serial.println("🚨 حرکت تشخیص داده شد → بازر فعال شد");
    }
  }

  // --- موشن وقتی دزدگیر خاموش است ---
  // if (!isArmed && digitalRead(MOTION_PIN) == HIGH && !motionLedOn) {
  //   digitalWrite(TOUCH_LED_PIN, HIGH);
  //   motionLedOn = true;
  //   motionLedStartTime = now;
  //   Serial.println("👣 حرکت تشخیص داده شد (دزدگیر خاموش) → LED روشن شد");
  // }

  // --- خاموشی خودکار LED موشن ---
  if (motionLedOn && (now - motionLedStartTime >= 20000)) {
    // digitalWrite(TOUCH_LED_PIN, LOW);
    motionLedOn = false;
    Serial.println("💡 LED موشن خاموش شد");
  }

  // --- اولتراسونیک ---
  if (now - lastUltrasonicRead >= ultrasonicInterval) {
    lastUltrasonicRead = now;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 20000);  // کاهش تایم‌آوت
    if (duration > 0) {
      float distance = duration * 0.0343 / 2;
      Serial.print("📏 فاصله: ");
      Serial.print(distance);
      Serial.println(" cm");

      if (distance > closeDistance) {
        ledcWrite(SERVO_PIN,10);
        doorOpened = false;
        Serial.println("🚗 ماشین در پارکینگ نیست");
      } else if (distance >= openDistanceMin && distance <= openDistanceMax && !doorOpened) {
        ledcWrite(SERVO_PIN,200);
        doorOpened = true;
        lastDoorActionTime = now;
        Serial.println("🔓 باز کردن درب...");
      } else if (distance < openDistanceMin) {
        Serial.println("✅ ماشین در پارکینگ هست");
      }

      if (doorOpened && now - lastDoorActionTime > 3000) {
        ledcWrite(SERVO_PIN,10);
        Serial.println("🔒 درب بسته شد");
        doorOpened = false;
      }
    }
  }

  // --- خاموشی خودکار بازر ---
  if (buzzerOn && (now - buzzerStartTime >= buzzerDuration)) {
    // digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    Serial.println("🔕 بازر خاموش شد خودکار");
  }

  // --- خواندن سنسورها هر 5 ثانیه ---
  if (now - lastSensorCheck > 5000) {
    lastSensorCheck = now;

    // --- شعله ---
    bool flame = (digitalRead(IR_FLAME_PIN) == LOW);

    if (flame) {
      Serial.println("🔥 شعله تشخیص داده شد!");
      analogWrite(R, 255);
      analogWrite(G, 19);
      analogWrite(B, 0);
    }

    // --- دود ---
    int smokeVal = analogRead(SMOKE_AO_PIN);
    Serial.print("🌫️ دود: ");
    Serial.println(smokeVal);
    if (smokeVal > 300) {
      Serial.println("⚠️ دود زیاد!");
      analogWrite(R, 255);
      analogWrite(G, 0);
      analogWrite(B, 22);
    } else if (!flame) {
      analogWrite(R, 255);
      analogWrite(G, 255);
      analogWrite(B, 255);
    }

    // --- دما و رطوبت محیط ---
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();

    if (!isnan(temp) && !isnan(hum)) {
      Serial.print("🌡️ دما: ");
      Serial.print(temp);
      Serial.print(" °C | 💧 رطوبت: ");
      Serial.print(hum);
      Serial.println(" %");
    } else {
      Serial.println("⚠️ خطا در خواندن DHT");
    }

    // --- LDR ---
    int ldrStatus = digitalRead(LDR_DO_PIN);
    if (ldrStatus == LOW) {
      Serial.println("🌙 تاریکی → LED خاموش");
    } else {
      Serial.println("☀️ نور کافی → LED روشن");
    }
  }
}
