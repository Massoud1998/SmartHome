#include <Arduino.h>
#include <DHT.h>

// -------------------------
// Pins & configuration
// -------------------------
#define B_PIN 5
#define R_PIN 18
#define G_PIN 19

#define DHT_TYPE DHT22
#define DHT_PIN 12

#define FAN_PIN 15
#define SERVO_PIN 13
#define TRIG_PIN 27
#define ECHO_PIN 26
#define MOTION_PIN 25
#define LDR_DO_PIN 33
#define SMOKE_AO_PIN 32
#define IR_FLAME_PIN 35

#define ULTRASONIC_WIDTH_2_DISTANCE_COEFFICIENT (0.0343f / 2.0f)

DHT dht(DHT_PIN, DHT_TYPE);

// -------------------------
// Hysteresis / timing
// -------------------------
const float openDistanceMin = 5.0f;
const float openDistanceMax = 10.0f;
const float closeDistanceFar = 20.0f;
const float closeDistanceNear = 3.0f;

const unsigned long ultrasonicInterval = 2000;
const unsigned long sensorInterval = 5000;
const unsigned long motionInterval = 1000;
// -------------------------
// Shared state (protected by simple critical sections)
// -------------------------
enum DoorState { CLOSED,
                 OPEN };
volatile DoorState doorState = CLOSED;
volatile unsigned long autoCloseDelay = 3000;


enum AlarmState {
  IDLE,       // No alarm triggered
  MOTION,     // Motion detected
  FLAME,      // Flame detected
  SMOKE_HIGH  // High smoke density detected
};

volatile AlarmState alarmState = IDLE;


volatile bool isArmed = true;
volatile bool buzzerOn = false;
volatile unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 20000;

volatile bool motionLedOn = false;
volatile unsigned long motionLedStartTime = 0;

// -------------------------
// FreeRTOS task handles
// -------------------------
TaskHandle_t TaskUltrasonicHandle = NULL;
TaskHandle_t TaskSensorsHandle = NULL;
TaskHandle_t TaskMotionHandle = NULL;
TaskHandle_t TaskRGBHandle = NULL;
// -------------------------
// FreeRTOS Queue handles
// -------------------------
QueueHandle_t debugQueue;
// -------------------------.
// -------------------------
// FreeRTOS Mutex handles
// -------------------------
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;
// -------------------------.


// RGB PWM properties
const int RGB_FREQ = 1000;  // 1kHz for LEDs
const int RGB_RES = 8;      // 8-bit resolution

// -------------------------
// Servo timer
// -------------------------
hw_timer_t* servoTimer = NULL;
volatile uint32_t servoPulseUs = 1000;  // default pulse (us)
// Servo state machine
enum ServoState { SERVO_LOW,
                  SERVO_HIGH };
volatile ServoState servoState = SERVO_LOW;

void IRAM_ATTR onServoTimerAlarmISR() {
  portENTER_CRITICAL_ISR(&servoMux);

  if (servoState == SERVO_LOW) {
    timerRestart(servoTimer);
    // Start pulse
    digitalWrite(SERVO_PIN, HIGH);
    servoState = SERVO_HIGH;
    // Set next alarm for pulse width duration
    timerAlarm(servoTimer, servoPulseUs, false, 0);  // pulse width
  } else {
    // End pulse
    digitalWrite(SERVO_PIN, LOW);
    servoState = SERVO_LOW;
    // Set next alarm for remainder of 20 ms period
    timerAlarm(servoTimer, 20000, false, 0);  // remainder
  }

  portEXIT_CRITICAL_ISR(&servoMux);
}


// -------------------------
// Prototypes
// -------------------------
void TaskUltrasonic(void* pvParameters);
void TaskSensors(void* pvParameters);
void TaskMotion(void* pvParameters);
void TaskRGB(void* pvParameters);
void TaskDebug(void* pvParameters);

void setServoAngle(int angle);
// simple helper to set RGB (0-255 each)
inline void setRGB(uint8_t r, uint8_t g, uint8_t b);

// helper function to add msgs to the queue
inline void logMessage(const char* msg);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting!");

  // pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(IR_FLAME_PIN, INPUT);
  pinMode(MOTION_PIN, INPUT);
  pinMode(LDR_DO_PIN, INPUT);

  ledcSetClockSource(LEDC_USE_APB_CLK);
  // Setup RGB channels
  ledcAttach(R_PIN, RGB_FREQ, RGB_RES);
  ledcAttach(G_PIN, RGB_FREQ, RGB_RES);
  ledcAttach(B_PIN, RGB_FREQ, RGB_RES);
  Serial.println("pin Setup Done!");
  // default white
  setRGB(255, 255, 255);

  servoTimer = timerBegin(1000000);
  if (servoTimer == NULL) {
    Serial.println("Failed to Start Timer!!");
    while (1) {
      delay(1000);
    }
  }
  analogSetPinAttenuation(SMOKE_AO_PIN, ADC_11db);
  analogReadResolution(12);
  // Attach update (overflow) interrupt
  timerAttachInterrupt(servoTimer, &onServoTimerAlarmISR);

  timerAlarm(servoTimer, servoPulseUs, false, 0);

  dht.begin();

  debugQueue = xQueueCreate(20, sizeof(char) * 256);
  // 10 messages, each up to 64 chars

  logMessage("🚀 سیستم راه‌اندازی شد (FreeRTOS tasks)");
  delay(5000);
  // Core 0: timing-critical
  xTaskCreatePinnedToCore(TaskUltrasonic, "TaskUltrasonic", 3072, NULL, 2, &TaskUltrasonicHandle, 0);
  xTaskCreatePinnedToCore(TaskMotion, "TaskMotion", 2048, NULL, 1, &TaskMotionHandle, 0);

  // Core 1: background / non-critical
  xTaskCreatePinnedToCore(TaskSensors, "TaskSensors", 4096, NULL, 3, &TaskSensorsHandle, 1);
  xTaskCreatePinnedToCore(TaskRGB, "TaskRGB", 2048, NULL, 2, &TaskRGBHandle, 1);
  xTaskCreatePinnedToCore(TaskDebug, "TaskDebug", 4096, NULL, 1, NULL, 1);

  timerStart(servoTimer);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

// -------------------------
// Tasks
// -------------------------

// Ultrasonic task: measure distance every ultrasonicInterval and control servo/door
void TaskUltrasonic(void* pvParameters) {
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();
  static unsigned long doorOpenTime = 0;
  while (1) {
    unsigned long now = millis();
    // Trigger ultrasonic pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 10000);
    if (duration > 0) {
      float distance = duration * ULTRASONIC_WIDTH_2_DISTANCE_COEFFICIENT;

      char buf[256];
      sprintf(buf, "📏 فاصله: %.1f cm", distance);
      logMessage(buf);

      taskENTER_CRITICAL(&timerMux);
      switch (doorState) {
        case CLOSED:
          if (distance >= openDistanceMin && distance <= openDistanceMax) {
            doorState = OPEN;
            doorOpenTime = now;
            setServoAngle(90);  // open
            logMessage("🔓 باز کردن درب...");
          }
          break;

        case OPEN:
          if ((distance > closeDistanceFar) || (now - doorOpenTime >= autoCloseDelay) || (distance < closeDistanceNear)) {
            doorState = CLOSED;
            setServoAngle(0);  // close
            logMessage("🔒 درب بسته شد");
          }
          break;
      }
      if (distance < closeDistanceNear) {
        logMessage("ماشین در پارکینگ است");
      } else {
        logMessage("ماشین در پارکینگ نیست");
      }
      taskEXIT_CRITICAL(&timerMux);
    } else {
      logMessage("ماشینی در نزدیکی نیست!");
      setServoAngle(0);
    }

    // Wait until next cycle
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ultrasonicInterval));
  }
}


// Sensors task: runs every sensorInterval and reads flame, smoke, DHT, LDR
void TaskSensors(void* pvParameters) {
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    unsigned long now = millis();
    char buf[256];

    // --- Flame detection ---
    bool flame = (digitalRead(IR_FLAME_PIN) == LOW);
    if (flame) {
      logMessage("🔥 شعله تشخیص داده شد!");
      alarmState = FLAME;  // Flame has highest priority
    } else if (alarmState == FLAME) {
      alarmState = IDLE;
    }

    // --- Gas / Smoke detection ---
    int smokeVal = analogRead(SMOKE_AO_PIN);

    sprintf(buf, "🌫️ دود: %d", smokeVal);
    logMessage(buf);

    if (smokeVal > 500) {
      logMessage("⚠️ دود زیاد!");
      if (alarmState != FLAME) {
        alarmState = SMOKE_HIGH;
      }  // Medium priority
    } else if (alarmState == SMOKE_HIGH) {
      alarmState = IDLE;
    }

    // --- Temperature & Humidity ---
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    if (!isnan(temp) && !isnan(hum)) {
      sprintf(buf, "🌡️ دما: %.1f °C | 💧 رطوبت: %.1f %%", temp, hum);
      logMessage(buf);
    } else {
      logMessage("⚠️ خطا در خواندن DHT");
    }

    // --- LDR / Light detection ---
    int ldrStatus = digitalRead(LDR_DO_PIN);
    if (ldrStatus == LOW) {
      logMessage("🌙 تاریکی → LED خاموش");
    } else {
      logMessage("☀️ نور کافی → LED روشن");
    }

    // Wait until next sensor check
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(sensorInterval));
  }
}

void TaskMotion(void* pvParameters) {
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    unsigned long now = millis();
    bool motionDetected = (digitalRead(MOTION_PIN) == HIGH);
    static unsigned long lastMotionDetection = 0;
    // Home security motion has the lowest priority
    if (motionDetected) {
      lastMotionDetection = now;
      if (isArmed) {
        if (alarmState != FLAME && alarmState != SMOKE_HIGH) {
          alarmState = MOTION;
        }
        logMessage("🚨 حرکت تشخیص داده شد (امنیت خانه)");
      } else {
        logMessage("👣 حرکت تشخیص داده شد (دزدگیر خاموش)");
      }
    } else if (now - lastMotionDetection > 5000) {
      alarmState = IDLE;
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(motionInterval));
  }
}


// RGB manager: placeholder task
void TaskRGB(void* pvParameters) {
  (void)pvParameters;
  enum MotionLED { RED,
                   BLUE };
  static MotionLED motionLED = RED;
  while (1) {
    switch (alarmState) {
      case FLAME:
        setRGB(255, 45, 0);
        break;
      case SMOKE_HIGH:
        setRGB(255, 0, 45);
        break;
      case MOTION:
        {
          if (motionLED == RED) {
            motionLED=BLUE;
            setRGB(0, 0, 255);
          } else {
            motionLED=RED;
            setRGB(255, 0, 0);
          }
        }
        break;
      case IDLE:
        setRGB(255, 255, 255);
        break;
      default:
        logMessage("Invalid Error State!!!!");
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void TaskDebug(void* pvParameters) {
  while (1) {
    // Wait indefinitely until a message arrives
    char msg[256];
    if (xQueueReceive(debugQueue, msg, portMAX_DELAY) == pdPASS) {
      Serial.println(msg);
    }
  }
}

// -------------------------
// Function Declrations
// -------------------------
void setServoAngle(int angle) {
  // Clamp to safe range
  angle = constrain(angle, 0, 90);

  // Map angle (0-90) to pulse (1000–2000 µs)
  int pulse = map(angle, 0, 90, 1000, 2000);

  // Update global pulse width
  servoPulseUs = pulse;
}
void logMessage(const char* msg) {
  char buf[256];
  strncpy(buf, msg, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';
  xQueueSend(debugQueue, buf, 0);
}
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(R_PIN, r);
  ledcWrite(G_PIN, g);
  ledcWrite(B_PIN, b);
}
// -------------------------.