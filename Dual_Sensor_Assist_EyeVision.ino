#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// ================= 핀 =================
const int PIN_TRIG    = 3;
const int PIN_ECHO    = 2;
const int PIN_BUZZER  = 9;

// ================= 샘플 주기 (10 Hz) =================
const uint32_t SAMPLE_DT_MS = 100;

// ================= ToF 파라미터 =================
const float  ALPHA_TOF          = 0.3f;     // EMA
const float  DIST_NOISE_TOF_CM  = 1.5f;     // 흔들림 완화
const float  SPEED_MIN_TOF_CM_S = 6.0f;     // 속도 컷
const float  TOF_FLOOR_MIN_CM   = 15.0f;
const float  TOF_FLOOR_MAX_CM   = 180.0f;

// [핵심] ToF가 울리기까지 필요한 "연속 시간"
const uint16_t TOF_TRIGGER_MS   = 300;     // 0.3초 연속 조건


const int TOF_ALERT_FREQ        = 2500;    // 2.5kHz: 매우 날카롭고 찢어지는 듯한 큰 소리
const int TOF_TONE_INTERVAL_MS  = 150;     // 토글 간격

// ================= 초음파 파라미터 =================
const float  ALPHA_US           = 0.3f;    // EMA
const float  DIST_NOISE_US_CM   = 1.0f;
const float  SPEED_MIN_US_CM_S  = 5.0f;

// 초음파 최대거리 제한(2m)
const float    US_MAX_CM      = 200.0f;
const uint32_t US_TIMEOUT_US  = 15000;

// ====== 초음파 "변형(변화) 유무" 판단 ======
const uint8_t US_VAR_WIN     = 6;         // 6개=0.6초
const float   US_VAR_EPS_CM  = 8.0f;      // 변화폭 <= 8cm면 "변형 없음"

// ================= 전역 상태 =================
// ToF
Adafruit_VL53L1X lox;
float tofFilt = 0.0f, tofLastDist = 0.0f;
unsigned long tofLastTime = 0;
float tofSpeed = 0.0f;

// ToF 조건 연속 시간 누적
uint32_t tofGoodMs = 0;
bool  tofHazard = false;

// 초음파
float usFilt = 0.0f, usLastDist = 0.0f;
unsigned long usLastTime = 0;
int   usBand = 0;
float usSpeed = 0.0f;

// 초음파 변형 판단용 링버퍼
float usHist[US_VAR_WIN] = {0};
uint8_t usHistIdx = 0;
bool usHistFilled = false;

bool  usNoVar = false;      // 변형 없음(=계단 후보로 가정)
float usVarRange = 0.0f;

// 타이밍/부저
unsigned long nextSampleMs = 0;
unsigned long tonePrevMs = 0;
bool toneOn = false;

// 초음파 경보 (우선순위 1위)
bool usAlarmActive = false;

// ================= 프로토타입 =================
void updateUltrasonic(unsigned long now);
void updateToF(unsigned long now);
void buzzerUpdateToFAlert();
void buzzerUpdateUS(int band);
void buzzerOff();

void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  Wire.begin();
  if (!lox.begin()) {
    Serial.println("❌ VL53L1X init fail (I2C/배선 확인)");
    //while (1) delay(100);
  }
  lox.startRanging();

  Serial.println("us_cm,us_v,us_band,us_range,us_novar,tof_cm,tof_v,tof_goodms,tof_hazard");
  nextSampleMs = millis();
}

void loop() {
  unsigned long now = millis();

  // 10 Hz 스케줄러 (논블로킹)
  if ((long)(now - nextSampleMs) < 0) {
    if (usAlarmActive) buzzerUpdateUS(usBand);
    else if (tofHazard) buzzerUpdateToFAlert();
    return;
  }
  nextSampleMs += SAMPLE_DT_MS;

  // 1) 초음파 먼저 업데이트
  updateUltrasonic(now);

  // 2) 초음파 경보 판단 (우선순위 1위)
  usAlarmActive = false;
  if (usBand == 4 && usSpeed < 0) {
    usAlarmActive = true;
  } else if (usBand == 3 && usSpeed < -SPEED_MIN_US_CM_S) {
    usAlarmActive = true;
  }

  // 3) ToF 업데이트 (초음파 변형 없을 때만 + 시간 누적)
  updateToF(now);

  // 4) 부저 우선순위
  if (usAlarmActive) {
    // 초음파 울릴 땐 ToF 리셋 (왔다갔다 방지)
    tofGoodMs = 0;
    tofHazard = false;
    buzzerUpdateUS(usBand);
  } else if (tofHazard) {
    buzzerUpdateToFAlert();
  } else {
    buzzerOff();
  }

  // 로그
  Serial.print("[US] "); 
  Serial.print(usFilt, 1); 
  Serial.print("cm ("); 
  Serial.print(usSpeed, 1); 
  Serial.print("cm/s) ");
}

// ================= 초음파 업데이트 =================
void updateUltrasonic(unsigned long now) {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long echo_us = pulseIn(PIN_ECHO, HIGH, US_TIMEOUT_US);
  float rawDist = echo_us / 58.0f; // cm

  if (rawDist <= 0.0f || rawDist > US_MAX_CM) {
    if (usFilt > 0.0f) rawDist = usFilt;
    else               rawDist = US_MAX_CM;
  }

  // EMA
  if (usFilt == 0.0f) usFilt = rawDist;
  else                usFilt = ALPHA_US * rawDist + (1.0f - ALPHA_US) * usFilt;

  float dt = (now - usLastTime) / 1000.0f;
  if (dt > 0.05f) {
    float delta = usFilt - usLastDist;

    // 속도
    if (fabs(delta) < DIST_NOISE_US_CM) usSpeed = 0.0f;
    else {
      usSpeed = delta / dt;
      if (fabs(usSpeed) < SPEED_MIN_US_CM_S) usSpeed = 0.0f;
    }

    usLastDist = usFilt;
    usLastTime = now;

    // 밴드
    if (usFilt >= 10 && usFilt < 60)       usBand = 4;
    else if (usFilt >= 60 && usFilt < 120) usBand = 3;
    else                                   usBand = 0;

    // 변형(변화폭) 계산
    usHist[usHistIdx++] = usFilt;
    if (usHistIdx >= US_VAR_WIN) { usHistIdx = 0; usHistFilled = true; }

    if (usHistFilled) {
      float mn = usHist[0], mx = usHist[0];
      for (uint8_t i = 1; i < US_VAR_WIN; i++) {
        if (usHist[i] < mn) mn = usHist[i];
        if (usHist[i] > mx) mx = usHist[i];
      }
      usVarRange = mx - mn;
      usNoVar = (usVarRange <= US_VAR_EPS_CM);
    } else {
      usVarRange = 0.0f;
      usNoVar = false; 
    }
  }
}

// ================= ToF 업데이트 (시간 누적 방식) =================
void updateToF(unsigned long now) {
  int16_t mm = lox.distance();
  if (mm < 50 || mm > 4000) {
    tofGoodMs = 0;
    tofHazard = false;
    return;
  }

  float cm = mm / 10.0f;

  if (tofFilt == 0.0f) tofFilt = cm;
  else                 tofFilt = ALPHA_TOF * cm + (1.0f - ALPHA_TOF) * tofFilt;

  uint32_t dt_ms = (uint32_t)(now - tofLastTime);
  float dt = dt_ms / 1000.0f;

  if (dt > 0.05f) {
    float d = tofFilt - tofLastDist;
    float v = 0.0f;

    if (fabs(d) >= DIST_NOISE_TOF_CM) {
      v = d / dt;
      if (fabs(v) < SPEED_MIN_TOF_CM_S) v = 0.0f;
    }
    tofSpeed = v;

    bool allowToF = (usNoVar && !usAlarmActive);

    bool tofCond =
      allowToF &&
      !isnan(tofFilt) &&
      (tofFilt >= TOF_FLOOR_MIN_CM && tofFilt <= TOF_FLOOR_MAX_CM) &&
      (v > 0.0f);

    if (tofCond) {
      tofGoodMs += dt_ms;
      if (tofGoodMs > 5000) tofGoodMs = 5000;
    } else {
      tofGoodMs = 0;
    }

    tofLastDist = tofFilt;
    tofLastTime = now;
  }

  tofHazard = (tofGoodMs >= TOF_TRIGGER_MS);
}

// ================= 부저 =================
void buzzerUpdateToFAlert() {
  unsigned long now = millis();
  if (now - tonePrevMs >= (unsigned long)TOF_TONE_INTERVAL_MS) {
    tonePrevMs = now;
    if (toneOn) {
      noTone(PIN_BUZZER);
      toneOn = false;
    } else {
      tone(PIN_BUZZER, TOF_ALERT_FREQ); // 3500Hz
      toneOn = true;
    }
  }
}

void buzzerUpdateUS(int band) {
  static unsigned long prevToneMillis = 0;
  static bool usToneOn = false;
  static int toneInterval = 0;
  int freq = 0;

  if (band == 4) { 
    freq = 2000;         // 900 -> 2000 Hz (가까움: 높고 큰 소리)
    toneInterval = 100; 
  }
  else if (band == 3) { 
    freq = 1000;         // 500 -> 1000 Hz (접근: 적당히 큰 소리)
    toneInterval = 200; 
  }
  else { buzzerOff(); return; }

  unsigned long now = millis();
  if (now - prevToneMillis >= (unsigned long)(toneInterval / 2)) {
    prevToneMillis = now;
    if (usToneOn) { noTone(PIN_BUZZER); usToneOn = false; }
    else { tone(PIN_BUZZER, freq); usToneOn = true; }
  }
}

void buzzerOff() {
  noTone(PIN_BUZZER);
  toneOn = false;
}