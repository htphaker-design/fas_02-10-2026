#include <Arduino.h>
#include <AccelStepper.h>

// ============================================================================
// --- CẤU HÌNH CẢM BIẾN & NÚT NHẤN ---
// ============================================================================
#define LINE_STATE         LOW     // Mức kích cảm biến dò vạch (LOW = nhận vạch)
#define START_BUTTON_PIN   53      // Nút nhấn Start (INPUT_PULLUP)
#define STOP_BUTTON_PIN    52      // Nút nhấn Stop (INPUT_PULLUP - Dừng ngay về IDLE)
#define SAFETY_SENSOR_PIN  A15     // Cảm biến an toàn (Kích mức LOW -> Dừng khẩn cấp)

// --- CẤU HÌNH ĐÈN & CÒI TÍN HIỆU ---
#define RED_LED_PIN        48
#define YELLOW_LED_PIN     49
#define GREEN_LED_PIN      50
#define HORN_PIN           51     

#define LED_ON             HIGH
#define LED_OFF            LOW
#define HORN_ON            HIGH     
#define HORN_OFF           LOW

// --- CẤU HÌNH ĐỘNG CƠ BƯỚC ---
#define PUL_L 6
#define DIR_L 5
#define PUL_R 8
#define DIR_R 7
#define ENA_PIN 9

AccelStepper stepperL(AccelStepper::DRIVER, PUL_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, PUL_R, DIR_R);

// --- CẤU HÌNH CẢM BIẾN DÒ VẠCH (10 CẢM BIẾN: A1 -> A10) ---
const int NUM_SENSORS = 10;
const int sensorPins[NUM_SENSORS] = {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};
bool sensorStates[NUM_SENSORS];

// ============================================================================
// --- CẤU HÌNH TỐC ĐỘ 2000 STEPS/S & RẼ NGÃ 3 ỔN ĐỊNH ---
// ============================================================================
const float TARGET_BASE_SPEED     = 2000.0f;  // Tốc độ nền 2000 steps/s
const float OUTER_TURN_BASE_SPEED = 600.0f;   // Tốc độ cua vạch ngoài
const float SLOW_BASE_SPEED       = 400.0f;   
const float MAX_MOTOR_SPEED       = 2500.0f;  // Tốc độ trần bánh xe
const float MIN_MOTOR_SPEED       = 40.0f;    

// TỐC ĐỘ RẼ NGÃ 3
const float BRANCH_HARD_SPEED_L   = 300.0f;   
const float BRANCH_HARD_SPEED_R   = 850.0f;  

const float ACCEL_RAMP_TIME       = 8.0f;     // Thời gian khởi động 8.0 giây
const float DECEL_RATE            = 900.0f;   
const float MAX_STEPPER_ACCEL     = 1400.0f;  // Giới hạn gia tốc chống mất bước

// THỜI GIAN LỌC NHIỄU NGÃ 3
const unsigned long BRANCH_DEBOUNCE_TIME = 15; 

// THỜI GIAN GIẢM TỐC VÀ DỪNG TRẠM
const unsigned long STATION_DECEL_TIME = 4500;  
const unsigned long STATION_PAUSE_TIME = 23000; 

const float NORMAL_RAMP_TAU       = 0.080f;   
const float FAST_RAMP_TAU         = 0.035f;   

const unsigned long START_DELAY_TIME       = 3000; 
const unsigned long LOST_LINE_DELAY_INNER = 380;  
const unsigned long LOST_LINE_DELAY_OUTER = 1100; 
const unsigned long OBSTACLE_WAIT_TIME     = 5000; 

// ============================================================================
// --- BỘ THAM SỐ PID CHẠY THẲNG & VÀO CUA ---
// ============================================================================
const float SET_POSITION = 4.5f; 
const float DEADBAND     = 0.05f;             

const float KP_STRAIGHT  = 60.0f;   
const float KD_STRAIGHT  = 20.0f;   

const float KP_TURN      = 85.0f;  
const float KD_TURN      = 28.0f;   

const float ERROR_TURN_THRESHOLD = 0.32f; 
const float DERIV_TURN_THRESHOLD = 8.5f;  

float lastError = 0.0f;
float filteredError = 0.0f; 
float lastPosition = SET_POSITION;
int lastOuterDirection = 0; 

unsigned long lastControlTaskTime = 0;
unsigned long lastRampTime = 0;
const unsigned long CONTROL_INTERVAL = 5;     

// ============================================================================
// --- MÁY TRẠNG THÁI & BIẾN TOÀN CỤC ---
// ============================================================================
enum AGVState { IDLE, COUNTDOWN, RUNNING, BRANCH_TURNING_RIGHT, STOPPING_STATION, PAUSE_23S, OBSTACLE_WAIT };
AGVState agvState = IDLE;

float currentBaseSpeed = 0.0f; 
float targetBaseSpeed  = 0.0f; 

float actualSpeedL = 0.0f;
float actualSpeedR = 0.0f;

unsigned long countdownStartMillis = 0;
unsigned long lostLineStartMillis = 0;
unsigned long obstacleClearStartMillis = 0;
unsigned long rampStartMillis = 0; 

unsigned long branchTurnStartMillis = 0;
unsigned long branchDetectStartMillis = 0; 
int branchTurnPhase = 0; 

bool prevLeftStationState = false;    
unsigned long stationDecelStartMillis = 0;
float stationDecelStartSpeed = 0.0f;
unsigned long pause23sStartMillis = 0;  

unsigned long lastStartPress = 0;
unsigned long lastStopPress = 0;

// ============================================================================
// --- HÀM XỬ LÝ DỪNG KHẨN CẤP / DỪNG HẲN (STOP/SAFETY) ---
// ============================================================================
inline void forceStopMotors() {
  actualSpeedL = 0.0f;
  actualSpeedR = 0.0f;
  currentBaseSpeed = 0.0f;
  targetBaseSpeed = 0.0f;
  stepperL.setSpeed(0);
  stepperR.setSpeed(0);
}

// ============================================================================
// --- HÀM ĐỌC CẢM BIẾN & TÍNH VỊ TRÍ ---
// ============================================================================
inline void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorStates[i] = (digitalRead(sensorPins[i]) == LINE_STATE);
  }
}

float getLinePosition() {
  float sum = 0.0f;
  int count = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorStates[i]) {
      sum += (float)i;
      count++;
    }
  }

  if (count > 0) {
    lastPosition = sum / (float)count;
    if (sensorStates[NUM_SENSORS - 1]) lastOuterDirection = 1;
    else if (sensorStates[0]) lastOuterDirection = -1;
    else lastOuterDirection = 0;
    
    return lastPosition;
  }
  
  return -1.0f; 
}

inline bool checkLeftStationTrigger() {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (sensorStates[i]) count++;
  }
  return (count >= 4);
}

void updateLEDsAndHorn() {
  switch (agvState) {
    case IDLE:
      digitalWrite(RED_LED_PIN, LED_ON);    
      digitalWrite(YELLOW_LED_PIN, LED_OFF);
      digitalWrite(GREEN_LED_PIN, LED_OFF);
      digitalWrite(HORN_PIN, HORN_OFF);
      break;

    case COUNTDOWN:
    case OBSTACLE_WAIT:
    case BRANCH_TURNING_RIGHT:
      digitalWrite(RED_LED_PIN, LED_OFF);
      digitalWrite(YELLOW_LED_PIN, LED_ON);  
      digitalWrite(GREEN_LED_PIN, LED_OFF);
      digitalWrite(HORN_PIN, HORN_ON);       
      break;

    case STOPPING_STATION:
    case PAUSE_23S:
      digitalWrite(RED_LED_PIN, LED_OFF);
      digitalWrite(YELLOW_LED_PIN, LED_ON);  
      digitalWrite(GREEN_LED_PIN, LED_OFF);
      digitalWrite(HORN_PIN, HORN_ON);       
      break;

    case RUNNING:
      digitalWrite(RED_LED_PIN, LED_OFF);
      digitalWrite(YELLOW_LED_PIN, LED_OFF);
      digitalWrite(GREEN_LED_PIN, LED_ON);  
      digitalWrite(HORN_PIN, HORN_ON);       
      break;
  }
}

// ============================================================================
// --- TÁC VỤ ĐIỀU KHIỂN NỀN (200 HZ) ---
// ============================================================================
void processControlTask(unsigned long currentMillis) {
  readSensors();

  // --------------------------------------------------------------------------
  // KIỂM TRA CẢM BIẾN AN TOÀN A15 (TẠM DỪNG KHI CÓ VẬT CẢN)
  // --------------------------------------------------------------------------
  if (digitalRead(SAFETY_SENSOR_PIN) == LOW) {
    forceStopMotors();
    if (agvState != IDLE && agvState != COUNTDOWN) {
      agvState = OBSTACLE_WAIT;
      obstacleClearStartMillis = currentMillis;
    }
  }

  float currentPos = getLinePosition();
  bool hasLine = (currentPos >= 0.0f);

  if (!hasLine && agvState != BRANCH_TURNING_RIGHT && agvState != IDLE) {
    if (lostLineStartMillis == 0) lostLineStartMillis = currentMillis;
    unsigned long maxLostDelay = (lastOuterDirection != 0) ? LOST_LINE_DELAY_OUTER : LOST_LINE_DELAY_INNER;

    if (currentMillis - lostLineStartMillis < maxLostDelay) {
      if (lastOuterDirection == -1) currentPos = 1.0f;
      else if (lastOuterDirection == 1) currentPos = 8.0f;
      else currentPos = lastPosition;
      hasLine = true;
    }
  } else if (hasLine) {
    lostLineStartMillis = 0;
  }

  bool leftStationNow = checkLeftStationTrigger();
  bool leftStationTriggered = leftStationNow && !prevLeftStationState;
  prevLeftStationState = leftStationNow;

  // --------------------------------------------------------------------------
  // 1. MÁY TRẠNG THÁI CHÍNH
  // --------------------------------------------------------------------------
  switch (agvState) {
    case IDLE:
      forceStopMotors();
      lastError = 0.0f;
      filteredError = 0.0f;
      lastOuterDirection = 0;
      branchTurnPhase = 0;
      branchDetectStartMillis = 0;
      
      if (digitalRead(START_BUTTON_PIN) == LOW && (currentMillis - lastStartPress > 300)) {
        lastStartPress = currentMillis;
        agvState = COUNTDOWN;
        countdownStartMillis = currentMillis;
      }
      break;

    case COUNTDOWN:
      targetBaseSpeed = 0.0f;
      if (currentMillis - countdownStartMillis >= START_DELAY_TIME) {
        if (hasLine) {
          agvState = RUNNING;
          rampStartMillis = currentMillis; 
        } else {
          agvState = IDLE;
        }
      }
      break;

    case RUNNING:
      if (!hasLine) {
        agvState = IDLE;
        forceStopMotors();
        branchDetectStartMillis = 0;
        break;
      }

      // LOGIC PHÁT HIỆN NGÃ 3 (BỎ QUA KHI CÓ 5, 6, 7 CẢM BIẾN BÊN PHẢI)
      {
        int rightSensorCount = 0;
        for (int i = 3; i < 10; i++) {
          if (sensorStates[i]) rightSensorCount++;
        }

        bool rightSensors = sensorStates[7] || sensorStates[8] || sensorStates[9];
        bool centerLeftSensors = sensorStates[2] || sensorStates[3] || sensorStates[4] || sensorStates[5];

        bool isIgnoredPattern = (rightSensorCount >= 5 && rightSensorCount <= 7);
        bool rightBranchDetected = rightSensors && centerLeftSensors && !isIgnoredPattern;

        if (rightBranchDetected) {
          if (branchDetectStartMillis == 0) {
            branchDetectStartMillis = currentMillis;
          } else if (currentMillis - branchDetectStartMillis >= BRANCH_DEBOUNCE_TIME) {
            agvState = BRANCH_TURNING_RIGHT;
            branchTurnStartMillis = currentMillis;
            branchTurnPhase = 1; 
            branchDetectStartMillis = 0;
            break; 
          }
        } else {
          branchDetectStartMillis = 0; 
        }
      }

      if (leftStationTriggered) {
        agvState = STOPPING_STATION;
        stationDecelStartMillis = currentMillis;
        stationDecelStartSpeed = currentBaseSpeed;
        branchDetectStartMillis = 0;
        break;
      }

      targetBaseSpeed = TARGET_BASE_SPEED;
      break;

    case BRANCH_TURNING_RIGHT:
      {
        unsigned long elapsed = currentMillis - branchTurnStartMillis;
        bool centerLine = sensorStates[3] || sensorStates[4] || sensorStates[5];

        if (branchTurnPhase == 1) {
          if (elapsed >= 280) { 
            branchTurnPhase = 2; 
          }
        } 
        else if (branchTurnPhase == 2) {
          if (centerLine || elapsed > 1800) { 
            agvState = RUNNING;
            currentBaseSpeed = OUTER_TURN_BASE_SPEED;
            rampStartMillis = currentMillis;
            lastError = 0.0f;
            filteredError = 0.0f;
            branchTurnPhase = 0;
            break;
          }
        }
      }
      break;

    case STOPPING_STATION:
      if (!hasLine) {
        agvState = IDLE;
        forceStopMotors();
        break;
      }

      {
        float elapsed = (float)(currentMillis - stationDecelStartMillis) / (float)STATION_DECEL_TIME;
        if (elapsed >= 1.0f) {
          targetBaseSpeed = 0.0f;
          agvState = PAUSE_23S;
          pause23sStartMillis = currentMillis;
        } else {
          targetBaseSpeed = stationDecelStartSpeed * (1.0f - elapsed);
        }
      }
      break;

    case PAUSE_23S:
      targetBaseSpeed = 0.0f;
      if (currentMillis - pause23sStartMillis >= STATION_PAUSE_TIME) {
        agvState = RUNNING;
        rampStartMillis = currentMillis; 
      }
      break;

    case OBSTACLE_WAIT:
      targetBaseSpeed = 0.0f;
      forceStopMotors();
      
      if (digitalRead(SAFETY_SENSOR_PIN) == LOW) {
        obstacleClearStartMillis = currentMillis;
      } else if (currentMillis - obstacleClearStartMillis >= OBSTACLE_WAIT_TIME) {
        agvState = RUNNING;
        rampStartMillis = currentMillis; 
      }
      break;
  }

  // --------------------------------------------------------------------------
  // 2. RAMP TỐC ĐỘ NỀN & TÍNH TOÁN TỐC ĐỘ BÁNH XE
  // --------------------------------------------------------------------------
  float dtRamp = (currentMillis - lastRampTime) / 1000.0f;
  lastRampTime = currentMillis;

  if (agvState == RUNNING && targetBaseSpeed == TARGET_BASE_SPEED) {
    float progress = (float)(currentMillis - rampStartMillis) / (ACCEL_RAMP_TIME * 1000.0f);
    if (progress > 1.0f) progress = 1.0f;

    float sFactor = progress * progress * (3.0f - 2.0f * progress);
    float calculatedSpeed = targetBaseSpeed * sFactor;

    if (calculatedSpeed > currentBaseSpeed) {
      currentBaseSpeed = calculatedSpeed;
    } else if (currentBaseSpeed > targetBaseSpeed) {
      currentBaseSpeed -= DECEL_RATE * dtRamp;
      if (currentBaseSpeed < targetBaseSpeed) currentBaseSpeed = targetBaseSpeed;
    }
  } else {
    if (currentBaseSpeed > targetBaseSpeed) {
      currentBaseSpeed -= DECEL_RATE * dtRamp;
      if (currentBaseSpeed < targetBaseSpeed) currentBaseSpeed = targetBaseSpeed;
    } else if (currentBaseSpeed < targetBaseSpeed) {
      currentBaseSpeed += DECEL_RATE * dtRamp;
      if (currentBaseSpeed > targetBaseSpeed) currentBaseSpeed = targetBaseSpeed;
    }
  }

  float targetSpeedL = 0.0f;
  float targetSpeedR = 0.0f;
  bool isCurvePredicted = false;

  if (agvState == BRANCH_TURNING_RIGHT) {
    targetSpeedL = BRANCH_HARD_SPEED_L;
    targetSpeedR = BRANCH_HARD_SPEED_R;
  } 
  else if ((agvState == RUNNING || agvState == STOPPING_STATION) && currentBaseSpeed > 0.0f) {
    float dt = (float)CONTROL_INTERVAL / 1000.0f;
    float rawError = currentPos - SET_POSITION;

    if (abs(rawError) < DEADBAND) rawError = 0.0f;

    filteredError = 0.60f * rawError + 0.40f * filteredError;

    float dError = (filteredError - lastError) / dt;
    lastError = filteredError;

    isCurvePredicted = (abs(filteredError) > ERROR_TURN_THRESHOLD) || (abs(dError) > DERIV_TURN_THRESHOLD);

    float activeKp = isCurvePredicted ? KP_TURN : KP_STRAIGHT;
    float activeKd = isCurvePredicted ? KD_TURN : KD_STRAIGHT;

    float pidOutput = (activeKp * filteredError) + (activeKd * dError);

    float pidRampScale = constrain(currentBaseSpeed / (TARGET_BASE_SPEED * 0.4f), 0.2f, 1.0f);
    pidOutput *= pidRampScale;

    float maxPidOffset = currentBaseSpeed * 0.35f; 
    pidOutput = constrain(pidOutput, -maxPidOffset, maxPidOffset);

    float dynamicBaseSpeed = currentBaseSpeed - (abs(filteredError) * 100.0f);

    if (sensorStates[0] || sensorStates[NUM_SENSORS - 1]) {
      if (dynamicBaseSpeed > OUTER_TURN_BASE_SPEED) {
        dynamicBaseSpeed = OUTER_TURN_BASE_SPEED;
      }
    }

    targetSpeedL = dynamicBaseSpeed - pidOutput;
    targetSpeedR = dynamicBaseSpeed + pidOutput;

    targetSpeedL = constrain(targetSpeedL, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    targetSpeedR = constrain(targetSpeedR, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  } else {
    targetSpeedL = 0.0f;
    targetSpeedR = 0.0f;
  }

  // --------------------------------------------------------------------------
  // 3. GIỚI HẠN GIA TỐC VÀ XUẤT XUNG BÁNH XE
  // --------------------------------------------------------------------------
  if (agvState != OBSTACLE_WAIT && agvState != IDLE) {
    float dt = (float)CONTROL_INTERVAL / 1000.0f;
    float activeTau = (agvState == BRANCH_TURNING_RIGHT || isCurvePredicted) ? FAST_RAMP_TAU : NORMAL_RAMP_TAU;
    float alpha = dt / (activeTau + dt);

    float desiredChangeL = (targetSpeedL - actualSpeedL) * alpha;
    float desiredChangeR = (targetSpeedR - actualSpeedR) * alpha;

    float maxAllowedChange = MAX_STEPPER_ACCEL * dt;
    desiredChangeL = constrain(desiredChangeL, -maxAllowedChange, maxAllowedChange);
    desiredChangeR = constrain(desiredChangeR, -maxAllowedChange, maxAllowedChange);

    actualSpeedL += desiredChangeL;
    actualSpeedR += desiredChangeR;
  } else {
    actualSpeedL = 0.0f;
    actualSpeedR = 0.0f;
  }

  if (abs(actualSpeedL) < 1.0f) actualSpeedL = 0.0f;
  if (abs(actualSpeedR) < 1.0f) actualSpeedR = 0.0f;

  stepperL.setSpeed(actualSpeedL);
  stepperR.setSpeed(-actualSpeedR);

  updateLEDsAndHorn();
}

// ============================================================================
// --- SETUP HỆ THỐNG ---
// ============================================================================
void setup() {
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SAFETY_SENSOR_PIN, INPUT_PULLUP);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, LED_OFF);
  digitalWrite(YELLOW_LED_PIN, LED_OFF);
  digitalWrite(GREEN_LED_PIN, LED_OFF);
  digitalWrite(HORN_PIN, HORN_OFF);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }

  stepperL.setMaxSpeed(4000);
  stepperR.setMaxSpeed(4000);

  lastControlTaskTime = millis();
  lastRampTime = millis();
}

// ============================================================================
// --- VÒNG LẶP CHÍNH ---
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // ĐỌC NÚT STOP (CHÂN 52) - ƯU TIÊN CAO, DỪNG TỨC THÌ
  if (digitalRead(STOP_BUTTON_PIN) == LOW && (currentMillis - lastStopPress > 150)) {
    lastStopPress = currentMillis;
    agvState = IDLE;
    forceStopMotors();
  }

  if (currentMillis - lastControlTaskTime >= CONTROL_INTERVAL) {
    lastControlTaskTime = currentMillis;
    processControlTask(currentMillis);
  }

  stepperL.runSpeed();
  stepperR.runSpeed();
}
