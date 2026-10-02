#include <AccelStepper.h>
#define PUL_PIN A0
#define DIR_PIN A1
#define ENA_PIN A2

#define CAM_BIEN_12 12
#define PIN_TIEN    22
#define PIN_LUI     24

AccelStepper stepper(1, PUL_PIN, DIR_PIN);

const float STEPS_PER_REV = 1000.0;
float GOC_DO = 220.0;             

float TOC_DO_TIM_GOC = 300.0; 

float TOC_DO_LAM_VIEC = 1200.0;  
float GIA_TOC_LAM_VIEC = 1500.0; 
const unsigned long THOI_GIAN_CHONG_NHIEU = 30; 

void setup() {
  pinMode(CAM_BIEN_12, INPUT_PULLUP);
  pinMode(PIN_TIEN, INPUT_PULLUP);
  pinMode(PIN_LUI, INPUT_PULLUP);

  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); 

  stepper.setSpeed(TOC_DO_TIM_GOC); 

  unsigned long thoiGianBatDauLow = 0;
  bool dangXacNhanGoc = false;

  while (true) {
    stepper.runSpeed(); 

    if (digitalRead(CAM_BIEN_12) == LOW) {
      if (!dangXacNhanGoc) 
      {
        // Vừa mới thấy LOW, bắt đầu ghi nhận thời gian
        dangXacNhanGoc = true;
        thoiGianBatDauLow = millis();
      } 
      else {
        // Đang là LOW, kiểm tra xem đã giữ LOW đủ 30ms chưa?
        if (millis() - thoiGianBatDauLow >= THOI_GIAN_CHONG_NHIEU) {
          // Đã giữ LOW liên tục 30ms -> Chắc chắn là công tắc, không phải nhiễu!
          break; // Thoát vòng lặp, hoàn thành tìm gốc
        }
      }
    } 
    else 
    {
      dangXacNhanGoc = false; 
    }
  }

  // Cập nhật mốc 0 tuyệt đối
  stepper.setCurrentPosition(0); 
  delay(500); 

  stepper.setMaxSpeed(TOC_DO_LAM_VIEC); 
  stepper.setAcceleration(GIA_TOC_LAM_VIEC);
}

void loop() {

  if (digitalRead(PIN_TIEN) == LOW) {
    long xungLamViec = (GOC_DO * STEPS_PER_REV) / 360.0; 
    stepper.moveTo(xungLamViec);
    stepper.runToPosition(); 
  }

  if (digitalRead(PIN_LUI) == LOW) {
    
    if (digitalRead(CAM_BIEN_12) == HIGH) {
      stepper.moveTo(0); 
      
      unsigned long thoiGianBaoVe = 0;
      bool dangXacNhanLui = false;

      while (stepper.distanceToGo() != 0) {
        stepper.run(); 

        if (digitalRead(CAM_BIEN_12) == LOW) {
          if (!dangXacNhanLui) {
            dangXacNhanLui = true;
            thoiGianBaoVe = millis();
          } else if (millis() - thoiGianBaoVe >= THOI_GIAN_CHONG_NHIEU) 
          {
            stepper.stop(); 
            stepper.runToPosition(); 
            stepper.setCurrentPosition(0); 
            break; 
          }
        } else {
          dangXacNhanLui = false;
        }
      }
    }
  }
}
  /*
  #include <AccelStepper.h>
  #define PUL_PIN A0
  #define DIR_PIN A1
  #define ENA_PIN A2
  #define CAM_BIEN_12 12
  #define PIN_TIEN    22
  #define PIN_LUI     24
    AccelStepper stepper(1, PUL_PIN, DIR_PIN);
    const float STEPS_PER_REV = 1000.0;          
    float GOC_DO = 220.0;             
    float TOC_DO_QUET  = 400.0;  
    float GIA_TOC_QUET = 800.0;  
    float TOC_DO_CHAY  = 1200.0; 
    float GIA_TOC_CHAY = 1500.0; 

  float TOC_DO_TIM_GOC = 300.0; // Tốc độ quay tròn   tìm gốc (nhỏ để dừng cho chính xác)

  // --- 2. THÔNG SỐ CHO LÚC LÀM VIỆC (Góc 220 độ) ---
  float TOC_DO_LAM_VIEC = 1200.0; // Tốc độ chạy nhanh khi làm việc
  float GIA_TOC_LAM_VIEC = 1500.0; // Gia tốc để khởi động và dừng êm ái
  void setup() {
    pinMode(CAM_BIEN_12, INPUT_PULLUP);
    pinMode(PIN_TIEN, INPUT_PULLUP);
    pinMode(PIN_LUI, INPUT_PULLUP);

    pinMode(ENA_PIN, OUTPUT);
    digitalWrite(ENA_PIN, LOW); 

    // --- BẮT ĐẦU QUÁ TRÌNH TÌM GÓC ---
    // Cài đặt tốc độ quay đều để tìm gốc
    stepper.setSpeed(TOC_DO_TIM_GOC); 

    // Vòng lặp quay tròn cho đến khi chạm cảm biến
    while (digitalRead(CAM_BIEN_12) == HIGH) {
      stepper.runSpeed(); // Hàm này cho động cơ chạy đều đặn (không gia tốc)
    }

    // Ngay khi chạm cảm biến, thoát vòng lặp và đánh dấu gốc 0
    stepper.setCurrentPosition(0); 
    
    delay(500); // Nghỉ 0.5s cho cơ khí ổn định

    // --- THIẾT LẬP THÔNG SỐ CHO LÚC LÀM VIỆC (GÓC 220 ĐỘ) ---
    // Cài đặt tốc độ và gia tốc mới, không liên quan gì đến tốc độ tìm gốc ở trên nữa
    stepper.setMaxSpeed(TOC_DO_LAM_VIEC); 
    stepper.setAcceleration(GIA_TOC_LAM_VIEC);
  }

  void loop() {
    if (digitalRead(PIN_TIEN) == LOW) {
      long xungLamViec = (GOC_DO * STEPS_PER_REV) / 360.0; 
      stepper.moveTo(xungLamViec);
      stepper.runToPosition(); 
    }

    if (digitalRead(PIN_LUI) == LOW) {
      if (digitalRead(CAM_BIEN_12) == HIGH) {
        stepper.moveTo(-1000000); 
        
        while (digitalRead(CAM_BIEN_12) == HIGH) {
          stepper.run(); 
        }

        stepper.setCurrentPosition(0); 
        stepper.moveTo(0);             
        stepper.setSpeed(0);           
      }
    }
  }
  */
