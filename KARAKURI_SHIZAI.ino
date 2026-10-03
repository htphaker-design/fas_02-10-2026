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
const unsigned long THOI_GIAN_CHONG_NHIEU = 10; 

// Biến trạng thái để khóa/mở hệ thống
bool daTimDuocGoc = false; 
unsigned long thoiGianBatDauLow = 0;
bool dangXacNhanGoc = false;

void setup() {
  pinMode(CAM_BIEN_12, INPUT_PULLUP);
  pinMode(PIN_TIEN, INPUT_PULLUP);
  pinMode(PIN_LUI, INPUT_PULLUP);

  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); 

  stepper.setMaxSpeed(2000.0); 
}

void loop() {

  if (!daTimDuocGoc) {
    stepper.setSpeed(TOC_DO_TIM_GOC);
    stepper.runSpeed(); 

    if (digitalRead(CAM_BIEN_12) == LOW) {
      if (!dangXacNhanGoc) {
        dangXacNhanGoc = true;
        thoiGianBatDauLow = millis();
      } else if (millis() - thoiGianBatDauLow >= THOI_GIAN_CHONG_NHIEU) {
        // Đã đụng cảm biến 12 -> Dừng ngay lập tức!
        stepper.stop(); 
        stepper.setCurrentPosition(0); // Lưu vị trí này là 0
        
        // Cài đặt lại thông số để sẵn sàng chạy làm việc
        stepper.setMaxSpeed(TOC_DO_LAM_VIEC); 
        stepper.setAcceleration(GIA_TOC_LAM_VIEC);
        
        daTimDuocGoc = true; // Chuyển sang TRẠNG THÁI 2 (Mở khóa)
        dangXacNhanGoc = false;
      }
    } else {
      dangXacNhanGoc = false;
    }
  } 
  // --------------------------------------------------------
  // TRẠNG THÁI 2: ĐÃ CÓ GỐC -> MỞ KHÓA CHÂN 22 VÀ 24
  // --------------------------------------------------------
  else {
    // CHÂN 22: Kích chạy tới 220 độ
    if (digitalRead(PIN_TIEN) == LOW) {
      long xungLamViec = (GOC_DO * STEPS_PER_REV) / 360.0; 
      stepper.moveTo(xungLamViec);
    }

    // CHÂN 24: Kích để lùi (Bản chất là quay lại TRẠNG THÁI 1 để tự dò gốc)
    if (digitalRead(PIN_LUI) == LOW) {
      // Ép biến này về false. Hệ thống sẽ lập tức hủy mọi lệnh đang chạy 
      // và vòng lặp tiếp theo sẽ nhảy lên TRẠNG THÁI 1 để quay dò tìm ngay lập tức!
      daTimDuocGoc = false; 
    }

    // Lệnh bắt buộc để động cơ chạy tới đích khi bấm Tiến
    stepper.run(); 
  }
}
