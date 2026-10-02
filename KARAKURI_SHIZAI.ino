#include <AccelStepper.h>

#define PUL_PIN A0
#define DIR_PIN A1
#define ENA_PIN A2

#define CAM_BIEN_12 12
#define PIN_TIEN    22
#define PIN_LUI     24

AccelStepper stepper(1, PUL_PIN, DIR_PIN);

const float STEPS_PER_REV = 1000.0;
float GOC_QUET = 180.0;           
float GOC_DO = 220.0;             

float TOC_DO_QUET  = 400.0;  
float GIA_TOC_QUET = 800.0;  

float TOC_DO_CHAY  = 1200.0; 
float GIA_TOC_CHAY = 1500.0; 

void setup() {
  pinMode(CAM_BIEN_12, INPUT_PULLUP);
  pinMode(PIN_TIEN, INPUT_PULLUP);
  pinMode(PIN_LUI, INPUT_PULLUP);

  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); 

  stepper.setMaxSpeed(TOC_DO_QUET); 
  stepper.setAcceleration(GIA_TOC_QUET);

  long xungQuet = (GOC_QUET * STEPS_PER_REV) / 360.0; 
  bool dangQuetLen = true; 
  
  stepper.moveTo(xungQuet); 

  while (digitalRead(CAM_BIEN_12) == HIGH) {
    stepper.run(); 
    
    if (stepper.distanceToGo() == 0) {
      dangQuetLen = !dangQuetLen; 
      
      if (dangQuetLen) {
        stepper.moveTo(xungQuet);  
      } else {
        stepper.moveTo(-xungQuet); 
      }
    }
  }

  stepper.setCurrentPosition(0); 
  stepper.moveTo(0);             
  stepper.setSpeed(0);           
  
  delay(500); 

  stepper.setMaxSpeed(TOC_DO_CHAY); 
  stepper.setAcceleration(GIA_TOC_CHAY);
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