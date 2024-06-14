#include <Wire.h> 
#include <Servo.h>

Servo servoMotor;  // Create a servo object to control a servo motor

#define SLAVE_ADDR 5
#define ANSWERSIZE 3

//ปุ่มหน้าจอ
int button_pin[4] = {24, 26, 28, 22};
int button_enter = 20;
int buttonState;

//step motor
const int stepPin = 16;
const int dirPin = 15;
const int enPin = 14;

//relay advice
const int Relay_PumpSp = 11;
const int Relay_PumpWater = 12;
const int Start_button = 13;           //used to be relay_fan 

//LED
const int Relay_LED_Red = 8;
const int Relay_LED_Green=  9 ; 
const int Relay_LED_Blue = 10;
//SW
const int limitSwitchPin = 3;
const int limitmotor = 2;
//dc_motor_หมุน
const int driveDC_PWM = 7;
const int driveDC_INA = 5;
const int driveDC_INB = 4;

int servoPin = 50; 


int position = 0;
int startposition = 90;

unsigned long previousMillis = 0;
const long interval = 500;




void setup() {
  Wire.begin(SLAVE_ADDR);
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  pinMode(Relay_PumpSp, OUTPUT);
  pinMode(Relay_LED_Green, OUTPUT);
  pinMode(Relay_LED_Red, OUTPUT);
  pinMode(Relay_PumpWater, OUTPUT);
  pinMode(Relay_LED_Blue, OUTPUT);
  pinMode(limitmotor, INPUT_PULLUP);
  pinMode(limitSwitchPin, INPUT_PULLUP);
  pinMode(button_pin[0], INPUT_PULLUP);
  pinMode(button_pin[1], INPUT_PULLUP);
  pinMode(button_pin[2], INPUT_PULLUP);
  pinMode(button_pin[3], INPUT_PULLUP);

  digitalWrite(enPin, LOW);
  digitalWrite(Relay_LED_Red, LOW);
  digitalWrite(Relay_LED_Green, HIGH);
  digitalWrite(Relay_LED_Blue, HIGH);
  digitalWrite(Relay_PumpSp, LOW);
  digitalWrite(Relay_PumpWater, LOW);

  servoMotor.attach(servoPin);
  servoMotor.write(90);

  // -------------------------------  SETUP MANUAL  ---------------------------------- //
  write_i2c(7);
  while(digitalRead(button_pin[0]) != LOW){           // รอคำสั่งจากปุ่ม 1
    Serial.println("-------press for setup--------");
  }
  //  เปิด pump spray
  servoMotor.write(90);// อยู่ตำแหน่งปิด 
  delay(100);
  digitalWrite(Relay_PumpSp, HIGH);
  delay(500);
  write_i2c(6);

  while(digitalRead(button_pin[1]) != LOW){           // รอคำสั่งจากปุ่ม 2
    Serial.println("-------press for done--------");
  }
  //  ปิด pump spray
  digitalWrite(Relay_PumpSp, LOW);
  delay(500);
  write_i2c(8);

  // ------------------------------------------------------------------------------- //

  Serial.println(">>>>>>>>>  setup done");
  digitalWrite(Relay_LED_Red, HIGH);

}

void loop() {
  write_i2c(8);
  buttonState = digitalRead(button_pin[2]);  // อ่านสถานะปุ่มปัจจุบัน

  servoMotor.write(startposition);// อยู่ตำแหน่งปิด 
  delay(1000);

  digitalWrite(Relay_PumpWater, HIGH);
  Serial.println("Start------------");

  // ------------------ STAGE 1 --------------------//
  // --   setup stepper motor to start position   --//
  digitalWrite(Relay_LED_Green, LOW);
  //write_i2c(9); // display working
  while(digitalRead(button_pin[3]) != LOW){
    
    Serial.println("Waiting for press green button...");
    if(digitalRead(button_pin[2]) == LOW){
      Serial.println("yellow pressed.");
      digitalWrite(Relay_LED_Green, HIGH);
      digitalWrite(Relay_LED_Red, LOW);
      servoMotor.write(0);// อยู่ตำแหน่งปิด 
      delay(1500);
      digitalWrite(Relay_PumpSp, HIGH);
      delay(500);
      write_i2c(6);
      delay(500);
      while(digitalRead(button_pin[2]) != LOW){
        Serial.println("Waiting for stop yellow.");
      }
      digitalWrite(Relay_PumpSp, LOW);
      delay(500);
      write_i2c(8);
      digitalWrite(Relay_LED_Green, LOW);
      digitalWrite(Relay_LED_Red, HIGH);
    }
  }
  digitalWrite(Relay_LED_Green, HIGH);
  
  // ------------------ STAGE 2 -------------------//
  //--------- set zero them start the pump --------//
  digitalWrite(Relay_LED_Blue, LOW);
  write_i2c(9);
  //ทำงาานแขนพ่น อันนี้เช็คว่าอยู่ตำแหน่งเริ่มต้นยัง
  servoMotor.write(startposition);// อยู่ตำแหน่งปิด 
  delay(1000);
  digitalWrite(Relay_PumpWater, LOW);
  delay(1000);
  digitalWrite(dirPin, HIGH);
  while(digitalRead(limitmotor) == HIGH){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  Serial.println("start stepper--------");
  delay(1000);
  // ทำงานแขนพ่น
  // digitalWrite(dirPin, LOW);
  // for (int R = 0; R < 400; R++) {
  //   digitalWrite(stepPin, HIGH);
  //   delayMicroseconds(500);
  //   digitalWrite(stepPin, LOW);
  //   delayMicroseconds(500);
  // }

  Serial.println("start dc and spray--------");
  //แท่นหมุน
  analogWrite(driveDC_PWM, 255);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, HIGH);
  delay(1000);// อยู่ตำแหน่งปิด 
  digitalWrite(Relay_PumpSp, HIGH);
  delay(1000);
  servoMotor.write(0);

  // digitalWrite(dirPin, LOW);
  //   for (int R = 0; R < 800; R++) {
  //     digitalWrite(stepPin, HIGH);
  //     delayMicroseconds(700);
  //     digitalWrite(stepPin, LOW);
  //     delayMicroseconds(700);
  //    }
  for (int i = 0; i < 3; i++) {
    digitalWrite(dirPin, LOW);
    for (int R = 0; R < 600; R++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
    }
    delay(100);
    digitalWrite(dirPin, HIGH);
    for (int R = 0; R < 600; R++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
    }
  }
  // delay(26000); 
    // digitalWrite(dirPin, LOW);
  // for (int R = 0; R < 1200; R++) {
  //   digitalWrite(stepPin, HIGH);
  //   delayMicroseconds(500);
  //   digitalWrite(stepPin, LOW);
  //   delayMicroseconds(500);
  // } // เวลาในการพ่นสเป
  servoMotor.write(90);// อยู่ตำแหน่งปิด 
  delay(1500);
  digitalWrite(Relay_PumpSp, LOW);
  delay(2000);
  
  //หยุดแท่นหมุน
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, LOW);
  delay(500);
  
  Serial.println("----------------------- DONE -----------------------");
  write_i2c(8);
  digitalWrite(Relay_LED_Blue, HIGH);
 }


//----------------------------------------------------------------------------------------------------//
//------------------------------------------------  ALL  ---------------------------------------------//
// ---------------------------------------- FUNCTION BELOW HERE --------------------------------------//

// เขียน i2c ส่ง
void write_i2c(int value) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(value);
  Wire.endTransmission();
}

//รับ i2c
String receive_i2c(){
  Serial.println("Receive data");
  // Read response from Slave---------
  Wire.requestFrom(SLAVE_ADDR,ANSWERSIZE);
  
  // Add characters to string
  String text_response = "";
  byte response[ANSWERSIZE];
  while (Wire.available()) {
    for (byte i=0;i<ANSWERSIZE;i++) {
       response[i] = (byte)Wire.read();
    }
  } 
  for (byte i = 0; i < ANSWERSIZE; i++) {
    Serial.print(char(response[i]));
    text_response += (char)response[i];
  }
  Serial.println("");
  return text_response;

}

