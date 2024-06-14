#include <Wire.h> 

#define SLAVE_ADDR 5
#define ANSWERSIZE 3

//ปุ่มหน้าจอ
int button_pin[4] = {24, 26, 28, 22};
int button_enter = 20;
int buttonState[4];

String final_menu = "";

//step motor
const int stepPin = 16;
const int dirPin = 15;
const int enPin = 14;

//relay advice
const int Relay_PumpSp = 11;
//const int Relay_PumpWater = 11;
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

bool setup_complete = false;
bool door_status = false;
bool complet = false;

unsigned long previousMillis = 0;
const long interval = 500;

// ---- FUNCTIONS ----// 
void MachineSetup();
String MenuBegin();
void operationWhenMenu1();
void operationWhenMenu2();
void write_i2c(int value);
String receive_i2c();
void limitSwitchInterrupt();
void when_door_opened();


void setup() {
  Wire.begin(SLAVE_ADDR);
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(Relay_PumpSp, OUTPUT);
  pinMode(Relay_LED_Green, OUTPUT);
  pinMode(Relay_LED_Red, OUTPUT);
  pinMode(Relay_LED_Blue, OUTPUT);
  pinMode(limitmotor, INPUT_PULLUP);
  pinMode(limitSwitchPin, INPUT_PULLUP);
  pinMode(button_pin[0], INPUT_PULLUP);
  pinMode(button_pin[1], INPUT_PULLUP);
  pinMode(button_pin[2], INPUT_PULLUP);
  pinMode(button_pin[3], INPUT_PULLUP);

  digitalWrite(enPin, LOW);
  digitalWrite(Relay_LED_Red, HIGH);
  digitalWrite(Relay_LED_Green, HIGH);
  digitalWrite(Relay_LED_Blue, HIGH);
  digitalWrite(Relay_PumpSp, LOW);

  //  interrupt
  attachInterrupt(digitalPinToInterrupt(limitSwitchPin), limitSwitchInterrupt, HIGH);
  //  set spray pump for minute
  write_i2c(7);
  while(digitalRead(button_pin[3]) != LOW){
    Serial.println("Waiting for press green button...");
  }
  MachineSetup_SP();
  
}

void loop() {

  Serial.println(digitalRead(limitSwitchPin));
  Serial.println(digitalRead(setup_complete));
  //Serial.println(digitalRead(door_status));

  // ------------------ STAGE 1 --------------------//
  // --   setup stepper motor to start position   --//
  digitalWrite(Relay_LED_Green, LOW);
  write_i2c(9); // display working
  while(digitalRead(button_pin[3]) != LOW){
    Serial.println("Waiting for press green button...");
    write_i2c(9);
    delay(60); 
  }
  
  // ------------------ STAGE 2 -------------------//
  //--------- set zero them start the pump --------//
  MachineSetup();
  start_spray();
  Serial.println("----------------------- DONE -----------------------");


  
  // ------------------ STAGE 2 (NOT USE)---------------------------//
  // -- put the obj into station then select type of obj --//

  // if(setup_complete == true){
  //   write_i2c(9);
  //   Serial.println("Start Select Menu");
  //   final_menu = MenuBegin();
  //   Serial.println("/////////////////------------FINAL MENU COMPLETE-------------///////////////////////"); 
  //   if (final_menu == "150200") {
  //     // กรณีที่จะทำงานเมื่อ select_menu เป็น "100"
  //     Serial.println("<<-------------------- Big_Plate --------------------------->>");
  //     Big_Plate();
  //     delay(500);
  //   } else if (final_menu == "150150") {
  //     // กรณีที่จะ reset ระบบ
  //     Serial.println("<<-------------------- Med_Plate --------------------------->>");
  //     Med_Plate();
  //     delay(500);
  //   }
  //   else if (final_menu == "150100") {
  //     // กรณีที่จะ reset ระบบ
  //     Serial.println("<<-------------------- Small_Plate --------------------------->>");
  //     Small_Plate();
  //     delay(500);
  //   }
  //   else if (final_menu == "100200") {
  //     // กรณีที่จะ reset ระบบ
  //     Serial.println("<<-------------------- Big_Bowl  --------------------------->>");
  //     Big_Bowl();
  //     delay(500);
  //   }
  //   else if (final_menu == "100150") {
  //     // กรณีที่จะ reset ระบบ
  //     Serial.println("<<-------------------- Med_Bowl --------------------------->>");
  //     Med_Bowl();
  //     delay(500);
  //   }
  //   else if (final_menu == "100100") {
  //     // กรณีที่จะ reset ระบบ
  //     Serial.println("<<-------------------- Small_Bowl --------------------------->>");
  //     Small_Bowl();
  //     delay(500);
  //   }
  // }
  // write_i2c(8);
  // ------------------------------------------- ---------------------------------------------------------------//
}


//----------------------------------------------------------------------------------------------------//
//------------------------------------------------  ALL  ---------------------------------------------//
// ---------------------------------------- FUNCTION BELOW HERE --------------------------------------//

void MachineSetup_SP(){
  digitalWrite(Relay_LED_Green, HIGH);
  digitalWrite(Relay_LED_Red, HIGH);
  digitalWrite(Relay_LED_Blue, LOW);
  Serial.println("start SP setup");
  delay(100);
  
  //------------------ setup system -------------------//
  
  digitalWrite(dirPin, HIGH);
  while(digitalRead(limitmotor) == HIGH){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    Serial.println("door state : " + String(door_status));
  }
  digitalWrite(Relay_PumpSp, HIGH);
  delay(1000);  // เวลาในการพ่นสเป
  digitalWrite(Relay_PumpSp, LOW);
  
  delay(1000);
  digitalWrite(Relay_LED_Blue, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, LOW);

  //----------------- complete setup system --------------//
  Serial.println("------->>> Setup Subsytem Complete ");
  Serial.println("------->>> Setup Complete <<<------------- ");
  
}
void MachineSetup(){
  digitalWrite(Relay_LED_Green, HIGH);
  digitalWrite(Relay_LED_Red, HIGH);
  digitalWrite(Relay_LED_Blue, LOW);
  Serial.println("start");
  delay(100);
  
  //------------------ setup system -------------------//
  digitalWrite(dirPin, HIGH);
  Serial.println("door state : " + String(door_status));
  while(digitalRead(limitmotor) == HIGH && door_status == true){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    Serial.println("door state : " + String(door_status));
  }
  digitalWrite(dirPin, LOW);
  for(int x =0; x < 200;x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    Serial.println("door state : " + String(door_status));
  }
  digitalWrite(Relay_LED_Blue, HIGH);
  digitalWrite(Relay_LED_Red, HIGH);
  digitalWrite(Relay_LED_Green, LOW);
  //----------------- complete setup system --------------//
  Serial.println("------->>> Setup Subsytem Complete ");

  setup_complete = true;
  Serial.println("------->>> Setup Complete <<<------------- ");
  
}


//เมนูเดียวจบครบ

void start_spray() {
  Serial.println("start_2");
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue, LOW);
  delay(1000);
  //digitalWrite(Relay_PumpWater, HIGH);
  //delay(1000);
  //digitalWrite(Relay_Fan, HIGH);
  //delay(1000);

  //ทำงาานแขนพ่น อันนี้เช็คว่าอยู่ตำแหน่งเริ่มต้นยัง
  digitalWrite(dirPin, HIGH);
  while(digitalRead(limitmotor) == HIGH){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  // ทำงานแขนพ่น
  digitalWrite(dirPin, LOW);
  for (int R = 0; R < 1000; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  //แท่นหมุน
  analogWrite(driveDC_PWM, 150);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, HIGH);
  delay(1000);
  digitalWrite(Relay_PumpSp, HIGH);
  
  digitalWrite(dirPin, HIGH);
  for(int i = 0; i < 10; i++);{
    for (int R = 0; R < 1000; R++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
   }
    digitalWrite(dirPin, LOW);
    for (int R = 0; R < 1000; R++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
   }
  }
  //delay(10000);  // เวลาในการพ่นสเป
  digitalWrite(Relay_PumpSp, LOW);
  delay(500);
  
  //หยุดแท่นหมุน
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, LOW);
  delay(500);
  //digitalWrite(Relay_PumpWater, LOW);
  //delay(500);
  //digitalWrite(Relay_Fan, LOW);
  //delay(500);
  digitalWrite(dirPin, HIGH);
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
 
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, LOW);
  delay(100);
  digitalWrite(Relay_LED_Blue, HIGH);
  delay(1000);
}




 //--------------------------------------------- NOT USE NOW -----------------------------------//
String MenuBegin(){
  int menu = 0;
  final_menu = "";
  String select_menu = "";
  String select_menu1 = "";

  for (int i = 0; i < 4; i++) {
    buttonState[i] = digitalRead(button_pin[i]);
  }

  //-------------------------- menu 0 ----------------------------//
  while(menu == 0){
    delay(100);
    Serial.println("-------------- menu 0 ------------------");
    for (int i = 0; i < 4; i++) {
      buttonState[i] = digitalRead(button_pin[i]);
    }
    
    // --------------------- Print Buttons State -----------------// 
    for (int i = 0; i < 4; i++) {
      Serial.print(buttonState[i]);
      Serial.print(" , ");
    }
    Serial.println("");
    // ----------------------------------------------------------//
    
    if (buttonState[0] == LOW) {
      write_i2c(1);
      delay(20);

    } else if (buttonState[1] == LOW) {
      write_i2c(2);
      delay(20);
    } else if (buttonState[2] == LOW) {
      write_i2c(3);
      delay(500);
    }
    else if (buttonState[3] == LOW) {
      write_i2c(4);
      select_menu = receive_i2c();
      delay(20);
      menu = 1;
    }
  }


  // --------------------- menu 1 -----------------------------//
  while(menu == 1){
    delay(200);
    Serial.println("-------------- menu 1 ------------------");
    for (int i = 0; i < 4; i++) {
      buttonState[i] = digitalRead(button_pin[i]);
    }
    
    // --------------------- Print Buttons State -----------------// 
    for (int i = 0; i < 4; i++) {
      Serial.print(buttonState[i]);
      Serial.print(" , ");
    }
    Serial.println("");
    // ----------------------------------------------------------//
    
    if (buttonState[0] == LOW) {
      write_i2c(1);
      delay(20);

    } else if (buttonState[1] == LOW) {
      write_i2c(2);
      delay(20);
    } else if (buttonState[2] == LOW) {
      write_i2c(3);
      menu = 0;
      delay(20);
    }
    else if (buttonState[3] == LOW) {
      write_i2c(4);
      delay(20);
      menu = 2;
      buttonState[3] == HIGH;
      select_menu1 = receive_i2c();
    }
  }

  if(menu == 2){
    Serial.print("Menu 0 : "+select_menu);
    Serial.println(",  Menu 2 : "+select_menu1);
    Serial.println("Final_Menu : "+final_menu);
    return select_menu + select_menu1;

    if (buttonState[3] == LOW) {
      write_i2c(4);
      menu = 0;
    }
  }
}

void Big_Plate() {
  Serial.println("start_1");
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue, LOW);
  delay(1000);
  //digitalWrite(Relay_PumpWater, HIGH);
  //delay(1000);
  //digitalWrite(Relay_Fan, HIGH);
  //delay(1000);
  // set ตำแหน่งแขนกล
  digitalWrite(dirPin, HIGH); //HIGH คือเคลื่อนที่ตูดลงหัวขึ้น
  while(digitalRead(limitmotor) == HIGH && door_status == true){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  // ทำงานแขนพ่น
  digitalWrite(dirPin, LOW);//LOW คือเคลื่อนที่ตูดขึ้นหัวลง
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  analogWrite(driveDC_PWM, 150);//แก้ความเร็วหมุน
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, HIGH);
  delay(1000);
  digitalWrite(Relay_PumpSp, HIGH);
  delay(10000);  // เวลาในการพ่นสเป
  digitalWrite(Relay_PumpSp, LOW);
  delay(500);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, LOW);
  delay(500);
  //digitalWrite(Relay_PumpWater, LOW);
  //delay(500);
  //digitalWrite(Relay_Fan, LOW);
  //delay(500);
  digitalWrite(dirPin, HIGH);
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, LOW);
  delay(100);
  digitalWrite(Relay_LED_Blue, HIGH);
  delay(1000);
}
//เมนู 2 
void Med_Plate() {
  Serial.println("start_2");
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue, LOW);
  delay(1000);
  //digitalWrite(Relay_PumpWater, HIGH);
  //delay(1000);
  //digitalWrite(Relay_Fan, HIGH);
  //delay(1000);

  //ทำงาานแขนพ่น อันนี้เช็คว่าอยู่ตำแหน่งเริ่มต้นยัง
  digitalWrite(dirPin, HIGH);
  while(digitalRead(limitmotor) == HIGH && door_status == true){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  // ทำงานแขนพ่น
  digitalWrite(dirPin, LOW);
  for (int R = 0; R < 1000; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  //แท่นหมุน
  analogWrite(driveDC_PWM, 150);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, HIGH);
  delay(1000);
  digitalWrite(Relay_PumpSp, HIGH);
  delay(10000);  // เวลาในการพ่นสเป
  digitalWrite(Relay_PumpSp, LOW);
  delay(500);
  //หยุดแท่นหมุน
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, LOW);
  delay(500);
  //digitalWrite(Relay_PumpWater, LOW);
  //delay(500);
  //digitalWrite(Relay_Fan, LOW);
  //delay(500);
  digitalWrite(dirPin, HIGH);
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, LOW);
  delay(100);
  digitalWrite(Relay_LED_Blue, HIGH);
  delay(1000);
}
//เมนู 3
void Small_Plate() {
  Serial.println("start_3");
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue, LOW);
  delay(1000);
  //digitalWrite(Relay_PumpWater, HIGH);
  //delay(1000);
  //digitalWrite(Relay_Fan, HIGH);
  //delay(1000);
  // set ตำแหน่งแขนกล
  digitalWrite(dirPin, HIGH); //HIGH คือเคลื่อนที่ตูดลงหัวขึ้น
  while(digitalRead(limitmotor) == HIGH && door_status == true){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  // ทำงานแขนพ่น
  digitalWrite(dirPin, LOW);//LOW คือเคลื่อนที่ตูดขึ้นหัวลง
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  analogWrite(driveDC_PWM, 150);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, HIGH);
  delay(1000);
  digitalWrite(Relay_PumpSp, HIGH);
  delay(10000);  // เวลาในการพ่นสเป
  digitalWrite(Relay_PumpSp, LOW);
  delay(500);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, LOW);
  delay(500);
  //digitalWrite(Relay_PumpWater, LOW);
  //delay(500);
  //digitalWrite(Relay_Fan, LOW);
  //delay(500);
  digitalWrite(dirPin, HIGH);
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  digitalWrite(Relay_LED_Red, LOW);
  delay(100);
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue, LOW);
  delay(1000);
}
//เมนู 4
void Big_Bowl() {
  Serial.println("start_4");
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue, LOW);
  delay(1000);
  //digitalWrite(Relay_PumpWater, HIGH);
  //delay(1000);
  //digitalWrite(Relay_Fan, HIGH);
  //delay(1000);
  // set ตำแหน่งแขนกล
  digitalWrite(dirPin, HIGH); //HIGH คือเคลื่อนที่ตูดลงหัวขึ้น
  while(digitalRead(limitmotor) == HIGH && door_status == true){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  // ทำงานแขนพ่น
  digitalWrite(dirPin, LOW);//LOW คือเคลื่อนที่ตูดขึ้นหัวลง
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  analogWrite(driveDC_PWM, 150);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, HIGH);
  delay(1000);
  digitalWrite(Relay_PumpSp, HIGH);
  delay(10000);  // เวลาในการพ่นสเป
  digitalWrite(Relay_PumpSp, LOW);
  delay(500);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, LOW);
  delay(500);
  //digitalWrite(Relay_PumpWater, LOW);
  //delay(500);
  //digitalWrite(Relay_Fan, LOW);
  //delay(500);
  digitalWrite(dirPin, HIGH);
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, LOW);
  delay(100);
  digitalWrite(Relay_LED_Blue, HIGH);
  delay(1000);
}

//เมนู 5
void Med_Bowl() {
  Serial.println("start_5");
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue, LOW);
  delay(1000);
  //digitalWrite(Relay_PumpWater, HIGH);
  //delay(1000);
  //digitalWrite(Relay_Fan, HIGH);
  //delay(1000);
  // set ตำแหน่งแขนกล
  digitalWrite(dirPin, HIGH); //HIGH คือเคลื่อนที่ตูดลงหัวขึ้น
  while(digitalRead(limitmotor) == HIGH && door_status == true){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  // ทำงานแขนพ่น
  digitalWrite(dirPin, LOW);//LOW คือเคลื่อนที่ตูดขึ้นหัวลง
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  analogWrite(driveDC_PWM, 150);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, HIGH);
  delay(1000);
  digitalWrite(Relay_PumpSp, HIGH);
  delay(10000);  // เวลาในการพ่นสเป
  digitalWrite(Relay_PumpSp, LOW);
  delay(500);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, LOW);
  delay(500);
  //digitalWrite(Relay_PumpWater, LOW);
  //delay(500);
  //digitalWrite(Relay_Fan, LOW);
  //delay(500);
  digitalWrite(dirPin, HIGH);
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, LOW);
  delay(100);
  digitalWrite(Relay_LED_Blue, HIGH);
  delay(1000);
}

//เมนู 6
void Small_Bowl() {
  Serial.println("start_6");
  digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue, LOW);
  delay(1000);
  //digitalWrite(Relay_PumpWater, HIGH);
  //delay(1000);
  //digitalWrite(Relay_Fan, HIGH);
  //delay(1000);
  // set ตำแหน่งแขนกล
  digitalWrite(dirPin, HIGH); //HIGH คือเคลื่อนที่ตูดลงหัวขึ้น
  while(digitalRead(limitmotor) == HIGH && door_status == true){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  // ทำงานแขนพ่น
  digitalWrite(dirPin, LOW);//LOW คือเคลื่อนที่ตูดขึ้นหัวลง
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  analogWrite(driveDC_PWM, 150);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, HIGH);
  delay(1000);
  digitalWrite(Relay_PumpSp, HIGH);
  delay(10000);  // เวลาในการพ่นสเป
  digitalWrite(Relay_PumpSp, LOW);
  delay(500);
  digitalWrite(driveDC_INA, LOW);
  digitalWrite(driveDC_INB, LOW);
  delay(500);
  //digitalWrite(Relay_PumpWater, LOW);
  //delay(500);
  //digitalWrite(Relay_Fan, LOW);
  //delay(500);
  digitalWrite(dirPin, HIGH);
  for (int R = 0; R < 940; R++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
   digitalWrite(Relay_LED_Red, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Green, LOW);
  delay(100);
  digitalWrite(Relay_LED_Blue, HIGH);
  delay(1000);
}

void write_i2c(int value) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(value);
  Wire.endTransmission();
}

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

void limitSwitchInterrupt() {
  //if(complet == false){
  door_status = false;
  setup_complete = false;
  when_door_opened();
  //}
}
//ทำงานตอนประตูเปิด
void when_door_opened(){
  Serial.println("Door is Opened...");
  delay(200);
  write_i2c(8);
  unsigned long currentMillis = millis();
  //if(currentMillis - previousMillis >= interval){
    //if( digitalRead(Relay_LED_Red)== HIGH ){
     digitalWrite(Relay_LED_Red, LOW);
     delay(100);
    //}
    //else{
      //digitalWrite(Relay_LED_Red, HIGH);
    //}

 // }
  digitalWrite(Relay_LED_Green, HIGH);
  delay(100);
  digitalWrite(Relay_LED_Blue,HIGH);
  delay(100);
  digitalWrite(Relay_PumpSp, LOW);
  //delay(1000);
  //digitalWrite(Relay_PumpWater, LOW);
  //delay(1000);
  //digitalWrite(Relay_Fan, LOW);
  //delay(1000);
  digitalWrite(driveDC_INA, LOW);
  //delay(1000);
  digitalWrite(driveDC_INB, LOW);
  //setupSystemExecuted = false;
  //delay(1000);
  //write_i2c(5); 
  //delay(1000);
}

///-------------------------------------------------------------------////
/// More Feature Idea is useing timer interrupt for check door status ////
///-------------------------------------------------------------------////

