/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* NOTE: Please check if your DYNAMIXEL supports Current-based Position Control mode 
* Supported Products : MX-64(2.0), MX-106(2.0), All X series(except XL-320, XL430-W250)
*/

#include <Dynamixel2Arduino.h>
#include <Servo.h>      // Servoライブラリの読み込

// Please modify it to suit your hardware.
//シリアル通信のパラメータ
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 
//ロボットのパラメータ
#define LEG_WAIT_TIME 700
#define LINEAR_WAIT_TIME 700
#define CLOSE_ANGLE 2200 //機構上の最大角度（閉脚時の角度2200
#define OPEN_ANGLE 1024  //機構上の最小角度（開脚時の角度)
#define POSITION_PGAIN 800
#define MIN_LENGTH  180 //リニアアクチュエータの最小伸展時（サーボモータの角度）
#define MAX_LENGTH  0 //リニアアクチュエータの最大伸展時（サーボモータの角度）
#define LIMIT_CURRENT  200 //モータの電流値がこの値に制限される（壁との接触時に指定したトルクで押しつけ可能）

const uint8_t FIRST_SEGMENT_SHADOW_ID = 1;
const uint8_t SECOND_SEGMENT_SHADOW_ID = 4;

int32_t Acceleration_Limit = 50; //最大化加速度が右の値に制限　Acceleration [rpm²] = Value * 214.577
int32_t Velocity_Limit = 250;     //最大速度が右の値に制限　Velocity [rpm] = Value * 0.229 [rpm]


const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
Servo first_servo, second_servo;

int state;  //状態遷移を示す

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 115200bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(115200);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(1);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(254);
  dxl.writeControlTableItem(SHUTDOWN, 254, 52);
  dxl.setOperatingMode(254, OP_CURRENT_BASED_POSITION);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, 254, Acceleration_Limit);    //最大化加速度が右の値に制限　Acceleration [rpm²] = Value * 214.577
  dxl.writeControlTableItem(PROFILE_VELOCITY, 254, Velocity_Limit);       //最大速度が右の値に制限　Velocity [rpm] = Value * 0.229 [rpm]
  dxl.torqueOn(254);
  dxl.setGoalPosition(254, CLOSE_ANGLE);
  dxl.setGoalCurrent(254, LIMIT_CURRENT );

  first_servo.attach(5);  //1体節目のサーボモータのpin番号を指定
  second_servo.attach(6);  //1体節目のサーボモータのpin番号を指定
  first_servo.write(MIN_LENGTH);   //1体節目のリニアアクチュエータ収縮
  second_servo.write(MIN_LENGTH);  //2体節目のリニアアクチュエータ収縮
  state = 1;
  delay(2000);
}

void loop() {

  switch (state) {
    case 1: //1体節目開脚
      dxl.setGoalPosition(FIRST_SEGMENT_SHADOW_ID, OPEN_ANGLE);
      DEBUG_SERIAL.println(1);
      delay(LEG_WAIT_TIME);
      state = 2;
      break;
    case 2: //2体節目閉脚
      dxl.setGoalPosition(SECOND_SEGMENT_SHADOW_ID, CLOSE_ANGLE);
      DEBUG_SERIAL.println(2);
      delay(LEG_WAIT_TIME);
      state = 3;
      break;
    case 3: //1,2体節目のリニアアクチュエータ伸展

      first_servo.write(MAX_LENGTH);   //1体節目のリニアアクチュエータ伸展
      second_servo.write(MAX_LENGTH);  //2体節目のリニアアクチュエータ伸展
      DEBUG_SERIAL.println(3);
      delay(LINEAR_WAIT_TIME);
      state = 4;
      break;

    case 4: //2体節目開脚
      dxl.setGoalPosition(SECOND_SEGMENT_SHADOW_ID, OPEN_ANGLE);
      DEBUG_SERIAL.println(4);
      delay(LEG_WAIT_TIME);
      state = 5;
      break;
    case 5: //1体節目閉脚
      dxl.setGoalPosition(FIRST_SEGMENT_SHADOW_ID, CLOSE_ANGLE);
      DEBUG_SERIAL.println(5);
      delay(LEG_WAIT_TIME);
      state = 6;
      break;

    case 6: //1,2体節目のリニアアクチュエータ収縮
      first_servo.write(MIN_LENGTH);   //1体節目のリニアアクチュエータ収縮
      second_servo.write(MIN_LENGTH);  //2体節目のリニアアクチュエータ収縮
      Serial.println(6);
      delay(LINEAR_WAIT_TIME);
      state = 1;
      break;
   }
  
}
