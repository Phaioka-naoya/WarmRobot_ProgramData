/*待ち時間の設定*/
#define LINEAR_WAIT_TIME 500

/*リニアアクチュエータの初期パラメータなど*/
#include "NPM_Library.h" //ライブラリの読み込み

//ボードのピン設定
#define SEG1_CLK 22
#define SEG1_DIR 23
#define SEG2_CLK 26
#define SEG2_DIR 27
#define SEG3_CLK 30
#define SEG3_DIR 31

//モータドライバとモータの仕様に準じる
#define T_CL 10         // CLKパルスのLo/Hiレベル応答幅[µs]
#define ONESTEP_DEG 1.8 // 1ステップ（パルス）あたりの移動角度[degree]
#define LEAD 1          //ねじリード．ネジが1回転したときに進む距離[mm]

//ユーザ設定<全体節共通>
#define AC_TIME 400 //加速時間[ms]
//#define TRAVEL_SPEED 0.08      //移動速度[mm/ms]
#define TRAVEL_SPEED 0.16      //移動速度[mm/ms]
#define MIN_TRAVEL_SPEED 0.001 //開始速度[mm/ms]
#define TRAVEL_STEP 7500       //移動距離の設定

/*LinearClassのstaicデータメンバの初期化*/
int LinearClass::t_cl = T_CL;
double LinearClass::onestep_deg = ONESTEP_DEG;
double LinearClass::start_travel_speed = MIN_TRAVEL_SPEED;
int LinearClass::ac_time = AC_TIME;
int LinearClass::lead = LEAD;

/*
LinearClass Seg1_Linear(SEG1_DIR, SEG1_CLK, TRAVEL_SPEED, TRAVEL_STEP);
LinearClass Seg2_Linear(SEG2_DIR, SEG2_CLK, TRAVEL_SPEED, TRAVEL_STEP);
LinearClass Seg3_Linear(SEG3_DIR, SEG3_CLK, TRAVEL_SPEED, TRAVEL_STEP);
*/

LinearClass LinearArray[3] ={
  LinearClass(SEG1_DIR, SEG1_CLK, TRAVEL_SPEED, TRAVEL_STEP),
  LinearClass(SEG2_DIR, SEG2_CLK, TRAVEL_SPEED, TRAVEL_STEP),
  LinearClass(SEG3_DIR, SEG3_CLK, TRAVEL_SPEED, TRAVEL_STEP),
};


/*状態遷移*/
int state;

void setup()
{
  state = 1;
  Serial.begin(115200);
  Serial.println("Program START");
  delay(2000);
}

//LinearClass LinearArray[] = {Seg1_Linear, Seg2_Linear, Seg3_Linear}; // LinearClassの配列．複数のリニアアクチュエータを同時に利用する際に利用．

void loop()
{

  switch (state)
  {
  case 1: // 2体節目開脚，3体節目開脚
    Serial.println("state1");
    digitalWrite(LinearArray[0].dir_pin, LOW); //伸展方向
    digitalWrite(LinearArray[1].dir_pin, LOW); //伸展方向
    digitalWrite(LinearArray[2].dir_pin, LOW); //伸展方向
    multi_move(LinearArray, 3);
    delay(LINEAR_WAIT_TIME);
    state = 2;
    break;

  case 2:                    
    Serial.println("state2");                 // 1体節目伸展
    digitalWrite(LinearArray[0].dir_pin, HIGH); //収縮方向
    digitalWrite(LinearArray[1].dir_pin, HIGH); //収縮方向
    digitalWrite(LinearArray[2].dir_pin, HIGH); //収縮方向
    multi_move(LinearArray, 3);
    delay(LINEAR_WAIT_TIME);
    state = 3;
    break;

  case 3:
    Serial.println("state3");  
    digitalWrite(LinearArray[0].dir_pin, LOW); //伸展方向
    LinearArray[0].single_move();
    delay(LINEAR_WAIT_TIME);
    state = 4;
    break;
  
  case 4:
    Serial.println("state4");
    digitalWrite(LinearArray[0].dir_pin, HIGH); //収縮方向
    LinearArray[0].single_move();
    delay(LINEAR_WAIT_TIME);
    state = 1;
    break;
  }
}
