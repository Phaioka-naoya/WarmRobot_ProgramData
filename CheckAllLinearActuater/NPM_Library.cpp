#include <Arduino.h>
#include "NPM_Library.h"

void multi_acceleration_move(LinearClass object[], int array_num)
{

  unsigned long start_time;//「time」をunsigned longで変数宣言
  unsigned long finish_time;//「time」をunsigned longで変数宣言
  //  DEBUG_SERIAL.println("start");
  // 360度回転すると1mm移動するアクチュエータなので，
  double max_degree_speed = object[0].travel_speed * 360;
  double min_degree_speed = object[0].min_travel_speed * 360;

  double degree_acceleration = (max_degree_speed - min_degree_speed) / LinearClass::ac_time; //角加速度[degree/µs^2]

  // 1pulseで1.8度回転する
  double tck = LinearClass::onestep_deg / min_degree_speed; //開始時の速度のパルス応答幅[ms]

  int traveled_step = 0;              //現在の移動ステップ量
  int travel_step_a_dcceleration = 0;   //加速時に移動したステップ量
  double current_time = 0;            //加速を開始した時を0とした現在の時間[ms]
  double unitl_deceleration_time = 0; //減速を開始するまでの経過時間
  double current_degree_speed;        //現在の角速度

  int deceleration_step = 500; // traveled_step - travel_step <deceleration_stepになったら減速を開始する

  int int_min_tck;           //等速度に移行した際のパルス幅を入れる変数
  bool over_ac_time = false; //等速度運動に移行したか否かのフラグ

  int state = 1; //状態遷移変数

  while (traveled_step < object[0].travel_step - 1)
  {
  //  start_time = micros();  
    traveled_step++;     //移動したステップの数を更新
    current_time += tck; //現在の時間[ms]を更新


    switch(state){
      case 1:   //加速
      current_degree_speed = degree_acceleration * current_time + min_degree_speed; //角加速度に基づき，現在の時間での角速度を更新
      tck = LinearClass::onestep_deg / current_degree_speed;                        //パルス応答幅[ms]を更新
      travel_step_a_dcceleration = traveled_step;                                     //加速に要したステップ量を現在の移動量に更新
      
      Serial.print("accele\t");
      Serial.println((int)(tck * 1000));
      Serial.println(traveled_step);
      
      /*現在の時間が加速時間を超えた場合*/
      if(current_time> LinearClass::ac_time){ 
                
        state = 2;  //定速に遷移
      }
      break;
   

      case 2: //定速
      
      // tckを変更しない
      
      Serial.print("const\t");
      Serial.println((int)(tck * 1000)); 
      Serial.println(traveled_step); 
    delayMicroseconds(50);
      ///すで移動したステップ量＋加速に要したステップ量 > 移動予定のステップ量なら
      if(traveled_step + travel_step_a_dcceleration > object[0].travel_step){ 
        state = 3; //減速に遷移
        current_time = 0;
        max_degree_speed = current_degree_speed;
      }
      break;

      case 3: //減速
      current_degree_speed = max_degree_speed - (degree_acceleration * current_time ) + min_degree_speed; //角加速度に基づき，現在の時間での角速度を更新
      tck = LinearClass::onestep_deg / current_degree_speed; 
    
      Serial.print("deaccele\t");
      Serial.println((int)(tck * 1000)); 
      Serial.println(traveled_step);  
    
      break; 
    }      
    multi_onepulse(object, array_num, (int)(tck * 1000)); //定速のパルス幅(int_min_tck)を送出
  //  finish_time = micros(); 
  //  Serial.println(finish_time - start_time);
  }
}

  /*LinearClassを格納した配列と，その数を引数とし，各々の要素全てに動作パルスを生成する関数*/
  void multi_onepulse(LinearClass object[], int array_num, int width)
  {
    int i;
    for (i = 0; i < array_num; i++)
    {
      digitalWrite(object[i].clc_pin, LOW);
    }

    delayMicroseconds(LinearClass::t_cl);
    for (i = 0; i < array_num; i++)
    {
      digitalWrite(object[i].clc_pin, HIGH);
    }
    delayMicroseconds(width - LinearClass::t_cl);
  }

  /*コンストラクタ*/
  LinearClass::LinearClass(int d, int c, double tspeed, int tstep)
  {
    dir_pin = d;
    clc_pin = c;
    travel_speed = tspeed;
    travel_step = tstep;

    pinMode(clc_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(clc_pin, HIGH);
    digitalWrite(dir_pin, HIGH);
  }

  /*パルス幅を受け取り，動作パルスを生成する関数*/
  void LinearClass::onepulse(int width)
  {
    digitalWrite(clc_pin, LOW);
    delayMicroseconds(t_cl);
    digitalWrite(clc_pin, HIGH);
    delayMicroseconds(width - t_cl);
  }

  /*等速で動作パルスを送る関数 使えない
  void LinearClass::constant_move(int travel_speed, int travel_step) {
    double tck = 0.15; //パルス幅[ms]を算出 仮置き？？
    for (int i = 0; i < travel_step; i++) {
      onepulse((int)(tck * 1000));
    }
  }
  */

  /*加速する動作のパルス幅を生成する関数*/
  /*加速時間{ms]，最大移動速度[mm/ms]，最低移動速度[mm/ms]を引数とする*/
  /*(最大移動速度-最低移動速度)/加速時間=加速度 とする*/
  void LinearClass::acceleration_move()
  {
    // 360度回転すると1mm移動するアクチュエータなので，
    double max_degree_speed = travel_speed * 360;
    double min_degree_speed = min_travel_speed * 360;

    double degree_acceleration = (max_degree_speed - min_degree_speed) / ac_time; //角加速度[degree/µs^2]

    // 1pulseで1.8度回転する
    double tck = onestep_deg / min_degree_speed; //開始時の速度のパルス応答幅[ms]

    int traveled_step = 0;       //加速時間に移動したステップ量
    double current_time = 0;     //加速を開始した時を0とした現在の時間[ms]
    double current_degree_speed; //現在の角速度

    int int_min_tck;           //等速度に移行した際のパルス幅を入れる変数
    bool over_ac_time = false; //等速度運動に移行したか否かのフラグ

    while (1)
    {
      /*加速時間を超えていなければ加速し，超えていた場合定速のパルス幅を送出*/
      if (over_ac_time == false)
      {
        onepulse((int)(tck * 1000)); //パルス幅tck[ms]でパルスを送出
      }
      else
      {
        onepulse(int_min_tck);
      }

      traveled_step++;                                                              //移動したステップの数を更新
      current_time += tck;                                                          //現在の時間[ms]を更新
      current_degree_speed = degree_acceleration * current_time + min_degree_speed; //角加速度に基づき，現在の時間での角速度を更新

      tck = onestep_deg / current_degree_speed; //パルス応答幅[ms]を更新

      /*現在の時間が加速時間を超えた場合*/
      if (current_time > ac_time)
      {
        if (over_ac_time == false)
        {                                  //フラグがfalseの場合
          int_min_tck = (int)(tck * 1000); //現在のパルス幅で今後パルスを送れるように新しい変数に入れる
          over_ac_time = true;             //次のループサイクルで更新することがないように，フラグをtrueにする
        }
      }

      if (traveled_step > travel_step - 1)
      { //移動量が指定された移動ステップに達したなら
        break;
      }
    }
    // return traveled_step; //移動したステップ量を返す(今後のプログラムの拡張性に期待した実装)
  }
