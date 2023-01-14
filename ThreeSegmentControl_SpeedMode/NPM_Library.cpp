#include <Arduino.h>
#include "NPM_Library.h"

void multi_move(LinearClass object[], int array_num)
{

  double degree_acceleration = (object[0].goal_degree_speed - object[0].start_degree_speed) / LinearClass::ac_time; //角加速度[degree/µs^2]

  // 1pulseで1.8度回転する
  double tck = LinearClass::onestep_deg / object[0].start_degree_speed; //開始時の速度のパルス応答幅[ms]

  int traveled_step = 0;              //現在の移動ステップ量
  int travel_step_a_dcceleration = 0;   //加減速時に移動したステップ量
  double current_time = 0;            //加速を開始した時を0とした現在の時間[ms]
  double current_degree_speed;        //現在の角速度

  int state = 1; //状態遷移変数

  
  while (traveled_step < object[0].travel_step - 1) //指定された移動距離を超えるまで繰り返す
  {
    traveled_step++;     //移動したステップの数を更新
    current_time += tck; //現在の時間[ms]を更新


    switch(state){
      case 1:   //加速
      current_degree_speed = degree_acceleration * current_time + object[0].start_degree_speed; //角加速度に基づき，現在の時間での角速度を更新
      tck = LinearClass::onestep_deg / current_degree_speed;                        //パルス応答幅[ms]を更新
      travel_step_a_dcceleration = traveled_step;                                     //加速に要したステップ量を現在の移動量に更新
  
      /*移動距離の半分を超えたなら(加速と減速には同等のステップ数が必要)減速に移行する*/
      if(traveled_step > object[0].travel_step/2){
        state = 3; //減速に遷移
        current_time = 0;   //現在時間のリセット（そちらのほうが減速時の加速度を計算しやすい）
        object[0].goal_degree_speed = current_degree_speed; //次の処理から減速を行うので，現在の速度を最大速度とする
      }
      /*現在の時間が加速時間を超えた場合*/
      else if(current_time> LinearClass::ac_time) state = 2;  //定速に遷移

      break;
   

      case 2: //状態が定速
      
      // tckを変更しない
  
      delayMicroseconds(50);  //プログラムの実行時間が加減速時と比較し少ないため，delayで調整を行う．50はハードウェア固有の値
      /*すで移動したステップ量＋加速に要したステップ量 > 移動予定のステップ量なら*/
      if(traveled_step + travel_step_a_dcceleration > object[0].travel_step){  
        state = 3; //減速に遷移
        current_time = 0;//現在時間のリセット（そちらのほうが減速時の加速度を計算しやすい）
      }
      break;

      case 3: //減速
      current_degree_speed = object[0].goal_degree_speed - (degree_acceleration * current_time ) + object[0].start_degree_speed; //角加速度に基づき，現在の時間での角速度を更新
      tck = LinearClass::onestep_deg / current_degree_speed;   
      break; 
    }    

    /*
      Serial.print(state);
      Serial.println((int)(tck * 1000)); 
      Serial.println(traveled_step); 
    */
    multi_onepulse(object, array_num, (int)(tck * 1000)); //パルス幅(tck)を送出
 
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
    goal_travel_speed = tspeed;
    travel_step = tstep;

    pinMode(clc_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(clc_pin, HIGH);
    digitalWrite(dir_pin, HIGH);
    goal_degree_speed =  travel_to_degree_speed(goal_travel_speed);
    start_degree_speed = travel_to_degree_speed(start_travel_speed);
  }

  /*パルス幅を受け取り，動作パルスを生成する関数*/
  void LinearClass::onepulse(int width)
  {
    digitalWrite(clc_pin, LOW);
    delayMicroseconds(t_cl);
    digitalWrite(clc_pin, HIGH);
    delayMicroseconds(width - t_cl);
  }

  double LinearClass::travel_to_degree_speed(double travel_speed){
    double degree_speed = travel_speed * 360 / lead;
    return degree_speed;
  }

  /*加速する動作のパルス幅を生成する関数*/
  /*加速時間{ms]，最大移動速度[mm/ms]，最低移動速度[mm/ms]を引数とする*/
  /*(最大移動速度-最低移動速度)/加速時間=加速度 とする*/
  void LinearClass::single_move()
  {
    
  double degree_acceleration = (goal_degree_speed - start_degree_speed) / ac_time; //角加速度[degree/µs^2]

  // 1pulseで1.8度回転する
  double tck = onestep_deg / start_degree_speed; //開始時の速度のパルス応答幅[ms]

  int traveled_step = 0;              //現在の移動ステップ量
  int travel_step_a_dcceleration = 0;   //加減速時に移動したステップ量
  double current_time = 0;            //加速を開始した時を0とした現在の時間[ms]
  double current_degree_speed;        //現在の角速度

  int state = 1; //状態遷移変数

  
  while (traveled_step < travel_step - 1) //指定された移動距離を超えるまで繰り返す
  {
    traveled_step++;     //移動したステップの数を更新
    current_time += tck; //現在の時間[ms]を更新


    switch(state){
      case 1:   //加速
      current_degree_speed = degree_acceleration * current_time + start_degree_speed; //角加速度に基づき，現在の時間での角速度を更新
      tck = onestep_deg / current_degree_speed;                        //パルス応答幅[ms]を更新
      travel_step_a_dcceleration = traveled_step;                                     //加速に要したステップ量を現在の移動量に更新
  
      /*移動距離の半分を超えたなら(加速と減速には同等のステップ数が必要)減速に移行する*/
      if(traveled_step > travel_step/2){
        state = 3; //減速に遷移
        current_time = 0;   //現在時間のリセット（そちらのほうが減速時の加速度を計算しやすい）
        goal_degree_speed = current_degree_speed; //次の処理から減速を行うので，現在の速度を最大速度とする
      }
      /*現在の時間が加速時間を超えた場合*/
      else if(current_time> ac_time) state = 2;  //定速に遷移
    
      break;
   

      case 2: //状態が定速
      
      // tckを変更しない
  
      delayMicroseconds(50);  //プログラムの実行時間が加減速時と比較し少ないため，delayで調整を行う．50はハードウェア固有の値
      /*すで移動したステップ量＋加速に要したステップ量 > 移動予定のステップ量なら*/
      if(traveled_step + travel_step_a_dcceleration > travel_step){  
        state = 3; //減速に遷移
        current_time = 0;//現在時間のリセット（そちらのほうが減速時の加速度を計算しやすい）
      }
      break;

      case 3: //減速
      current_degree_speed = goal_degree_speed - (degree_acceleration * current_time ) + start_degree_speed; //角加速度に基づき，現在の時間での角速度を更新
      tck = onestep_deg / current_degree_speed;   
      break; 
    }    

    /*
      Serial.print(state);
      Serial.println((int)(tck * 1000)); 
      Serial.println(traveled_step); 
    */
    
    onepulse((int)(tck * 1000));
 
  }
 
  }
