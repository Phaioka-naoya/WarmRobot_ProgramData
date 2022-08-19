#include <Arduino.h>
#include "NPM_Library.h"

void multi_acceleration_move(LinearClass object[], int array_num)
{
//  DEBUG_SERIAL.println("start");
  //360度回転すると1mm移動するアクチュエータなので，
  double max_degree_speed = object[0].travel_speed * 360;
  double min_degree_speed = object[0].min_travel_speed * 360;

  double degree_acceleration = (max_degree_speed - min_degree_speed) /LinearClass:: ac_time; //角加速度[degree/µs^2]

  //1pulseで1.8度回転する
  double tck = LinearClass::onestep_deg / min_degree_speed; //開始時の速度のパルス応答幅[ms]

  int traveled_step = 0; //加速時間に移動したステップ量
  double current_time = 0; //加速を開始した時を0とした現在の時間[ms]
  double current_degree_speed ;  //現在の角速度

  int int_min_tck;  //等速度に移行した際のパルス幅を入れる変数
  bool over_ac_time = false;  //等速度運動に移行したか否かのフラグ

  while (1)
  {
    /*加速時間を超えていなければ加速し，超えていた場合定速のパルス幅を送出*/
    if(over_ac_time == false){  
      multi_onepulse(object, array_num,(int)(tck * 1000)); //パルス幅tck[ms]でパルスを送出
    }else{
    multi_onepulse(object, array_num,int_min_tck);
    }

    traveled_step++;  //移動したステップの数を更新
    current_time += tck;      //現在の時間[ms]を更新
    current_degree_speed = degree_acceleration * current_time + min_degree_speed; //角加速度に基づき，現在の時間での角速度を更新


    tck = LinearClass::onestep_deg / current_degree_speed; //パルス応答幅[ms]を更新

 /*現在の時間が加速時間を超えた場合*/
    if (current_time > LinearClass::ac_time) {
      if (over_ac_time == false) {  //フラグがfalseの場合
        int_min_tck = (int)(tck * 1000);  //現在のパルス幅で今後パルスを送れるように新しい変数に入れる
        over_ac_time = true;  //次のループサイクルで更新することがないように，フラグをtrueにする
      }
    }


    if (traveled_step > object[0].travel_step - 1) { //移動量が指定された移動ステップに達したなら
    //DEBUG_SERIAL.println("finish");
     
      break;
    }

  }
}



/*LinearClassを格納した配列と，その数を引数とし，各々の要素全てに動作パルスを生成する関数*/
void multi_onepulse(LinearClass object[], int array_num , int width){
  int i;
  for(i=0; i< array_num; i++){
    digitalWrite(object[i].clc_pin, LOW);
  }
  
  delayMicroseconds(LinearClass::t_cl);
  for(i=0; i<array_num; i++){
     digitalWrite(object[i].clc_pin, HIGH); 
  }
  delayMicroseconds(width - LinearClass::t_cl);

}


/*コンストラクタ*/
LinearClass::LinearClass(int d, int c, double tspeed, int tstep) {
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
void LinearClass::onepulse(int width) {
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
  //360度回転すると1mm移動するアクチュエータなので，
  double max_degree_speed = travel_speed * 360;
  double min_degree_speed = min_travel_speed * 360;

  double degree_acceleration = (max_degree_speed - min_degree_speed) / ac_time; //角加速度[degree/µs^2]

  //1pulseで1.8度回転する
  double tck = onestep_deg / min_degree_speed; //開始時の速度のパルス応答幅[ms]

  int traveled_step = 0; //加速時間に移動したステップ量
  double current_time = 0; //加速を開始した時を0とした現在の時間[ms]
  double current_degree_speed ;  //現在の角速度

  int int_min_tck;  //等速度に移行した際のパルス幅を入れる変数
  bool over_ac_time = false;  //等速度運動に移行したか否かのフラグ

  while (1)
  {
    /*加速時間を超えていなければ加速し，超えていた場合定速のパルス幅を送出*/
    if(over_ac_time == false){  
      onepulse((int)(tck * 1000)); //パルス幅tck[ms]でパルスを送出
    }else{
    onepulse(int_min_tck);
    }

    traveled_step++;  //移動したステップの数を更新
    current_time += tck;      //現在の時間[ms]を更新
    current_degree_speed = degree_acceleration * current_time + min_degree_speed; //角加速度に基づき，現在の時間での角速度を更新


    tck = onestep_deg / current_degree_speed; //パルス応答幅[ms]を更新

 /*現在の時間が加速時間を超えた場合*/
    if (current_time > ac_time) {
      if (over_ac_time == false) {  //フラグがfalseの場合
        int_min_tck = (int)(tck * 1000);  //現在のパルス幅で今後パルスを送れるように新しい変数に入れる
        over_ac_time = true;  //次のループサイクルで更新することがないように，フラグをtrueにする
      }
    }


    if (traveled_step > travel_step - 1) { //移動量が指定された移動ステップに達したなら
      break;
    }

  }
  //return traveled_step; //移動したステップ量を返す(今後のプログラムの拡張性に期待した実装)
}
