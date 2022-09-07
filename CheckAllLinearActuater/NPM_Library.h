#include <Arduino.h>
#include <Dynamixel2Arduino.h>


class LinearClass {


  public:
    //複数種類のモータの利用は想定していないため，モータ固有の値は静的メンバ変数で定義
    static  int t_cl;  //CLKパルスのLo/Hiレベル応答幅[µs]
    static  double onestep_deg;  //1ステップ（パルス）あたりの移動角度[degree]
    static  int lead;  //ねじリード．ネジが1回転したときに進む距離[mm]

    //ここも，モータ毎で異なるとうざい気がしたので，統一
    static double start_travel_speed;  //開始速度[mm/ms]
    static int ac_time;  //加速時間[ms]
        
    /*コンストラクタで定義される各インスタンスの変数*/
    int dir_pin;              //ボードの方向ピン
    int clc_pin;              //ボードのクロックピン
    int travel_step;          //移動予定のステップ量
    double goal_travel_speed;      //移動速度[mm/ms]
    double goal_degree_speed;  //最大角速度
    double start_degree_speed;  //最小角速度

    LinearClass(int d, int c, double tspeed, int tstep); //コンストラクタ

    void onepulse(int width); /*パルス幅を受け取り，動作パルスを生成する関数*/       
    void single_move();/*加速する動作のパルス幅を生成する関数*/
    double travel_to_degree_speed(double travel_speed); /*移動速度を元に，角速度を返す関数*/



};

void multi_onepulse(LinearClass object[], int array_num , int width);
void multi_move(LinearClass object[], int array_num);

