#include <Arduino.h>
#include <Dynamixel2Arduino.h>


class LinearClass {


  public:
    //複数種類のモータの利用は想定していないため，モータ固有の値は静的メンバ変数で定義
    static  int t_cl;  //CLKパルスのLo/Hiレベル応答幅[µs]
    static  double onestep_deg;  //1ステップ（パルス）あたりの移動角度[degree]

    //ここも，モータ毎で異なるとうざい気がしたので，統一
    static double min_travel_speed;  //開始速度[mm/ms]
    static int ac_time;  //加速時間[ms]
        
    /*コンストラクタで定義される各インスタンスの変数*/
    int dir_pin; //ボードの方向ピン
    int clc_pin; //ボードのクロックピン
    int travel_step;  //移動距離
    double travel_speed;//移動速度[mm/ms]

    LinearClass(int d, int c, double tspeed, int tstep); //コンストラクタ

    void onepulse(int width); /*パルス幅を受け取り，動作パルスを生成する関数*/       
    void constant_move(int travel_speed, int travel_step);/*等速で動作パルスを送る関数 */
    void acceleration_move();/*加速する動作のパルス幅を生成する関数*/



};

void Testprint();
void multi_onepulse(LinearClass object[], int array_num , int width);
void multi_acceleration_move(LinearClass object[], int array_num);
