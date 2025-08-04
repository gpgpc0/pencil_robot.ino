/**
   @file pencil_robot.ino
   @brief これはパラレルリンク機構で渦巻きと直線を描くスケッチです
   @author 2022B34 4B 巻渕 友慎
   @date 2025/04/08

   @note gitで構成管理しておくべきか分からない@n
   あと直線を描画するのに問題があって頭が悪い方式に変更した
*/

/*ライブラリ*/
#include <VarSpeedServo.h> //サーボの動作の最適化
#include <math.h> //計算用
/*memo
  0 700
  90 1500
  180 2300

  1600
  -900=-180?
*/
/** @def
   オプション
*/
#define INTERVAL 0.5 //線の間隔1=0.5cm 0.2=1mm
#define INITIAL 115//初期座標(115mm,115mm)
#define INITIAL_DEGREE 375 //90degree 1450 //初期角度
#define PULSE_MAX 2200 //最大パルス幅
#define PULSE_MIN 700 //最小パルス幅
#define SV1_PIN 5
#define SV2_PIN 6
#define SW_PIN 7
#define START_PIN 8
#define LENGTH 200 //腕の一辺の長さ115mm

/**二次元座標*/
typedef struct {
  double x;
  double y;
} vector2;

/**サーボ初期設定*/
unsigned int degree = 0;
VarSpeedServo sv1, sv2;
bool wire = 0;
long previousTime = 0;

/**直線非計算式用定数*/
int i1 = 1000, i2 = 1200;//100
/**
   @brief 範囲変換
   @details この関数はxの範囲をin_min～in_outからout_min～out_maxに変換してくれる関数です@n
   まんま、Arduinoのmap()をdoubleにしただけです

   @param [in] x 変換元
   @param in_min 変換前の最小値
   @param in_max 変換前の最大値
   @param out_min 変換後の最小値
   @param out_max 変換後の最大値
   @return temp 変換結果
*/
double user_map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/**
   @brief 座標計算(渦巻)
   @details この関数はアルキメデスの螺旋の式より現在の角度を@n
   アルキメデスの螺旋を描画する二次元座標を出力する関数です

   @param [in] d 現在の角度(rad)
   @param interval 線の間隔
   @param initial 初期位置
   @return v 計算結果(直交座標)
*/
vector2 archimedesVector(double d, double interval, double initial) {
  vector2 v;
  v.x = initial + interval * d * cos(d);
  v.y = initial + (1) * interval * d * sin(d);
  return v;
}
/**
   @brief 座標計算(直線)
   @details この関数は一次関数より現在の角度を@n
   斜めの直線を描画する二次元座標を出力する関数です


   @param [in] d 現在の角度(rad)
   @return v 計算結果(直交座標)
*/
vector2 wireVector(double d) {
  vector2 v;
  v.x = d;
  v.y = d;
  return v;
}
/**
   @brief 角度計算
   @details この関数は座標から@n
   パラレルリンク機構のサーボモーターの角度を計算する関数です

   @param [in] L 腕の長さ
   @param d 現在のステップ数
   @param sv サーボ番号
   @param iswire 直線モード
   @return temp サーボモーターの角度
*/
double writeSV(int L, int d, int sv, bool iswire) {
  double theta = radians(d);
  vector2 vector = (iswire == true) ? wireVector(theta) : archimedesVector(theta, INTERVAL, INITIAL);
  double arct = atan2(vector.y, vector.x);//定義域:実数
  double arcc_temp = sqrt(pow(vector.x, 2) + pow(vector.y, 2)) / (2 * L);
  double arcc = (arcc_temp <= 1) ? acos(arcc_temp) : 0; //おにぎりの原因 acosの定義域(-1~1)外に値が飛び出ることで未定義となってしまう
  double rad = (sv == 1) ? (arct - arcc) : -(arct + arcc);
  Serial.print("degree:");
  Serial.print(d);
  Serial.print("arcc:");
  Serial.print(arcc);
  Serial.print(",arct:");
  Serial.println(arct);
  return user_map(rad, -PI, PI, PULSE_MIN, PULSE_MAX);
}
/**
   @brief マイクロ秒用遅延関数
   @details この関数はマイクロ秒単位で待機できるようにした関数です

   @param [in] 待機時間
   @return is 待機時間がたっているかどうか
*/
bool newdelay(unsigned long interval) {
  unsigned long currentTime = micros();
  bool is = ((currentTime - previousTime) > interval);
  if (is)previousTime = currentTime;
  return is;
}
/**
   @brief セットアップ関数
   @details この関数はピン設定、サーボ設定、現在のモードの判別、@n
   シリアル設定、レジスタ設定、初期角度の設定を行う関数です
   @note GNDをD7につなぐと直線を描く
*/
void setup() {
  pinMode(SV1_PIN, OUTPUT);
  pinMode(SV2_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);
  sv1.attach(SV1_PIN);//, PULSE_MIN, PULSE_MAX);
  sv2.attach(SV2_PIN);//, PULSE_MIN, PULSE_MAX);
  wire = !digitalRead(SW_PIN);
  if (wire) {
    sv1.writeMicroseconds(i1);
    sv2.writeMicroseconds(i2);
  } else {
    degree = 20;
    sv1.writeMicroseconds(1450);
    sv2.writeMicroseconds(1450);
  }

  Serial.begin(9600);
  //カウンタ0 割り込み許可レジスタ
  TIMSK0 = 0;
}
/**
   @brief ループ関数
   @details この関数は連続的にサーボモーターの角度を変えて図形を描画するメインの関数です
   @note GNDをD7につなぐと直線を描く
*/
void loop() {
  while (1) {
    /*直線を描く際どうしても最初がずれてしまう問題があったが最初は上のサーボを動かさずにし、
       少したってから上のサーボを動かすとずれが解消された。
       また、直線の長さを調整するためにステップ数と長さの関係を見つける作業が必要となった。
    */
    if (((wire == 1) ? newdelay(10) : newdelay(1)) && !digitalRead(START_PIN) && ((wire == 1) ? degree <= 563 : true)) {
      wire = !digitalRead(SW_PIN);
      //      Serial.print("deg:");
      //      Serial.print(degree);
      //      Serial.print("SV1:");
      //      Serial.print(writeSV(LENGTH, degree, 1, wire));
      //      Serial.print("SV2:");
      //      Serial.println(writeSV(LENGTH, degree, 2, wire) + INITIAL_DEGREE);
      sv1.writeMicroseconds((wire == 0) ? writeSV(LENGTH, degree, 1, wire) : i1 + degree); //90degree=INITIAL_DEGREEMicroSeconds
      sv2.writeMicroseconds((wire == 0) ? writeSV(LENGTH, degree, 2, wire) + INITIAL_DEGREE : i2 + ((degree <= 40) ? degree * 0 : degree));
      degree+=1;
    }
  }
}
