#ifndef LASERL1_H
#define LASERL1_H
/*---------------------INCLUDES----------------------*/
#include "arm_math.h"

/*---------------------VARIABLES---------------------*/


const float p_0 = 1.01325f;//1标准大气压
const float T_0 = 273.15f;
const float C = 0.47f;//球的阻力系数
const float MS = 0.0032f;//17mm的质量
const float DS = 0.017f;//17mm
const float MB_N = 0.041f;//42mm不发光
const float DB_N = 0.042f;//42mm不发光
const float MB_F = 0.043f;//42mm发光
const float DB_F = 0.042f;//42mm发光

#define SERVO_ERROR 12

#define iLD_1 7
#define iLD_0 6
#define iCM 2
#define iACM 3
#define iFACM 4
#define iHALT 5
#define iINT 10

#define LASER_BUF_NUM 8
#define LASER_BUF_LEN 8
#define LASER_FIFO_BUF_NUM 32
#define LASER_FIFO_BUF_LEN 32

#define AMMOG 9.7803f //重力加速度
#define AMMO1ONE 1.0f  //1
#define AMMOHUNDRED 100.0f  //100
#define AMMOTHOUSAND 1000.0f //1000
#define AMMOLEVEL 30.0f //水平
#define AMMOPI 3.14159265358979f //Π
#define AMMO180 180.0f //180


//180
#ifndef PI
#define PI 3.14159265358979f
#endif
//90
#ifndef PIp2
#define PIp2 1.57079632679489f
#endif
//45
#ifndef PIp4
#define PIp4 0.78539816339744f
#endif
//-90
#ifndef PIp2_negative
#define PIp2_negative -1.5707963267948f
#endif
//-15
#ifndef PIp12_negative
#define PIp12_negative -0.2617993877991f
#endif
//30
#ifndef PIp6
#define PIp6 0.52359877559829f
#endif
//-30
#ifndef PIp6_negative
#define PIp6_negative -0.5235987755982f
#endif
//1 rad
#ifndef ONERAD
#define ONERAD 57.2957795130823f
#endif

/*---------------------FUNCTIONS---------------------*/
//数据结构体


typedef struct  
{
 const float p_0;
 const float T_0;
 const float C;
 const float MS;
 const float DS;
 const float MB_N;
 const float DB_N;
 const float MB_F;
 const float DB_F;
	
 float p_t;//现场气压
 float T_t;//现场温度
 float row;//空气密度
 float m;//子弹质量
 float D;//子弹直径
 float area;//面积计算
 float k;//结果
 int Choose;
}trajecyory_constant ;

typedef struct
{

//  float GL_angel;        //激光对水平角度 角度值	
  float Caluculat_angel; //解算角,炮管对水平 角度制
  float TOA;             //炮弹飞行时间

  float L1_Distance;  //激光测得的长度
  float L1_Angel;     //激光测得数据时与水平的夹角 角度值
  float L1_AmmoSpeed; //弹速
  uint8_t Server;     //舵机编号

  float GB_angel;       //枪管对水平角度 角度值
  float BL_angel;       //激光对枪管角度（中位90°） 角度值
  float Delta_B;        //实时运算 计算炮管角度与实际角度差 上正
  uint8_t L1_ACM_State; //测量是否可信
  uint8_t L1_Operation; //指令

  uint8_t E_FPS; //有效帧数
  uint8_t tE_FPS;

} L1_DATA_T;
/*此结构体主要用来配合调试使用*/
typedef struct
{
  uint16_t TotalCalcu_Numbers;      //计算次数
  uint16_t TotalCalcu_last_numbers; //上次计算次数

  uint16_t Flag_timer;
  uint16_t Beat_timer;

  uint8_t Calcu_flag;

  float precision;
  uint16_t fps;
  uint16_t Pfps;

} L1_ITERATION_T;








extern L1_DATA_T L1_Data;
extern L1_ITERATION_T L1_Iteration;

void Angel_approx(L1_DATA_T *L1_Data, L1_ITERATION_T *L1_Iteration,trajecyory_constant *constant);
void init(trajecyory_constant *constant);
float angel_change(L1_DATA_T *L1_Data);
#endif
