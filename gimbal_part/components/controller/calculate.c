
#include "arm_math.h"
#include "calculate.h"
#include "shoot.h"
//#include "remote_control.h"

//extern RC_ctrl_t rc_ctrl;                 //声明遥控器储存结构体
shoot_control_t shoot_pwm;



void init(trajecyory_constant *constant)
{
//	constant->Choose = rc_ctrl.rc.s[0];
	switch (constant->Choose)
	{
		case 1://小
		{
			constant->m = MS;
			constant->D = DS;
			break;
		}
		case 2://大-不发光
		{
			constant->m = MB_N;
			constant->D = DB_N;
			break;
		}
		case 3://大-发光
		{
			constant->m = MB_F;
			constant->D = DB_N;
			break;
		}
		default:
			break;
	}

  constant->row = 1.293f * (constant->p_t / constant->p_0) * (constant->T_0 / (constant->T_0 + constant->T_t));//空气密度计算
 	constant->area = PI * 0.25f * constant->D * constant->D;//受风面积
	constant->k = 0.5f * constant->C * constant->area * constant->row/constant->m;//计算常数k
}



void Angel_approx(L1_DATA_T *L1_Data, L1_ITERATION_T *L1_Iteration,trajecyory_constant *constant)
{ 
  
 	//常数变量化，以便arm_math库调用
	
  float AmmoK = constant->k;          //1/2CρS
  float AmmoG = AMMOG;          //重力加速度
  float AmmoM = constant->m;          //发光弹丸质量
  float AmmoMG = AmmoM * AmmoG; //发光弹丸重力
  float AmmoM2GpK2 = AmmoM * AmmoM * AmmoG / (AmmoK * AmmoK);  //m^2*g/k^2
  float AmmoMGpK = AmmoM * AmmoG / AmmoK;                      //m*g/k
  float AmmoKpM = AmmoK / AmmoM;                               //k/m
  float AmmoMpK_negative = -AmmoM / AmmoK;                     //-m/k
  float Ammo1One = 1.0f;
  float OneRad = ONERAD;
	L1_Data->L1_Angel=(((shoot_pwm.Pwm_L1-5.0f*AMMOHUNDRED)/(2.0f*AMMOTHOUSAND))*AMMO180)-AMMOLEVEL;
//  float OneRad_e = 0.0174532f;
  float precision = 0.01f;
	
	//函数运算全部采用arm_math库加速
  //theta角是待求角。alpha角是激光与水平夹角

  float Vx;
  float Vy;
  float Dx;
  float Dy;
  float C_Dy;

  float theta[2] = {PIp6_negative, PIp2};
  float theta_mid = 0.0f;

  float Sin_alpha;
  float Cos_alpha;
  float Sin_theta;
  float Cos_theta;
  //第一个分式
  float exp1;          // x/vo(cos(theta)) 
  float exp2;          // (mg/k)*(exp1) P1
  float exp3;          // exp1*VyP2
  float exp4;          // k/m*e	xp1
  float exp5;          // 1-exp4
  float exp6;          //ln(exp5)
  float exp7;          // exp6^(m*m*G/(k*k))P3
  float exp8;          //P1+P2
  float result = 0.0f; //P1+P2
  float abs_result = 0.0f;

  Sin_alpha = arm_sin_f32(L1_Data->L1_Angel);
  Cos_alpha = arm_cos_f32(L1_Data->L1_Angel);

  arm_mult_f32(&L1_Data->L1_Distance, &Cos_alpha, &Dx, 1);
  arm_mult_f32(&L1_Data->L1_Distance, &Sin_alpha, &Dy, 1);
//  arm_add_f32(&Dy, &PIT_CORRECT, &Dy, 1); //修正距离
//  L1_Data->L1_AmmoSpeed = robot_referee_status.shoot_data.bullet_speed_last;
  if (L1_Data->L1_AmmoSpeed <= 1)
  {
    L1_Data->L1_AmmoSpeed = 14.2f;
  }
  //二分法
  //当角度大于-15度时，定义域为-30 到 90度
  //当角度小于-15度时，定义域为-90 到 30度

  if (L1_Data->L1_Angel >= PIp12_negative)
  {
  } 
  else
  {
    theta[0] = PIp2_negative;
    theta[1] = PIp6;
  } //-15度时，定义域为-30 到 30度

  do
  {
    arm_mean_f32(theta, 2, &theta_mid); //确定新的运算核心

    Sin_theta = arm_sin_f32(theta_mid);
    Cos_theta = arm_cos_f32(theta_mid);
    arm_mult_f32(&L1_Data->L1_AmmoSpeed, &Cos_theta, &Vx, 1); //Vx	V0Costheat
    arm_mult_f32(&L1_Data->L1_AmmoSpeed, &Sin_theta, &Vy, 1); //Vy	V0Sintheta
    exp1 = Dx / Vx;                                           //exp1
    arm_mult_f32(&AmmoMGpK, &exp1, &exp2, 1);                 //exp2
    arm_mult_f32(&exp1, &Vy, &exp3, 1);                       //exp3
    arm_mult_f32(&AmmoKpM, &exp1, &exp4, 1);                  //exp4

    arm_sub_f32(&Ammo1One, &exp4, &exp5, 1);    //exp5
    exp6 = logf(exp5);                          //exp6
    arm_mult_f32(&AmmoM2GpK2, &exp6, &exp7, 1); //exp7
    arm_add_f32(&exp2, &exp3, &exp8, 1);        //exp8
    arm_add_f32(&exp7, &exp8, &C_Dy, 1);        //C_Dy
    L1_Iteration->TotalCalcu_Numbers++;
    arm_sub_f32(&C_Dy, &Dy, &result, 1);
    arm_abs_f32(&result, &abs_result, 1);
    if (abs_result < precision)
    {
        arm_mult_f32(&theta_mid, &OneRad, &L1_Data->Caluculat_angel, 1); //角度用°表示
        arm_mult_f32(&AmmoMpK_negative, &exp6, &L1_Data->TOA, 1);        //弹道飞行时间计算
        L1_Iteration->Pfps++;
			  shoot_pwm.Pwm_GB=(((L1_Data->Caluculat_angel+AMMOLEVEL)/AMMO180)*(2.0f*AMMOTHOUSAND))+5.0f*AMMOHUNDRED;	

      L1_Iteration->TotalCalcu_last_numbers = L1_Iteration->TotalCalcu_Numbers;
      L1_Iteration->TotalCalcu_Numbers = 0;
      return;
    }
    else if (result > 0)
    {
      theta[1] = theta_mid;
    }
    else
    {
      theta[0] = theta_mid;
    }

    L1_Iteration->TotalCalcu_Numbers++;
  } while (L1_Iteration->TotalCalcu_Numbers < 15);

  L1_Iteration->TotalCalcu_last_numbers = L1_Iteration->TotalCalcu_Numbers;
  L1_Iteration->TotalCalcu_Numbers = 0;
}










