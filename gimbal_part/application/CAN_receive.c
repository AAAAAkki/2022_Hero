/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
void get_motor_measure_ecd(motor_measure_t *motor_measure, uint8_t data[8]){
		motor_measure->last_ecd = motor_measure->ecd;                          
    motor_measure->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           
    motor_measure->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     
    motor_measure->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); 
    motor_measure->temperate = (data)[6];
		if (motor_measure->ecd - motor_measure->last_ecd > 8191/2)
    {
        motor_measure->ecd_count--;
    }
    else if (motor_measure->ecd - motor_measure->last_ecd < -8191/2)
    {
        motor_measure->ecd_count++;
    }
}

#define get_cap_measure(ptr, data)                               \
  {                                                                \
		(ptr)->InputVot = ((int16_t) ((data)[1]  | (data)[0]<< 8))/100;      \
		(ptr)->CapVot = ((int16_t) ((data)[3]  | (data) [2]<< 8))/100; \
		(ptr) -> TestCurrent = ((int16_t) ((data)[5]  | (data) [4]<< 8))/100;   \
		(ptr) -> Target_Power = ((int16_t) ((data)[7]  | (data) [6]<< 8))/100;   \
	}
//		(ptr)->InputVot = ((int16_t) ((data)[1] << 8 | (data)[0]))/100;      \
		(ptr)->CapVot = ((int16_t) ((data)[3] <<8 | (data) [2]))/100; \
		(ptr) -> TestCurrent = ((int16_t) ((data)[5] <<8 | (data) [4]))/100;   \
		(ptr) -> Target_Power = ((int16_t) ((data)[7] <<8 | (data) [6]))/100;   \
  
	
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
	4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机 7:左摩擦轮电机 3508 8：右摩擦轮电机 3508 9:scope motor*/
motor_measure_t motor_chassis[10];
chassis_data_t chassis_data_receive;
//global var to receive data from chassis
static cap_measure_t cap_measure;
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef fric_tx_message;
static uint8_t fric_can_send_data[8];

static uint8_t cap_can_send_data[8];
static CAN_TxHeaderTypeDef cap_tx_measure;
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[9];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  if (hcan == &hcan2)
  {
    switch (rx_header.StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
      static uint8_t i = 0;
      //get motor id
      i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      detect_hook(YAW_GIMBAL_MOTOR_TOE);
      break;
    }
		case CAN_TRIGGER_MOTOR_ID:
    {
      static uint8_t i = 0;
      //get motor id
      i = 6;
      get_motor_measure_ecd(&motor_chassis[i], rx_data);
      detect_hook(TRIGGER_MOTOR_TOE);
      break;
    }
		
		//merge heat and limit data
		//detect REFEREE_TOE
		case heat_data_id:
		{
			chassis_data_receive.shooter_heat0=(uint16_t)(rx_data[0]<<8|rx_data[1]);
			chassis_data_receive.shooter_heat0_limit=(uint16_t)(rx_data[2]<<8|rx_data[3]);
			chassis_data_receive.chassis_power_limit=(uint16_t)(rx_data[4]<<8|rx_data[5]);
			detect_hook(REFEREE_TOE);
			break;
		}
		case shoot_data_id:
		{
			chassis_data_receive.bullet_type=rx_data[0];
			chassis_data_receive.bullet_freq=rx_data[1];
			chassis_data_receive.bullet_speed=(float)(rx_data[2]<<8|rx_data[3])/100;
			break;
		}
    default:
    {
      break;
    }
    }
  }
  if (hcan == &hcan1)
  {
    switch (rx_header.StdId)
    {
		 
    case CAN_FRIC_LIFT_MOTOR_ID:
    {
      static uint8_t i = 0;
      //get motor id
      i = 7;
      get_motor_measure(&motor_chassis[i], rx_data);
      detect_hook(FRIC_LEFT_MOTOR_TOE);
      break;
    }
    case CAN_FRIC_RIGHT_MOTOR_ID:
    {
      static uint8_t i = 0;
      //get motor id
      i = 8;
      get_motor_measure(&motor_chassis[i], rx_data);
      detect_hook(FRIC_RIGHT_MOTOR_TOE);
      break;
    }
    case CAN_PIT_MOTOR_ID:
    {
      static uint8_t i = 0;
      //get motor id
      i = 5;
      get_motor_measure(&motor_chassis[i], rx_data);
      detect_hook(PITCH_GIMBAL_MOTOR_TOE);
      break;
    }
		case gimbal_scope_motor_id:
		{
      get_motor_measure(&motor_chassis[9], rx_data);
      //detect_hook(CHASSIS_MOTOR1_TOE + i);
			break;
		}
    default:
    {
      break;
    }
    }
  }
}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
	gimbal_can_send_data[2] = (shoot >> 8);
  gimbal_can_send_data[3] = shoot;
  HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
  gimbal_can_send_data[0] = (pitch >> 8);
  gimbal_can_send_data[1] = pitch;

  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
/**
  * @brief          send fric current
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送摩擦轮电机电流
  * @param[in]      none
  * @retval         none
  */
void CAN_CMD_FRIC(int16_t motor1, int16_t motor2, int16_t scope)
{
  uint32_t send_mail_box;
  fric_tx_message.StdId = 0x200;
  fric_tx_message.IDE = CAN_ID_STD;
  fric_tx_message.RTR = CAN_RTR_DATA;
  fric_tx_message.DLC = 0x08;
  fric_can_send_data[0] = (motor1 >> 8);
  fric_can_send_data[1] = motor1;
  fric_can_send_data[2] = (motor2 >> 8);
  fric_can_send_data[3] = motor2;
	fric_can_send_data[4] = (scope>>8);
	fric_can_send_data[5] = scope;
  HAL_CAN_AddTxMessage(&hcan1, &fric_tx_message, fric_can_send_data, &send_mail_box);
}

void CAN_CMD_CAP(uint16_t power, uint16_t buffer)
{
	uint16_t powertx;
	powertx = power * 100;
  uint32_t send_mail_box;
  cap_tx_measure.StdId = 0x210;
  cap_tx_measure.IDE = CAN_ID_STD;
  cap_tx_measure.RTR = CAN_RTR_DATA;
  cap_tx_measure.DLC = 0x08;
//	cap_can_send_data[0] = powertx>>8;
//	cap_can_send_data[1] = powertx;
//	cap_can_send_data[2] = buffer>>8;
//	cap_can_send_data[3] = buffer;
	cap_can_send_data[0] = buffer>>8;
	cap_can_send_data[1] = buffer;
  HAL_CAN_AddTxMessage(&hcan2, &cap_tx_measure, cap_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis[5];
}

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis[6];
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}
/**
  * @brief          return the fric 3508 motor data point
  * @param[in]      i: motor number,range [1,2]
  * @retval         motor data point
  */
/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[1,2]
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric_motor_measure_point(uint8_t i)
{
  return &motor_chassis[i + 6];
}

/*Addtional functions*/
const motor_measure_t *get_scope_gimbal_motor_measure_point(void)
{
    return &motor_chassis[9];
}

static CAN_TxHeaderTypeDef  chassis_board_tx_message;
uint8_t              chassis_board_can_send_data[8]={0};


void CAN_chassis_transfer(int16_t vx_set, int16_t vy_set, int16_t wz_set, uint16_t chassis_mode){
		uint32_t send_mail_box;
		//int16_t temp[4];
    chassis_board_tx_message.StdId = chassis_motor_cmd_id;
    chassis_board_tx_message.IDE = CAN_ID_STD;
    chassis_board_tx_message.RTR = CAN_RTR_DATA;
    chassis_board_tx_message.DLC = 0x08;
    chassis_board_can_send_data[0]=vx_set>>8;
		chassis_board_can_send_data[1]=vx_set;
		chassis_board_can_send_data[2]=vy_set>>8;
		chassis_board_can_send_data[3]=vy_set;
		chassis_board_can_send_data[4]=wz_set>>8;
		chassis_board_can_send_data[5]=wz_set;
		chassis_board_can_send_data[6]=chassis_mode>>8;
		chassis_board_can_send_data[7]=chassis_mode;
    HAL_CAN_AddTxMessage(&hcan2, &chassis_board_tx_message, chassis_board_can_send_data, &send_mail_box);
}


static CAN_TxHeaderTypeDef  ui_info_tx_message;
uint8_t              ui_info_can_send_data[8]={0};


void send_gimbal_motor_state(uint8_t shoot_mode, fp32 pitch_angel){
		uint32_t send_mail_box;
		int16_t pitch = (int16_t)(pitch_angel*100);
    chassis_board_tx_message.StdId = ui_info_id;
    chassis_board_tx_message.IDE = CAN_ID_STD;
    chassis_board_tx_message.RTR = CAN_RTR_DATA;
    chassis_board_tx_message.DLC = 0x08;
    ui_info_can_send_data[0]=shoot_mode>>8;
		ui_info_can_send_data[1]=shoot_mode;
		ui_info_can_send_data[2]=pitch>>8;
		ui_info_can_send_data[3]=pitch;
//		ui_info_can_send_data[4]=wz_set>>8;
//		ui_info_can_send_data[5]=wz_set;
		
    HAL_CAN_AddTxMessage(&hcan2, &chassis_board_tx_message, ui_info_can_send_data, &send_mail_box);
}


