#ifndef VISION_H
#define VISION_H

#include "main.h"
#include "protocol.h"
#include "remote_control.h"

union vision_4_byte_t
{
		float f;
		unsigned char buf[4];
};

typedef __packed struct
{
	u8 delay;
	u8 target_flag;
	fp32 x,y,z;
	
	
//	 fp32 yaw_add;
//	 fp32 pitch_add;
//	 uint8_t fire_single:1;
//	 uint8_t mode:1;
//	 uint8_t target_single:1;
} vision_data_rx_t;

typedef __packed struct
{
	
	
	 fp32 yaw_add;
	 fp32 pitch_add;	
	 uint8_t fire_single:1;
	 uint8_t mode:1;
	 uint8_t target_single:1;
} windwill_data_rx_t;



typedef __packed struct
{
	uint8_t header;
	uint8_t mode:1;
} vision_data_tx_t;

typedef __packed struct
{
	uint8_t header;
	float bullet_v;
	float angle[2];
	float coord[3][3];
} vision_data_auto_shoot_tx_t;

typedef __packed struct
{
	uint8_t flag;
	float angle[2];
} vision_data_auto_shoot_rx_t;

typedef enum
{
	WINDMILL,
	ARMOUR,
}vision_mode_t;

typedef struct 
{
	vision_data_rx_t rx[2];
	windwill_data_rx_t windwill_rx;
	uint8_t rx_buf_ind;
	uint8_t update_flag[2];
	vision_data_tx_t tx;
}vision_data_t;

typedef struct 
{
	vision_data_t vision_data;
	vision_mode_t vision_mode;
	const RC_ctrl_t *vision_rc;
}vision_control_t;

void Vision_task(void const *pvParameters);
vision_control_t * get_vision_data(void);
#endif
