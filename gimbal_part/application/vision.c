#include "vision.h"

#include <arm_math.h>
#include <math.h>

#include "cmsis_os.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "kalman_filter.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "user_lib.h"
#include "referee.h"
extern uint8_t UserRxBufferFS[32];

extern uint32_t nRxLength;
extern uint8_t uRxBufIndex;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t uLastRxBufIndex;
extern gimbal_control_t gimbal_control;
extern osThreadId vision_task_handle;
vision_control_t vision_control;
uint8_t target_init_flag;
void Vision_Unpack(void);
void Vision_Init(vision_control_t *init);
void Vision_mode_switch(vision_control_t *mode_change);

static void Vision_AutoShoot_Predict(void);
static void Vision_Windmill_Predict(void);
SemaphoreHandle_t vision_rx_buf_ind_mutex;
osThreadId vision_unpack_handle;
vision_data_rx_t *cur_rx;
uint8_t *cur_update_flag;
void Vision_task(void const *pvParameters) {
    Vision_Init(&vision_control);
    target_init_flag = 0;
    vision_rx_buf_ind_mutex = xSemaphoreCreateMutex();
    osThreadDef(VISION_UNPACK_TASK, Vision_Unpack, osPriorityHigh, 0, 128);
    vision_unpack_handle = osThreadCreate(osThread(VISION_UNPACK_TASK), NULL);
    // return;
    while (1) {
        // Vision_Unpack();
        xSemaphoreTake(vision_rx_buf_ind_mutex, portMAX_DELAY);
        cur_rx = &vision_control.vision_data.rx[vision_control.vision_data.rx_buf_ind];
        cur_update_flag = &vision_control.vision_data
                               .update_flag[vision_control.vision_data.rx_buf_ind];
        xSemaphoreGive(vision_rx_buf_ind_mutex);
        Vision_mode_switch(&vision_control);
        switch (vision_control.vision_mode) {
            case ARMOUR:
                Vision_AutoShoot_Predict();
                break;
            case WINDMILL:
                Vision_Windmill_Predict();
                break;
            default:
                break;
        }
        vTaskDelay(1);
    }
}
#define QUEUE_SIZE 200

typedef struct {
    fp32 val[QUEUE_SIZE];
    short front;
} queue_f32_t;

queue_f32_t yaw_queue, pitch_queue;

void queue_init(queue_f32_t *queue) {
    memset(queue, 0, sizeof(queue_f32_t));
    queue->front = -1;
}

void queue_update(queue_f32_t *queue, fp32 val) {
    if (queue->front < QUEUE_SIZE - 1) {
        /*stack not overflow*/
        queue->front++;

    } else {
        /*go back to 0*/
        queue->front = 0;
    }
    queue->val[queue->front] = val;
}

/*no is the nth recent data*/
fp32 queue_access(queue_f32_t *queue, unsigned short no) {
    if (no <= queue->front)
        return queue->val[queue->front - no];
    else
        return queue->val[QUEUE_SIZE - (no - queue->front)];
}

typedef struct {
    fp32 x;
    fp32 y;
    fp32 dis;
    fp32 height_difference;
} target_position_t;

target_position_t target_armor;

arm_matrix_instance_f32 delay_trans_matrix, predicted_x, predicted_y, predicted_height_difference;
arm_matrix_instance_f32 coord_trans_matrix, world_coord, camera_coord, milli_trans_matrix, cur_x, cur_y, cur_height;
fp32 predicted_x_data[3], predicted_y_data[3];
fp32 predicted_horizon;
fp32 frame_pitch = 0, frame_yaw, cur_pitch_degree, cur_yaw_degree;
fp32 predicted_yaw, predicted_pitch, predicted_dis, predicted_height_difference_data[3], cur_dis;
fp32 predicted_yaw_degree, predicted_pitch_degree, cur_x_data[3], cur_y_data[3], cur_height_data[3];
fp32 world_coord_data[3], camera_coord_data[3];
KalmanFilter_t vision_kf[3];
float P_Init[9] = {
    10, 0, 0, 0, 30, 0, 0, 0, 10,
};
#define current_yaw gimbal_control.gimbal_yaw_motor.absolute_angle
#define current_pitch gimbal_control.gimbal_pitch_motor.absolute_angle
void Update_Horizon_Base_With_Shoot_Delay(fp32 speed) {}

void IMU_Pose_Update() {
    /*update current pose*/
    queue_update(&yaw_queue, current_yaw);
    queue_update(&pitch_queue, current_pitch);
}

void Sync_Pose() {
    static fp32 sin_pitch, sin_yaw, cos_pitch, cos_yaw;
    frame_pitch = queue_access(&pitch_queue, cur_rx->delay);
    frame_yaw = queue_access(&yaw_queue, cur_rx->delay);
    arm_sin_cos_f32(frame_pitch / PI * 180.0f, &sin_pitch, &cos_pitch);
    arm_sin_cos_f32(-frame_yaw / PI * 180.0f, &sin_yaw, &cos_yaw);
    /*perform coord transformation*/
    coord_trans_matrix.pData[0] = cos_yaw;
    coord_trans_matrix.pData[1] = -sin_pitch * sin_yaw;
    coord_trans_matrix.pData[2] = cos_pitch * sin_yaw;
    coord_trans_matrix.pData[3] = 0;
    coord_trans_matrix.pData[4] = cos_pitch;
    coord_trans_matrix.pData[5] = sin_pitch;
    coord_trans_matrix.pData[6] = -sin_yaw;
    coord_trans_matrix.pData[7] = -sin_pitch * cos_yaw;
    coord_trans_matrix.pData[8] = cos_pitch * cos_yaw;
    camera_coord.pData[0] = cur_rx->x;
    camera_coord.pData[1] = -cur_rx->y;
    camera_coord.pData[2] = cur_rx->z;
    Matrix_Multiply(&coord_trans_matrix, &camera_coord, &world_coord);
    target_armor.x = world_coord.pData[2];
    target_armor.y = world_coord.pData[0];
    arm_sqrt_f32(target_armor.x * target_armor.x + target_armor.y * target_armor.y,
                 &target_armor.dis);
    target_armor.height_difference = world_coord.pData[1];
}

void Observer_Matrix_Update(fp32 Ts) {
    vision_kf[0].F_data[3] = vision_kf[2].F_data[3] = vision_kf[1].F_data[3] = Ts;
    vision_kf[0].F_data[7] = vision_kf[2].F_data[7] = vision_kf[1].F_data[7] = Ts;
    vision_kf[0].F_data[6] = vision_kf[2].F_data[6] = vision_kf[1].F_data[6] = 0.5 * Ts * Ts;
}

void Initialize_Observer_With_Target() {
    vision_kf[0].xhat_data[2] = target_armor.x;
    vision_kf[1].xhat_data[2] = target_armor.y;
    vision_kf[2].xhat_data[2] = target_armor.height_difference;
}

void Update_Observer_With_Target() {
    vision_kf[0].MeasuredVector[0] = target_armor.x;
    vision_kf[1].MeasuredVector[0] = target_armor.y;
    vision_kf[2].MeasuredVector[0] = target_armor.height_difference;
    Kalman_Filter_Update(&vision_kf[0]);
    Kalman_Filter_Update(&vision_kf[1]);
    Kalman_Filter_Update(&vision_kf[2]);
}

float delay_trans_matrix_data[9] = {
    1, 0, 0, 0, 1, 0, 0, 0, 1,
};

void Future_Predict(fp32 horizon) {
    // Update prediction matrix
    delay_trans_matrix_data[3] = delay_trans_matrix_data[7] = horizon;
    delay_trans_matrix_data[6] = 0.5 * horizon * horizon;
    // Predict angle
    Matrix_Multiply(&delay_trans_matrix, &cur_x, &predicted_x);
    Matrix_Multiply(&delay_trans_matrix, &cur_y, &predicted_y);
    Matrix_Multiply(&delay_trans_matrix, &cur_height, &predicted_height_difference);
    /*predict yaw*/
    predicted_yaw = -atan2f(predicted_y_data[2], predicted_x_data[2]);
    if (predicted_yaw != predicted_yaw) {
        if (predicted_x_data[2] < 0) {
            predicted_yaw = 0.0f;
        } else
            predicted_yaw = PI;
    }
    /*predict pitch*/
    arm_sqrt_f32(predicted_x_data[2] * predicted_x_data[2] + predicted_y_data[2] * predicted_y_data[2], &predicted_dis);
    predicted_pitch = atan2f(predicted_height_difference_data[2], predicted_dis);
#ifdef DEBUG_MODE
    predicted_yaw_degree = predicted_yaw / PI * 180.0f;
    predicted_pitch_degree = predicted_pitch / PI * 180.0f;
#endif
}

void Update_Current_State() {
    Matrix_Multiply(&milli_trans_matrix, &cur_x, &cur_x);
    Matrix_Multiply(&milli_trans_matrix, &cur_y, &cur_y);
    Matrix_Multiply(&milli_trans_matrix, &cur_height, &cur_height);
    /*predict pitch*/
    arm_sqrt_f32(cur_x_data[2] * cur_x_data[2] + cur_y_data[2] * cur_y_data[2], &cur_dis);
#ifdef DEBUG_MODE
    cur_yaw_degree = -atan2f(cur_y_data[2], cur_x_data[2]) / PI * 180.0f;
    if (cur_yaw_degree != cur_yaw_degree) {
        if (predicted_x_data[2] < 0) {
            cur_yaw_degree = 0.0f;
        } else
            cur_yaw_degree = 180;
    }

    cur_pitch_degree = atan2f(cur_height_data[2], cur_dis) * 180.0f / PI;
#endif
}

void Motion_Pause() {
    gimbal_control.vision_angle_set[0] = current_pitch;
    gimbal_control.vision_angle_set[1] = current_yaw;
}

static float GIMBAL_DELAY = 0.25f;

static vision_data_auto_shoot_tx_t pkg;
static vision_data_auto_shoot_rx_t pkg_R;

void Vision_AutoShoot_Computation_Query_Callback(void) {
    memcpy(&pkg_R, UserRxBufferFS + 1, sizeof(vision_data_auto_shoot_rx_t));
}

static void Vision_AutoShoot_Computation_Query(void) {
    pkg.header = 0xF7;
    pkg.bullet_v = get_bullet_speed();
    pkg.angle[0] = current_pitch;
    pkg.angle[1] = current_yaw;
    memcpy((uint8_t *)pkg.coord[0], predicted_x_data, sizeof(predicted_x_data));
    memcpy((uint8_t *)pkg.coord[1], predicted_y_data, sizeof(predicted_y_data));
    memcpy((uint8_t *)pkg.coord[2], predicted_height_difference_data, sizeof(predicted_height_difference_data));
    CDC_Transmit_FS((uint8_t *)&pkg, sizeof(vision_data_auto_shoot_tx_t));
    vTaskPrioritySet(vision_task_handle, osPriorityHigh);
    if (ulTaskNotifyTake(pdTRUE, 1) == pdPASS) {
    }else{
			pkg_R.flag = 0;
		}
    vTaskPrioritySet(vision_task_handle, osPriorityNormal);
}

static void Vision_AutoShoot_Predict(void) {
    IMU_Pose_Update();
    /*update target information*/
    if (*cur_update_flag) {
        if (cur_rx->target_flag) {
            Sync_Pose();
            if (!target_init_flag) {
                Initialize_Observer_With_Target();
                target_init_flag = 1;
            } else {
                Observer_Matrix_Update(cur_rx->delay * 1e-3);
                Update_Observer_With_Target();
            }
            memcpy(cur_x_data, vision_kf[0].xhat_data, sizeof(float) * 3);
            memcpy(cur_y_data, vision_kf[1].xhat_data, sizeof(float) * 3);
            memcpy(cur_height_data, vision_kf[2].xhat_data, sizeof(float) * 3);
            // predicted_horizon = vision_control.vision_data.rx.delay * 1e-3;
        } else {
            // target lost,reset filter
            target_init_flag = 0;
            Kalman_Filter_Reset(&vision_kf[0]);
            Kalman_Filter_Reset(&vision_kf[1]);
            Kalman_Filter_Reset(&vision_kf[2]);
            memcpy(vision_kf[0].P_data, P_Init, sizeof(P_Init));
            memcpy(vision_kf[1].P_data, P_Init, sizeof(P_Init));
            memcpy(vision_kf[2].P_data, P_Init, sizeof(P_Init));
        }
    }
    if (cur_rx->target_flag) {
        Future_Predict(cur_rx->delay * 1e-3 + GIMBAL_DELAY);
        Vision_AutoShoot_Computation_Query();
        if (pkg_R.flag) {
            gimbal_control.vision_angle_set[0] = pkg_R.angle[0];
            gimbal_control.vision_angle_set[1] = pkg_R.angle[1];
        } else {
            gimbal_control.vision_angle_set[0] = predicted_pitch;
            gimbal_control.vision_angle_set[1] = predicted_yaw;
        }
        Update_Current_State();
    } else {
        Motion_Pause();
    }
    *cur_update_flag = 0;
}
/*blank function*/
static void Vision_Windmill_Predict(void) {}

void Vision_Unpack_Callback(void) {
    memcpy((uint8_t *)&(vision_control.vision_data.windwill_rx), UserRxBufferFS + 1, sizeof(windwill_data_rx_t));
}

void Vision_Unpack(void) {
    while (1) {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }
        vision_control.vision_data.update_flag[vision_control.vision_data.rx_buf_ind ^ 1] = 1;
        xSemaphoreTake(vision_rx_buf_ind_mutex, 1);
        vision_control.vision_data.rx_buf_ind ^= 1;
        xSemaphoreGive(vision_rx_buf_ind_mutex);
        detect_hook(VISION);
    }
}

#define dt 1e-3

float F_Init[9] = {
    1, 0, 0, dt, 1, 0, 0.5 * dt *dt, dt, 1,
};

float H_Init[3] = {
    0,
    0,
    1,
};

float B_Init[3] = {
    0,
    0,
    0,
};

float Q_Init[9] = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
float R_Init[1] = {0.5};

float Q_Init_height[9] = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};

float R_Init_height[1] = {4};

float state_min_variance[3] = {0.03, 0.005, 0.1};

float coord_trans_matrix_data[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float milli_trans_matrix_data[9] = {
    1, 0, 0, dt, 1, 0, 0.5 * dt *dt, dt, 1,
};

void Vision_Config_Send() {}

void Vision_Init(vision_control_t *init) {
    init->vision_rc = get_remote_control_point();
    init->vision_mode = ARMOUR;
    init->vision_data.rx_buf_ind = 0;
    init->vision_data.rx[init->vision_data.rx_buf_ind].target_flag = 0;
    init->vision_data.tx.mode = 0;  // armour
    init->vision_data.tx.header = 0xF1;
    Matrix_Init(&camera_coord, 3, 1, camera_coord_data);
    Matrix_Init(&coord_trans_matrix, 3, 3, coord_trans_matrix_data);
    Matrix_Init(&delay_trans_matrix, 3, 3, delay_trans_matrix_data);
    Matrix_Init(&world_coord, 3, 1, world_coord_data);
    Matrix_Init(&predicted_x, 3, 1, predicted_x_data);
    Matrix_Init(&predicted_y, 3, 1, predicted_y_data);
    Matrix_Init(&cur_x, 3, 1, cur_x_data);
    Matrix_Init(&cur_y, 3, 1, cur_y_data);
    Matrix_Init(&cur_height, 3, 1, cur_height_data);
    Matrix_Init(&predicted_height_difference, 3, 1, predicted_height_difference_data);
    Matrix_Init(&milli_trans_matrix, 3, 3, milli_trans_matrix_data);
    Kalman_Filter_Create(&vision_kf[0], 3, 0, 1);
    memcpy(vision_kf[0].F_data, F_Init, sizeof(F_Init));
    memcpy(vision_kf[0].B_data, B_Init, sizeof(B_Init));
    memcpy(vision_kf[0].H_data, H_Init, sizeof(H_Init));
    memcpy(vision_kf[0].R_data, R_Init, sizeof(R_Init));
    memcpy(vision_kf[0].Q_data, Q_Init, sizeof(Q_Init));
    memcpy(vision_kf[0].P_data, P_Init, sizeof(P_Init));
    memcpy(vision_kf[0].StateMinVariance, state_min_variance, sizeof(state_min_variance));
    Kalman_Filter_Create(&vision_kf[1], 3, 0, 1);
    memcpy(vision_kf[1].F_data, F_Init, sizeof(F_Init));
    memcpy(vision_kf[1].B_data, B_Init, sizeof(B_Init));
    memcpy(vision_kf[1].H_data, H_Init, sizeof(H_Init));
    memcpy(vision_kf[1].R_data, R_Init, sizeof(R_Init));
    memcpy(vision_kf[1].Q_data, Q_Init, sizeof(Q_Init));
    memcpy(vision_kf[1].P_data, P_Init, sizeof(P_Init));
    memcpy(vision_kf[1].StateMinVariance, state_min_variance, sizeof(state_min_variance));
    vision_kf[0].UseAutoAdjustment = vision_kf[1].UseAutoAdjustment = 0;
    Kalman_Filter_Create(&vision_kf[2], 3, 0, 1);
    memcpy(vision_kf[2].F_data, F_Init, sizeof(F_Init));
    memcpy(vision_kf[2].B_data, B_Init, sizeof(B_Init));
    memcpy(vision_kf[2].H_data, H_Init, sizeof(H_Init));
    memcpy(vision_kf[2].R_data, R_Init_height, sizeof(R_Init));
    memcpy(vision_kf[2].Q_data, Q_Init_height, sizeof(Q_Init));
    memcpy(vision_kf[2].P_data, P_Init, sizeof(P_Init));
    memcpy(vision_kf[2].StateMinVariance, state_min_variance, sizeof(state_min_variance));
    vision_kf[2].UseAutoAdjustment = 0;
}

void Vision_mode_switch(vision_control_t *mode_change) {
    static uint8_t time = 0;
    if (time) {
        time--;
    }
    if (!time) {
        if (mode_change->vision_rc->key.v & KEY_PRESSED_OFFSET_B) {
            target_init_flag = 0;
            mode_change->vision_data.tx.mode = mode_change->vision_data.tx.mode ^ 1;
            CDC_Transmit_FS((uint8_t *)&mode_change->vision_data.tx, sizeof(vision_data_tx_t));
            time = 200;
        }
    }
}

vision_control_t *get_vision_data(void) { return &vision_control; }