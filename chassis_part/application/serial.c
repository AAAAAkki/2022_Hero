#include "serial.h"
#include "string.h"
#include "crc.h"
#include "usart.h"
#include "bsp_usart.h"
#include "CRC8_CRC16.h"
#include "main.h"

uint8_t rx_buf1[SEE_RX_BUF_SIZE];		//DMA接收缓冲区
uint8_t rx_buf2[SEE_RX_BUF_SIZE];	

uint8_t rx_data[SEE_RX_BUF_SIZE];
VisionData_t vision_data;
static void vision_hook1(void);
static void vision_hook2(void);
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/**
  * @brief  miniPC串口DMA初始化
  * @param  None
  * @retval None
  * @note   rx_buf 串口数据接受缓存
  */
void vision_init(void)
{
	//视觉信息初始化防止编译器未定义错误
	vision_data.status = 66;
	vision_data.pitch.value = 180.00;
	vision_data.yaw.value = 320.000;
	usart1_init(rx_buf1,rx_buf2,SEE_RX_BUF_SIZE);
}

//void USART1_IRQHandler(void)
//{
//	int nice;
//  if (huart1.Instance->SR & UART_FLAG_RXNE)
//  {
//    __HAL_UART_CLEAR_PEFLAG(&huart1);
//  }
//  else if (huart1.Instance->SR & UART_FLAG_IDLE)
//	{
//		
//   	 __HAL_UART_CLEAR_PEFLAG(&huart1);
//		__HAL_DMA_DISABLE(&hdma_usart1_rx);
//		nice = huart1.Instance->SR;
//		nice = huart1.Instance->DR;
//		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF0_4|DMA_FLAG_HTIF0_4);
//		__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_IDLE);
//		__HAL_DMA_ENABLE(&hdma_usart1_rx);
//			
//		memcpy(rx_data, rx_buf, SEE_RX_BUF_SIZE);
//		memset(rx_buf, 0, SEE_RX_BUF_SIZE);
//		
//		//Handle vision data
//		vision_hook();
//	}
//}


void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SEE_RX_BUF_SIZE - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SEE_RX_BUF_SIZE;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
						
						vision_hook1();

        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SEE_RX_BUF_SIZE - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SEE_RX_BUF_SIZE;

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
					
						vision_hook2();
        }
    }

}
void vision_hook1(void)
{				
	
			memcpy(rx_data, rx_buf1, SEE_RX_BUF_SIZE);
			memset(rx_buf1, 0, SEE_RX_BUF_SIZE);
			if(rx_data[0] == 0xA5 && verify_CRC16_check_sum(rx_data, SEE_RX_BUF_SIZE)) 
		{
			vision_data.status = rx_data[1];
			for(int i = 0; i < 4; ++i){
				vision_data.pitch.byte[i] = rx_data[2 + i];
			}
			for(int i = 0; i < 4; ++i){
				vision_data.yaw.byte[i] = rx_data[6 + i];
			}
		}
}
void vision_hook2()
{				
	
			memcpy(rx_data, rx_buf2, SEE_RX_BUF_SIZE);
			memset(rx_buf2, 0, SEE_RX_BUF_SIZE);
			if(rx_data[0] == 0xA5 && verify_CRC16_check_sum(rx_data, SEE_RX_BUF_SIZE)) 
		{
			vision_data.status = rx_data[1];
			for(int i = 0; i < 4; ++i){
				vision_data.pitch.byte[i] = rx_data[2 + i];
			}
			for(int i = 0; i < 4; ++i){
				vision_data.yaw.byte[i] = rx_data[6 + i];
			}
		}
}
void uart1_tx(uint8_t	*USART_RX_BUF ,int len)
{
			int t;
			for(t=0;t<len;t++)
			{
				HAL_UART_Transmit_DMA(&huart1, &USART_RX_BUF[t],1);         //向串口1发送数据
				while(HAL_UART_GetState(&huart1)==HAL_UART_STATE_BUSY_TX);//等待发送结束
			}
			
		//	HAL_UART_Transmit_DMA(&huart1, USART_RX_BUF[t],len);
		//while(HAL_UART_GetState(&huart1)==HAL_UART_STATE_BUSY_TX);
}

/*
void getVisionData(u8* data, VisionData_t* vision_data)
{
	
	if(data[0] == 0xA5 && Verify_CRC16_Check_Sum(data,18)) 
	{
		vision_data->status = data[1];
		for(int i = 0; i < 4; ++i){
			vision_data->pitch.byte[i] = data[2 + i];
		}
		for(int i = 0; i < 4; ++i){
			vision_data->yaw.byte[i] = data[6 + i];
		}
	}	
}
*/
