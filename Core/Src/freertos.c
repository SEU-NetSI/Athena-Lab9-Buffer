/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_drv.h"
#include "spi_drv.h"
#include "uart_hal.h"
#include "tca6408a.h"
#include "vl53l5cx_api.h"
#include "test_tof.h"
#include "calibration.h"
#include "w25q64_ll.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FIFO_CAPACITY 2
#define DATA_SIZE     16
typedef struct {
    uint8_t  data[FIFO_CAPACITY][DATA_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} FifoBuffer;

static VL53L5CX_Configuration vl53l5dev_f;
static VL53L5CX_ResultsData vl53l5_res_f;
SemaphoreHandle_t txComplete = NULL;
SemaphoreHandle_t rxComplete = NULL;
SemaphoreHandle_t spiMutex = NULL;

static FifoBuffer g_fifo;
// Flash 写基地址
static uint32_t g_flash_write_addr = 0x123456;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void buffer_init(FifoBuffer *fifo);
bool buffer_push(FifoBuffer *fifo, const uint8_t *src);
void read_all_data(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	txComplete = xSemaphoreCreateBinary();
	rxComplete = xSemaphoreCreateBinary();
	spiMutex = xSemaphoreCreateMutex();

	if (txComplete == NULL || rxComplete == NULL || spiMutex == NULL)
	{
	    // 处理信号量创建失败
	    while (1);
	}

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  BSP_W25Qx_Init();
	  uint8_t ID[2]={0};
	  BSP_W25Qx_Read_ID(ID);
	  BSP_W25Qx_Erase_Block(0x123456);
//    uint8_t tx_data[10];
//	  uint8_t rx_data[10] = {0x00};
//    for(int i=0; i<10; i++)
//    {
//      tx_data[i] = i;
//    }
//	  BSP_W25Qx_Write(tx_data, 0x123456, 10);
//	  BSP_W25Qx_Read(rx_data, 0x123456, 10);
//	  BSP_W25Qx_Erase_Block(0x123456);
//	  BSP_W25Qx_Read(rx_data, 0x123456, 10);
//	  LL_mDelay(1);
	  buffer_init(&g_fifo);

	  uint8_t sample[DATA_SIZE];
	  for (int n = 0; n < 4; n++) {
	    for (int i = 0; i < DATA_SIZE; i++) {
	        sample[i] = (uint8_t)(n * DATA_SIZE + i);  // 数据内容随 n 变化
	    }
	        buffer_push(&g_fifo, sample);
	  }

	  // 打印全部数据（先 Flash 里的 2 条，再 RAM 里的 2 条）
	  read_all_data();
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void buffer_init(FifoBuffer *fifo)
{
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

// 向 FIFO 写入一条长度为 DATA_SIZE 的记录；当 FIFO 已满时，先把整个 FIFO
// 中按时间顺序整理，再一次性写入 Flash，
// 然后清空 FIFO，把当前这条新数据作为第一条写入。
bool buffer_push(FifoBuffer *fifo, const uint8_t *src)
{
	// 如果当前有效数据条数小于容量，说明 FIFO 还未满
    if (fifo->count < FIFO_CAPACITY) {
        // 缓存未满，直接写；
    	// 把 src 指向的 DATA_SIZE 字节拷贝到 FIFO 的 tail 位置（下一条写入位置）
    	// tail 前移一格，取模保证在 0..FIFO_CAPACITY-1 的环形范围内
    	// 有效数据条数 +1


        return true;
    }

    // 走到这里说明 FIFO 已经满了，需要先把 RAM 中数据按时间顺序整理到一块连续的临时缓冲里再写入 Flash
    uint8_t temp_block[FIFO_CAPACITY * DATA_SIZE];
    uint16_t idx = fifo->head;
    //依次把记录搬到 temp_block 中
    for (uint16_t i = 0; i < FIFO_CAPACITY; ++i) {

    }

    // 整块写入 Flash，同时计算下次起始地址
    BSP_W25Qx_Write(temp_block, g_flash_write_addr, FIFO_CAPACITY * DATA_SIZE);


    // 清空缓存


    // 写入新数据作为第一条


    return true;
}

// 读取全部数据：先读 Flash 历史，再读 RAM 未落盘数据
void read_all_data(void)
{
    // 1. 读 Flash 历史数据
    uint32_t bytes = g_flash_write_addr - 0x123456; // 写入的总字节数
    uint32_t blocks = bytes / (FIFO_CAPACITY * DATA_SIZE);

    for (uint32_t i = 0; i < blocks; ++i) {
    	uint8_t buf[FIFO_CAPACITY * DATA_SIZE];
    	// 从 Flash 把这一块读到 buf。

    	// 占位语句（打断点使用，忽略即可）。
    	int a = 0;
    }

    // 2. 读 RAM 中未落盘的数据
    // 取得全局 FIFO 的指针，准备访问 RAM 中还未写入 Flash 的条目。

    // 计算当前 FIFO 的“队首”索引，

    //存储读取的数据
    uint8_t buf[FIFO_CAPACITY][DATA_SIZE];
    // 遍历 RAM 中尚未落盘的所有条目（共 count 条）
    for (uint16_t i = 0; i < fifo->count; i++) {


    }
}

/* USER CODE END Application */

