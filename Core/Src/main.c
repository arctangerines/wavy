/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "fatfs.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/// Utility I wrote
#include "axl.h"
/// Networking library, never really used it...
#include "mongoose.h"
#include "stdlib.h"
#include "string.h"
/// Common functions
#include "wavy.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma location = 0x30000000
ETH_DMADescTypeDef
    DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location = 0x30000080
ETH_DMADescTypeDef
    DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined(__CC_ARM) /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef
    DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000080))) ETH_DMADescTypeDef
    DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined(__GNUC__) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((
    section(".RxDescripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((
    section(".TxDescripSection"))); /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

COM_InitTypeDef BspCOMInit;

ETH_HandleTypeDef heth;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t         defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */
void
parse_nmea(void);
int
_write(int            fd,
       unsigned char* buf,
       int            len)
{
    HAL_UART_Transmit(&hcom_uart[COM1], buf, len, HAL_MAX_DELAY);
    return len;
}

/// NOTE: The whole firmware init should include verification of sd card,
/// the fs, settings bit flags on if we are saving to sd or just transmitting
/// (should also include the adxl section etc etc
// TODO: Initialization takes into consideration other sensors
// TODO: Check frequency
// FIXME: add some led blinking showing initialization (partly implemented)

/// Thread metadata
/// The stack numbers were sort of arbitrarily chosen...
osThreadId_t         pps_thread_handle;
const osThreadAttr_t pps_thread_attributes = {
    .name       = "ppsTask",
    .stack_size = DEFAULT_STACK_SIZE * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
osThreadId_t         gps_thread_handle;
const osThreadAttr_t gps_thread_attributes = {
    .name       = "gpsTask",
    .stack_size = DEFAULT_STACK_SIZE * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
osThreadId_t         adxl_thread_handle;
const osThreadAttr_t adxl_thread_attributes = {
    .name       = "adxlTask",
    .stack_size = DEFAULT_STACK_SIZE * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
osThreadId_t         mngs_thread_handle;
const osThreadAttr_t mngs_thread_attributes = {
    .name       = "mngsTask",
    .stack_size = DEFAULT_STACK_SIZE * 16,
    .priority   = (osPriority_t)osPriorityNormal,
};
osThreadId_t         sdcard_thread_handle;
const osThreadAttr_t sdcard_thread_attributes = {
    .name = "sdcardTask",
    // FIXME: Maybe we can lower this? lol
    .stack_size = DEFAULT_STACK_SIZE * 32,
    .priority   = (osPriority_t)osPriorityNormal,
};

/// Main mongoose thread that will be associated with the handle
void
mngs_thread(void* argument)
{
    struct mg_mgr mgr;
    mg_mgr_init(&mgr);
    mg_log_set(MG_LL_DEBUG);
    while (1)
    {
        mg_mgr_poll(&mgr, 0);
    }
}
// FIXME: how should sensitivity be chosen and in what variable will it be kept
WAVYD1 struct wavyGPS wavyTimeData = {
    .format = {'\0'}, .date_str = {'\0'}, .time_str = {'\0'}};
uint8_t wavyBuf[ADXL_BUFF_SIZE_UNALIGNED] = {0};
// BUG: Test this without WAVY_D1
WAVYD1 uint8_t wavyBackBuf[ADXL_BUFF_SIZE] = {0};
uint8_t        wavyFifo[ADXL_BUFF_SIZE]    = {0};

WAVYD1 FATFS  fs;
WAVYD1 FATFS* fs_ptr;
WAVYD1 FIL    file;
WAVYD1 BYTE   work[512];

#define SDPREPW   0b00000001
#define SDREADYW  0b00000010
#define SDUNMOUNT 0b10000000
uint8_t sdflags = 0;
/// All of this function is very very messy
void
sdcard_thread(void* argument)
{
    FRESULT ecr, ew, efc, em, eum, efree;
    DWORD   freeclust;
    // ecr = f_mkfs("", FM_EXFAT, 0, work, 512);
    osThreadFlagsWait(SDINIT_THR_FLAG, osFlagsWaitAny, osWaitForever);
    char fname[16];
    memcpy(fname, wavyTimeData.time_str, 6);
    fname[6] = '_';
    memcpy(fname + 7, wavyTimeData.date_str, 6);
    fname[13] = '_';
    int m     = '0';
    fname[14] = m;
    fname[15] = '\0';
    UINT    x;
    UINT    y;
    uint8_t wMarker[32] = {'w', 'a', 'v', 'y'};
    while (1)
    {
        if (!(sdflags & SDREADYW))
        {
            osThreadFlagsWait(SDPREPW_THR_FLAG, osFlagsWaitAny, osWaitForever);
            em = f_mount(&fs, "", 1);
            if (em == FR_OK)
            {
                HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
            }
            else
                Error_Handler();
            fname[14] += 1;
            em = f_open(&file, fname, FA_CREATE_ALWAYS | FA_WRITE);

            RTC_TimeTypeDef time;
            RTC_DateTypeDef date;
            HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BCD);
            HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BCD);
            wMarker[4] = date.Year;
            wMarker[5] = date.Month;
            wMarker[6] = date.Date;
            wMarker[7] = time.Seconds;
            wMarker[8] = time.Minutes;
            wMarker[9] = time.Hours;
            // wMarker[10] = delta;
            ew = f_write(&file, wMarker, 32, &y);
            if (em != FR_OK) Error_Handler();
            sdflags |= SDREADYW;
        }
        osThreadFlagsWait(SDWRITE_THR_FLAG, osFlagsWaitAny, osWaitForever);
        ew = f_write(&file, wavyBackBuf, ADXL_BUFF_SIZE, &x);
        if ((sdflags & SDUNMOUNT))
        {
            efc     = f_close(&file);
            eum     = f_mount(NULL, "", 0);
            sdflags = 0;
            __NOP();
        }
    }
}

void
package_prev_second()
{
    struct WavyPack wavy;
    memcpy(wavy.xyz, wavyBackBuf, ADXL_BUFF_SIZE_UNALIGNED);
    __NOP();
}
uint32_t       start_time;
uint32_t       end_time;
uint32_t       tot_time;
uint8_t        pps_enabled      = 0;
WAVYD1 uint8_t gps_rx[GPSRXSIZ] = {'a'};
/// The next two functions are used to basically just get the first tick,
/// then everything is handed off to the internal timer
/// In theory PPS is the best way to keep time, however...
/// It is better to have a safer mechanism, like syncing once with pps
/// then keeping time with something that cannot be lost (the pps signal lol)
void
pps_thread(void* argument)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, gps_rx, GPSRXSIZ);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    while (1)
    {
        osThreadFlagsWait(PPS_TICK_THR_FLAG, osFlagsWaitAny, osWaitForever);
        // package_prev_second();
        // osThreadFlagsSet(gps_thread_handle, GPS_RX_THR_FLAG);
        // osDelay(1);
    }
}
uint8_t adxltoggle = 0;
uint8_t peep       = 0;
void
HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        pps_enabled = 1;
        start_time  = osKernelGetTickCount();
        HAL_TIM_Base_Start_IT(&htim5);

        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
    }
}

void
HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                           uint16_t            Size)
{
    if (huart->Instance == USART2)
    {
        // should be enabled after the first tick
        if (pps_enabled)
        {
            parse_nmea();
            HAL_UART_DeInit(&huart2);
        }
        else
        {
            // turns on the uart, this is to avoid synchronization issues with
            // huart
            HAL_UARTEx_ReceiveToIdle_IT(&huart2, gps_rx, GPSRXSIZ);
            memset(gps_rx, '\0', GPSRXSIZ);
        }
    }
    // osThreadFlagsSet(gps_thread_handle, NMEA_PROC_THR_FLAG);
}
uint32_t errs = 0;
void
HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    // BUG: nasty hal bug
    __HAL_UART_CLEAR_OREFLAG(huart);
    errs++;
    if (errs == 2)
    {
        __NOP();
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, gps_rx, GPSRXSIZ);
}

void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ADXL_DRDY_Pin)
    {
        osThreadFlagsSet(adxl_thread_handle, ADXL_DATA_THR_FLAG);
    }
    if (GPIO_Pin == BLUEBTN_Pin)
    {
        HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
        if (sdflags == 0)
        {
            sdflags |= SDPREPW;
        }
        if (sdflags & SDREADYW)
        {
            sdflags |= SDUNMOUNT;
        }
    }
}

void
parse_nmea(void)
{
    char    buf[12]        = {'\0'};
    uint8_t datetime_flags = 0x00;
    uint8_t scratch        = 0;
    uint8_t section        = 0;
    // FIXME: this should save the whole time and date string,
    // then parse that, as opposed to reading 2 then 2
    for (int idx = 0; idx < GPSRXSIZ; idx++)
    {
        if (gps_rx[idx] == '$')
        {
            idx++;
            // Not enough data
            if (idx >= 1018) break;
            // FIXME: I dont like this...
            memcpy(wavyTimeData.format, gps_rx + idx, 5);
            wavyTimeData.format[5] = '\0';
            // NOTE: GNRMC state machine, we favor GNRMC because it provides the
            // most data
            if (strcmp("GNRMC", wavyTimeData.format) == 0)
            {
                idx += 5;
                size_t jdx = 0;
                while (idx < GPSRXSIZ)
                {
                    if (gps_rx[idx] == ',')
                    {
                        idx++;
                        section++;
                        continue;
                    }
                    if (section == PARSE_TIME)
                    {
                        buf[jdx] = gps_rx[idx];
                        if (jdx == 5 && !(datetime_flags & TIME_OBTAINED))
                        {
                            memcpy(wavyTimeData.time_str, buf, 6);
                            wavyTimeData.time_str[6] = '\0';
                            char* seconds_slice      = buf + 4;
                            char* minutes_slice      = buf + 2;
                            char* hours_slice        = buf;
                            scratch = strtol(seconds_slice, NULL, 10);
                            wavyTimeData.time.Seconds = scratch;
                            seconds_slice[0]          = '\0'; // End string here

                            scratch = strtol(minutes_slice, NULL, 10);
                            wavyTimeData.time.Minutes = scratch;
                            minutes_slice[0]          = '\0';

                            scratch = strtol(hours_slice, NULL, 10);
                            wavyTimeData.time.Hours          = scratch;
                            wavyTimeData.time.SubSeconds     = 0;
                            wavyTimeData.time.SecondFraction = 0;
                            wavyTimeData.time.DayLightSaving =
                                RTC_DAYLIGHTSAVING_NONE;
                            wavyTimeData.time.StoreOperation =
                                RTC_STOREOPERATION_RESET;

                            datetime_flags |= TIME_OBTAINED;
                            // FIXME: the size should be a macro
                            memset(buf, '\0', 12);
                            jdx = 0;
                            // skip the .000 for now
                            idx += 4;
                            idx++;
                            continue;
                        }
                        jdx++;
                    }
                    else if (section == PARSE_DATE)
                    {
                        buf[jdx] = gps_rx[idx];
                        if (jdx == 5 && !(datetime_flags & DATE_OBTAINED))
                        {
                            memcpy(wavyTimeData.date_str, buf, 6);
                            wavyTimeData.date_str[6] = '\0';
                            char* year_slice         = buf + 4;
                            char* month_slice        = buf + 2;
                            char* day_slice          = buf;

                            scratch                = strtol(year_slice, NULL, 10);
                            wavyTimeData.date.Year = scratch;
                            year_slice[0]          = '\0';

                            scratch = strtol(month_slice, NULL, 16);
                            wavyTimeData.date.Month = scratch;
                            month_slice[0]          = '\0';

                            scratch                 = strtol(day_slice, NULL, 10);
                            wavyTimeData.date.Date  = scratch;
                            wavyTimeData.date.WeekDay = RTC_WEEKDAY_MONDAY;

                            datetime_flags |= DATE_OBTAINED;
                            // while (1)
                            // {
                            //     osDelay(1);
                            // }

                            jdx = 0;
                            idx++;
                            break;
                        }
                        jdx++;
                    }
                    idx++;
                }
            }
            else
            {
                // Skip and parse
                idx += 4;
            }
            if ((datetime_flags & TIME_OBTAINED) &&
                (datetime_flags & DATE_OBTAINED))
                break;
        }
    }
}

// FIXME: Add configurability for frequency of sensor, this includes the
// ADXL_BUFF_SIZE and ADXL_CHUNK_SIZE
void
adxl_thread(void* argument)
{
    HAL_NVIC_EnableIRQ(ADXL_DRDY_EXTI_IRQn);

    uint16_t buf_idx  = 0;
    uint16_t cbuf_idx = ADXL_CHUNK_SIZE;
    uint8_t  mult     = 1;
    for (int i = ADXL_BUFF_SIZE_UNALIGNED; i < ADXL_BUFF_SIZE; i++)
    {
        wavyFifo[i]    = 1;
        wavyBackBuf[i] = 1;
    }
    while (1)
    {
        // FIXME: This could actually all be implemented into 1 fifo and no
        // wavybuf
        osThreadFlagsWait(ADXL_DATA_THR_FLAG, osFlagsWaitAll, osWaitForever);
        adxl355_read_data(&hspi1, wavyBuf + buf_idx);
        // FIXME: isnt buf_idx always the same as cbuf_idx?
        // THINK: We might be able to get away with deleting cbuf_idx
        // FIXME: This whole double idx is weird
        buf_idx += 9;
        if (buf_idx >= cbuf_idx)
        {
            memcpy(wavyFifo, wavyFifo + ADXL_CHUNK_SIZE,
                   ADXL_BUFF_SIZE_UNALIGNED - ADXL_CHUNK_SIZE);
            memcpy(wavyFifo + (ADXL_BUFF_SIZE_UNALIGNED - ADXL_CHUNK_SIZE),
                   wavyBuf + (cbuf_idx - ADXL_CHUNK_SIZE), ADXL_CHUNK_SIZE);
            mult++;
            cbuf_idx = ADXL_CHUNK_SIZE * mult;
            if (cbuf_idx > ADXL_BUFF_SIZE_UNALIGNED)
            {
                cbuf_idx = ADXL_CHUNK_SIZE;
                mult     = 1;
            }
        }
        if (buf_idx == ADXL_BUFF_SIZE_UNALIGNED)
        {
            buf_idx = 0;
            HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
        }
    }
}
uint8_t time_synced = 0;
void
HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef* hrtc)
{
    __NOP();
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config(void);
static void
MPU_Config(void);
static void
MX_GPIO_Init(void);
static void
MX_DMA_Init(void);
static void
MX_SPI1_Init(void);
static void
MX_ETH_Init(void);
static void
MX_RNG_Init(void);
static void
MX_RTC_Init(void);
static void
MX_SDMMC1_SD_Init(void);
static void
MX_USART2_UART_Init(void);
static void
MX_TIM2_Init(void);
static void
MX_TIM5_Init(void);
void
StartDefaultTask(void* argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP *
/

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick.
     */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_ETH_Init();
    MX_RNG_Init();
    MX_FATFS_Init();
    MX_RTC_Init();
    MX_SDMMC1_SD_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_TIM5_Init();
    /* USER CODE BEGIN 2 */
    // HAL_UART_DeInit(&huart2);
    //  Required for interrupts
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    // HAL_UART_DeInit(&huart2);
    // This ensures the interrupt doesnt fire immediately
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    // We want to choose when to enable this
    HAL_NVIC_DisableIRQ(ADXL_DRDY_EXTI_IRQn);
    adxl355_init(&hspi1);
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

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
    defaultTaskHandle =
        osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    pps_thread_handle = osThreadNew(pps_thread, NULL, &pps_thread_attributes);
    //  gps_thread_handle  = osThreadNew(gps_thread, NULL,
    //  &gps_thread_attributes);
    adxl_thread_handle = osThreadNew(adxl_thread, NULL, &adxl_thread_attributes);
    // mngs_thread_handle   = osThreadNew(mngs_thread, NULL,
    // &mngs_thread_attributes);
    sdcard_thread_handle =
        osThreadNew(sdcard_thread, NULL, &sdcard_thread_attributes);
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

    /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity
     */
    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI48 |
                                            RCC_OSCILLATORTYPE_HSI |
                                            RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = 64;
    RCC_OscInitStruct.HSI48State          = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4;
    RCC_OscInitStruct.PLL.PLLN            = 24;
    RCC_OscInitStruct.PLL.PLLP            = 1;
    RCC_OscInitStruct.PLL.PLLQ            = 3;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN        = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void
MX_ETH_Init(void)
{

    /* USER CODE BEGIN ETH_Init 0 */

    /* USER CODE END ETH_Init 0 */

    static uint8_t MACAddr[6];

    /* USER CODE BEGIN ETH_Init 1 */

    /* USER CODE END ETH_Init 1 */
    heth.Instance            = ETH;
    MACAddr[0]               = 0x00;
    MACAddr[1]               = 0x80;
    MACAddr[2]               = 0xE1;
    MACAddr[3]               = 0x00;
    MACAddr[4]               = 0x00;
    MACAddr[5]               = 0x00;
    heth.Init.MACAddr        = &MACAddr[0];
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = DMATxDscrTab;
    heth.Init.RxDesc         = DMARxDscrTab;
    heth.Init.RxBuffLen      = 1536;

    /* USER CODE BEGIN MACADDRESS */

    /* USER CODE END MACADDRESS */

    if (HAL_ETH_Init(&heth) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ETH_Init 2 */

    /* USER CODE END ETH_Init 2 */
}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void
MX_RNG_Init(void)
{

    /* USER CODE BEGIN RNG_Init 0 */

    /* USER CODE END RNG_Init 0 */

    /* USER CODE BEGIN RNG_Init 1 */

    /* USER CODE END RNG_Init 1 */
    hrng.Instance                 = RNG;
    hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
    if (HAL_RNG_Init(&hrng) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RNG_Init 2 */

    /* USER CODE END RNG_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void
MX_RTC_Init(void)
{

    /* USER CODE BEGIN RTC_Init 0 */

    /* USER CODE END RTC_Init 0 */

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    /* USER CODE BEGIN RTC_Init 1 */

    /* USER CODE END RTC_Init 1 */

    /** Initialize RTC Only
     */
    hrtc.Instance            = RTC;
    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv   = 127;
    hrtc.Init.SynchPrediv    = 255;
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN Check_RTC_BKUP */

    /* USER CODE END Check_RTC_BKUP */

    /** Initialize RTC and set the Time and Date
     */
    sTime.Hours          = 0x0;
    sTime.Minutes        = 0x0;
    sTime.Seconds        = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month   = RTC_MONTH_JANUARY;
    sDate.Date    = 0x1;
    sDate.Year    = 0x0;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }

    /** Enable the WakeUp
     */
    if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 2048, RTC_WAKEUPCLOCK_RTCCLK_DIV16) !=
        HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */

    /* USER CODE END RTC_Init 2 */
}

/**
 * @brief SDMMC1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_SDMMC1_SD_Init(void)
{

    /* USER CODE BEGIN SDMMC1_Init 0 */

    /* USER CODE END SDMMC1_Init 0 */

    /* USER CODE BEGIN SDMMC1_Init 1 */

    /* USER CODE END SDMMC1_Init 1 */
    hsd1.Instance                 = SDMMC1;
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_1B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv            = 320;
    if (HAL_SD_Init(&hsd1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SDMMC1_Init 2 */

    /* USER CODE END SDMMC1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 0x0;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    hspi1.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    hspi1.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.TxCRCInitializationPattern =
        SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.RxCRCInitializationPattern =
        SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.MasterSSIdleness        = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi1.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi1.Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi1.Init.IOSwap                  = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef      sConfigIC     = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 4294967295;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_TIM5_Init(void)
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 19200 - 1;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 10000 - 1;
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 9600;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) !=
        HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) !=
        HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void
MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void
MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

    /*Configure GPIO pin : BLUEBTN_Pin */
    GPIO_InitStruct.Pin  = BLUEBTN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BLUEBTN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB14 */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_14;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : ADXL_DRDY_Pin */
    GPIO_InitStruct.Pin  = ADXL_DRDY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADXL_DRDY_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PD8 PD9 */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : PG3 */
    GPIO_InitStruct.Pin  = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin : ADXL_CS_SS_Pin */
    GPIO_InitStruct.Pin   = ADXL_CS_SS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ADXL_CS_SS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PE1 */
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(BLUEBTN_EXTI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(BLUEBTN_EXTI_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void
StartDefaultTask(void* argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END 5 */
}

/* MPU Configuration */

void
MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x0;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    if (htim->Instance == TIM5)
    {
        end_time = osKernelGetTickCount();
        tot_time = end_time - start_time;
        if (!time_synced)
        {
            // HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
            HAL_RTC_SetTime(&hrtc, &wavyTimeData.time, RTC_FORMAT_BCD);
            HAL_RTC_SetDate(&hrtc, &wavyTimeData.date, RTC_FORMAT_BCD);
            time_synced = 1;
            osThreadFlagsSet(sdcard_thread_handle, SDINIT_THR_FLAG);
            osThreadFlagsSet(sdcard_thread_handle, SDPREPW_THR_FLAG);
        }
        // THINK: Somewhere in here, there should be something that triggers
        //  packaging the previous seconds time data,
        //  however, it shouldnt trigger until after the first 2 seconds
        // FIXME: how do we know that this does not trigger twice??????
        // HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_SET, 0);
        // FIXME: Decide if this is the best way
        memcpy(wavyBackBuf, wavyFifo, ADXL_BUFF_SIZE_UNALIGNED);
        if (sdflags & SDPREPW)
        {
            osThreadFlagsSet(sdcard_thread_handle, SDPREPW_THR_FLAG);
            sdflags &= ~SDPREPW;
        }
        osThreadFlagsSet(sdcard_thread_handle, SDWRITE_THR_FLAG);
        // RTC_TimeTypeDef v;
        // RTC_DateTypeDef x;
        // HAL_RTC_GetTime(&hrtc, &v, RTC_FORMAT_BCD);
        // HAL_RTC_GetDate(&hrtc, &x, RTC_FORMAT_BCD);
        HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
        // end_time = osKernelGetTickCount();
        // tot_time = end_time - start_time;
        __NOP();
    }
    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
        HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
        HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
        HAL_Delay(250);
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void
assert_failed(uint8_t* file,
              uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
