/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 小车底盘主程序（X5 串口控制 + DT7 遥控兜底 + CAN1 电机指令）
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "can_bus.h"
#include "protocol.h"
#include "x5_link.h"
#include "DBUS.h"
#include "uart_rx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CTRL_PERIOD_MS      10u    /* 电机指令周期：控制器要求 5~20 ms */
#define STATUS_PERIOD_MS    100u   /* 上行状态帧周期：10 Hz */
#define RC_UNIT_MAX         660    /* DT7 摇杆满量程（±660） */
#define CMD_UNIT_MAX        1000   /* 下发给控制器的无量纲指令范围 */
#define RC_DEADZONE         20     /* 摇杆死区，防抖 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t s_src = PROTO_SRC_NONE;   /* 当前占上风的控制源，随状态帧上报 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int16_t rc_to_units(int16_t channel);
static void car_control_tick(void);
static void car_status_tick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  摇杆值 [-660,660] -> 指令单位 [-1000,1000]，并钳位
 */
static int16_t rc_to_units(int16_t channel)
{
    int32_t v = ((int32_t)channel * (int32_t)CMD_UNIT_MAX) / RC_UNIT_MAX;

    if (v > CMD_UNIT_MAX)
    {
        v = CMD_UNIT_MAX;
    }
    if (v < -CMD_UNIT_MAX)
    {
        v = -CMD_UNIT_MAX;
    }
    return (int16_t)v;
}

/**
 * @brief  10 ms 一次的仲裁 + 下发
 *
 *   s2 打到 3（硬急停）  >  串口（X5 300 ms 内有有效帧）  >  遥控器  >  停
 *
 * 串口优先而不是遥控器优先：跑建图/走圈时人是松手的，遥控器摇杆回中会一直
 * 抢控制权，反而更危险。急停留了物理开关，不依赖优先级。
 */
static void car_control_tick(void)
{
    int16_t speed = 0;
    int16_t angular = 0;
    uint8_t src = PROTO_SRC_NONE;

    if (DBUS_Estop() != 0u)
    {
        /* 硬急停：不采纳任何来源 */
        src = PROTO_SRC_NONE;
    }
    else if (X5_Link_GetCmd(&speed, &angular) != 0)
    {
        src = PROTO_SRC_SERIAL;
    }
    else if (DBUS_Alive() != 0u)
    {
        int16_t gain = (rc.s2 == 2u) ? 2 : 1;   /* s2=1 全速，s2=2 半速 */

        speed = (int16_t)(rc_to_units(rc.ch3) / gain);
        angular = (int16_t)(rc_to_units(rc.ch0) / gain);
        if ((speed < RC_DEADZONE) && (speed > -RC_DEADZONE))
        {
            speed = 0;
        }
        if ((angular < RC_DEADZONE) && (angular > -RC_DEADZONE))
        {
            angular = 0;
        }
        src = PROTO_SRC_RC;
    }
    else
    {
        /* 两个来源都没有：发 0，让控制器判"有信号但速度为 0"而不是信号丢失 */
        src = PROTO_SRC_NONE;
    }

    s_src = src;
    CAN_Send_MIX(angular, speed);
}

/**
 * @brief  100 ms 一次的上行状态帧（X5 侧解析成 /car/status）
 */
static void car_status_tick(void)
{
    uint8_t status = 0u;

    if (CAN_Bus_Okay() != 0u)
    {
        status |= PROTO_ST_CAN_OK;
    }
    if (CAN_Bus_AckOkay() != 0u)
    {
        status |= PROTO_ST_CAN_ACK;
    }
    if (X5_Link_Alive() != 0u)
    {
        status |= PROTO_ST_LINK_OK;
    }
    if (DBUS_Alive() != 0u)
    {
        status |= PROTO_ST_RC_OK;
    }
    if (DBUS_Estop() != 0u)
    {
        status |= PROTO_ST_ESTOP;
    }

    X5_Link_SendStatus(status, s_src);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t now = 0u;
  uint32_t last_ctrl = 0u;
  uint32_t last_status = 0u;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /*
   * 电机链路：片内 CAN1 (PA11/PA12) -> SN65HVD230 -> 控制器。
   * USART3 只是备用/调试口，不参与电机控制。
   */

  /* 三路接收中断（全工程唯一的 HAL_UART_RxCpltCallback 在 uart_rx.c） */
  UART_Rx_Start();

  /* 链路与遥控初始化 */
  X5_Link_Init(&huart1);
  DBUS_Init();

  /* 启动先发一帧 status：让 X5 立刻知道 C8T6 活了。即使后面 CAN 初始化卡住，
     bridge_node 也能看到这一帧，便于区分“串口没通”和“CAN 初始化失败”。 */
  X5_Link_SendStatus(0u, PROTO_SRC_NONE);

  /* CAN：配置过滤器（全收，便于收 ACK）→ Start → RX 中断 */
  CAN_Bus_Init(&hcan);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* 注意：这几个变量声明在块首（C89 风格）。工程虽然开着 C99，但 ARMCC 的
     --c90 模式下"语句之后不能再声明变量"，放块首就不用赌编译选项。 */
  now = 0u;
  last_ctrl = HAL_GetTick();
  last_status = last_ctrl;

  while (1)
  {

    now = HAL_GetTick();

    if ((now - last_ctrl) >= CTRL_PERIOD_MS)
    {
      last_ctrl = now;
      car_control_tick();
    }

    if ((now - last_status) >= STATUS_PERIOD_MS)
    {
      last_status = now;
      car_status_tick();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
