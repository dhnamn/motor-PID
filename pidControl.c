#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct PID1 {
	int setPoint;
	double Kp;
	double Ki;
	double Kd;
	int lastError;
	int prevError;
} PID1;

typedef struct PID2 {
	int setPoint;
	double Kp;
	double Ki;
	double Kd;
	int prevError1;
	int prevError2;
	int prevError3;
} PID2;

static PID1 pid;
static PID1* ptr = &pid;

static PID2 pid2;
static PID2* ptr2 = &pid2;


void pidInit1(void);
int pidCalc1(int curPoint);
void pidInit2(void);
int pidCalc2(int curPoint);
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Pdata 1;
#define Idata 0.21;
#define Ddata 0.11;
/* USER CODE END PD */

/* USER CODE BEGIN PV */
int abs(int a) {
	if (a<0) return -a;
	return a;
}
int getCurrentSpeed(void);
void runMotor(uint32_t runTime);
void stopMotor(uint32_t stopTime);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
uint32_t period = 50;
int runTimePercent = 20;
uint32_t position = 0;
int curSpeed = 0;
int newSpeed = 0;
double hi = 1;
int speed = 24;
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	pidInit1();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint32_t runTime = runTimePercent * period / 100;
		uint32_t stopTime = period - runTime;
		for(int i=0; i<1000; i=i+period) {
			runMotor(runTime);
			stopMotor(stopTime);
		}

		curSpeed = getCurrentSpeed();
		if (curSpeed != ptr->setPoint) {
			if (curSpeed < ptr->setPoint) {
				newSpeed = curSpeed + pidCalc1(curSpeed);
			} else {
				newSpeed = curSpeed - pidCalc1(curSpeed);
			}
			if (newSpeed < 0) newSpeed = 0;
			hi =  (double)(newSpeed*1.0/curSpeed);
			runTimePercent = (int)(runTimePercent * hi);
			if (runTimePercent > 100) runTimePercent = 100;
			if (runTimePercent < 10) runTimePercent = 10;
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_5) {
		position = position + 1;
		// 1s = 2.100.000
		for (int x = 1500; x > 0; x--) {}
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
		HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	}
}

int getCurrentSpeed() {
	int s = (int)(position*1.0/32);
	position = 0;
	if (s < 1) s = 1;
	return s;
}

void runMotor(uint32_t runTime) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	HAL_Delay(runTime);
}

void stopMotor(uint32_t stopTime) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	HAL_Delay(stopTime);
}

void pidInit1() {
	ptr->Kp = Pdata;
	ptr->Ki = Idata;
	ptr->Kd = Ddata;
	ptr->lastError = 0;
	ptr->prevError = 0;
	ptr->setPoint = speed;
}

void pidInit2() {
	ptr2->Kp = Pdata;
	ptr2->Ki = Idata;
	ptr2->Kd = Ddata;
	ptr2->prevError1 = 0;
	ptr2->prevError2 = 0;
	ptr2->prevError3 = 0;
	ptr2->setPoint = 14;
}

int pidCalc1(int curPoint) {
	int pidIncVal, pidErr;
	pidErr = abs(ptr->setPoint - curPoint);
	pidIncVal = ptr->Kp * (pidErr - ptr->lastError)     						  // Kp*(Ek - Ek-1)        
         + ptr->Ki * ptr->lastError																	// + Ki*(Ek-1)	
         + ptr->Kd * (pidErr - 2*ptr->lastError + ptr->prevError);  //+ Kd*(Ek - 2Ek-1 + Ek-2)  
	ptr->prevError = ptr->lastError;
	ptr->lastError = pidErr;
	return pidIncVal;
}

int pidCalc2(int curPoint) {
	int pidIncVal, pidErr;
	pidErr = abs(ptr2->setPoint - curPoint);
	pidIncVal = ptr2->Kp * (pidErr - ptr2->prevError1)     						  // Kp*(Ek - Ek-1)        
         + ptr2->Ki * (pidErr + 1/2 * ptr2->prevError1)																	// + Ki*(1/2Ek + 1/2Ek-1)	
         + ptr->Kd * (3/2*pidErr - 7/2*ptr2->prevError1 + 5/2*ptr2->prevError2 - 1/2 * ptr2->prevError3);  //+ Kd*(3/2Ek - 7/2Ek-1 + 5/2Ek-2 - 1/2Ek-3)  
	ptr2->prevError3 = ptr2->prevError2;
	ptr2->prevError2 = ptr2->prevError1;
	ptr2->prevError1 = pidErr;
	return pidIncVal;
}
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

#ifdef  USE_FULL_ASSERT
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
