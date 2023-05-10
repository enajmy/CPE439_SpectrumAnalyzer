/* --- main.c --- */
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "gpio.h"
#include "LPUART.h"
#include "usart.h"
#include "delay.h"
#include "ADC.h"
#include "DMA.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/* Private defines -----------------------------------------------------------*/
#define CLK_SPEED 80000000
#define DMA_MEM_ADDR (ADC1->DR)

#define SAMPLING_RATE 8192
#define FFT_SIZE (SAMPLING_RATE / 2)
#define BIN_SIZE (FFT_SIZE / NUM_COLS)

#define CAL_FACTOR 70

#define NUM_COLS 78
#define NUM_ROWS 20

#define IRQ_Enable 1
#define IRQ_Disable 0

/* Private function prototypes -----------------------------------------------*/
/* System Functions */
void SystemClock_Config(void);

/* Task Functions */
void FFT_Task(void *argument);
void LPUARTPrint_Task(void *argument);

/* Private variables ---------------------------------------------------------*/
// Task Handlers
TaskHandle_t FFT_TaskHandler, LPUARTPrint_TaskHandler;

// Global array for DMA/ADC Storage & FFT Usage
float32_t fft_inputArray[ELEMENT_COUNT] = {0};
uint8_t scaled_amplitudes[NUM_COLS] = {0};

int main(void)
{

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();
	SysTick_Init();

	/* Configure GPIO */
	GPIOInit();

	/* Configure LPUART */
	LPUARTInit();
	LPUART_Setup_SpectrumAnalyzer();

	/* Configure ADC and DMA Timer Sampling & Memory Transfer */
	ADC1Init(IRQ_Enable);
	DMAInit(&DMA_MEM_ADDR, &fft_inputArray, IRQ_Enable);

	/* Configure timer for proper ADC sampling */
	// Enable timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	// Set TIM1 Master Mode Selection to 'Update Event Mode' on TRGO2
	TIM1->CR2 &= ~(TIM_CR2_MMS);
	TIM1->CR2 |= (TIM_CR2_MMS_1);
	// Set timer trigger value based on clock and desired sampling
	TIM1->ARR = (CLK_SPEED / SAMPLING_RATE) + CAL_FACTOR;
	// Update interrupt event
	TIM1->EGR |= TIM_EGR_UG;
	// Clear flag created by UG (& all other flags)
	TIM1->SR = 0;

	/* -- Initialize RTOS -- */
	/* Task Creation Variable */
	// Used to check proper task creation
	BaseType_t retVal;

	/* Init scheduler */
	osKernelInitialize();

	NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(4,4,0));

	/* Creation of Tasks */
	retVal = xTaskCreate(FFT_Task, "FFT_Task", 200 * configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &FFT_TaskHandler);
	if (retVal != pdPASS) { while(1); } // task creation failed, get stuck in while loop

	retVal = xTaskCreate(LPUARTPrint_Task, "LPUARTPrint_Task", 20* configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &LPUARTPrint_TaskHandler);
	if (retVal != pdPASS) { while(1); } // task creation failed, get stuck in while loop

	// Start ADC Conversions
	ADC1->CR |= ADC_CR_ADSTART;
	// Start Timer
	TIM1->CR1 |= TIM_CR1_CEN;
	// Enable DMA
	DMA1_Channel1->CCR |= (DMA_CCR_EN);

	/* Start FreeRTOS Scheduler */
	vTaskStartScheduler();


	while (1)
	{
	}
}

/* ---- Interrupt Handler Functions ---- */
// DMA IRQ Handler Function
void DMA1_Channel1_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (DMA1->ISR & DMA_ISR_TCIF1){
		// Turn off timer to prevent more conversions
		TIM1->CR1 &= ~TIM_CR1_CEN;
		// Clear flag created by UG (& all other flags)
		TIM1->SR = 0;
		// Clear flag
		DMA1->IFCR |= (DMA_IFCR_CTCIF1 | DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1);
		// Disable DMA to reset CNDTR
		DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
		// Set DMA # of data value
		DMA1_Channel1->CNDTR = ELEMENT_COUNT;

		vTaskNotifyGiveFromISR(FFT_TaskHandler, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

// ADC IRQ Handler Function
void ADC1_2_IRQHandler(void){
	// Toggle GPIO on for timing measurement
	GPIO_DEF_A->ODR ^= (GPIO_ODR_OD7);

	// Clear flag
	ADC1->ISR |= ADC_ISR_EOC;
}

/* --- RTOS Tasks --- */
// FFT Task - used to perform the FFT and calculations for printing
void FFT_Task(void *argument) {

	/* Initialize Variables */

	// Arrays for DMA/ADC Storage & FFT Output
	float32_t fft_outputArray[FFT_SIZE] = {0};
	float32_t fft_plotArray[NUM_COLS], fft_plotArrayAbs[NUM_COLS] = {0};

	for (;;){


		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		GPIO_DEF_A->ODR |= (GPIO_ODR_OD5);

		// Hanning Window Implementation
		for (uint16_t i = 0; i < FFT_SIZE; i++) {
			fft_inputArray[i] *= 0.5 * (1 - arm_cos_f32(2 * PI * i / (FFT_SIZE - 1)));
		}

		// CMSIS FFT Stuff
		arm_rfft_fast_instance_f32 FFT_ARRAY_INST;
		arm_rfft_fast_init_f32(&FFT_ARRAY_INST, FFT_SIZE);
		arm_rfft_fast_f32(&FFT_ARRAY_INST, (float32_t *)fft_inputArray, (float32_t *)fft_outputArray, 0);
		arm_abs_f32((float32_t *)fft_outputArray, (float32_t *)fft_plotArrayAbs, FFT_SIZE);

		fft_plotArrayAbs[0] = 0;

		for (uint8_t i = 0; i < (NUM_COLS); i++){
			float32_t sum = 0;
			for (uint8_t j = 0; j < (BIN_SIZE); j++) {
			      sum += fft_plotArrayAbs[(i * BIN_SIZE) + j];
			    }
			fft_plotArray[i] = (sum / BIN_SIZE);
		}

		float32_t max_amp = fft_plotArray[0];

		    // Find the maximum amplitude in the array
		for (uint8_t i = 1; i < NUM_COLS; i++) {
			if (fft_plotArray[i] > max_amp) {
				max_amp = fft_plotArray[i];
			}
		}

		// Normalize the amplitudes and generate the bar graph
		for (int j = 0; j < NUM_COLS; j++) {
			scaled_amplitudes[j] = (uint8_t)round(fft_plotArray[j] / max_amp * NUM_ROWS);

		}

		GPIO_DEF_A->ODR &= ~(GPIO_ODR_OD5);

		xTaskNotifyGive(LPUARTPrint_TaskHandler);
	}
}

// LPUART Print Task - used for printing to the LPUART
void LPUARTPrint_Task(void *argument) {

	/* Initialize Variables */
	char ColPosition[8];

	for (;;){

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		GPIO_DEF_A->ODR |= (GPIO_ODR_OD6);

		LPUART_Setup_SpectrumAnalyzer();
		LPUART_ESC_Code("[23;2H");
		for (uint8_t i = 3; i <= 80; i++){
			for (uint8_t j = 0; j < scaled_amplitudes[i - 3]; j++){
				LPUART_print("|");
				LPUART_ESC_Code("[1A\b");
			}
			sprintf(ColPosition, "[23;%uH", i);
			LPUART_ESC_Code(ColPosition);
		}

		GPIO_DEF_A->ODR &= ~(GPIO_ODR_OD6);

		// Reenable timers for ADC conversions
		TIM1->EGR |= TIM_EGR_UG;
		TIM1->CR1 |= TIM_CR1_CEN;

		// Enable DMA to after reseting CNDTR
		DMA1_Channel1->CCR |= (DMA_CCR_EN);
	}
}

/* ---- System Functions ---- */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

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
