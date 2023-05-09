/* --- main.c --- */
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "gpio.h"
#include "LPUART.h"
#include "delay.h"
#include "ADC.h"
#include "DMA.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/* Private defines -----------------------------------------------------------*/
#define FFT_SIZE 4096
#define DMA_MEM_ADDR (ADC1->DR)
#define CLK_SPEED 80000000
#define SAMPLING_RATE 8192
#define CAL_FACTOR 175
#define NUM_COLS 78

#define IRQ_Enable 1
#define IRQ_Disable 0

/* Private function prototypes -----------------------------------------------*/
/* System Functions */
void SystemClock_Config(void);

/* Task Functions */
void Task1(void *argument);
void Task2(void *argument);

/* Private variables ---------------------------------------------------------*/
// Task Handlers
TaskHandle_t Task1Handler, Task2Handler;

// Global array for DMA/ADC Storage & FFT Usage
float32_t fft_inputArray[ELEMENT_COUNT] = {0};
uint16_t fft_max_freqs_indx[8] = {0};

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
	ADC1Init(IRQ_Disable);
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
	retVal = xTaskCreate(Task1, "Task1", 200 * configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &Task1Handler);
	if (retVal != pdPASS) { while(1); } // task creation failed, get stuck in while loop

	retVal = xTaskCreate(Task2, "Task2", 20* configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &Task2Handler);
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

		vTaskNotifyGiveFromISR(Task1Handler, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	//	// Clear flags
	//	DMA1->IFCR |= (DMA_IFCR_CTCIF1 | DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1);
}

// ADC IRQ Handler Function
void ADC1_2_IRQHandler(void){
	// Toggle GPIO on for timing measurement
	GPIO_DEF_A->ODR ^= (GPIO_ODR_OD7);

	// Clear flag
	ADC1->ISR |= ADC_ISR_EOC;
}

/* --- RTOS Tasks --- */
// Task 1 - explanation
void Task1(void *argument) {

	/* Initialize Variables */

	// Arrays for DMA/ADC Storage & FFT Output
	float32_t fft_outputArray[ELEMENT_COUNT / 2] = {0};
	// Used to hold FFT maximum magnitude
	float32_t fft_max_magnitude = 0;
	// Amplitude threshold for capturing data peaks
	float32_t threshold = 0;
	// Used for indexing into the max freq array
	uint8_t elementCount = 1;

	for (;;){


		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Hanning Window Implementation
		for (uint16_t i = 0; i < FFT_SIZE; i++) {
			fft_inputArray[i] *= 0.5 * (1 - arm_cos_f32(2 * PI * i / (FFT_SIZE - 1)));
		}

		// CMSIS FFT Stuff
		arm_rfft_fast_instance_f32 FFT_ARRAY_INST;
		arm_rfft_fast_init_f32(&FFT_ARRAY_INST, FFT_SIZE);
		arm_rfft_fast_f32(&FFT_ARRAY_INST, (float32_t *)fft_inputArray, (float32_t *)fft_outputArray, 0);
		arm_max_f32(&fft_outputArray[1], FFT_SIZE - 1, &fft_max_magnitude, (uint32_t *) &fft_max_freqs_indx[0]);

		// Iterate over the output array to find other major peaks
		threshold = fft_max_magnitude * 0.1; // Set a threshold of 10% of the maximum value
		elementCount = 1;
		for (uint16_t i = 1; i < FFT_SIZE - 1; i++) {
			if (fft_outputArray[i] > threshold &&
					fft_outputArray[i] > fft_outputArray[i-1] &&
					fft_outputArray[i] > fft_outputArray[i+1] &&
					i > fft_max_freqs_indx[elementCount - 1] - 300 &&
					i > fft_max_freqs_indx[elementCount - 1] + 300) {
				// Found a local maximum
				fft_max_freqs_indx[elementCount++] = i;
			}
		}

		for (uint16_t i = 0; i < (FFT_SIZE / NUM_COLS); )

		xTaskNotifyGive(Task2Handler);
	}
}

// Task 2 - explanation
void Task2(void *argument) {

	/* Initialize Variables */
	char peakColPositons[7][11];
	char peakFreqToPrint[7][11];

	for (;;){

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Line 23 is above x-axis
		for (uint8_t i = 0; i < 7; i++){
			if (fft_max_freqs_indx[i] != 0){
				sprintf(peakColPositons[i], "[23;%uH", (fft_max_freqs_indx[i] / (FFT_SIZE / NUM_COLS)));
				sprintf(peakFreqToPrint[i], "%u", fft_max_freqs_indx[i]);
			}
		}

		LPUART_Setup_SpectrumAnalyzer();
		for (uint8_t i = 0; i < 7; i++){
			LPUART_ESC_Code(peakColPositons[i]);
			LPUART_print("!");
		}
		LPUART_ESC_Code("[28;1H");
		for (uint8_t i = 0; i < 7; i++){
			LPUART_print(peakFreqToPrint[i]);
			LPUART_ESC_Code("\n\r ");
		}

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
