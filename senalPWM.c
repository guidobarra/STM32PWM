#include 'main.h'
#include 'stm32f0xx_hal.h'
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
static __IO uint32_t usTiempoDesfase=0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void SysTick_Handler();// SysTick_Handler esta funcion sera llamada cada 1us
void DelayInit();
void DelayUs(uint32_t us);

//variacion de la tension de entrada
//90% ====> 3686 ====> 30us
//10% ====> 410  ====> 3us
//recta y= 0,0245x - 0,012 porcentaje de la ventana, para el PWM
//recta y= 0,00824x - 0,37 tiempo On, para el sacar el tiempo de desfasaje
int main(void)
{
 uint32_t usTiempoOn = 0;
 uint32_t usDesfase = 0;
 uint16_t adc_value = 0;
 uint8_t porVentanaOn = 0;
 //ADC
 MX_ADC_Init();
 // start conversion
 ADC_Cmd (ADC1,ENABLE);	//enable ADC1
 ADC_SoftwareStartConvCmd(ADC1, ENABLE);	// start conversion (will be endless as we are in continuous mode)

 //Delay
 DelayInit()
 
 //PWM
 HAL_Init();
 SystemClock_Config();
 MX_GPIO_Init();
 MX_TIM1_Init();
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
 while (1)
 {

	adc_value = ADC_GetConversionValue(ADC1);
	if(410<=adc_value && adc_value<=3686)
	{
		porVentanaOn = (uint8_t)(0,0245*adc_value - 0,012);
		usTiempoOn = (uint32_t)(0,00824*adc_value - 0,37);
		usDesfase = usTiempoOn/2;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, porVentanaOn);
		DelayUs(usDesfase);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, porVentanaOn);
	} else if(adc_value<=410) 
	{
		usDesfase = 3/2;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);
		DelayUs(usDesfase);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 10);
	}  else if(3686<=adc_value) 
	{
		usDesfase = 30/2;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 90);
		DelayUs(usDesfase);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 90);
	}
}





void MX_ADC_Init() 
{
	//ADC
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	// input of ADC (it doesn't seem to be needed, as default GPIO state is floating input)
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 ;		// that's ADC1 (PA1 on STM32)
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//clock for ADC (max 14MHz --> 72/6=12MHz)
	RCC_ADCCLKConfig (RCC_PCLK2_Div6);
	// enable ADC system clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// define ADC config
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	// we work in continuous sampling mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_RegularChannelConfig(ADC1,ADC_Channel_1, 1,ADC_SampleTime_28Cycles5); // define regular conversion config
	ADC_Init ( ADC1, &ADC_InitStructure);	//set config of ADC1

	// enable ADC
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1

	//	ADC calibration (optional, but recommended at power on)
	ADC_ResetCalibration(ADC1);	// Reset previous calibration
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);	// Start new calibration (ADC must be off at that time)
	while(ADC_GetCalibrationStatus(ADC1));

}

void SystemClock_Config(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct;
 RCC_ClkInitTypeDef RCC_ClkInitStruct;
 /**Initializes the CPU, AHB and APB busses clocks
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState = RCC_HSE_ON;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
 {
 _Error_Handler(__FILE__, __LINE__);
 }
 /**Initializes the CPU, AHB and APB busses clocks
 */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
 |RCC_CLOCKTYPE_PCLK1;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
 {
 _Error_Handler(__FILE__, __LINE__);
 }
 /**Configure the Systick interrupt time
 */
 HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
 /**Configure the Systick
 */
 HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
 /* SysTick_IRQn interrupt configuration */
 HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM14 init function */
static void MX_TIM1_Init(void)
{
 TIM_OC_InitTypeDef sConfigOC;
 htimInstance = TIM14;
 htimInit.Prescaler = 7;
 htimInit.CounterMode = TIM_COUNTERMODE_UP;
 htimInit.Period = 1000;
 htimInit.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htimInit.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
 {
 _Error_Handler(__FILE__, __LINE__);
 }
 if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
 {
 _Error_Handler(__FILE__, __LINE__);
 }
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = 0;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
 {
 _Error_Handler(__FILE__, __LINE__);
 }
 HAL_TIM_MspPostInit(&htim1);
}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
*/

static void MX_GPIO_Init(void)
{
 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOF_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
}

void _Error_Handler(char *file, int line)
{
 while(1)
 {
 }
}

void DelayUs(uint32_t us)
{
    usTiempoDesfase = us;
    // esperar esta que usTiempoDesfase sea 0
    while (usTiempoDesfase);
}

void DelayInit()
{
    // Actualizar el valor de SystemCoreClock
    SystemCoreClockUpdate();
    // Configuracion para el temporizador SysTick cada 1 us
    SysTick_Config(SystemCoreClock / 1000000);
}

void SysTick_Handler()
{
	if (usTiempoDesfase != 0)
    {
        usTiempoDesfase--;
    }
}