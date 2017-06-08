/**
******************************************************************************
* @file    main.c 
* @author  lchaooo
* @version V1.0.0

*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cube_hal.h"

#include "osal.h"
#include "sample_service.h"
#include "role_type.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"

#include <math.h>
#include <stdio.h>

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @defgroup SampleAppThT
 *  @{
 */

/** @defgroup MAIN 
 * @{
 */

/** @defgroup MAIN_Private_Defines 
 * @{
 */ 
/* Private defines -----------------------------------------------------------*/
#define BDADDR_SIZE 6
#define FILTER_SAMPLE           5
#define PRECISION               100
#define SAMPLE_PERIOD           20
#define MAX_VALUE               4096
#define MIN_TIMEWIN				350
#define MAX_TIMEWIN				2000
#define STEPTHRESHOLD			4

//typedef enum 
//{  
//  PedometerMode_STEADY = 0,
//  PedometerMode_WALK = 1
//}PedometerMode_TypeDef;

/**
 * @}
 */

/* Private macros ------------------------------------------------------------*/

/** @defgroup MAIN_Private_Variables
 * @{
 */ 
/* Private variables ---------------------------------------------------------*/
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */   

float TEMPERATURE_Value;        /*!< Temperature Value */
SensorAxes_t ACC_Value;                  /*!< Acceleration Value */
void *ACCELERO_handle = NULL;
void *TEMPERATURE_handle = NULL;
void *GYRO_handle = NULL;
void *PRESSURE_handle = NULL;
int32_t x[FILTER_SAMPLE], y[FILTER_SAMPLE], z[FILTER_SAMPLE];
int32_t  x_max = - MAX_VALUE;
int32_t x_min = MAX_VALUE;
int32_t  y_max = - MAX_VALUE;
int32_t y_min = MAX_VALUE;
int32_t  z_max = - MAX_VALUE;
int32_t z_min = MAX_VALUE;
int32_t  x_th, y_th, z_th;
int32_t x_absmax,y_absmax,z_absmax;
int32_t  x_new, x_old, y_new, y_old, z_new, z_old;
int32_t  x_result, y_result, z_result;
int32_t d1, d2;
float temperature;
int counter;
int32_t steps;
int tempSteps = 0;
int formerSteps = -1;
//PedometerMode_TypeDef mode;
int active;
uint32_t old_time, new_time;

#ifdef SERVER_ROLE
  BLE_RoleTypeDef BLE_Role = SERVER;
#endif
#ifdef CLIENT_ROLE
  BLE_RoleTypeDef BLE_Role = CLIENT;
#endif

#ifdef THROUGHPUT_TEST
  uint8_t throughput_test = 1; /* enable the test for the estimation of the throughput */
#else
  uint8_t throughput_test = 0; /* disable the test for the estimation of the throughput */
#endif

UART_HandleTypeDef UartHandle;

extern volatile uint8_t set_connectable;
extern volatile int connected;
extern volatile uint8_t notification_enabled;

extern volatile uint8_t start_read_tx_char_handle;
extern volatile uint8_t start_read_rx_char_handle;
extern volatile uint8_t end_read_tx_char_handle;
extern volatile uint8_t end_read_rx_char_handle;

/**
 * @}
 */

/** @defgroup MAIN_Private_Function_Prototypes
 * @{
 */ 
/* Private function prototypes -----------------------------------------------*/
void User_Process(void);
static void start(void);
static void newStep(void);
static void initializeAllSensors(void);
static void enableAllSensors(void);
uint8_t intToChar(int32_t);
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );
static int32_t int32abs(int32_t);
/**
 * @}
 */

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  /* Enable USART1 clock */
  USARTx_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;
  
  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
    
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;
    
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
}

/** 
* @brief  Init the VCOM.
* @param  None
* @return None
*/
void vcom_init(void)
{
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;
  
  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

void change_absmax(void) 
{
	x_absmax = int32abs(x_max) > int32abs(x_min) ? int32abs(x_max) : int32abs(x_min);
	y_absmax = int32abs(y_max) > int32abs(y_min) ? int32abs(y_max) : int32abs(y_min);
	z_absmax = int32abs(z_max) > int32abs(z_min) ? int32abs(z_max) : int32abs(z_min);
}

/**
 * @brief  Main function to show how to use the BlueNRG STM32 expansion board 
 *         to exchange data between two STM32 Nucleo boards with their
 *         respective BlueNRG STM32 expansion boards.
 *         One board will act as Server-Peripheral and the other as 
 *         Client-Central.
 *         After connection has been established, by pressing the USER button
 *         on one board, the LD2 LED on the other one gets toggled and
 *         viceversa.
 *         The communication is done using a vendor specific profile.
 * @param  None
 * @retval None
 */
int main(void)
{ 
  uint8_t CLIENT_BDADDR[] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02};
  uint8_t SERVER_BDADDR[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  uint8_t  hwVersion;
  uint16_t fwVersion;
  
  int ret;
	
  /* STM32Cube HAL library initialization:
   *  - Configure the Flash prefetch, Flash preread and Buffer caches
   *  - Systick timer is configured by default as source of time base, but user 
   *    can eventually implement his proper time base source (a general purpose 
   *    timer for example or other time source), keeping in mind that Time base 
   *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
   *    handled in milliseconds basis.
   *  - Low Level Initialization
   */
  HAL_Init();

  /* Configure LED2 */
  BSP_LED_Init(LED2);
  
  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize the VCOM interface */
  vcom_init();
  
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();

  /* Initialize the BlueNRG HCI */
  HCI_Init();
  
  /* Reset BlueNRG hardware */
  BlueNRG_RST();
  
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /* 
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  BlueNRG_RST();
  
  PRINTF("HWver %d, FWver %d\n", hwVersion, fwVersion);
  
  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1; 
  }
  
  if(BLE_Role == CLIENT) {
    Osal_MemCpy(bdaddr, CLIENT_BDADDR, sizeof(CLIENT_BDADDR));
  } else {
    Osal_MemCpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));
  }
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret){
    PRINTF("Setting BD_ADDR failed 0x%02x.\n", ret);
  }
  
  ret = aci_gatt_init();    
  if(ret){
    PRINTF("GATT_Init failed.\n");
  }
  
  if(BLE_Role == SERVER) {
    if (bnrg_expansion_board == IDB05A1) {
      ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    }
    else {
      ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    }
  }
  else {
    if (bnrg_expansion_board == IDB05A1) {
      ret = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    }
    else {
      ret = aci_gap_init_IDB04A1(GAP_CENTRAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    }
  }
  
  if(ret != BLE_STATUS_SUCCESS){
    PRINTF("GAP_Init failed.\n");
  }
    
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret == BLE_STATUS_SUCCESS) {
    PRINTF("BLE Stack Initialized.\n");
  }
  
  if(BLE_Role == SERVER) {
    PRINTF("SERVER: BLE Stack Initialized\n");
    ret = Add_Sample_Service();
    
    if(ret == BLE_STATUS_SUCCESS)
      PRINTF("Service added successfully.\n");
    else
      PRINTF("Error while adding service.\n");
    
  } else {
    PRINTF("CLIENT: BLE Stack Initialized\n");
  }
  
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);
  
	initializeAllSensors();
  enableAllSensors();
	
	active = 1; 
	//mode = PedometerMode_STEADY;
	
  while(1)
  {
    HCI_Process();
    User_Process();
  }
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send a LED toggle command to the remote board.
 * @param  None
 * @retval None
 */
void User_Process(void)
{
  if(set_connectable){
    /* Establish connection with remote device */
    Make_Connection();
    set_connectable = FALSE;
  }
	
	start();
	if(steps != formerSteps) 
	{
		unsigned char dataOut[20] = {'s','t','e','p',':',intToChar(steps/1000%10), intToChar(steps/100%10),intToChar(steps/10%10),intToChar(steps%10),',','t','e','m','p',':',intToChar(d1/10%10),intToChar(d1%10),'.',intToChar(d2)};
		//sprintf(dataOut, "steps: %d", (int)steps);
		sendData(dataOut, sizeof(dataOut));
		formerSteps = steps;
	}
	
  /* Check if the user has pushed the button */
  if(BSP_PB_GetState(BUTTON_KEY) == RESET)
  {
    while (BSP_PB_GetState(BUTTON_KEY) == RESET);
    
    if(connected && notification_enabled){
      /* Send a toggle command to the remote device */
      uint8_t data[20] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J'};
      sendData(data, sizeof(data));
      
			steps = 0;
			formerSteps = -1;
			//mode = PedometerMode_STEADY;
			
      //BSP_LED_Toggle(LED2);  // toggle the LED2 locally.
                               // If uncommented be sure BSP_LED_Init(LED2) is
                               // called in main().
                               // E.g. it can be enabled for debugging.
    }
  }
	
	HAL_Delay(20);
}

/**
 * @brief  Get the current tick value in millisecond
 * @param  None
 * @retval The tick value
 */
uint32_t user_currentTimeGetTick(void)
{
  return HAL_GetTick();
}

/**
 * @brief  Get the delta tick value in millisecond from Tick1 to the current tick
 * @param  Tick1 the reference tick used to compute the delta
 * @retval The delta tick value
 */
uint32_t user_currentTimeGetElapsedMS(uint32_t Tick1)
{
  volatile uint32_t Delta, Tick2;

  Tick2 = HAL_GetTick();

  /* Capture computation */
  Delta = Tick2 - Tick1;
  return Delta;
}

static void start()
{
	if(active) 
	{
		/*SUM FILTERING */
		BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
		x[0] = ACC_Value.AXIS_X;
    y[0] = ACC_Value.AXIS_Y;
    z[0] = ACC_Value.AXIS_Z;
		BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
		x[1] = ACC_Value.AXIS_X;
    y[1] = ACC_Value.AXIS_Y;
    z[1] = ACC_Value.AXIS_Z;
		BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
		x[2] = ACC_Value.AXIS_X;
    y[2] = ACC_Value.AXIS_Y;
    z[2] = ACC_Value.AXIS_Z;
		BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
		x[3] = ACC_Value.AXIS_X;
    y[3] = ACC_Value.AXIS_Y;
    z[3] = ACC_Value.AXIS_Z;
		x_result = (x[0] + x[1] + x[2] + x[3])/4;
    y_result = (y[0] + y[1] + y[2] + y[3])/4;
    z_result = (z[0] + z[1] + z[2] + z[3])/4;
    /*END SUM FILTERING*/
		
		/*FIND MAX AND MIN VALUE*/
    if (x_result > x_max) {x_max = x_result;change_absmax();}
		if (x_result < x_min) {x_min = x_result;change_absmax();}
		
		if (y_result > y_max) {y_max = y_result;change_absmax();}
		if (y_result < y_min) {y_min = y_result;change_absmax();}

		if (z_result > z_max) {z_max = z_result;change_absmax();}
		if (z_result < z_min) {z_min = z_result;change_absmax();}
		/*END FIND MAX AND MIN VALUE*/
		
		++counter;
		
		if(counter > 50) 
		{
			counter = 0;
			//¼ÆËããÐÖµ
			x_th = (x_max + x_min) >>1;
			y_th = (y_max + y_min) >>1;
			z_th = (z_max + z_min) >>1;
			
			/*INIT VALUE*/
			/*
			x_max = -MAX_VALUE;
			x_min = MAX_VALUE;
			y_max = -MAX_VALUE;
			y_min = MAX_VALUE;
			z_max = -MAX_VALUE;
			z_min = MAX_VALUE;
			*/
			x_max = x_result;
			x_min = x_result;
			y_max = y_result;
			y_min = y_result;
			z_max = z_result;
			z_min = z_result;
			
			BSP_TEMPERATURE_Get_Temp( TEMPERATURE_handle, &temperature );
			floatToInt( temperature, &d1, &d2, 2 );
			while(d2>10) d2 /= 10;
		}
		
		x_old = x_new;
		y_old = y_new;
		z_old = z_new;

		if (x_new - x_result > PRECISION || x_result - x_new > PRECISION) x_new = x_result;
		if (y_new - y_result > PRECISION || y_result - y_new > PRECISION) y_new = y_result;
		if (z_new - z_result > PRECISION || z_result - z_new > PRECISION) z_new = z_result;
		
		
		
		/*FIND LARGEST ACCEL AXIS*/
		if(x_absmax > y_absmax && x_absmax > z_absmax) 
		{
			if(x_old > x_th && x_new < x_th) 
			{
				newStep();
			}
		} else if (y_absmax > x_absmax && y_absmax > z_absmax)
		{
			if(y_old > y_th && y_new < y_th) 
			{
				newStep();
				
			}
		} else
		{
			if(z_old > z_th && z_new < z_th) 
			{
				newStep();
			}
		}
	}
}

static void newStep()
{
	old_time = new_time;
	new_time = HAL_GetTick();
	uint32_t time = new_time - old_time;
	
	//if(time > MAX_TIMEWIN) 
	//{
		//if(mode == PedometerMode_WALK) 
		//{
		//	mode = PedometerMode_STEADY;
		//}
		//tempSteps = 1;
	//}
	//else 
	if(time >= MIN_TIMEWIN && time <= MAX_TIMEWIN) 
	{
		//if(mode == PedometerMode_WALK) 
		//{
			++steps;
		//} 
		//else {
		//	if(tempSteps < STEPTHRESHOLD) 
		//	{
		//		tempSteps++;
		//		return;
		//	} 
		//	else {
		//		tempSteps = 0;
		//		steps = steps + STEPTHRESHOLD + 1;
		//		mode = PedometerMode_WALK;
		//	}
	//	}
	}
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors(void)
{
  /* Try to use LSM6DS3 DIL24 if present, otherwise use LSM6DS0 on board */
  BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle );
  /* Force to use HTS221 */
  BSP_TEMPERATURE_Init( HTS221_T_0, &TEMPERATURE_handle );
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors(void)
{
  BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
  BSP_TEMPERATURE_Sensor_Enable( TEMPERATURE_handle );
}

uint8_t intToChar(int32_t num)
{
	switch(num){
		case 0: return '0';
		case 1: return '1';
		case 2: return '2';
		case 3: return '3';
		case 4: return '4';
		case 5: return '5';
		case 6: return '6';
		case 7: return '7';
		case 8: return '8';
		case 9: return '9';
	}
	return '0';
}

static int32_t int32abs(int32_t num)
{
	return num >= 0 ? num : -num;
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  //printf("Error_Handler\n");
  /* Blink LED2 */
  while(1)
  {
    BSP_LED_Toggle(LED2);
    HAL_Delay(100);
  }
}

/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{

  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
 * @}
 */
 
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
