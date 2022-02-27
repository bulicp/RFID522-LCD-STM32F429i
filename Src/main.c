/**
  ******************************************************************************
  * @file    BSP/Src/main.c 
  * @author  MCD Application Team
  * @brief   This example code shows how to use the STM32429I-Discovery BSP Drivers
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stlogo.h"
#include "MFRC522.h"
#include "RFID-tag.h"
#include "retarget.h"

//#define __READ_NEW_CARD__
//#define __INIT_NEW_CARD__
#define __ACCESS_CONTROL__
//#define __READ_UID__



/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO ITStatus 	UartRXReady = RESET;  // UART RX Callback sets this flag
__IO ITStatus 	UartTXReady = RESET;  // UART TX Callback sets this flag

UART_HandleTypeDef 	UartHandle;
SPI_HandleTypeDef SpiHandle;

static uint8_t DemoIndex = 0;
#ifdef EE_M24LR64
uint8_t NbLoop = 1;
#endif /* EE_M24LR64 */
__IO uint8_t ubKeyPressed = RESET; 



struct_block_access_bits B0;
struct_block_access_bits B1;
struct_block_access_bits B2;
struct_block_access_bits TB;
MIFARE_Key key;
MIFARE_Key new_key;
MIFARE_CLASSIC_1K_Card_t card;
UID uid;								// Used by PICC_ReadCardSerial().
char strUID[14];
bool cardPresent = false;

#define NRFIDCODES4 	13
#define NRFIDCODES16 	1

const uint8_t RFIDcodes[][4] = {
	{0xe6, 0xd4, 0x5e, 0xaf}, 		//	testni key
	//{0xbb, 0x49, 0x65, 0x0a} ,
	//{0x99, 0x0b, 0xfe, 0x4b} ,
	//{0xb9, 0x74, 0x07, 0x7f} ,
	//{0x57, 0xd5, 0x8e, 0x38} ,  	// Blaz Fit-FIt
	{0x44, 0x8F, 0x60, 0x4C} ,		// Blaz key
	{0x04, 0x67, 0x60, 0x4C} ,		// Luka key
	{0x74, 0xC2, 0x61, 0x4C} ,		// Marko key
	{0x24, 0x68, 0x62, 0x4C} ,		// Jozi key
	{0xB4, 0x74, 0x61, 0x4C} ,		// Pa3 key
	{0x99, 0x0B, 0xFE, 0x4B} ,		// Blaz card
	{0xA2, 0xCE, 0xF8, 0x1B} ,		// Jozi card
	{0x63, 0x1A, 0xB2, 0x1A}		// Pa3 card
};


char *sNames[] = {
		"Test key",
		"Blaz",
		"Luka",
		"Marko",
		"Jozica",
		"Pa3cio",
		"Blaz CARD",
		"Jozica CARD",
		"Pa3cio CARD"
};



const uint8_t RFIDcodesSize16[NUM_CODES_SIZE16][SIZE16] = {
		{0x12, 0xd6, 0x8a, 0xb1, 0x44, 0x55, 0x0c, 0x03, 0xaf, 0x47, 0x4f, 0xf2, 0x75, 0x55, 0x00, 0x9b}
};

const uint8_t cKEYA[6] = {0x5e, 0x24, 0xa6, 0x62, 0x69, 0xfd};



BSP_DemoTypedef BSP_examples[]=
{
  {Touchscreen_demo, "TOUCHSCREEN", 0}, 
  {LCD_demo, "LCD", 0}, 
  {Log_demo, "LCD LOG", 0},     
  {MEMS_demo, "MEMS", 0}, 
#ifdef EE_M24LR64
  {EEPROM_demo, "EEPROM", 0}, 
#endif /* EE_M24LR64 */
};

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Display_DemoDescription(void);
static void printUID(UID* uid);
static void UIDtoString(UID* uid, char* stringUID);
static void Display_InitContent(void);
static void Display_AccesGranted(void);
static void Clear_AccesGranted(void);
static void Display_AccesDenied(void);

static void AccessControl();

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{ 
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure RELAY port */
  GPIOB_Init();
  // turn off relay
  HAL_GPIO_WritePin(RELAY_PORT, RELAY_IN_PIN, GPIO_PIN_SET);


  /* Configure LED3 and LED4 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4); 
  
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();
  
  /* Configure USER Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  


  USART_Init(&UartHandle);
  RetargetInit(&UartHandle);

  printf("\n RFID STM32F429i Discovery \n\n");


  // Prepare the security key for the read and write functions - all six key bytes are set to 0xFF at chip delivery from the factory.
  // Since the cards in the kit are new and the keys were never defined, they are 0xFF
  // if we had a card that was programmed by someone else, we would need to know the key to be able to access it. This key would then need to be stored in 'key' instead.

  for (byte i = 0; i < 6; i++) {
	  key.keyA[i] = 0xFF;	// keyA is defined in the "MIFARE_Key" 'struct' definition in the .h file of the library
	  key.keyB[i] = 0xFF;
  }
  for (byte i = 0; i < 6; i++) {
  	  new_key.keyA[i] = cKEYA[i];	// keyA is defined in the "MIFARE_Key" 'struct' definition in the .h file of the library
  	  new_key.keyB[i] = 0xFF;
  }


  //Set transport configuration for access bits:
  B0.c1bit = ACC_BIT_RESET; B0.c2bit = ACC_BIT_RESET; B0.c3bit = ACC_BIT_RESET;
  B1.c1bit = ACC_BIT_RESET; B1.c2bit = ACC_BIT_RESET; B1.c3bit = ACC_BIT_RESET;
  B2.c1bit = ACC_BIT_RESET; B2.c2bit = ACC_BIT_RESET; B2.c3bit = ACC_BIT_RESET;
  TB.c1bit = ACC_BIT_RESET; TB.c2bit = ACC_BIT_RESET; TB.c3bit = ACC_BIT_SET;




  /*##-1- Initialize the LCD #################################################*/
  /* Initialize the LCD */
  BSP_LCD_Init();
  
  /* Initialize the LCD Layers */
  BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER);
  
  Display_InitContent();

  PCD_Init();


  while(1){
	  //clearBuffers();
	  do {
		  //Display_InitContent();
		  BSP_LED_On(LED3);
		  HAL_Delay(80);
		  BSP_LED_Off(LED3);
		  HAL_Delay(80);
		  cardPresent = PICC_IsNewCardPresent(); 	// sets successRead to 1 when we get read from reader otherwise 0
	  } while (!cardPresent); 	//the program will not go further while you not get a successful read

	  cardPresent = false;

#ifdef __ACCESS_CONTROL__
	  AccessControl();
#endif

	  /*

	  if (PICC_ReadCardSerial(&uid)) {
		  UIDtoString(&uid, strUID);
		  Display_AccesGranted();
		  HAL_GPIO_WritePin(RELAY_PORT, RELAY_IN_PIN, GPIO_PIN_RESET);
		  BSP_LED_On(LED3);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(RELAY_PORT, RELAY_IN_PIN, GPIO_PIN_SET);
		  HAL_Delay(1000);
		  printf("\n\n***************************************************************\n");
		  printUID(&uid);
		  printf("\n");
		  printf("***************************************************************\n");
		  PICC_HaltA();
		  Clear_AccesGranted();
		  Display_InitContent();
	  }
	  */

  }






  Display_DemoDescription();
  
  /* Wait For User inputs */
  while (1)
  {
    if(BSP_PB_GetState(BUTTON_KEY) == RESET)
    {
      while (BSP_PB_GetState(BUTTON_KEY) == RESET);
      
      BSP_examples[DemoIndex++].DemoFunc();
      
      if(DemoIndex >= COUNT_OF_EXAMPLE(BSP_examples))
      {
#ifdef EE_M24LR64
        /* Increment number of loops which be used by EEPROM example */
        NbLoop++;
#endif /* EE_M24LR64 */
        DemoIndex = 0;
      }
      Display_DemoDescription();
    }
  }
}



static void printUID(UID* uid){
	int i;
	uint8_t nibbleH;
	uint8_t nibbleL;
	uint8_t c1, c2;
	for (i = 0; i<uid->size; i++){
		nibbleH = (uid->uids[i] >> 4) & 0x0F;
		nibbleL = uid->uids[i] & 0x0F;
		if (nibbleH < 10) {
			c1 = nibbleH + 0x30;
		}
		else c1 = nibbleH + 0x37;
		if (nibbleL < 10) {
			c2 = nibbleL + 0x30;
		}
		else c2 = nibbleL + 0x37;

		printf("%c%c", c1,c2);
	}
}


static void UIDtoString(UID* uid, char* stringUID){
	int i;
	uint8_t nibbleH;
	uint8_t nibbleL;
	uint8_t c1, c2;
	for (i = 0; i<uid->size; i++){
		nibbleH = (uid->uids[i] >> 4) & 0x0F;
		nibbleL = uid->uids[i] & 0x0F;
		if (nibbleH < 10) {
			c1 = nibbleH + 0x30;
		}
		else c1 = nibbleH + 0x37;
		if (nibbleL < 10) {
			c2 = nibbleL + 0x30;
		}
		else c2 = nibbleL + 0x37;
		strUID[2*i] = c1;
		strUID[2*i+1] = c2;
	}
}


static void AccessControl(){
	StatusCode_t status;
	uint8_t* id;
	int8_t dummy;

	if (PICC_ReadCardSerial(&uid)) {
		card.uid = &uid;
		uid.index = -1;

		UIDtoString(&uid, strUID);
		// 1. check 4-byte UID - allow only known RFIDs
		status = TAG_CheckID(uid.uids, (uint8_t*)RFIDcodes, NRFIDCODES4, 4, &(uid.index));
		if (status == STATUS_OK){
			//2. allowed card, read the sector 15:
			status = TAG_ReadMifareClassicSector(&uid, &card, 15, &new_key, AUTH_KEY_A);
			if (status == STATUS_OK){
				// 3. check check 16-byte ID from sector 15, block 60;
				id = &(card.sector[15].Block[0][0]);
				status = TAG_CheckID(id, &RFIDcodesSize16[0][0], 1, 16, &dummy);
				if (status == STATUS_OK){
					Display_AccesGranted();
					HAL_GPIO_WritePin(RELAY_PORT, RELAY_IN_PIN, GPIO_PIN_RESET);
					// RFID allowed:
					BSP_LED_On(LED3);
					HAL_Delay(1000);
					HAL_GPIO_WritePin(RELAY_PORT, RELAY_IN_PIN, GPIO_PIN_SET);
					HAL_Delay(1000);
					Clear_AccesGranted();
					Display_InitContent();
				}
			}
			else{
				//printf("!!!ACCESS DENIED!!!! \n");
				Display_AccesDenied();
				HAL_Delay(1000);
				Clear_AccesGranted();
				Display_InitContent();
			}
		}
		else{
			//printf("!!!ACCESS DENIED!!!! \n");
			Display_AccesDenied();
			HAL_Delay(1000);
			Clear_AccesGranted();
			Display_InitContent();
		}
		PICC_HaltA();
		PCD_StopCrypto1();
	}

}



/**
  * @brief  Display main messages
  * @param  None
  * @retval None
  */
static void Display_InitContent(void)
{
  uint8_t desc[50];

  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
  //BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"STM32F429I BSP", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 6, (uint8_t*)"Kontrola", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 26, (uint8_t*)"dostopa", CENTER_MODE);
  //BSP_LCD_SetFont(&Font16);
  //BSP_LCD_DisplayStringAt(0, 55, (uint8_t*)"Prislonite RFID", CENTER_MODE);
  //BSP_LCD_DisplayStringAt(0, 68, (uint8_t*)"kljucek", CENTER_MODE);

  /* Draw Bitmap */
  //BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 80)/2, 75, (uint8_t *)stlogo);
  PB_BSP_LCD_DrawBitmap(0, 50, &bmimg_hisa);
  //PB_BSP_LCD_DrawBitmap(0, 50, &bmimg_hisa888);

  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize( )- 40, (uint8_t*)"Razvil:", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize( )- 20, (uint8_t*)"Pa3cio, 2022", CENTER_MODE);


  /*
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 15, BSP_LCD_GetXSize(), 60);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 30, (uint8_t*)"Prislonite RFID", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 50, (uint8_t*)"kljucek", CENTER_MODE);
  */


  /* Draw Bitmap */
  //BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 180)/2, 0, (uint8_t *)stlogo);
  //PB_BSP_LCD_DrawBitmap(0, 20, (uint8_t *)hisa);
  //sprintf((char *)desc,"%s example", BSP_examples[DemoIndex].DemoName);
  //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 45, (uint8_t *)desc, CENTER_MODE);
}


static void Display_AccesGranted(void)
{
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
	BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 35, BSP_LCD_GetXSize(), 80);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 35 + 10, (uint8_t*)"Dostop:", CENTER_MODE);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 35 + 30, sNames[uid.index], CENTER_MODE);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 35 + 60, strUID, CENTER_MODE);

	/*
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 15, BSP_LCD_GetXSize(), 60);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 30, (uint8_t*)"RFID prebran!", CENTER_MODE);
	//BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 50, (uint8_t*)"kljucek", CENTER_MODE);
	 */
}

static void Display_AccesDenied(void)
{
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 35, BSP_LCD_GetXSize(), 80);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 35 + 10, (uint8_t*)"Dostop", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 35 + 30, (uint8_t*)"zavrnjen!", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 35 + 50, strUID, CENTER_MODE);
/*
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 15, BSP_LCD_GetXSize(), 60);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 30, (uint8_t*)"RFID prebran!", CENTER_MODE);
	//BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 50, (uint8_t*)"kljucek", CENTER_MODE);
	 *
	 */
}


static void Clear_AccesGranted(void)
{
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 - 85, BSP_LCD_GetXSize(), 60);

	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 15, BSP_LCD_GetXSize(), 60);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 30, (uint8_t*)"Prislonite RFID", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 50, (uint8_t*)"kljucek", CENTER_MODE);
}







/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}



/**
  * @brief  Display main demo messages
  * @param  None
  * @retval None
  */
static void Display_DemoDescription(void)
{
  uint8_t desc[50];
  
  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);
  
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  
  /* Clear the LCD */ 
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE); 
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  
  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);  
  
  /* Display LCD messages */
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"STM32F429I BSP", CENTER_MODE);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t*)"Drivers examples", CENTER_MODE);
  
  /* Draw Bitmap */
  BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 80)/2, 65, (uint8_t *)stlogo);
  
  BSP_LCD_SetFont(&Font8);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()- 20, (uint8_t*)"Copyright (c) STMicroelectronics 2017", CENTER_MODE);
  
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 15, BSP_LCD_GetXSize(), 60);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE); 
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 30, (uint8_t*)"Press USER Button to start:", CENTER_MODE);
  sprintf((char *)desc,"%s example", BSP_examples[DemoIndex].DemoName);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 45, (uint8_t *)desc, CENTER_MODE);   
}

/**
  * @brief  Check for user input
  * @param  None
  * @retval Input state (1 : active / 0 : Inactive)
  */
uint8_t CheckForUserInput(void)
{
  if(BSP_PB_GetState(BUTTON_KEY) == RESET)
  {
    while (BSP_PB_GetState(BUTTON_KEY) == RESET);
    return 1;
  }
  return 0;
}

/**
  * @brief  Toggle LEDs
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  static uint8_t ticks = 0;
  
  if(ticks++ > 100)
  {
    //BSP_LED_Toggle(LED3);
    //BSP_LED_Toggle(LED4);
    ticks = 0;
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if (GPIO_Pin == KEY_BUTTON_PIN)
 {
   ubKeyPressed = SET;
 }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
