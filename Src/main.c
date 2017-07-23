/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "helper.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#define CMD_READID 'i'
#define CMD_READPAGE 'p'
#define CMD_READSPARE 's'
#define CMD_PING1 '1'
#define CMD_PING2 '2'
#define CMD_ERASE 'e'
#define CMD_WRITEPAGE 'w'
#define CMD_WRITESPARE 'r'
#define CMD_WRITE_PAGE_SPARE 'o'

#define CMD_RET_OK 0x00
#define CMD_RET_ERROR 0xFF

/* Private variables ---------------------------------------------------------*/
NAND_HandleTypeDef hnand1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static NAND_IDTypeDef NAND_ID = {0};
static uint8_t aRxBuffer[4096+224];
struct RespondHeader {
   uint32_t ret;
   uint32_t payload_sz;
   uint32_t crc;
} cmd_header;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
void Send(uint8_t* buf, size_t len);
void SendChar(uint8_t ch);
uint8_t CalCRC(uint8_t* buf, size_t len)
{
	return 0; //return 0 for now
}
uint8_t RecvChar();

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_FSMC_Init();
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	NAND_AddressTypedef Address;

	uint16_t pcount = 0;

	while(1) {
		if(receive_buff_size()){
			switch(RecvChar())
			{
			case CMD_READID:
				//Some NAND need to reset first to get the correct ID
				HAL_NAND_Reset(&hnand1);
				if (HAL_NAND_Read_ID(&hnand1, &NAND_ID) != HAL_OK) {
					cmd_header.ret=CMD_RET_ERROR;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				} else {
					aRxBuffer[0] = NAND_ID.Maker_Id;
					aRxBuffer[1] = NAND_ID.Device_Id;
					aRxBuffer[2] = NAND_ID.Third_Id;
					aRxBuffer[3] = NAND_ID.Fourth_Id;
					cmd_header.payload_sz = 4;
					cmd_header.ret = CMD_RET_OK;
					cmd_header.crc = CalCRC(aRxBuffer, 4);
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
					Send(aRxBuffer, cmd_header.payload_sz);
				}
				break;
			case CMD_READPAGE:
				Address.Page = RecvChar();
				Address.Page |= RecvChar() << 8;
				Address.Block = RecvChar();
				Address.Block |= RecvChar() << 8;
				Address.Plane = RecvChar();
				Address.Plane |= RecvChar() << 8;

				if (HAL_NAND_Read_Page(&hnand1, &Address, aRxBuffer, 1) != HAL_OK) {
					cmd_header.ret=CMD_RET_ERROR;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				} else {
					cmd_header.payload_sz = hnand1.Config.PageSize;
					cmd_header.ret = CMD_RET_OK;
					cmd_header.crc = CalCRC(aRxBuffer, hnand1.Config.PageSize);
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
					Send(aRxBuffer, cmd_header.payload_sz);
				}
				break;
			case CMD_READSPARE:
				Address.Page = RecvChar();
				Address.Page |= RecvChar() << 8;
				Address.Block = RecvChar();
				Address.Block |= RecvChar() << 8;
				Address.Plane = RecvChar();
				Address.Plane |= RecvChar() << 8;

				if (HAL_NAND_Read_SpareArea(&hnand1, &Address, aRxBuffer, 1) != HAL_OK) {
					cmd_header.ret=CMD_RET_ERROR;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				} else {
					cmd_header.payload_sz = hnand1.Config.SpareAreaSize;
					cmd_header.ret = CMD_RET_OK;
					cmd_header.crc = CalCRC(aRxBuffer, hnand1.Config.SpareAreaSize);
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
					Send(aRxBuffer, cmd_header.payload_sz);
				}
				break;
			case CMD_ERASE:
				Address.Page = 0;
				Address.Block = RecvChar();
				Address.Block |= RecvChar() << 8;
				Address.Plane = RecvChar();
				Address.Plane |= RecvChar() << 8;

				if (HAL_NAND_Erase_Block(&hnand1, &Address) != HAL_OK) {
					cmd_header.ret=CMD_RET_ERROR;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				} else {
					cmd_header.ret=CMD_RET_OK;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				}
				break;
			case CMD_WRITEPAGE:
				Address.Page = RecvChar();
				Address.Page |= RecvChar() << 8;
				Address.Block = RecvChar();
				Address.Block |= RecvChar() << 8;
				Address.Plane = RecvChar();
				Address.Plane |= RecvChar() << 8;

				for(pcount = 0; pcount < hnand1.Config.PageSize; pcount++) {
					aRxBuffer[pcount] = RecvChar();
				}

				if (HAL_NAND_Write_Page(&hnand1, &Address, aRxBuffer, 1) != HAL_OK) {
					cmd_header.ret=CMD_RET_ERROR;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				} else {
					cmd_header.ret=CMD_RET_OK;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				}
				break;
			case CMD_WRITESPARE:
				Address.Page = RecvChar();
				Address.Page |= RecvChar() << 8;
				Address.Block = RecvChar();
				Address.Block |= RecvChar() << 8;
				Address.Plane = RecvChar();
				Address.Plane |= RecvChar() << 8;

				for(pcount = 0; pcount < hnand1.Config.SpareAreaSize; pcount++) {
					aRxBuffer[pcount] = RecvChar();
				}

				if (HAL_NAND_Write_SpareArea(&hnand1, &Address, aRxBuffer, 1) != HAL_OK) {
					cmd_header.ret=CMD_RET_ERROR;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				} else {
					cmd_header.ret=CMD_RET_OK;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				}
				break;
			case CMD_WRITE_PAGE_SPARE:
				Address.Page = RecvChar();
				Address.Page |= RecvChar() << 8;
				Address.Block = RecvChar();
				Address.Block |= RecvChar() << 8;
				Address.Plane = RecvChar();
				Address.Plane |= RecvChar() << 8;

				for(pcount = 0; pcount < (hnand1.Config.SpareAreaSize+hnand1.Config.PageSize); pcount++) {
					aRxBuffer[pcount] = RecvChar();
				}

				if (HAL_NAND_Write_Page_Oob_8b(&hnand1, &Address, aRxBuffer, 1) != HAL_OK) {
					cmd_header.ret=CMD_RET_ERROR;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				} else {
					cmd_header.ret=CMD_RET_OK;
					cmd_header.payload_sz=0;
					cmd_header.crc=0;
					Send((uint8_t*)&cmd_header, sizeof(cmd_header));
				}
				break;
			case CMD_PING1:
				SendChar('a');
				break;
			case CMD_PING2:
				SendChar('b');
				break;
			default:
				continue;
			}
		}
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

/* FSMC initialization function */
static void MX_FSMC_Init(void) {
	FSMC_NAND_PCC_TimingTypeDef ComSpaceTiming;
	FSMC_NAND_PCC_TimingTypeDef AttSpaceTiming;

	/** Perform the NAND1 memory initialization sequence
	 */
	hnand1.Instance = FSMC_NAND_DEVICE;
	/* hnand1.Init */
	hnand1.Init.NandBank = FSMC_NAND_BANK2;
	hnand1.Init.Waitfeature = FSMC_NAND_PCC_WAIT_FEATURE_ENABLE;
	hnand1.Init.MemoryDataWidth = FSMC_NAND_PCC_MEM_BUS_WIDTH_8;
	hnand1.Init.EccComputation = FSMC_NAND_ECC_DISABLE;
	hnand1.Init.ECCPageSize = FSMC_NAND_ECC_PAGE_SIZE_4096BYTE;
	hnand1.Init.TCLRSetupTime = 2;
	hnand1.Init.TARSetupTime = 2;
	/* hnand1.Config */
	hnand1.Config.PageSize = 0x1000;
	hnand1.Config.SpareAreaSize = 0xe0;
	hnand1.Config.BlockSize = 0x100;
	hnand1.Config.BlockNbr = 0x800;
	hnand1.Config.PlaneNbr = 2;
	hnand1.Config.PlaneSize = 0x400;
	hnand1.Config.ExtraCommandEnable = DISABLE;
	/* ComSpaceTiming */
	ComSpaceTiming.SetupTime = 5;//3;
	ComSpaceTiming.WaitSetupTime = 10;//10;
	ComSpaceTiming.HoldSetupTime = 4;//4;
	ComSpaceTiming.HiZSetupTime = 14;//14;
	/* AttSpaceTiming */
	AttSpaceTiming.SetupTime = 5;//3;
	AttSpaceTiming.WaitSetupTime = 10;//10;
	AttSpaceTiming.HoldSetupTime = 4;//4;
	AttSpaceTiming.HiZSetupTime = 14;//14;

	if (HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  SendChar(ch);

  return ch;
}


void Send(uint8_t* buf, size_t len)
{
  while(CDC_Transmit_FS((uint8_t *)buf, len) != USBD_OK) {
	  continue;
  }
}

void SendChar(uint8_t ch)
{
  while(CDC_Transmit_FS((uint8_t *)&ch, 1) != USBD_OK) {
	  continue;
  }
}

uint8_t RecvChar()
{
	uint8_t c;
	while(receive_buff_dequeue(&c, 1)) {
		continue;
	}

	return c;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
