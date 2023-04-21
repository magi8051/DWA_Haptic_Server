/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : rednoah3_app.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Dongwoon Anatech.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REDNOAH3_APP_H
#define __REDNOAH3_APP_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */
  typedef unsigned char u8;
  typedef signed char s8;
  typedef unsigned short u16;
  typedef signed short s16;
  typedef unsigned int u32;
  typedef signed int s32;

  /*Boot Define*/
#define REDNOAH_FW_INFO 0x23042100 /* INFO -> (0x/year/month/date/rev 0 to 255) */
#define REDNOAH_RESET SCB->AIRCR = 0x05FA0000 | 0x04
#define REDNOAH_FLASH_20 ((u32)0x08180000)
/* Packet Define */
#define RX_SIZE (1024 * 40 + 32)  // 40Kb
#define RTP_SIZE (1024 * 40 + 32) // 4Kb

/*UART Define*/
#define BHD6 6
#define UART1_BUAD 921600 // 1Mbps
#define LEDON 0
#define LEDOF 1
#define STOP 0xFF
#define PLAY 0xCC
#define RUN 0x11
#define TIMER_10 10
#define TIMER_7 7
#define PI_ 3.14159265

/*I2C Define*/
#define I2C_50KHZ 136
#define I2C_100KHZ 68
#define I2C_400KHZ 16
#define I2C_1MHZ 5
#define I2C_1_2MHZ 4
#define I2C_2MHZ 2
#define I2C_3MHZ 1
#define I2C_8BIT 1
#define I2C_16BIT 2
#define I2C_ACK 0

/* BD ID */
#define BOOT_ID0 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define BOOT_ID1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)
#define BOOT_ID2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)

/* LED */
#define LED5(in) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, (GPIO_PinState)in)
#define LED4(in) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, (GPIO_PinState)in)
#define LED3(in) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, (GPIO_PinState)in)
#define LED2(in) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, (GPIO_PinState)in)
#define LED1(in) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, (GPIO_PinState)in)
#define LED1_T HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3)
#define LED2_T HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_4)
#define LED3_T HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5)
#define LED4_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_11)
#define LED5_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_12)

/* KEY */
#define KEY3 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)
#define KEY2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)
#define KEY1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)

/* LDO */
#define ADJ_LDO_EN(in) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, (GPIO_PinState)in)

/* SPI NSS */
#define SPI3_NSS2(in) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, (GPIO_PinState)in)
#define SPI3_NSS1(in) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, (GPIO_PinState)in)
#define SPI3_NSS0(in) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, (GPIO_PinState)in)
#define SPI5_NSS(in) HAL_GPIO_WritePin(GPIOF, GPIO_PIN6, (GPIO_PinState)in)

/* IO */
#define IO0(in) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (GPIO_PinState)in)
#define IO1(in) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (GPIO_PinState)in)
#define IO2(in) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (GPIO_PinState)in)
#define IO3(in) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (GPIO_PinState)in)

  /*bit register stuct*/
  struct bit_reg
  {
    u32
        rx_done : 1,
        rx_fail : 1,
        tx_done : 1,
        tx_fail : 1,
        i2c_act : 1,
        timer10_act : 1,
        timer7_act : 1,
        scope_act : 1,
        scope_cal : 1,
        rtp_act : 1,
        rtp_stop : 1,
        trig_act : 1;
  };

  /*stream buffer stuct*/
  struct rtp_packet
  {
    u8 buf[2][RTP_SIZE];
    u8 dev;
    u8 id;
    u8 pk;
    u8 mode;
    u16 play;
    u16 psize;
    u16 cnt[2];
    u16 size[2];
    u16 loop[2];
  };

  /*uart buffer stuct*/
  struct uart_packet
  {
    u8 buf[RX_SIZE];
    u8 hw;
    u8 get;
    u8 echo;
    u32 dev_num;
    u32 get_size;
  };

  /* USER CODE END EFP */

  extern TIM_HandleTypeDef htim7;
  extern TIM_HandleTypeDef htim10;
  extern UART_HandleTypeDef huart1;
  extern UART_HandleTypeDef huart3;
  extern DMA_HandleTypeDef hdma_usart1_tx;
  extern DMA_HandleTypeDef hdma_usart1_rx;

  /* Private defines -----------------------------------------------------------*/
  /* USER CODE BEGIN Private defines */
  u16 _8u16(u8 *in);
  u32 _8u32(u8 *in);
  void _16u8(u16 *_u16, u8 *_u8);

  u8 i2c_8bit_r(u8 addr);
  u8 i2c_8bit_w(u8 addr, u8 data);
  int play_rtp_task(void);
  int init_rtp_task(void);
  void uart_transfer_task(u32 tx_size);
  void init_redhoah3_system(void);
  void g_sensor_read_out(void);
  void g_sensor_set_task(void);
  void rednoah3_main_task(void);
  static void i2c_task(void);
  static void sys_led_task(void);
  static void sys_timer_set(u32 timer, u32 mode, u32 usec);
  static void board_set_task(void);
  static void device_info_request(void);
  static void delay_us(int t);
  static void triger_ctrl_task(void);

#ifdef __cplusplus
}
#endif

#endif /* __REDNOAH3_APP_H */
