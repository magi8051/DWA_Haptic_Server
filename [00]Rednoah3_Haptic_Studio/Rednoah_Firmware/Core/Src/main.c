/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ae_i2c.h"
#include "sensor.h"
#include "boot_loader.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef _DEBUG_JKS
#define jks_debug(...) printf(__VA_ARGS__)
#else
#define jks_debug(...) NULL
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/*Base Struct for BBB*/
volatile struct bit_reg s_bit; // bit reigster
struct uart_packet s_upk;      // uart buffer
struct rtp_packet s_rtp;       // rtp buffer

/*board info*/
u8 g_board_info;
u8 g_ic_info;

/*boot pv*/
u32 g_flash_jmp;

/*i2c pv*/
u8 g_i2c_info[] = {I2C_50KHZ, I2C_100KHZ, I2C_400KHZ, I2C_1MHZ, I2C_1_2MHZ, I2C_2MHZ, I2C_3MHZ};
u8 g_i2c_clk;
u8 g_i2c_id;

/*timer pv*/
u16 g_tmr_psc;
u16 g_tmr_period;
u32 g_tmr_cnt_1ms;

enum board_type
{
  REDNOAH2 = 0,
  REDNOAH3
};

enum fsm
{
  SYS_FSM = 0,
  I2C_FSM,
  RTP_FSM,
  REV_FSM,
  SCP_FSM
};

enum dev
{
  DW7800 = 0,
  DW7802,
  DW7912,
  DW7914
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* Macro Data Conversion Function */
u16 _8u16(u8 *in) { return (u16)(((u16)in[0] << 8) | in[1]); }

u32 _8u32(u8 *in) { return (u32)((((u32)in[0] << 24) | ((u32)in[1] << 16) | ((u32)in[2] << 8) | ((u32)in[3] << 0))); }

s16 _twos(u16 in)
{ // two's complement
  s16 ret;
  if (in > 32767)
    ret = (s16)(in - 65536);
  else
    ret = in;
  return ret;
}

float _fp(u32 in, u32 num)
{
  return (float)in / (float)num;
}

void _16u8(u16 *_u16, u8 *_u8)
{
  *(_u8 + 0) = *(_u16 + 0) >> 8;
  *(_u8 + 1) = *(_u16 + 0) >> 0;
}

static void delay_us(int t)
{
  for (volatile int k = 0; k < 10; k++)
  {
    for (volatile int i = 0; i < t - 1; i++)
    {
      __asm volatile("NOP");
      __asm volatile("NOP");
      __asm volatile("NOP");
      __asm volatile("NOP");
      __asm volatile("NOP");
      __asm volatile("NOP");
    }
  }
}

/***************************************************************************
Function	: uart_transfer_task
Version		: 1.0
Descript 	:
***************************************************************************/
void uart_transfer_task(u32 tx_size)
{
  /*Default Header. Do not change!*/
  s_upk.buf[0] = 0xAE;
  s_upk.buf[1] = s_upk.hw;
  s_upk.buf[2] = tx_size >> 8;
  s_upk.buf[3] = tx_size >> 0;
  s_upk.buf[4] = 0;
  s_upk.buf[5] = s_upk.echo;

  for (int i = 0; i < tx_size; i++)
  {
    s_upk.buf[4] += s_upk.buf[6 + i];
  }

  LED5_T;
  HAL_UART_Transmit_DMA(&huart1, (u8 *)s_upk.buf, tx_size + BHD6);
  // HAL_UART_Transmit_IT(&huart1,(u8*)s_upk.buf, txsize + BHD6);
}

static void device_info_request(void)
{
  volatile u8 pin, size, i;
  uint32_t sn[3];

  i = 0;
  pin = i2c_pin_state();

  /*step: sw version*/
  s_upk.buf[6] = 0;
  s_upk.buf[7] = 0;
  s_upk.buf[8] = (u8)(REDNOAH_FW_INFO >> 24);
  s_upk.buf[9] = (u8)(REDNOAH_FW_INFO >> 16);
  s_upk.buf[10] = (u8)(REDNOAH_FW_INFO >> 8);
  s_upk.buf[11] = (u8)(REDNOAH_FW_INFO >> 0);

  /*step: serial number*/
  sn[0] = HAL_GetUIDw0();
  sn[1] = HAL_GetUIDw1();
  sn[2] = HAL_GetUIDw2();
  for (int i = 0; i < 3; i++)
  {
    s_upk.buf[12 + i * 4] = (u8)(sn[i] >> 24);
    s_upk.buf[13 + i * 4] = (u8)(sn[i] >> 16);
    s_upk.buf[14 + i * 4] = (u8)(sn[i] >> 8);
    s_upk.buf[15 + i * 4] = (u8)(sn[i] >> 0);
  }

  /* i2c pin setting error */
  if (pin != 3)
  {
    s_upk.buf[24] = 0xff;
    s_upk.buf[25] = pin;
    s_upk.buf[26] = 0;
    s_upk.buf[27] = 0;
    size = 19;
  }
  else
  {
    s_upk.buf[24] = 0;
    for (g_i2c_id = 0; g_i2c_id < 200; g_i2c_id += 2)
    {
      if (i2c_write_task(g_i2c_id, 0, 1, &g_i2c_id, 1, I2C_400KHZ) == I2C_ACK)
      {
        s_upk.buf[24]++;
        s_upk.buf[25 + i] = g_i2c_id;
        s_upk.buf[26 + i] = i2c_8bit_r(0x00);
        i += 2;
      }
    }
    size = 19 + s_upk.buf[24] * 2;
  }

  uart_transfer_task(size);
}

/***************************************************************************
Function	: trig pin control
Version		: 1.0
Descript 	: io 1 ~ 4
***************************************************************************/
static void trig_ctrl_task(int setup)
{
  u32 pin_temp;
  u8 temp, pin_num;
  static u16 delay_cnt, io;
  static u8 time_num;
  static u16 time_elapsed;
  u16 time[8], time_temp;
  static u32 pin[8];

  if (setup == 0)
  {
    /*init reg*/
    io = 0;
    delay_cnt = 0;
    time_elapsed = 0;

    pin_num = s_upk.buf[8];
    time_num = pin_num * 2;
    for (int i = 0; i < pin_num; i++)
    {
      pin[i] = GPIO_PIN_0 << s_upk.buf[9 + i * 5];
      pin[i + pin_num] = (uint32_t)GPIO_PIN_0 << (16 + s_upk.buf[9 + i * 5]);
      time[i] = _8u16(s_upk.buf + (10 + i * 5));
      time[i + pin_num] = _8u16(s_upk.buf + (12 + i * 5)) + time[i];
    }

    /* sort array */
    for (int i = 0; i < time_num; i++)
    {
      for (int j = 0; j < time_num - 1; j++)
      {
        if (time[j] > time[j + 1])
        {
          pin_temp = pin[j];
          pin[j] = pin[j + 1];
          pin[j + 1] = pin_temp;

          time_temp = time[j];
          time[j] = time[j + 1];
          time[j + 1] = time_temp;
        }
      }
    }

    /* remove duplicats time */
    for (int i = 0; i < time_num - 1; i++)
    {
      temp = time_num;
      for (int j = i + 1; j < temp; j++)
      {
        if (time[i] == time[i + 1])
        {
          time_num--;
          pin[i] |= pin[i + 1];
          for (int k = i + 1; k < time_num; k++)
          {
            pin[k] = pin[k + 1];
            time[k] = time[k + 1];
          }
        }
        else
        {
          break;
        }
      }
    }
    delay_cnt = time[io] - time_elapsed;
    time_elapsed = time[io];
  }

  else if (s_bit.trig_act == 1)
  {
    if (delay_cnt > 1)
    {
      delay_cnt--;
    }
    else
    {
      GPIOA->BSRR = pin[io];

      if (io > time_num) /* finish */
      {
        s_bit.trig_act = 0;
      }
      else
      {
        io++;
      }
      delay_cnt = time[io] - time_elapsed;
      time_elapsed = time[io];
    }
  }
}

/***************************************************************************
Function	: board_set_task
Version		: 1.0
Descript 	: chip info, reset, who am i
***************************************************************************/
static void board_set_task(void)
{
  switch (s_upk.buf[7])
  {
  case 0x00: /*board info */
    device_info_request();
    break;

  case 0x01: /*device info*/
    g_ic_info = s_upk.buf[8];
    g_i2c_id = s_upk.buf[9];
    if (s_upk.buf[5])
      uart_transfer_task(1); // echo
    break;

  case 0x02: /*board reset */
    REDNOAH_RESET;
    break;

  case 0x03: /*ldo control */
    ADJ_LDO_EN(s_upk.buf[8]);
    break;

  case 0x04: /* find auto i2c id */
    s_upk.buf[8] = 0;
    for (u8 id = 1; id < 119; id++)
    {
      if (i2c_write_task(id << 1, 0, 1, &id, 1, I2C_400KHZ) == I2C_ACK)
      {
        s_upk.buf[9 + s_upk.buf[8]] = id << 1;
        s_upk.buf[8]++;
      }
    }
    uart_transfer_task(3 + s_upk.buf[8]); // do not change!
    break;

  case 0x05: /* Trigger pin control */
    s_bit.trig_act = 1;
    trig_ctrl_task(0);
    break;

  case 0xFA: /*firmware update*/
    if (s_upk.buf[8] == 0x01)
    {
      bootloader_update_task();
      jump_vector_table();
    }
    break;
  }
}

/***************************************************************************
Function  : sys_timer_set
Version		: 1.0
Descript 	: timer control tick base is usec
***************************************************************************/
static void sys_timer_set(u32 timer, u32 mode, u32 usec)
{
  switch (timer)
  {
  case TIMER_1:
    if (mode == PLAY)
    {
      /*APB2_180Mhz*/
      g_tmr_psc = 180 - 1;
      s_bit.timer1_act = 1;
      g_tmr_period = usec - 1;
      MX_TIM1_Init();
      HAL_TIM_Base_Start_IT(&htim1);
    }
    else
    {
      s_bit.timer1_act = 0;
      HAL_TIM_Base_Stop_IT(&htim1);
    }
    break;

  case TIMER_7:
    if (mode == PLAY)
    {
      g_tmr_psc = 90 - 1;
      s_bit.timer7_act = 1;
      g_tmr_period = usec - 1;
      MX_TIM7_Init();
      HAL_TIM_Base_Start_IT(&htim7);
    }
    else
    {
      s_bit.timer7_act = 0;
      HAL_TIM_Base_Stop_IT(&htim7);
    }
    break;
  }
}

/***************************************************************************
Function	: init_rednoah_platform_board
Version		: 1.0
Descript 	: init function
***************************************************************************/
static void init_redhoah_system(void)
{
  /*LDO power on*/
  ADJ_LDO_EN(1); /* ADJ LDO Enable */

  /*Auto board check*/
  HAL_Delay(100);
  s_upk.hw = (BOOT_ID2 << 2) | (BOOT_ID1 << 1) | (BOOT_ID0 << 0);

  /*gpio i2c init start*/
  g_i2c_clk = g_i2c_info[1]; /* set 1mhz */
  init_i2c(0, 1);            /* rednoah default, i2c ch 1 */

  /* Accel sensor i2c init */
  si2c_init();

  /*uart get start*/
  HAL_UART_Receive_IT(&huart1, &s_upk.get, 1);
  // HAL_UART_Receive_DMA(&huart1, &s_upk.get, 1);

  /*AXL Sensor*/
  SPI3_NSS0(1);
  SPI3_NSS1(1);
  SPI3_NSS2(1);

  /* GPIO for Trigger */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pin : PA0,PA1,PA2,PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* for handshake */
  // device_info_request();

  /*boot messsage to PC*/
  sys_timer_set(TIMER_1, PLAY, 1000); /* systick 1ms */
}

/***************************************************************************
Function	: tick_led_task
Version		: 1.0
Descript 	: led blink tick 1ms
***************************************************************************/
static void sys_led_task(void)
{
  volatile static u32 t1, t2, n;

  t1++;
  g_tmr_cnt_1ms++;

  if (t1 > 200)
  {
    t1 = 0;
    LED4_T;
  }

  if (s_bit.rtp_act)
  {
    if (t2 > 100)
    {
      n++;
      t2 = 0;
      if (n == 1)
      {
        LED1(LEDON);
        LED2(LEDOF);
        LED3(LEDOF);
      }
      else if (n == 2)
      {
        LED1(LEDOF);
        LED2(LEDON);
        LED3(LEDOF);
      }
      else if (n == 3)
      {
        n = 0;
        LED1(LEDOF);
        LED2(LEDOF);
        LED3(LEDON);
      }
    }
    else
    {
      t2++;
    }
  }
}

/***************************************************************************
Function	: sys_key_task
Version		: 1.0
Descript 	: polling mode 10ms
***************************************************************************/
static void sys_key_task(void)
{
  volatile static u32 lock;
  u8 dat[2];

  if (!KEY1 || !KEY2 || !KEY3)
  {
    if (!KEY1 && !lock)
    {
      lock = 1;
      LED1_T;
      /* user code here!! */
      new_i2c_write_task(0xb2, dat, 2, I2C_1_2MHZ);
    }
    else if (!KEY2 && !lock)
    {
      lock = 1;
      LED2_T;
      /* user code here!! */
    }
    else if (!KEY3 && !lock)
    {
      lock = 1;
      LED3_T;
      /* user code here!! */
    }
  }
  else
  {
    lock = 0;
  }
}

/***************************************************************************
Function	: i2c task
Version		: 1.2
Descript 	: i2c r/w event
***************************************************************************/
static void i2c_task(void)
{
  static u8 scope;
  u8 type, mode, cnt, id;
  u16 dsize, asize, clk;

  if (s_bit.scope_act)
  {
    scope = 1;
    s_bit.scope_act = 0;
  }

  // id = g_i2c_id;
  s_bit.i2c_act = 1;
  type = s_upk.buf[7] & 0x10;
  asize = s_upk.buf[7] & 0x0f;
  /* clk set */
  g_i2c_clk = g_i2c_info[s_upk.buf[8]];
  // clk = g_i2c_clk;

  dsize = _8u16(s_upk.buf + 9);
  g_i2c_id = s_upk.buf[11];

  if (type == 0x10) /* i2c read mode */
  {
    if (new_i2c_write_task(g_i2c_id, (u8 *)s_upk.buf + 12, asize, g_i2c_clk) != I2C_ACK)
    {
      s_upk.buf[7] = 0xff;
      uart_transfer_task(2);
    }
    else
    {
      new_i2c_read_task(g_i2c_id, (u8 *)s_upk.buf + (12 + asize), dsize, g_i2c_clk);
      uart_transfer_task(BHD6 + asize + dsize);
    }
  }
  else /* i2c write mode */
  {
    if (new_i2c_write_task(g_i2c_id, (u8 *)s_upk.buf + 12, dsize + asize, g_i2c_clk) != I2C_ACK)
    {
      s_upk.buf[7] = 0xff;
    }
    else
    {
      s_upk.buf[7] = 0x00;
    }
    uart_transfer_task(2);
  }

  // g_i2c_clk = clk;
  // g_i2c_id = id;
  s_bit.i2c_act = 0;
  s_bit.scope_act = scope;
  scope = 0;
}

/***************************************************************************
Function	: i2c byte write
Version		: 1.0
Descript 	: 8bit addr, data format
***************************************************************************/
u8 i2c_8bit_w(u8 addr, u8 data)
{
  return i2c_write_task(g_i2c_id, addr, I2C_8BIT, &data, 1, g_i2c_clk);
}

/***************************************************************************
Function	: i2c byte read
Version		: 1.0
Descript 	: 8bit addr, data format
***************************************************************************/
u8 i2c_8bit_r(u8 addr)
{
  u8 data;
  i2c_read_task(g_i2c_id, addr, I2C_8BIT, &data, 1, g_i2c_clk);
  return data;
}

/***************************************************************************
Function	: init rtp task
Version		: 1.0
Descript 	: DW7800, DW7912, DW7914 rtp setup
***************************************************************************/
int init_rtp_task(void)
{
  static u8 p;
  u16 time;

  /* basic setup */
  if (s_upk.buf[8] == PLAY)
  {
    /* first time register clear*/
    if (s_bit.rtp_act == 0)
    {
      p = 0;
      s_rtp.play = 0;
      s_rtp.cnt[0] = 0;
      s_rtp.cnt[1] = 0;
    }

    s_rtp.play++;
    s_rtp.cnt[p] = 0;
    s_rtp.dev = s_upk.buf[7];
    s_rtp.size[p] = _8u16(s_upk.buf + 9);

    for (int i = 0; i < s_rtp.size[p]; i++)
    {
      s_rtp.buf[p][i] = s_upk.buf[11 + i];
    }
    p ^= 1;

    /* is first time run?*/
    if (s_bit.rtp_act == 0)
    {
      /* device setup */
      if (s_rtp.dev == DW7800)
      {
        time = 5000;
      }
      else if (s_rtp.dev == DW7802)
      {
        time = 5000;
      }
      else if (s_rtp.dev == DW7912)
      {
        /* for test set value */
        time = 5000;
        // i2c_8bit_w(0x09, 0x01);
      }
      else if (s_rtp.dev == DW7914)
      {
        time = 5000;
        // i2c_8bit_w(0x0B, 0x00); /* RTP */
      }

      s_bit.rtp_act = 1;
      s_bit.rtp_stop = 0;
      sys_timer_set(TIMER_7, PLAY, time); /* start timer */
    }
  }
  else /* stop */
  {
    if (s_rtp.dev == DW7800)
    {
      /* dw7800 sw reset */
      // i2c_8bit_w(0x05, 0x01);
    }
    else if (s_rtp.dev == DW7802)
    {
      i2c_8bit_w(0x0C, 0x00);
    }
    else if (s_rtp.dev == DW7912)
    {
      /* play stop & fifo fulsh */
      i2c_8bit_w(0x09, 0x00);
    }
    else if (s_rtp.dev == DW7914)
    {
      i2c_8bit_w(0x0C, 0x00);
    }

    s_bit.rtp_stop = 1;
  }

  return 0;
}

/***************************************************************************
Function	: play rtp task
Version		: 1.0
Descript 	: stream data for real time play
***************************************************************************/
int play_rtp_task(void)
{
  u8 buf[2];
  u16 fifo, play;
  static u8 echo, p;

  if (s_rtp.play && (s_bit.rtp_stop == 0))
  {
    /* DW7800 */
    if (s_rtp.dev == DW7800)
    {
      /* step : check fifo */
      fifo = i2c_8bit_r(0x03);

      if (fifo < 127)
      {
        /* what if fifo is empty?*/
        if (s_rtp.size[p] > s_rtp.cnt[p])
        {
          play = 127 - fifo;

          if (play > (s_rtp.size[p] - s_rtp.cnt[p]))
          {
            play = s_rtp.size[p] - s_rtp.cnt[p];
          }

          i2c_write_task(g_i2c_id, 0x04, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_1MHZ);
          s_rtp.cnt[p] += play;

          /* hand shake */
          if (echo == 0)
          {
            echo = 1;
            _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
            uart_transfer_task(5);
          }

          /* swap buffer */
          if (s_rtp.size[p] == s_rtp.cnt[p])
          {
            p ^= 1;
            echo = 0;
            s_rtp.play--;
            s_rtp.cnt[p] = 0;
          }
        }
      }
    }

    /* DW7802 */
    else if (s_rtp.dev == DW7802)
    {
      /* step : check fifo */
      i2c_read_task(g_i2c_id, 0x31, I2C_8BIT, buf, 2, I2C_1MHZ);
      fifo = _8u16(buf);

      /* what if fifo is empty?*/
      if (s_rtp.size[p] > s_rtp.cnt[p])
      {
        play = 2048 - fifo;

        if (play > (s_rtp.size[p] - s_rtp.cnt[p]))
        {
          play = s_rtp.size[p] - s_rtp.cnt[p];
        }

        i2c_write_task(g_i2c_id, 0x0D, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_1MHZ);
        // i2c_8bit_w(0x0C, 0x01);
        buf[0] = 0x01;
        i2c_write_task(g_i2c_id, 0x0C, I2C_8BIT, buf, 1, I2C_1MHZ);
        s_rtp.cnt[p] += play;

        /* hand shake */
        if (echo == 0)
        {
          echo = 1;
          _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
          uart_transfer_task(5);
        }

        /* swap buffer */
        if (s_rtp.size[p] == s_rtp.cnt[p])
        {
          p ^= 1;
          echo = 0;
          s_rtp.play--;
          s_rtp.cnt[p] = 0;
        }
      }
    }

    /* DW7912 */
    else if (s_rtp.dev == DW7912)
    {
      /* step : check fifo */
      fifo = i2c_8bit_r(0x0A);

      /* what if fifo is empty?*/
      if (s_rtp.size[p] > s_rtp.cnt[p])
      {
        play = 2048 - (fifo * 64 + 64);

        if (play > (s_rtp.size[p] - s_rtp.cnt[p]))
        {
          play = s_rtp.size[p] - s_rtp.cnt[p];
        }

        i2c_write_task(g_i2c_id, 0x0A, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_1MHZ);
        // i2c_8bit_w(0x09, 0x01);
        buf[0] = 0x01;
        i2c_write_task(g_i2c_id, 0x09, I2C_8BIT, buf, 1, I2C_1MHZ);
        s_rtp.cnt[p] += play;

        /* hand shake */
        if (echo == 0)
        {
          echo = 1;
          _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
          uart_transfer_task(5);
        }

        /* swap buffer */
        if (s_rtp.size[p] == s_rtp.cnt[p])
        {
          p ^= 1;
          echo = 0;
          s_rtp.play--;
          s_rtp.cnt[p] = 0;
        }
      }
    }

    /* DW7914 */
    else if (s_rtp.dev == DW7914)
    {
      /* step : check fifo */
      i2c_read_task(g_i2c_id, 0x4D, I2C_8BIT, buf, 2, I2C_1MHZ);
      fifo = _8u16(buf);

      /* what if fifo is empty?*/
      if (s_rtp.size[p] > s_rtp.cnt[p])
      {
        play = 2048 - fifo;

        if (play > (s_rtp.size[p] - s_rtp.cnt[p]))
        {
          play = s_rtp.size[p] - s_rtp.cnt[p];
        }

        i2c_write_task(g_i2c_id, 0x0D, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_1MHZ);
        // i2c_8bit_w(0x0C, 0x01);
        buf[0] = 0x01;
        i2c_write_task(g_i2c_id, 0x0C, I2C_8BIT, buf, 1, I2C_1MHZ);
        s_rtp.cnt[p] += play;

        /* hand shake */
        if (echo == 0)
        {
          echo = 1;
          _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
          uart_transfer_task(5);
        }

        /* swap buffer */
        if (s_rtp.size[p] == s_rtp.cnt[p])
        {
          p ^= 1;
          echo = 0;
          s_rtp.play--;
          s_rtp.cnt[p] = 0;
        }
      }
    }
  }
  else
  {
    /* play end or stop*/
    p = 0;
    echo = 0;
    s_bit.rtp_act = 0;
    sys_timer_set(TIMER_7, STOP, 0);

    /* for hand shake */
    s_upk.buf[6] = 0x02;
    s_upk.buf[7] = s_rtp.dev;
    s_upk.buf[8] = 0xff;
    uart_transfer_task(3);

    LED1(LEDON);
    LED2(LEDON);
    LED3(LEDON);
  }

  return 0;
}

/***************************************************************************
Function	: main_func_state_machine
Version		: 1.0
Descript 	: max packet 4kbyte
***************************************************************************/
static void main_func_state_machine(void)
{
  if (s_bit.rx_done == 1)
  {
    LED5(LEDON);

    switch (s_upk.buf[6])
    {
    case SYS_FSM:
      board_set_task();
      break;

    case I2C_FSM:
      i2c_task();
      break;

    case RTP_FSM:
      init_rtp_task();
      break;

    case SCP_FSM:
      g_sensor_set_task();
      break;
    }
    s_bit.rx_done = 0;

    LED5(LEDOF);
  }
}

/***************************************************************************
Function	: ADXL343 Sensor
Version		: 1.0
Descript 	: Data read out (200us)
***************************************************************************/
void g_sensor_read_out(void)
{
  u16 p, n;
  u8 dat[6], fifo;
  static u8 cks, mode;

  /* Auto trigger mode */
  if (s_rtp.mode == 0x01)
  {
    if (s_rtp.loop[0] > s_rtp.cnt[0])
    {
      /* all sensor read */
      dat[0] = 0x39;
      si2c_write_task(0, s_rtp.id, dat, 1, I2C_2MHZ);
      si2c_read_task(0, s_rtp.id, &fifo, 1, I2C_2MHZ);

      if (fifo + s_rtp.cnt[0] > s_rtp.loop[0])
      {
        fifo = s_rtp.loop[0] - s_rtp.cnt[0];
      }

      for (int i = 0; i < fifo; i++)
      {
        for (int ch = 0; ch < 3; ch++)
        {
          dat[0] = 0x32;
          si2c_write_task(ch, s_rtp.id, dat, 1, I2C_2MHZ);
          si2c_read_task(ch, s_rtp.id, dat, 6, I2C_2MHZ);

          p = s_rtp.cnt[0] * 18;
          n = (ch * 6) + p;
          s_upk.buf[12 + n] = dat[0];
          s_upk.buf[13 + n] = dat[1];
          s_upk.buf[14 + n] = dat[2];
          s_upk.buf[15 + n] = dat[3];
          s_upk.buf[16 + n] = dat[4];
          s_upk.buf[17 + n] = dat[5];
        }
        s_rtp.cnt[0]++;
      }
    }
    else
    {
      /* kill timer */
      s_rtp.cnt[0] = 0;
      s_bit.scope_act = 0;
      sys_timer_set(TIMER_7, STOP, 0);

      /* for hand shake */
      s_upk.buf[8] = 0xFF;
      s_upk.buf[9] = 0x00;
      s_upk.buf[10] = s_rtp.size[0] >> 8;
      s_upk.buf[11] = s_rtp.size[0] >> 0;
      uart_transfer_task(6 + s_rtp.size[0]);

      /* flush FIFO and set to BYPASS Mode */
      for (int ch = 0; ch < 3; ch++)
      {
        dat[0] = 0x38;
        dat[1] = 0x00;
        si2c_write_task(ch, s_rtp.id, dat, 2, I2C_2MHZ);
      }
    }
  }

  /* continuous mode */
  else if (s_rtp.mode == 0x02)
  {
    if (s_rtp.cnt[0] == 0)
    {
      cks = 0;
    }

    /* all sensor read */
    dat[0] = 0x39;
    for (int i = 0; i < 3; i++)
    {
      if (si2c_write_task(i, s_rtp.id, dat, 1, I2C_2MHZ) == I2C_ACK)
      {
        si2c_read_task(i, s_rtp.id, &fifo, 1, I2C_2MHZ);
        break;
      }
    }

    if (fifo + s_rtp.cnt[0] > s_rtp.loop[0])
    {
      fifo = s_rtp.loop[0] - s_rtp.cnt[0];
    }
    for (int i = 0; i < fifo; i++)
    {
      for (int ch = 0; ch < 3; ch++)
      {
        dat[0] = 0x32;
        si2c_write_task(ch, s_rtp.id, dat, 1, I2C_2MHZ);
        si2c_read_task(ch, s_rtp.id, dat, 6, I2C_2MHZ);

        if (s_rtp.cnt[1] < s_rtp.loop[0])
        {
          s_rtp.pk = 0;
        }
        else
        {
          s_rtp.pk = 1;
        }

        for (int k = 0; k < 6; k++)
        {
          n = ch * 6 + (s_rtp.cnt[0] * 18);
          s_rtp.buf[s_rtp.pk][(12 + k) + n] = dat[k];
          cks += dat[k];
        }
      }
      s_rtp.cnt[0]++;
      s_rtp.cnt[1]++;
    }

    if (s_rtp.cnt[1] == s_rtp.loop[0])
    {
      s_rtp.cnt[0] = 0;

      for (int i = 0; i < 6; i++)
      {
        cks += s_rtp.buf[0][6 + i];
      }
      s_rtp.buf[0][4] = cks;
      HAL_UART_Transmit_DMA(&huart1, s_rtp.buf[0], s_rtp.psize + BHD6);
    }

    else if (s_rtp.cnt[1] == s_rtp.loop[1])
    {
      s_rtp.cnt[0] = 0;
      s_rtp.cnt[1] = 0;

      for (int i = 0; i < 6; i++)
      {
        cks += s_rtp.buf[1][6 + i];
      }
      s_rtp.buf[1][4] = cks;
      HAL_UART_Transmit_DMA(&huart1, s_rtp.buf[1], s_rtp.psize + BHD6);
    }
  }
}

/***************************************************************************
Function	: ADXL343 Sensor
Version		: 1.0
Descript 	: Data set up
***************************************************************************/
void g_sensor_set_task(void)
{
  u16 time;
  u8 dat[2], rdat, rsize;

  rsize = 18;
  time = 1000;
  s_rtp.id = 0x3A;

  if (s_upk.buf[7] == 0x00)
  {
    /* sensor setting & detect */
    if (s_upk.buf[8] == 0x00)
    {
      /* auto contect detect */
      for (int ch = 0; ch < 3; ch++)
      {
        dat[0] = 0x00;
        si2c_write_task(ch, s_rtp.id, dat, 1, I2C_400KHZ);
        si2c_read_task(ch, s_rtp.id, &rdat, 1, I2C_400KHZ);
        s_upk.buf[9 + ch] = rdat;

        if (rdat == 0xE5)
        {
          /* Default Sensor setup */
          dat[0] = 0x2E; /* sensor setup */
          dat[1] = 0x00;
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

          dat[0] = 0x1D; /* Tab threshold */
          dat[1] = 20;
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

          dat[0] = 0x21; /* DUR register */
          dat[1] = 50;
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

          dat[0] = 0x22;
          dat[1] = 0;
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

          dat[0] = 0x23;
          dat[1] = 0;
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

          dat[0] = 0x2A; /* Tab Axes */
          dat[1] = 0x07;
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

          dat[0] = 0x2C; /* Data Rate 3200Hz */
          dat[1] = 0x0F;
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

          dat[0] = 0x2D; /* Power Control */
          dat[1] = 0x08;
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
        }
      }
      /* for handshake */
      uart_transfer_task(6);
    }
    /* g scale setting */
    else if (s_upk.buf[8] == 0x01)
    {
      for (int ch = 0; ch < 3; ch++)
      {
        if (s_upk.buf[9 + (ch * 2)] & 0x01)
        {
          /* sensor data format */
          dat[0] = 0x31;
          /* set to full resolution */
          dat[1] = 0x08 | s_upk.buf[10 + (ch * 2)];
          si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
        }

        /* for handshake */
        uart_transfer_task(4);
      }
    }
    /* Sensor Offset Calibration */
    else if (s_upk.buf[8] == 0x02)
    {
      sys_timer_set(TIMER_7, PLAY, 300);
      s_bit.scope_cal = 1;
    }
  }

  /* auto trigger setting*/
  else if (s_upk.buf[7] == 0x01)
  {
    s_rtp.mode = s_upk.buf[7];
    // s_rtp.loop[0] = _8u16(s_upk.buf + 11) * 1000 / time;
    s_rtp.loop[0] = _8u16(s_upk.buf + 11) * 3200 / 1000;
    s_rtp.size[0] = s_rtp.loop[0] * rsize; /* 3 Sensor x 6byte */
    s_rtp.cnt[0] = 0;

    /* Set to FIFO Mode */
    for (int ch = 0; ch < 3; ch++)
    {
      dat[0] = 0x38;
      dat[1] = 0x40;
      si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
    }

    /* timer set & measure started*/
    sys_timer_set(TIMER_7, s_upk.buf[8], time);

    if (s_upk.buf[8] == PLAY)
    {
      s_bit.scope_act = 1;
    }
    else
    {
      s_bit.scope_act = 0;
    }

    /* device setup */
    if (s_upk.buf[9] == DW7800)
    {
      /* not supported  */
    }
    else if (s_upk.buf[9] == DW7802)
    {
      /*  */
      i2c_8bit_w(0x0C, 0x01); /* play on */
    }
    else if (s_upk.buf[9] == DW7912)
    {
      /*  */
      i2c_8bit_w(0x09, 0x01); /* play on */
    }
    else if (s_upk.buf[9] == DW7914)
    {
      /*  */
      i2c_8bit_w(0x0C, 0x01); /* play on */
    }
  }

  /* continuous setting*/
  else if (s_upk.buf[7] == 0x02)
  {
    s_rtp.mode = s_upk.buf[7];
    // s_rtp.loop[0] = _8u16(s_upk.buf + 11) * 1000 / time;
    s_rtp.loop[0] = _8u16(s_upk.buf + 11) * 3200 / 1000;
    s_rtp.loop[1] = s_rtp.loop[0] << 1;
    s_rtp.psize = s_rtp.loop[0] * rsize + 6;
    s_rtp.cnt[0] = 0;
    s_rtp.cnt[1] = 0;

    /* stream dummy data */
    for (int i = 0; i < 10; i++)
    {
      s_rtp.buf[0][i] = s_upk.buf[i];
      s_rtp.buf[1][i] = s_upk.buf[i];
    }
    /* packet size redefine */
    s_rtp.buf[0][2] = s_rtp.psize >> 8;
    s_rtp.buf[0][3] = s_rtp.psize >> 0;
    s_rtp.buf[1][2] = s_rtp.psize >> 8;
    s_rtp.buf[1][3] = s_rtp.psize >> 0;

    s_rtp.buf[0][10] = (s_rtp.psize - 6) >> 8;
    s_rtp.buf[0][11] = (s_rtp.psize - 6) >> 0;
    s_rtp.buf[1][10] = (s_rtp.psize - 6) >> 8;
    s_rtp.buf[1][11] = (s_rtp.psize - 6) >> 0;

    /* Set to FIFO Mode */
    for (int ch = 0; ch < 3; ch++)
    {
      dat[0] = 0x38;
      dat[1] = 0x40;
      si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
    }

    /* timer set & measure started*/
    sys_timer_set(TIMER_7, s_upk.buf[8], time);

    if (s_upk.buf[8] == PLAY)
    {
      s_bit.scope_act = 1;
    }
    else
    {
      /* flush FIFO and set to BYPASS Mode */
      for (int ch = 0; ch < 3; ch++)
      {
        dat[0] = 0x38;
        dat[1] = 0x00;
        si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
      }

      uart_transfer_task(3);
      s_bit.scope_act = 0;
    }
  }
}

/***************************************************************************
Function	: ADXL343 Sensor
Version		: 1.0
Descript 	: Offset Calibration
***************************************************************************/
void g_sensor_ofs_cal(void)
{
  static s32 sum[9];
  static u16 cnt;
  u16 avg = 320;
  u8 dat[6], buf[6], fifo;
  s8 ofs[3];

  /* initialize offset registers */
  if (cnt == 0)
  {
    dat[0] = 0x1E;
    dat[1] = 0x00;
    dat[2] = 0x00;
    dat[3] = 0x00;
    for (int ch = 0; ch < 3; ch++)
    {
      si2c_write_task(ch, s_rtp.id, dat, 4, I2C_2MHZ);
    }

    /* Set to FIFO Mode */
    for (int ch = 0; ch < 3; ch++)
    {
      dat[0] = 0x38;
      dat[1] = 0x40;
      si2c_write_task(ch, s_rtp.id, dat, 2, I2C_2MHZ);
    }
  }

  /* read all sensors */
  dat[0] = 0x39;
  si2c_write_task(0, s_rtp.id, dat, 1, I2C_2MHZ);
  si2c_read_task(0, s_rtp.id, &fifo, 1, I2C_2MHZ);
  if (fifo + cnt > avg)
  {
    fifo = avg - cnt;
  }
  for (int i = 0; i < fifo; i++)
  {
    for (int ch = 0; ch < 3; ch++)
    {
      dat[0] = 0x32;
      si2c_write_task(ch, s_rtp.id, dat, 1, I2C_2MHZ);
      si2c_read_task(ch, s_rtp.id, dat, 6, I2C_2MHZ);

      /* swap bytes */
      buf[0] = dat[1];
      buf[1] = dat[0];
      buf[2] = dat[3];
      buf[3] = dat[2];
      buf[4] = dat[5];
      buf[5] = dat[4];

      /* summation of 0.1sec read data */
      sum[0 + (ch * 3)] += _twos(_8u16(&buf[0]));
      sum[1 + (ch * 3)] += _twos(_8u16(&buf[2]));
      sum[2 + (ch * 3)] += _twos(_8u16(&buf[4]));
    }
    cnt++;
  }

  /* write offset */
  if (cnt == avg)
  {
    for (int ch = 0; ch < 3; ch++)
    {
      ofs[0] = -sum[0 + (ch * 3)] / avg / 4; /* average and convert g values */
      ofs[1] = -sum[1 + (ch * 3)] / avg / 4; /* read data :  3.9mg/LSB in full resolution mode */
      ofs[2] = -sum[2 + (ch * 3)] / avg / 4; /* ofs reg   : 15.6mg/LSB */

      dat[0] = 0x1E;
      dat[1] = ofs[0];
      dat[2] = ofs[1];
      dat[3] = ofs[2];

      si2c_write_task(ch, s_rtp.id, dat, 4, I2C_2MHZ);
    }
    /* kill timer */
    sys_timer_set(TIMER_7, STOP, 0);
    s_bit.scope_cal = 0;

    /* for hand shake */
    s_upk.buf[9] = 0xFF;
    uart_transfer_task(4);

    /* flush FIFO and set to BYPASS Mode */
    for (int ch = 0; ch < 3; ch++)
    {
      dat[0] = 0x38;
      dat[1] = 0x00;
      si2c_write_task(ch, s_rtp.id, dat, 2, I2C_2MHZ);
    }

    /* initialize variables */
    cnt = 0;
    for (int i = 0; i < 9; i++)
    {
      sum[i] = 0;
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI3_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  init_redhoah_system();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (s_bit.timer1_act)
    {
      sys_led_task();
      sys_key_task();
      trig_ctrl_task(RUN);
      s_bit.timer1_act = 0;
    }

    if (s_bit.timer7_act)
    {
      if (s_bit.rtp_act)
      {
        play_rtp_task();
      }
      else if (s_bit.scope_act)
      {
        g_sensor_read_out();
      }
      else if (s_bit.scope_cal)
      {
        g_sensor_ofs_cal();
      }
      s_bit.timer7_act = 0;
    }

    main_func_state_machine();
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
   */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
   */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
   */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief SPI5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  htim1.Init.Prescaler = g_tmr_psc;
  htim1.Init.Period = g_tmr_period;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  htim7.Init.Prescaler = g_tmr_psc;
  htim7.Init.Period = g_tmr_period;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM7_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  huart1.Init.BaudRate = UART1_BUAD;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD3 PD4
                           PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG12 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/***************************************************************************
Function	: HAL_GPIO_EXTI_Callback
Date		: 2021.08.12
Descript 	: sw1, sw2, sw3
***************************************************************************/
void HAL_GPIO_EXTI_Callback(u16 GPIO_Pin)
{
}

/***************************************************************************
Function	: HAL_UART_RxCpltCallback
Date			: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: degign for speed
***************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  u8 cks;
  static u32 num, step, get_size;

  s_upk.buf[num++] = s_upk.get;

  switch (step)
  {
  case 0:
    if (s_upk.buf[0] != 0xAE)
      num = 0;
    else
      step = 1;
    break;
  case 1:
    if (num >= BHD6)
    {
      s_upk.get_size = _8u16(s_upk.buf + 2);
      get_size = s_upk.get_size + BHD6;
      step = 2;
    }
    break;
  case 2:
    if (num >= get_size)
    {
      cks = 0;
      for (int i = 0; i < s_upk.get_size; i++)
      {
        cks += s_upk.buf[6 + i];
      }

      /*checksum fail*/
      if (cks != s_upk.buf[4])
      {
        s_upk.buf[6] = 0x00;
        s_upk.buf[7] = 0xEC;
        s_upk.buf[8] = 0xE1;
        uart_transfer_task(3);
      }
      else
      {
        s_bit.rx_done = 1;
      }
      num = 0;
      step = 0;
    }
    break;
  }

  HAL_UART_Receive_IT(&huart1, &s_upk.get, 1);
#if 0
  HAL_UART_Receive_DMA(&huart1, &s_upk.get, 1);
#endif
}

/***************************************************************************
Function	: HAL_UART_TxCpltCallback
Date			: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: Reserve
***************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  // P0_T;
  s_bit.tx_done = 1;
}

/***************************************************************************
Function	: HAL_TIM_PeriodElapsedCallback
Date			: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: degign for speed
***************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  if (htim->Instance == TIM1)
  {
    s_bit.timer1_act = 1;
  }
  else if (htim->Instance == TIM7)
  {
    s_bit.timer7_act = 1;
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
