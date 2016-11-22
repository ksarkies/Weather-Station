/* ChaN's FAT Disk I/O library.

MODIFIED FILE

Defines for the microcontroller board used.

The file is substantially that provided by Martin Thomas in his STM driver
contribution to the ChaN FAT library. It has been modified for libopencm3.

Only the two ET boards have been tested.

*/
/* Copyright (c) 2010, Martin Thomas, ChaN
   Copyright (c) 2013 Ken Sarkies
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#ifndef BOARD_H_
#define BOARD_H_

/* set to 1 to provide a disk_ioctrl function even if not needed by the FatFs */
#define STM32_SD_DISK_IOCTRL_FORCE      0

//#define STM32_SD_USE_DMA

//#define USE_ET_STAMP_STM32
//#define USE_ET_STM32F103
//#define USE_EK_STM32F
//#define USE_STM32_P103
//#define USE_MINI_STM32

#if defined USE_ET_STM32F103
 #define CARD_SUPPLY_SWITCHABLE   0
 #define SOCKET_WP_CONNECTED      1 /* write-protect socket-switch */
 #define SOCKET_CP_CONNECTED      0 /* card-present socket-switch (clashes with USART1 on this board) */
 #define GPIO_PORT_WP             GPIOC
 #define RCC_GPIO                 RCC_APB2ENR
 #define RCC_GPIO_PORT_WP         RCC_APB2ENR_IOPCEN
 #define GPIOWP                   GPIO6
 #define GPIO_MODE_WP             GPIO_CNF_INPUT_FLOAT /* external resistor */
 #define GPIO_PORT_CP             GPIOA
 #define RCC_GPIO_PORT_CP         RCC_APB2ENR_IOPAEN
 #define GPIOCP                   GPIO8
 #define GPIO_MODE_CP             GPIO_CNF_INPUT_FLOAT /* external resistor */
 #define GPIO_PORT_CS             GPIOA
 #define RCC_GPIO_PORT_CS         RCC_APB2ENR_IOPAEN
 #define GPIOCS                   GPIO4
 #define SPI_SD                   SPI1
 #define DMA_CHANNEL_SPI_SD_RX    DMA_CHANNEL2
 #define DMA_CHANNEL_SPI_SD_TX    DMA_CHANNEL3
 #define DMA_FLAG_SPI_SD_TC_RX    DMA_TCIF
 #define DMA_FLAG_SPI_SD_TC_TX    DMA_TCIF
 #define GPIO_PORT_SPI_SD         GPIOA
 #define GPIOSPI_SD_SCK           GPIO5
 #define GPIOSPI_SD_MISO          GPIO6
 #define GPIOSPI_SD_MOSI          GPIO7
 #define RCC_SPI                  RCC_APB2ENR
 #define RCC_SPI_SD               RCC_APB2ENR_SPI1EN
/* - for SPI1 and full-speed APB2: 72MHz/4 */
 #define SPI_BaudRatePrescaler_fast    SPI_CR1_BR_FPCLK_DIV_4
 #define SPI_BaudRatePrescaler_slow    SPI_CR1_BR_FPCLK_DIV_256

#elif defined USE_ET_STAMP_STM32
 #define CARD_SUPPLY_SWITCHABLE   0
 #define SOCKET_WP_CONNECTED      0 /* write-protect socket-switch */
 #define SOCKET_CP_CONNECTED      0 /* card-present socket-switch (clashes with USART1 on this board) */
 #define GPIO_PORT_WP             GPIOB
 #define RCC_GPIO                 RCC_APB2ENR
 #define RCC_GPIO_PORT_WP         RCC_APB2ENR_IOPBEN
 #define GPIOWP                   GPIO11
 #define GPIO_MODE_WP             GPIO_CNF_INPUT_FLOAT /* external resistor */
 #define GPIO_PORT_CP             GPIOB
 #define RCC_GPIO_PORT_CP         RCC_APB2ENR_IOPBEN
 #define GPIOCP                   GPIO10
 #define GPIO_MODE_CP             GPIO_CNF_INPUT_FLOAT /* external resistor */
 #define GPIO_PORT_CS             GPIOB
 #define RCC_GPIO_PORT_CS         RCC_APB2ENR_IOPBEN
 #define GPIOCS                   GPIO12
 #define SPI_SD                   SPI2
 #define DMA_CHANNEL_SPI_SD_RX    DMA_CHANNEL4
 #define DMA_CHANNEL_SPI_SD_TX    DMA_CHANNEL5
 #define DMA_FLAG_SPI_SD_TC_RX    DMA_TCIF
 #define DMA_FLAG_SPI_SD_TC_TX    DMA_TCIF
 #define GPIO_PORT_SPI_SD         GPIOB
 #define GPIOSPI_SD_SCK           GPIO13
 #define GPIOSPI_SD_MISO          GPIO14
 #define GPIOSPI_SD_MOSI          GPIO15
 #define RCC_SPI                  RCC_APB1ENR
 #define RCC_SPI_SD               RCC_APB1ENR_SPI2EN

/* - for SPI2 and full-speed APB1: 36MHz/2 */
 #define SPI_BaudRatePrescaler_fast    SPI_CR1_BR_FPCLK_DIV_4
 #define SPI_BaudRatePrescaler_slow    SPI_CR1_BR_FPCLK_DIV_256

#elif defined USE_EK_STM32F
 #define CARD_SUPPLY_SWITCHABLE   1
 #define GPIO_PWR                 GPIOD
 #define RCC_APB2Periph_GPIO_PWR  RCC_APB2Periph_GPIOD
 #define GPIOPWR                  GPIO10
 #define GPIO_Mode_PWR            GPIO_Mode_Out_OD /* pull-up resistor at power FET */
 #define SOCKET_WP_CONNECTED      0
 #define SOCKET_CP_CONNECTED      0
 #define SPI_SD                   SPI1
 #define GPIO_CS                  GPIOD
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOD
 #define GPIOCS                     GPIO9
 #define DMA_Channel_SPI_SD_RX    DMA1_Channel2
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel3
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_TCIF
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_TCIF
 #define GPIO_SPI_SD              GPIOA
 #define GPIOSPI_SD_SCK      GPIO5
 #define GPIOSPI_SD_MISO     GPIO6
 #define GPIOSPI_SD_MOSI     GPIO7
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB2PeriphClockCmd
 #define RCC_APBPeriph_SPI_SD     RCC_APB2Periph_SPI1
 /* - for SPI1 and full-speed APB2: 72MHz/4 */
 #define SPI_BaudRatePrescaler_fast  SPI_BaudRatePrescaler_4
 #define SPI_BaudRatePrescaler_slow  SPI_CR1_BR_FPCLK_DIV_256

#elif defined USE_STM32_P103
 // Olimex STM32-P103 not tested!
 #define CARD_SUPPLY_SWITCHABLE   0
 #define SOCKET_WP_CONNECTED      1 /* write-protect socket-switch */
 #define SOCKET_CP_CONNECTED      1 /* card-present socket-switch */
 #define GPIO_WP                  GPIOC
 #define GPIO_CP                  GPIOC
 #define RCC_APBxPeriph_GPIO_WP   RCC_APB2Periph_GPIOC
 #define RCC_APBxPeriph_GPIO_CP   RCC_APB2Periph_GPIOC
 #define GPIOWP                   GPIO6
 #define GPIOCP                   GPIO7
 #define GPIO_Mode_WP             GPIO_Mode_IN_FLOATING /* external resistor */
 #define GPIO_Mode_CP             GPIO_Mode_IN_FLOATING /* external resistor */
 #define SPI_SD                   SPI2
 #define GPIO_CS                  GPIOB
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOB
 #define GPIOCS                   GPIO12
 #define DMA_Channel_SPI_SD_RX    DMA1_Channel4
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel5
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_TCIF
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_TCIF
 #define GPIO_SPI_SD              GPIOB
 #define GPIOSPI_SD_SCK           GPIO13
 #define GPIOSPI_SD_MISO          GPIO14
 #define GPIOSPI_SD_MOSI          GPIO15
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB1PeriphClockCmd
 #define RCC_APBPeriph_SPI_SD     RCC_APB1Periph_SPI2
 /* for SPI2 and full-speed APB1: 36MHz/2 */
 /* !! PRESCALE 4 used here - 2 does not work, maybe because
       of the poor wiring on the HELI_V1 prototype hardware */
 #define SPI_BaudRatePrescaler_SPI_SD_fast  SPI_BaudRatePrescaler_4
 #define SPI_BaudRatePrescaler_slow  SPI_CR1_BR_FPCLK_DIV_256

#elif defined USE_MINI_STM32
 #define CARD_SUPPLY_SWITCHABLE   0
 #define SOCKET_WP_CONNECTED      0
 #define SOCKET_CP_CONNECTED      0
 #define SPI_SD                   SPI1
 #define GPIO_CS                  GPIOB
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOB
 #define GPIOCS              GPIO6
 #define DMA_Channel_SPI_SD_RX    DMA1_Channel2
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel3
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_TCIF
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_TCIF
 #define GPIO_SPI_SD              GPIOA
 #define GPIOSPI_SD_SCK           GPIO5
 #define GPIOSPI_SD_MISO          GPIO6
 #define GPIOSPI_SD_MOSI          GPIO7
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB2PeriphClockCmd
 #define RCC_APBPeriph_SPI_SD     RCC_APB2Periph_SPI1
 /* - for SPI1 and full-speed APB2: 72MHz/4 */
 #define SPI_BaudRatePrescaler_SPI_SD_fast  SPI_BaudRatePrescaler_4
 #define SPI_BaudRatePrescaler_slow  SPI_CR1_BR_FPCLK_DIV_256

#else
#error "unsupported board"
#endif

/* Manley EK-STM32F board does not offer socket contacts -> dummy values: */
#define SOCKPORT	1			/* Socket contact port */
#define SOCKWP		0			/* Write protect switch (PB5) */
#define SOCKINS		0			/* Card detect switch (PB4) */

#endif

