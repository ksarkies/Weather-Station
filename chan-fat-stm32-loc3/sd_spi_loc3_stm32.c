/*-----------------------------------------------------------------------*/
/* MMC/SDSC/SDHC (in SPI mode) control module for STM32 Version 1.1.6    */
/* (C) Martin Thomas, 2010 - based on the AVR MMC module (C)ChaN, 2007   */
/*                                                                       */
/* This is the libopencm3 version                                        */
/*-----------------------------------------------------------------------*/

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

/* Boolean type */
#include <stdbool.h>
typedef bool BOOL;
#ifndef FALSE
#define FALSE false
#define TRUE true
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include "ffconf.h"
#include "diskio.h"
#include "board.h"

#ifdef STM32_SD_USE_DMA
// #warning "Information only: using DMA"
#pragma message "*** Using DMA for MMC Card Access ***"
#endif

#ifdef USE_ET_STM32F103
// #warning "Information only: using ET_STM32F103"
#pragma message "*** Using ET_STM32F103 board ***"
#endif

/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD9	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
#define ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

/* Card-Select Controls  (Platform dependent) */
#define SELECT()        gpio_clear(GPIO_PORT_CS, GPIOCS)    /* MMC CS = L */
#define DESELECT()      gpio_set(GPIO_PORT_CS, GPIOCS)      /* MMC CS = H */

#if (_MAX_SS != 512) || (_FS_READONLY == 0) || (STM32_SD_DISK_IOCTRL_FORCE == 1)
#define STM32_SD_DISK_IOCTRL   1
#else
#define STM32_SD_DISK_IOCTRL   0
#endif

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC        0x01        /* MMC ver 3 */
#define CT_SD1        0x02        /* SD ver 1 */
#define CT_SD2        0x04        /* SD ver 2 */
#define CT_SDC        (CT_SD1|CT_SD2)    /* SD */
#define CT_BLOCK      0x08        /* Block addressing */

/* Temporary Debug Variables */

/*--------------------------------------------------------------------------

   Module Private Functions and Variables

---------------------------------------------------------------------------*/

static const DWORD socket_state_mask_cp = (1 << 0);
static const DWORD socket_state_mask_wp = (1 << 1);

static volatile DSTATUS diskStatus = STA_NOINIT;	/* Disk status */

/* These are updated in the Systick ISR (see callback disk_timerproc())  */
static volatile DWORD Timer1;               /* 100Hz decrement timers */

static BYTE CardType;			            /* Card type flags */

/*---------------------------------------------------------------------------*/
/** @brief Check for timeout

Read the ticking clock given by Timer1, accounting for word rollover, and
comparing with the given time to see if it has gone past.

@param[in] DWORD timer: Original Time for comparison.
@param[in] DWORD delay: Delay desired.

Globals Timer1: DWORD incremented in the systick ISR by 10ms.
*/

BOOL timeout(DWORD timer, DWORD delay)
{
    if (Timer1 > timer) return (Timer1-timer > delay);
    else if (timer+delay > timer) return false;
    else return (timer+delay > Timer1);
}

/*---------------------------------------------------------------------------*/
/** @brief Set the SPI Interface Speed

Sets the SPI baudrate prescaler value to either divide by 4 or divide by 256.

@param[in] speed_setting: INTERFACE_SLOW, INTERFACE_FAST

*/

enum speed_setting { INTERFACE_SLOW, INTERFACE_FAST };

static void interface_speed( enum speed_setting speed )
{
	if ( speed == INTERFACE_SLOW )
    {
		/* Set slow clock (100k-400k) */
		spi_set_baudrate_prescaler(SPI_SD,SPI_BaudRatePrescaler_slow);
	}
    else
    {
		/* Set fast clock (depends on the CSD) */
		spi_set_baudrate_prescaler(SPI_SD,SPI_BaudRatePrescaler_fast);
	}
}
/*---------------------------------------------------------------------------*/
/** @brief Initialise GPIO to Read Write Protect Pin

*/

#if SOCKET_WP_CONNECTED

static void socket_wp_init(void)
{
/* Configure I/O for write-protect */
    rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_GPIO_PORT_WP);
    gpio_set_mode(GPIO_PORT_WP,GPIO_MODE_INPUT,GPIO_CNF_INPUT_PULL_UPDOWN,GPIOWP);
    gpio_set(GPIO_PORT_WP,GPIOWP);
}

#else

static void socket_wp_init(void)
{
	return;
}

#endif

/*---------------------------------------------------------------------------*/
/** @brief Read the Write Protect Pin

Socket's Write-Protection Pin: high = write-protected, low = writable

@returns 16 bit mask for the socket state if protected, or zero if writeable. 
*/

#if SOCKET_WP_CONNECTED

static DWORD socket_is_write_protected(void)
{
	return gpio_get(GPIO_PORT_WP,GPIOWP) ? socket_state_mask_wp : 0;
}

#else

static inline DWORD socket_is_write_protected(void)
{
	return 0;   /* fake not protected */
}

#endif /* SOCKET_WP_CONNECTED */

/*---------------------------------------------------------------------------*/
/** @brief Initialise the Card Present Pin

*/

#if SOCKET_CP_CONNECTED

static void socket_cp_init(void)
{
	/* Configure I/O for card-present */
    rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_GPIO_PORT_CP);
    gpio_set_mode(GPIO_PORT_CP,GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT,GPIOCP);
}

#else

static void socket_cp_init(void)
{
	return;
}

#endif

/*---------------------------------------------------------------------------*/
/** @brief Read the Card Present Pin

Socket's Card-Present Pin: high = socket empty, low = card inserted

@returns 16 bit mask for the socket state if empty, or zero if inserted. 
*/

#if SOCKET_CP_CONNECTED

static inline DWORD socket_is_empty(void)
{
	return gpio_get(GPIO_PORT_CP,GPIOCP) ? socket_state_mask_cp : 0;
}

#else

static inline DWORD socket_is_empty(void)
{
	return 0;   /* fake inserted */
}

#endif /* SOCKET_CP_CONNECTED */

/*---------------------------------------------------------------------------*/
/** @brief Switch on the Socket Power

The power control pin is set low to turn on (high side PMOS).
Internal pullup is used but shouldn't be needed as circuit should have this.

@param[in] on: Boolean.
*/

#if CARD_SUPPLY_SWITCHABLE

static void card_power(BOOL on)
{
	/* Turn on GPIO for power-control pin connected to FET's gate */
	/* Configure I/O for Power FET */
    rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_GPIO_PWR);
    gpio_set_mode(GPIO_PWR,GPIO_MODE_OUTPUT,GPIO_CNF_OUTPUT_PULL_UPDOWN,GPIOPWR);
	if (on) {
		gpio_clear(GPIO_PWR,GPIOPWR);
	}
    else
    {
		gpio_set(GPIO_PWR,GPIOPWR);
	}
}

#else

static void card_power(BYTE on)
{
	on=on;
}

#endif /* CARD_SUPPLY_SWITCHABLE */

/*---------------------------------------------------------------------------*/
/** @brief Check if Socket Power is on

The power control pin is set low to turn on.

@returns int 1 (TRUE) if the power is on.
*/

#if CARD_SUPPLY_SWITCHABLE

static int chk_power(void)		/* Socket power state: 0=off, 1=on */
{
	if ( gpio_get(GPIO_PWR,GPIOPWR) > 0 ) {
		return 0;
	} else {
		return 1;
	}
}

#else

static int chk_power(void)
{
	return 1; /* fake powered */
}
#endif

/*---------------------------------------------------------------------------*/
/** @brief Transmit/Receive a byte to MMC via SPI

@param[in] out: BYTE value to send.
@returns BYTE value read back from the SPI interface.
*/

static BYTE stm32_spi_rw( BYTE out )
{
    return spi_xfer(SPI_SD,out);
}

#define xmit_spi(dat)  stm32_spi_rw(dat)

/*---------------------------------------------------------------------------*/
/** @brief Receive a byte from MMC via SPI

Sends a byte 0xFF and picks up whatever returns.

@returns BYTE value received.
*/

static BYTE rcvr_spi(void)
{
	return stm32_spi_rw(0xff);
}

/* Alternative macro to receive data fast */
#define rcvr_spi_m(dst)  *(dst)=stm32_spi_rw(0xff)

/*---------------------------------------------------------------------------*/
/** @brief Wait for the Card to become Ready

A card that is ready will return 0xFF in response to the same value sent to it,
that is, it is no longer sending any data. Keep trying for 50 timer ticks.

@returns bool value of a status result: TRUE = success, otherwise timeout.
*/

static bool wait_ready(void)
{
    bool ready;

    DWORD timer = Timer1;           /* Wait for ready timeout of 500ms */
    rcvr_spi();                     /* Initial transmission */
    do
    {
        ready = (rcvr_spi() == 0xFF);      /* Get back the response */
    }
    while (!ready && !timeout(timer,50));
    return ready;
}

/*---------------------------------------------------------------------------*/
/** @brief Deselect the card and release SPI bus

*/

static void release_spi(void)
{
	DESELECT();
	rcvr_spi();
}

#ifdef STM32_SD_USE_DMA
/*---------------------------------------------------------------------------*/
/** @brief Transmit/Receive Block using DMA

@param[in] receive: Boolean false for sending to SPI, true for reading
@param[in] *buff: Pointer to buffer of BYTE
@param[in] btr: UINT byte count (multiple of 2 for send, 512 always for receive)
*/

static void stm32_dma_transfer(BOOL receive, const BYTE *buff, UINT btr)
{
	WORD rw_workbyte[] = { 0xffff };

/* Enable DMA1 Clock */
	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

/* Reset DMA channels*/
	dma_channel_reset(DMA1, DMA_CHANNEL_SPI_SD_RX);
	dma_channel_reset(DMA1, DMA_CHANNEL_SPI_SD_TX);

/* DMA1 read channel2 configuration SPI1 RX ---------------------------------------------*/
    dma_channel_reset(DMA1,DMA_CHANNEL_SPI_SD_RX);
    dma_set_peripheral_address(DMA1,DMA_CHANNEL_SPI_SD_RX,(DWORD) &SPI_DR(SPI_SD));
    dma_set_peripheral_size(DMA1,DMA_CHANNEL_SPI_SD_RX,DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1,DMA_CHANNEL_SPI_SD_RX,DMA_CCR_MSIZE_8BIT);
    dma_disable_peripheral_increment_mode(DMA1,DMA_CHANNEL_SPI_SD_RX);
    dma_set_number_of_data(DMA1,DMA_CHANNEL_SPI_SD_RX,btr);
    dma_set_priority(DMA1,DMA_CHANNEL_SPI_SD_RX,DMA_CCR_PL_VERY_HIGH);

/* DMA1 write channel3 configuration SPI1 TX ---------------------------------------------*/
    dma_channel_reset(DMA1,DMA_CHANNEL_SPI_SD_TX);
    dma_set_peripheral_address(DMA1,DMA_CHANNEL_SPI_SD_TX,(DWORD) &SPI_DR(SPI_SD));
    dma_set_peripheral_size(DMA1,DMA_CHANNEL_SPI_SD_TX,DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1,DMA_CHANNEL_SPI_SD_TX,DMA_CCR_MSIZE_8BIT);
    dma_disable_peripheral_increment_mode(DMA1,DMA_CHANNEL_SPI_SD_TX);
    dma_set_number_of_data(DMA1,DMA_CHANNEL_SPI_SD_TX,btr);
    dma_set_priority(DMA1,DMA_CHANNEL_SPI_SD_TX,DMA_CCR_PL_VERY_HIGH);

	if ( receive )
    {
/* DMA1 read channel configuration SPI1 RX ---------------------------------------------*/
    	dma_set_memory_address(DMA1,DMA_CHANNEL_SPI_SD_RX,(DWORD)buff);
    	dma_set_read_from_peripheral(DMA1,DMA_CHANNEL_SPI_SD_RX);
    	dma_enable_memory_increment_mode(DMA1,DMA_CHANNEL_SPI_SD_RX);

/* DMA1 write channel configuration SPI1 TX ---------------------------------------------*/
		dma_set_memory_address(DMA1,DMA_CHANNEL_SPI_SD_TX,(DWORD)rw_workbyte);
    	dma_set_read_from_memory(DMA1,DMA_CHANNEL_SPI_SD_TX);
    	dma_disable_memory_increment_mode(DMA1,DMA_CHANNEL_SPI_SD_TX);
	}
    else
    {
#if _FS_READONLY == 0
/* DMA1 read channel configuration SPI1 RX ---------------------------------------------*/
    	dma_set_memory_address(DMA1,DMA_CHANNEL_SPI_SD_RX,(DWORD)rw_workbyte);
    	dma_set_read_from_peripheral(DMA1,DMA_CHANNEL_SPI_SD_RX);
    	dma_disable_memory_increment_mode(DMA1,DMA_CHANNEL_SPI_SD_RX);

/* DMA1 write channel configuration SPI1 TX ---------------------------------------------*/
		dma_set_memory_address(DMA1,DMA_CHANNEL_SPI_SD_TX,(DWORD)buff);
    	dma_set_read_from_memory(DMA1,DMA_CHANNEL_SPI_SD_TX);
    	dma_enable_memory_increment_mode(DMA1,DMA_CHANNEL_SPI_SD_TX);
#endif
	}

/* Enable DMA Channels */
	dma_enable_channel(DMA1,DMA_CHANNEL_SPI_SD_RX);
	dma_enable_channel(DMA1,DMA_CHANNEL_SPI_SD_TX);

/* Enable SPI TX/RX requests */
    spi_enable_rx_dma(SPI_SD);
    spi_enable_tx_dma(SPI_SD);

/* Wait until DMA1_CHANNEL 2 Receive Complete */
    while (! dma_get_interrupt_flag(DMA1,DMA_CHANNEL_SPI_SD_RX,DMA_TCIF));

/* Disable DMA Channels */
    dma_disable_channel(DMA1,DMA_CHANNEL_SPI_SD_RX);
    dma_disable_channel(DMA1,DMA_CHANNEL_SPI_SD_TX);

/* Disable SPI RX/TX requests */
    spi_disable_rx_dma(SPI_SD);
    spi_disable_tx_dma(SPI_SD);
}
#endif /* STM32_SD_USE_DMA */

/*---------------------------------------------------------------------------*/
/** @brief Power Control and Interface-Initialization

All peripherals are initialised and the power is turned on to the card.
*/

static void power_on(void)
{
/* Enable GPIO clock for CS */
	rcc_peripheral_enable_clock(&RCC_GPIO, RCC_GPIO_PORT_CS);
/* Enable SPI clock, SPI1: APB2, SPI2: APB1 */
	rcc_peripheral_enable_clock(&RCC_SPI, RCC_SPI_SD);

	card_power(1);
	socket_cp_init();
	socket_wp_init();

    DWORD timer = Timer1;
    while (!timeout(timer,25));            /* Wait for 250ms */

/* Configure I/O for Card Chip select */
    gpio_set_mode(GPIO_PORT_CS, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			    GPIOCS | GPIOSPI_SD_MISO);

/* De-select the Card: Chip Select high */
	DESELECT();

/* Configure SPI pins: SCK and MOSI with default alternate function (not re-mapped) push-pull */
    gpio_set_mode(GPIO_PORT_SPI_SD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
			    GPIOSPI_SD_SCK | GPIOSPI_SD_MOSI);
/* Configure MISO as Input with internal pull-up */
    gpio_set_mode(GPIO_PORT_SPI_SD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
			    GPIOSPI_SD_MISO);

/* Set the pull-up/pull-down resistors to pull-up for the MISO input
in case board has no external resistor*/
    gpio_set(GPIO_PORT_SPI_SD,GPIOSPI_SD_MISO);

/* Clear all SPI and associated RCC registers */
	spi_reset(SPI_SD);

/* SPI configuration */
	spi_init_master(SPI_SD, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_set_full_duplex_mode(SPI_SD);
/*
Set NSS management to software.

Note:
Setting nss high is very important, even if we are controlling the GPIO
ourselves this bit needs to be at least set to 1, otherwise the spi
peripheral will not send any data out.
*/
	spi_enable_software_slave_management(SPI_SD);
	spi_set_nss_high(SPI_SD);
    spi_disable_crc(SPI_SD);

	spi_enable(SPI_SD);

    SELECT();

/* drain SPI */
	while (!(SPI_SR(SPI_SD) & SPI_SR_TXE));
//	dummyread = SPI_DR(SPI_SD);

#ifdef STM32_SD_USE_DMA
	/* enable DMA clock */
	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
#endif
}
/*---------------------------------------------------------------------------*/
/** @brief Power Off and Peripheral Disable

All peripherals are disabled and the power is turned off to the card.

Globals diskStatus: DSTATUS disk status.
*/

static void power_off(void)
{
	if (!(diskStatus & STA_NOINIT)) {
		SELECT();
		wait_ready();
		release_spi();
	}

	spi_disable(SPI_SD);
	rcc_peripheral_disable_clock(&RCC_SPI, RCC_SPI_SD);

	/* All SPI-Pins to input with weak internal pull-downs */
    gpio_set_mode(GPIO_PORT_SPI_SD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
			    GPIOSPI_SD_SCK | GPIOSPI_SD_MISO | GPIOSPI_SD_MOSI);

	card_power(0);

	diskStatus |= STA_NOINIT;		/* Set STA_NOINIT */
}

/*---------------------------------------------------------------------------*/
/** @brief Receive a Data Block from the Card

Can be done with DMA or programmed.
This is in response to a previously sent read command.

@param *buff: BYTE 512 byte data block to store received data 
@param numBytes: UINT Byte count (must be multiple of 4)
@returns BOOL valid token received.
*/

static BOOL rcvr_datablock(BYTE *buff,UINT numBytes)
{
	BYTE token;

    DWORD timer = Timer1;
/* Wait for a valid data packet (token), timeout of 200ms */
	do
    {							/* Wait for data packet in timeout of 100ms */
		token = rcvr_spi();
	}
    while ((token == 0xFF) && !timeout(timer,20));
	if(token != 0xFE) return FALSE;	/* If not valid data token, return with error */

#ifdef STM32_SD_USE_DMA
	stm32_dma_transfer( TRUE, buff, numBytes );
#else
	do
    {							/* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	}
    while (numBytes -= 4);
#endif /* STM32_SD_USE_DMA */

	rcvr_spi();						/* Discard CRC */
	rcvr_spi();

	return TRUE;					/* Return with success */
}

/*---------------------------------------------------------------------------*/
/** @brief Send a data packet to MMC

Only compiled if the filesystem is writeable.

@param *buff: BYTE 512 byte data block to be transmitted
@param token: BYTE Data/Stop token
@returns BOOL system available and packet accepted.
*/

#if _FS_READONLY == 0
static BOOL xmit_datablock(const BYTE *buff,BYTE token)
{
	BYTE response;
#ifndef STM32_SD_USE_DMA
	BYTE wc;
#endif

    if (!wait_ready()) return FALSE;

	xmit_spi(token);					/* transmit data token */
	if (token != 0xFD)
    {	                                /* Is data token */

#ifdef STM32_SD_USE_DMA
		stm32_dma_transfer( FALSE, buff, 512 );
#else
		wc = 0;
		do
        {							/* transmit the 512 byte data block to MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		}
        while (--wc);
#endif /* STM32_SD_USE_DMA */

		xmit_spi(0xFF);					/* CRC (Dummy) */
		xmit_spi(0xFF);
		response = rcvr_spi();			/* Receive data response */
		if ((response & 0x1F) != 0x05)	/* If not accepted, return with error */
        {
			return FALSE;
        }
	}

	return TRUE;
}
#endif /* _READONLY */

/*---------------------------------------------------------------------------*/
/** @brief Send a command packet to MMC

@param cmd: BYTE Command byte
@param arg: DWORD argument for command
@returns BYTE response
*/

static BYTE send_cmd(BYTE cmd,DWORD arg)
{
	BYTE n, response;

/* ACMD<n> is the command sequence of CMD55-CMD<n> */
    if (cmd & 0x80)
    {
        cmd &= 0x7F;
        response = send_cmd(CMD55, 0);
        if (response > 1) return response;
    }

/* Select the card and wait for ready */
    DESELECT();
    SELECT();
    if (!wait_ready()) return 0xFF;

/* Send command packet */
    xmit_spi(cmd);                      /* Start + Command index */
    xmit_spi((BYTE)(arg >> 24));        /* Argument[31..24] */
    xmit_spi((BYTE)(arg >> 16));        /* Argument[23..16] */
    xmit_spi((BYTE)(arg >> 8));         /* Argument[15..8] */
    xmit_spi((BYTE)arg);                /* Argument[7..0] */
    n = 0x01;                           /* Dummy CRC + Stop */
    if (cmd == CMD0) n = 0x95;          /* Valid CRC for CMD0(0) */
    if (cmd == CMD8) n = 0x87;          /* Valid CRC for CMD8(0x1AA) */
    xmit_spi(n);

/* Receive command response */
    if (cmd == CMD12) rcvr_spi();       /* Skip a stuff byte when stopping read */

/* Wait for a valid response in timeout of 10 attempts */
	n = 10;
	do
    {
		response = rcvr_spi();
    }
	while ((response & 0x80) && --n);

	return response;			        /* Return with the response value */
}

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** @brief Initialize Disk Drive

Power on the drive and send some commands to find steady state and determine
the type of card (SDSC or MMC). If not successful, power off and return a
STA_NOINIT status.

@param[in] drv: BYTE Physical drive number (only 0 allowed here)
@returns DSTATUS drive initialized/present/read only status.

Globals Timer1: DWORD incremented in the systick ISR by (configurable) 1ms.
        diskStatus: DSTATUS disk status. Bit 0=STA_NOINIT, 1=STA_NODISK, 2=STA_PROTECT
*/

DSTATUS disk_initialize(BYTE drv)
{
	BYTE n, cmd, ty, ocr[4];

	if (drv > 0) return STA_NOINIT;			    /* Supports only single drive */
	if (diskStatus & STA_NODISK) return diskStatus;	/* No card in the socket */

/* Force socket power on and initialize interface */
	power_on();

	interface_speed(INTERFACE_SLOW);
	for (n = 10; n; n--) rcvr_spi();	        /* 80 dummy clocks */

	ty = 0;
/* Enter Idle state */
	if (send_cmd(CMD0, 0) == 1)
    {
/* Initialization timeout of 1000 milliseconds */
        DWORD timer = Timer1;
/* SDHC */
		if (send_cmd(CMD8, 0x1AA) == 1)
        {
/* Get trailing return value of R7 response */
			for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
/* The card can work at VDD range of 2.7-3.6V */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            {
/* Wait until leaving idle state (ACMD41 with HCS bit) */
				while (!timeout(timer,100) && send_cmd(ACMD41, 1UL << 30));
				if (!timeout(timer,100) && send_cmd(CMD58, 0) == 0)
                {
/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
				}
			}
		}
/* Determine if SDSC or MMC */
        else
        {
			if (send_cmd(ACMD41, 0) <= 1) 	
            {
				ty = CT_SD1; cmd = ACMD41;	/* SDSC */
			}
            else
            {
				ty = CT_MMC; cmd = CMD1;	/* MMC */
			}
/* Wait until leaving idle state */
			while (!timeout(timer,100) && send_cmd(cmd, 0));
/* Set R/W block length to 512 */
			if (timeout(timer,100) || send_cmd(CMD16, 512) != 0)
            {
				ty = 0;
            }
		}
	}
	CardType = ty;
	release_spi();

	if (ty > 0)
    {			                            /* Initialization succeeded */
		diskStatus &= ~STA_NOINIT;		    /* Clear STA_NOINIT */
		interface_speed(INTERFACE_FAST);
	}
    else
    {			                            /* Initialization failed */
		power_off();
	}
	return diskStatus;
}

/*---------------------------------------------------------------------------*/
/** @brief Get Disk Status

@param[in] drv: BYTE Physical drive number (only 0 allowed)
@returns DSTATUS

Globals diskStatus: DSTATUS disk status.
*/

DSTATUS disk_status(BYTE drv)
{
	if (drv > 0) return STA_NOINIT;		/* Supports only single drive */
	return diskStatus;
}

/*---------------------------------------------------------------------------*/
/** @brief Read Disk Sectors

@param[in] drv: BYTE Physical drive number (only 0 allowed)
@param[in] *buff: BYTE Pointer to buffer
@param[in] sector: DWORD starting sector number
@param[in] count: BYTE number of sectors to read
@returns DRESULT success (RES_OK) or fail.

Globals diskStatus: DSTATUS disk status.
*/

DRESULT disk_read(BYTE drv, BYTE* buff, DWORD sector, UINT count)
{
    BYTE error = 0;             /* Error status for fine tracking */
    DRESULT res = RES_OK;
    if (drv || !count)          /* Zero count or non-zero drive number */
    {
        res = RES_PARERR;
    }
    else if (diskStatus & STA_NOINIT) /* Drive not initialized */
    {
        res = RES_NOTRDY;
    }
    if (res == RES_OK)
    {
/* Convert to byte address if needed */
    	if (!(CardType & CT_BLOCK)) sector *= 512;
/* Single block read */
	    if (count == 1)
        {
/* If the response to the command is non-zero, the block is not read */
		    if ((error = send_cmd(CMD17, sector)) == 0)
            {
			    if (rcvr_datablock(buff, 512))
                {
				    count = 0;
			    }
		    }
	    }
/* Multiple block read */
	    else
        {
/* If the response to the command is non-zero, the block is not read */
		    if ((error = send_cmd(CMD18, sector)) == 0)
            {
			    do
                {
				    if (!rcvr_datablock(buff, 512)) break;
				    buff += 512;
			    }
                while (--count);
			    send_cmd(CMD12, 0); /* Stop transmission */
		    }
	    }
	    release_spi();
        if (count > 0) res = RES_ERROR;
    }

    return res;
}

/*---------------------------------------------------------------------------*/
/** @brief Write Disk Sectors

Only compiled if disk is not read only.

@param[in] drv: BYTE Physical drive number (only 0 allowed)
@param[in] *buff: BYTE Pointer to buffer
@param[in] sector: DWORD starting sector number
@param[in] count: BYTE number of sectors to read
@returns DRESULT success (RES_OK) or fail.

Globals diskStatus: DSTATUS disk status.
*/

#if _FS_READONLY == 0

DRESULT disk_write(BYTE drv, const BYTE* buff, DWORD sector, UINT count)
{
    BYTE error = 0;             /* Error status for fine tracking */
    DRESULT res = RES_OK;
    if (drv || !count)          /* Zero count or non-zero drive number */
    {
        res = RES_PARERR;
    }
    else if (diskStatus & STA_NOINIT) /* Drive not initialized */
    {
        res = RES_NOTRDY;
    }
    else if (diskStatus & STA_PROTECT)/* Write protected */
    {
        res = RES_WRPRT;
    }
    if (res == RES_OK)
    {
/* Convert to byte address if needed */
        if (!(CardType & CT_BLOCK)) sector *= 512;
/* Single block write */
        if (count == 1)
        {
/* If the response to the command is non-zero, the block is not written */
            if ((error = send_cmd(CMD24, sector)) == 0)
            {
                if (xmit_datablock(buff, 0xFE))
                {
                    count = 0;
                }
            }
        }
/* Multiple block write */
        else
        {
            if (CardType & CT_SDC) send_cmd(ACMD23, count);
/* If the response to the command is non-zero, the block is not written */
            if ((error = send_cmd(CMD25, sector)) == 0)
            {
                do
                {
                    if (!xmit_datablock(buff, 0xFC)) break;
                    buff += 512;
                }
                while (--count);
                if (!xmit_datablock(0, 0xFD))    /* STOP_TRAN token */
                    count = 1;
            }
        }
        release_spi();
        if (count > 0) res = RES_ERROR;
    }

    return res;
}
#endif /* _READONLY == 0 */

/*---------------------------------------------------------------------------*/
/** @brief Disk I/O Control

@param[in] drv: BYTE Physical drive number (only 0 allowed)
@param[in] ctrl: BYTE Control Code.
                    CTRL_POWER (0=on,1=off,2=get setting)
                    CTRL_SYNC wait for writes to complete
                    GET_SECTOR_COUNT number of sectors on disk
                    GET_SECTOR_SIZE
                    GET_BLOCK_SIZE
                    MMC_GET_TYPE
                    MMC_GET_CSD     ???
                    MMC_GET_CID     Card ID
                    MMC_GET_OCR     Operating Conditions Register
                    MMC_GET_SDSTAT  Card status
@param[in] *buff: BYTE Pointer to buffer
@returns DRESULT success (RES_OK) or fail (RES_ERROR).

Globals diskStatus: DSTATUS disk status.
*/

#if (STM32_SD_DISK_IOCTRL == 1)
DRESULT disk_ioctl(BYTE drv,BYTE ctrl,void *buff)
{
    DRESULT res = RES_ERROR;
    BYTE n, csd[16], *ptr = buff;
    WORD csize;

    if (drv)
    {
        res = RES_PARERR;
    }
    else
    {
        if (ctrl == CTRL_POWER)
        {
            switch (*ptr)
            {
/* Sub control code == 0 (POWER_OFF) */
            case 0:
                if (chk_power())
                    power_off();            /* Power off */
                res = RES_OK;
                break;
/* Sub control code == 1 (POWER_ON) */
            case 1:
                power_on();                 /* Power on */
                res = RES_OK;
                break;
/* Sub control code == 2 (POWER_GET) */
            case 2:
                *(ptr+1) = (BYTE)chk_power();
                res = RES_OK;
                break;
            default :
                res = RES_PARERR;
            }
        }
        else
        {
            if (diskStatus & STA_NOINIT) res = RES_NOTRDY;
            else switch (ctrl)
            {
/* Make sure no pending write process */
            case CTRL_SYNC :
                SELECT();
                if (wait_ready()) res = RES_OK;
                break;
/* Get number of sectors on the disk (DWORD) */
            case GET_SECTOR_COUNT :
                if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16))
                {
/* SDC version 2.00 */
                    if ((csd[0] >> 6) == 1)
                    {
                        csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                        *(DWORD*)buff = (DWORD)csize << 10;
                    }
/* SDC version 1.XX or MMC*/
                    else
                    {
                        n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                        csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                        *(DWORD*)buff = (DWORD)csize << (n - 9);
                    }
                    res = RES_OK;
                }
                break;
/* Get R/W sector size (WORD) */
            case GET_SECTOR_SIZE :
                *(WORD*)buff = 512;
                res = RES_OK;
                break;
/* Get erase block size in unit of sector (DWORD) */
            case GET_BLOCK_SIZE :
                if (CardType & CT_SD2)
                {                                   /* SDC version 2.00 */
                    if (send_cmd(ACMD13, 0) == 0)
                    {                               /* Read SD status */
                        rcvr_spi();
                        if (rcvr_datablock(csd, 16))
                        {                           /* Read partial block */
/* Purge trailing data */
                            for (n = 64 - 16; n; n--) rcvr_spi();
                            *(DWORD*)buff = 16UL << (csd[10] >> 4);
                            res = RES_OK;
                        }
                    }
                }
                else
                {                                   /* SDC version 1.XX or MMC */
                    if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16))
                    {                               /* Read CSD */
                        if (CardType & CT_SD1)
                        {                           /* SDC version 1.XX */
                            *(DWORD*)buff = (((csd[10] & 63) << 1) + \
                                ((WORD)(csd[11] & 128) >> 7) + 1) \
                                << ((csd[13] >> 6) - 1);
                        }
                        else
                        {                           /* MMC */
                            *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) \
                                * (((csd[11] & 3) << 3) + \
                                ((csd[11] & 224) >> 5) + 1);
                        }
                        res = RES_OK;
                    }
                }
                break;
/* Get card type flags (1 byte) */
            case MMC_GET_TYPE :
                *ptr = CardType;
                res = RES_OK;
                break;
/* Receive CSD as a data block (16 bytes) */
            case MMC_GET_CSD :
                if (send_cmd(CMD9, 0) == 0          /* READ_CSD */
                    && rcvr_datablock(ptr, 16))
                    res = RES_OK;
                break;
/* Receive CID as a data block (16 bytes) */
            case MMC_GET_CID :
                if (send_cmd(CMD10, 0) == 0         /* READ_CID */
                    && rcvr_datablock(ptr, 16))
                    res = RES_OK;
                break;
/* Receive OCR as an R3 resp (4 bytes) */
            case MMC_GET_OCR :
                if (send_cmd(CMD58, 0) == 0)
                {                                   /* READ_OCR */
                    for (n = 4; n; n--) *ptr++ = rcvr_spi();
                    res = RES_OK;
                }
                break;
/* Receive SD status as a data block (64 bytes) */
            case MMC_GET_SDSTAT :
                if (send_cmd(ACMD13, 0) == 0)
                {                                   /* SD_STATUS */
                    rcvr_spi();
                    if (rcvr_datablock(ptr, 64))
                        res = RES_OK;
                }
                break;

            default:
                res = RES_PARERR;
            }
        }

        release_spi();
    }

    return res;
}
#endif /* _USE_IOCTL != 0 */

/*---------------------------------------------------------------------------*/
/** @brief Device Timer Interrupt Procedure

This function must be called in period of 10ms, generally by the systick ISR.
It counts up a timer and checks for write protect and card presence.

Globals diskStatus: DSTATUS disk status.
        Timer1: DWORD a real-time variable used to trigger events.
Statics pv: DWORD previous value of socket status.
*/

void disk_timerproc(void)
{
	static DWORD current;       /* last reading of socket status */
	DWORD previous;
	BYTE status;

    Timer1++;

	previous = current;
/* Sample the socket switch to check empty or write protect */
	current = socket_is_empty() | socket_is_write_protected();

    if (previous == current)    /* Debounce: no change since last reading */
    {                           /* Have contacts stabled? */
		status = diskStatus;

		if (current & socket_state_mask_wp)     /* WP is H (write protected) */
			status |= STA_PROTECT;
		else                                    /* WP is L (write enabled) */
			status &= ~STA_PROTECT;

		if (current & socket_state_mask_cp)     /* INS = H (Socket empty) */
			status |= (STA_NODISK | STA_NOINIT);
		else                                    /* INS = L (Card inserted) */
			status &= ~STA_NODISK;

/* diskStatus is a global status for card present, inserted and write protected */
		diskStatus = status;
	}
}

