/* rgb-led-matrix-bricklet
 * Copyright (C) 2017 Olaf Lüke <olaf@tinkerforge.com>
 *
 * matrix.c: WS2812B 8x8 Matrix driver
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "matrix.h"

#include "xmc_spi.h"
#include "xmc_gpio.h"

#include "configs/config_matrix.h"

#include "bricklib2/hal/system_timer/system_timer.h"

#include <string.h>

#define MATRIX_SPI_FREQUENCY 6400000
#define matrix_tx_irq_handler IRQ_Hdlr_12

extern Matrix matrix;

                                       // @6.4MHz
#define MATRIX_HIGH_PATTERN 0b11000000 // 0.25us  + 0.75us
#define MATRIX_LOW_PATTERN  0b11111000 // 0.625us + 0.375us

void __attribute__((optimize("-O3"))) matrix_tx_irq_handler(void) {
	while(!XMC_USIC_CH_TXFIFO_IsFull(MATRIX_USIC)) {
		MATRIX_USIC->IN[0] = matrix.buffer_out[matrix.frame_current_index];
		matrix.frame_current_index++;
		if(matrix.frame_current_index == MATRIX_STUFFED_SIZE) {
			matrix.frame_number++;
			XMC_USIC_CH_TXFIFO_DisableEvent(MATRIX_USIC, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
			return;
		}
	}
}

void matrix_draw_frame(Matrix *matrix) {
	uint32_t buffer_out_counter = 0;
	for(uint32_t i = 0; i < MATRIX_SIZE; i++) {
		for(uint8_t j = 0; j < MATRIX_CHANNELS; i++) {
			uint8_t byte = 0;
			switch(j) {
				case 0: byte = matrix->buffer_in.r[i]; break;
				case 1: byte = matrix->buffer_in.g[i]; break;
				case 2: byte = matrix->buffer_in.b[i]; break;
			}

			for(uint8_t k = 0; k < MATRIX_STUFFED_BITS_PER_BIT; k++) {
				matrix->buffer_out[buffer_out_counter] = (byte & (1 << k)) ? MATRIX_HIGH_PATTERN : MATRIX_LOW_PATTERN;
				buffer_out_counter++;
			}
		}
	}

	XMC_USIC_CH_TXFIFO_EnableEvent(MATRIX_USIC, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
	XMC_USIC_CH_TriggerServiceRequest(MATRIX_USIC, MATRIX_SERVICE_REQUEST_TX);
}

void matrix_init(Matrix *matrix) {
	memset(matrix, 0, sizeof(Matrix));

	// USIC channel configuration
	const XMC_SPI_CH_CONFIG_t channel_config = {
		.baudrate       = MATRIX_SPI_FREQUENCY,
		.bus_mode       = XMC_SPI_CH_BUS_MODE_MASTER,
		.selo_inversion = XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS,
		.parity_mode    = XMC_USIC_CH_PARITY_MODE_NONE
	};

	// MOSI pin configuration
	const XMC_GPIO_CONFIG_t mosi_pin_config = {
		.mode             = MATRIX_MOSI_PIN_AF,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};

	// Initialize USIC channel in SPI master mode
	XMC_SPI_CH_Init(MATRIX_USIC, &channel_config);
	MATRIX_USIC->SCTR &= ~USIC_CH_SCTR_PDL_Msk; // Set passive data level to 0

	XMC_SPI_CH_SetBitOrderMsbFirst(MATRIX_USIC);

	XMC_SPI_CH_SetWordLength(MATRIX_USIC, (uint8_t)8U);
	XMC_SPI_CH_SetFrameLength(MATRIX_USIC, (uint8_t)64U);

	XMC_SPI_CH_SetTransmitMode(MATRIX_USIC, XMC_SPI_CH_MODE_STANDARD);

	// SPI Mode: CPOL=1 and CPHA=1
	MATRIX_USIC_CHANNEL->DX1CR |= USIC_CH_DX1CR_DPOL_Msk;

	// Configure transmit FIFO
	XMC_USIC_CH_TXFIFO_Configure(MATRIX_USIC, 32, XMC_USIC_CH_FIFO_SIZE_32WORDS, 16);

	// Set service request for tx FIFO transmit interrupt
	XMC_USIC_CH_TXFIFO_SetInterruptNodePointer(MATRIX_USIC, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, MATRIX_SERVICE_REQUEST_TX);  // IRQ MATRIX_IRQ_TX

	// Set priority and enable NVIC node for transmit interrupt
	NVIC_SetPriority((IRQn_Type)MATRIX_IRQ_TX, MATRIX_IRQ_TX_PRIORITY);
	NVIC_EnableIRQ((IRQn_Type)MATRIX_IRQ_TX);

	// Start SPI
	XMC_SPI_CH_Start(MATRIX_USIC);

	// Configure MOSI pin
	XMC_GPIO_Init(MATRIX_MOSI_PIN, &mosi_pin_config);
}

void matrix_tick(Matrix *matrix) {
	if(matrix->frame_duration != 0) {
		if(system_timer_is_time_elapsed_ms(matrix->frame_last_time, matrix->frame_duration)) {
			matrix_draw_frame(matrix);
			matrix->frame_started = true;
			matrix->frame_last_time = system_timer_get_ms();
		}
	}
}
