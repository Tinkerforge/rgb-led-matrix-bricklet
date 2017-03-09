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
#include "xmc_vadc.h"

#include "configs/config_matrix.h"
#include "configs/config.h"

#include "bricklib2/hal/system_timer/system_timer.h"

#include <string.h>

#define MATRIX_SPI_FREQUENCY 6400000
#define MATRIX_HIGH_PATTERN 0b11111100
#define MATRIX_LOW_PATTERN  0b11000000

#define matrix_tx_irq_handler IRQ_Hdlr_12

extern Matrix matrix;
void __attribute__((optimize("-O3"))) matrix_tx_irq_handler(void) {
	while(!XMC_USIC_CH_TXFIFO_IsFull(MATRIX_USIC)) {
		MATRIX_USIC->IN[0] = matrix.buffer_out[matrix.frame_current_index];
		matrix.frame_current_index++;
		if(matrix.frame_current_index >= MATRIX_STUFFED_SIZE) {
			matrix.frame_current_index = 0;
			matrix.frame_number++;
			XMC_USIC_CH_TXFIFO_DisableEvent(MATRIX_USIC, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
			return;
		}
	}
}

void matrix_draw_frame(Matrix *matrix) {
	uint32_t buffer_out_counter = 0;
	for(uint32_t i = 0; i < MATRIX_SIZE; i++) {
		for(uint32_t j = 0; j < MATRIX_CHANNELS; j++) {
			uint8_t byte = 0;
			switch(j) {
				case 0: byte = matrix->buffer_in.g[i]; break;
				case 1: byte = matrix->buffer_in.r[i]; break;
				case 2: byte = matrix->buffer_in.b[i]; break;
			}

			for(int32_t k = MATRIX_STUFFED_BITS_PER_BIT-1; k >= 0 ; k--) {
				matrix->buffer_out[buffer_out_counter] = (byte & (1 << k)) ? MATRIX_HIGH_PATTERN : MATRIX_LOW_PATTERN;
				buffer_out_counter++;
			}
		}
	}

	XMC_USIC_CH_TXFIFO_EnableEvent(MATRIX_USIC, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
	XMC_USIC_CH_TriggerServiceRequest(MATRIX_USIC, MATRIX_SERVICE_REQUEST_TX);
}

void matrix_measure_voltage(Matrix *matrix) {
	static uint32_t last_time = 0;
	if(system_timer_is_time_elapsed_ms(last_time, 250)) {
		last_time = system_timer_get_ms();
		uint32_t result = XMC_VADC_GLOBAL_GetDetailedResult(VADC);
		if(result & (1 << 31)) {
			// Resistor divisor is 1k to 1k so we have to go from 0-4095 to 0-3.3V with a multiplier of (1+1)/1 = 2
			matrix->voltage = (result & 0xFFFF)*3300*2/4095;
		}
	}

}

void matrix_init_adc(Matrix *matrix) {
	// This structure contains the Global related Configuration.
	const XMC_VADC_GLOBAL_CONFIG_t adc_global_config = {
		.boundary0 = (uint32_t) 0, // Lower boundary value for Normal comparison mode
		.boundary1 = (uint32_t) 0, // Upper boundary value for Normal comparison mode

		.class0 = {
			.sample_time_std_conv     = 31,                      // The Sample time is (2*tadci)
			.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT, // 12bit conversion Selected

		},
		.class1 = {
			.sample_time_std_conv     = 31,                      // The Sample time is (2*tadci)
			.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT, // 12bit conversion Selected

		},

		.data_reduction_control         = 0, // Data Reduction disabled
		.wait_for_read_mode             = 0, // GLOBRES Register will not be overwritten until the previous value is read
		.event_gen_enable               = 0, // Result Event from GLOBRES is disabled
		.disable_sleep_mode_control     = 0  // Sleep mode is enabled
	};


	// Global iclass0 configuration
	const XMC_VADC_GLOBAL_CLASS_t adc_global_iclass_config = {
		.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
		.sample_time_std_conv	  = 31,
	};

	// Global Result Register configuration structure
	XMC_VADC_RESULT_CONFIG_t adc_global_result_config = {
		.data_reduction_control = 0, // No Accumulation
		.post_processing_mode   = XMC_VADC_DMM_REDUCTION_MODE,
		.wait_for_read_mode  	= 1, // Enabled
		.part_of_fifo       	= 0, // No FIFO
		.event_gen_enable   	= 0  // Disable Result event
	};

	// LLD Background Scan Init Structure
	const XMC_VADC_BACKGROUND_CONFIG_t adc_background_config = {
		.conv_start_mode   = XMC_VADC_STARTMODE_CIR,       // Conversion start mode selected as cancel inject repeat
		.req_src_priority  = XMC_VADC_GROUP_RS_PRIORITY_1, // Priority of the Background request source in the VADC module
		.trigger_signal    = XMC_VADC_REQ_TR_A,            // If Trigger needed then this denotes the Trigger signal
		.trigger_edge      = XMC_VADC_TRIGGER_EDGE_NONE,   // If Trigger needed then this denotes Trigger edge selected
		.gate_signal       = XMC_VADC_REQ_GT_A,			   // If Gating needed then this denotes the Gating signal
		.timer_mode        = 0,							   // Timer Mode Disabled
		.external_trigger  = 0,                            // Trigger is Disabled
		.req_src_interrupt = 0,                            // Background Request source interrupt Disabled
		.enable_auto_scan  = 1,
		.load_mode         = XMC_VADC_SCAN_LOAD_OVERWRITE
	};


	XMC_VADC_GLOBAL_Init(VADC, &adc_global_config);
	XMC_VADC_GLOBAL_StartupCalibration(VADC);

	// Initialize the Global Conversion class 0
	XMC_VADC_GLOBAL_InputClassInit(VADC, adc_global_iclass_config, XMC_VADC_GROUP_CONV_STD, 0);
	// Initialize the Global Conversion class 1
	XMC_VADC_GLOBAL_InputClassInit(VADC, adc_global_iclass_config, XMC_VADC_GROUP_CONV_STD, 1);

	// Initialize the Background Scan hardware
	XMC_VADC_GLOBAL_BackgroundInit(VADC, &adc_background_config);

	// Initialize the global result register
	XMC_VADC_GLOBAL_ResultInit(VADC, &adc_global_result_config);

	XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 0, MATRIX_ADC_CHANNEL);
	XMC_VADC_GLOBAL_SetResultEventInterruptNode(VADC, XMC_VADC_SR_SHARED_SR0);

	XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC);
}

void matrix_init_spi(Matrix *matrix) {
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
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW
	};

	// Initialize USIC channel in SPI master mode
	XMC_SPI_CH_Init(MATRIX_USIC, &channel_config);
	MATRIX_USIC->SCTR &= ~USIC_CH_SCTR_PDL_Msk; // Set passive data level to 0
	MATRIX_USIC->SCTR |= USIC_CH_SCTR_DOCFG_Msk; // Use correct polarity
	MATRIX_USIC->PCR_SSCMode &= ~USIC_CH_PCR_SSCMode_TIWEN_Msk; // Disable time between bytes

	XMC_SPI_CH_SetBitOrderMsbFirst(MATRIX_USIC);

	XMC_SPI_CH_SetWordLength(MATRIX_USIC, (uint8_t)8U);
	XMC_SPI_CH_SetFrameLength(MATRIX_USIC, (uint8_t)64U);

	XMC_SPI_CH_SetTransmitMode(MATRIX_USIC, XMC_SPI_CH_MODE_STANDARD);

	// SPI Mode: CPOL=1 and CPHA=1
	MATRIX_USIC_CHANNEL->DX1CR &= ~USIC_CH_DX1CR_DPOL_Msk;

	// Configure transmit FIFO
	XMC_USIC_CH_TXFIFO_Configure(MATRIX_USIC, 32, XMC_USIC_CH_FIFO_SIZE_32WORDS, 28);

	// Set service request for tx FIFO transmit interrupt
	XMC_USIC_CH_TXFIFO_SetInterruptNodePointer(MATRIX_USIC, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, MATRIX_SERVICE_REQUEST_TX);  // IRQ MATRIX_IRQ_TX

	// Set priority and enable NVIC node for transmit interrupt
	NVIC_SetPriority((IRQn_Type)MATRIX_IRQ_TX, MATRIX_IRQ_TX_PRIORITY);
	NVIC_EnableIRQ((IRQn_Type)MATRIX_IRQ_TX);

	// Start SPI
	XMC_SPI_CH_Start(MATRIX_USIC);

	// Configure MOSI pin
	XMC_GPIO_Init(MATRIX_MOSI_PIN, &mosi_pin_config);
	XMC_GPIO_SetHardwareControl(MATRIX_MOSI_PIN, XMC_GPIO_HWCTRL_DISABLED);
}

void matrix_init(Matrix *matrix) {
	memset(matrix, 0, sizeof(Matrix));

	matrix_init_spi(matrix);
	matrix_init_adc(matrix);
}


void matrix_tick(Matrix *matrix) {
	matrix_measure_voltage(matrix);

	if(matrix->frame_duration != 0) {
		if(system_timer_is_time_elapsed_ms(matrix->frame_last_time, matrix->frame_duration)) {
			matrix_draw_frame(matrix);
			matrix->frame_started = true;
			matrix->frame_last_time = system_timer_get_ms();
		}
	}
}
