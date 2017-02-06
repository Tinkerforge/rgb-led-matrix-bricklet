/* rgb-led-matrix-bricklet
 * Copyright (C) 2017 Olaf Lüke <olaf@tinkerforge.com>
 *
 * conig_matrix.c: Configuration for WS2812B 8x8 Matrix
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

#include "xmc_gpio.h"

// TODO: Add Voltage Input pin

#define MATRIX_USIC_CHANNEL         USIC0_CH1
#define MATRIX_USIC                 XMC_SPI0_CH1

#define MATRIX_MOSI_PIN             P0_7
#define MATRIX_MOSI_PIN_AF          (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P0_7_AF_U0C1_DOUT0)

#define MATRIX_SERVICE_REQUEST_TX   3

#define MATRIX_IRQ_TX               12
#define MATRIX_IRQ_TX_PRIORITY      3
