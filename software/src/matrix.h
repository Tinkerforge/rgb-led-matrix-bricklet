/* rgb-led-matrix-bricklet
 * Copyright (C) 2017 Olaf Lüke <olaf@tinkerforge.com>
 *
 * matrix.h: WS2812B 8x8 Matrix driver
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

#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>
#include <stdbool.h>

#define MATRIX_SIZE 64

typedef struct {
	uint8_t r[MATRIX_SIZE];
	uint8_t g[MATRIX_SIZE];
	uint8_t b[MATRIX_SIZE];
} MatrixBuffer;

typedef struct {
	MatrixBuffer buffer_in;
	MatrixBuffer buffer_out;
	uint16_t frame_duration;
	uint32_t frame_number;

	bool     frame_started;
	uint32_t frame_last_time;
} Matrix;

void matrix_init(Matrix *matrix);
void matrix_tick(Matrix *matrix);

void matrix_draw_frame(Matrix *matrix);

#endif
