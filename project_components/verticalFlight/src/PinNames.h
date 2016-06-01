/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2014, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#ifndef MBED_PINDEFS_H
#define MBED_PINDEFS_H

#ifdef __cplusplus
extern "C" {
#endif

// See stm32f4xx_hal_gpio.h and stm32f4xx_hal_gpio_ex.h for values of MODE, PUPD and AFNUM
#define STM_PIN_DATA(MODE, PUPD, AFNUM)  ((int)(((AFNUM) << 7) | ((PUPD) << 4) | ((MODE) << 0)))
#define STM_PIN_DATA_EXT(MODE, PUPD, AFNUM, CHANNEL, INVERTED)  ((int)(((INVERTED & 0x01) << 15) | ((CHANNEL & 0x0F) << 11) | ((AFNUM & 0x0F) << 7) | ((PUPD & 0x07) << 4) | ((MODE & 0x0F) << 0)))
#define STM_PIN_MODE(X)   (((X) >> 0) & 0x0F)
#define STM_PIN_PUPD(X)   (((X) >> 4) & 0x07)
#define STM_PIN_AFNUM(X)  (((X) >> 7) & 0x0F)
#define STM_PIN_CHANNEL(X)  (((X) >> 11) & 0x0F)
#define STM_PIN_INVERTED(X) (((X) >> 15) & 0x01)
#define STM_MODE_INPUT              (0)
#define STM_MODE_OUTPUT_PP          (1)
#define STM_MODE_OUTPUT_OD          (2)
#define STM_MODE_AF_PP              (3)
#define STM_MODE_AF_OD              (4)
#define STM_MODE_ANALOG             (5)
#define STM_MODE_IT_RISING          (6)
#define STM_MODE_IT_FALLING         (7)
#define STM_MODE_IT_RISING_FALLING  (8)
#define STM_MODE_EVT_RISING         (9)
#define STM_MODE_EVT_FALLING        (10)
#define STM_MODE_EVT_RISING_FALLING (11)
#define STM_MODE_IT_EVT_RESET       (12)

// High nibble = port number (0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, 7=H)
// Low nibble  = pin number
#define STM_PORT(X) (((uint32_t)(X) >> 4) & 0xF)
#define STM_PIN(X)  ((uint32_t)(X) & 0xF)

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

typedef enum {
    PullNone  = 0,
    PullUp    = 1,
    PullDown  = 2,
    OpenDrain = 3,
    PullDefault = PullNone
} PinMode;

#ifdef __cplusplus
}
#endif

#endif //MBED_PINDEFS_H
