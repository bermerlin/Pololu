/***************************************************************************
*
* Copyright (C) 2014 www.sailboatinstruments.blogspot.com
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
* copies of the Software, and to permit persons to whom the Software is furnished
* to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
* IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
****************************************************************************/

/*
 * This file contains a modified version (ported from C++ to C, adapted for AVR)
 * of the "Driver for the ST LSM303D MEMS accelerometer / magnetometer connected
 * via SPI" (file lsm303d.cpp) published by the PX4 Development Team.
 */
 /****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 
 /*
 This file contains a modified version (ported from C++ to C, adapted for AVR) of 
 the second order low pass filter (files LowPassFilter2p.hpp and LowPassFilter2p.cpp)
 published by the PX4 Development Team.
*/
 /****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <util/delay.h>
#include <math.h>
#include <uart.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
/* SPI protocol address bits */

#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT			(1<<6)

/* register addresses: A: accel, M: mag, T: temp */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM				0x49

#define ADDR_OUT_TEMP_L			0x05
#define ADDR_OUT_TEMP_H			0x06
#define ADDR_STATUS_M			0x07
#define ADDR_OUT_X_L_M          0x08
#define ADDR_OUT_X_H_M          0x09
#define ADDR_OUT_Y_L_M          0x0A
#define ADDR_OUT_Y_H_M			0x0B
#define ADDR_OUT_Z_L_M			0x0C
#define ADDR_OUT_Z_H_M			0x0D

#define ADDR_INT_CTRL_M			0x12
#define ADDR_INT_SRC_M			0x13
#define ADDR_REFERENCE_X		0x1c
#define ADDR_REFERENCE_Y		0x1d
#define ADDR_REFERENCE_Z		0x1e

#define ADDR_STATUS_A			0x27
#define ADDR_OUT_X_L_A			0x28
#define ADDR_OUT_X_H_A			0x29
#define ADDR_OUT_Y_L_A			0x2A
#define ADDR_OUT_Y_H_A			0x2B
#define ADDR_OUT_Z_L_A			0x2C
#define ADDR_OUT_Z_H_A			0x2D

#define ADDR_CTRL_REG0			0x1F
#define ADDR_CTRL_REG1			0x20
#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define ADDR_CTRL_REG5			0x24
#define ADDR_CTRL_REG6			0x25
#define ADDR_CTRL_REG7			0x26

#define ADDR_FIFO_CTRL			0x2e
#define ADDR_FIFO_SRC			0x2f

#define ADDR_IG_CFG1			0x30
#define ADDR_IG_SRC1			0x31
#define ADDR_IG_THS1			0x32
#define ADDR_IG_DUR1			0x33
#define ADDR_IG_CFG2			0x34
#define ADDR_IG_SRC2			0x35
#define ADDR_IG_THS2			0x36
#define ADDR_IG_DUR2			0x37
#define ADDR_CLICK_CFG			0x38
#define ADDR_CLICK_SRC			0x39
#define ADDR_CLICK_THS			0x3a
#define ADDR_TIME_LIMIT			0x3b
#define ADDR_TIME_LATENCY		0x3c
#define ADDR_TIME_WINDOW		0x3d
#define ADDR_ACT_THS			0x3e
#define ADDR_ACT_DUR			0x3f

#define REG1_RATE_BITS_A		((1<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_POWERDOWN_A		((0<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_3_125HZ_A		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_6_25HZ_A		((0<<7) | (0<<6) | (1<<5) | (0<<4))
#define REG1_RATE_12_5HZ_A		((0<<7) | (0<<6) | (1<<5) | (1<<4))
#define REG1_RATE_25HZ_A		((0<<7) | (1<<6) | (0<<5) | (0<<4))
#define REG1_RATE_50HZ_A		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define REG1_RATE_100HZ_A		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define REG1_RATE_200HZ_A		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_RATE_400HZ_A		((1<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_800HZ_A		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_1600HZ_A		((1<<7) | (0<<6) | (1<<5) | (0<<4))

#define REG1_BDU_UPDATE			(1<<3)
#define REG1_Z_ENABLE_A			(1<<2)
#define REG1_Y_ENABLE_A			(1<<1)
#define REG1_X_ENABLE_A			(1<<0)

#define REG2_ANTIALIAS_FILTER_BW_BITS_A	((1<<7) | (1<<6))
#define REG2_AA_FILTER_BW_773HZ_A		((0<<7) | (0<<6))
#define REG2_AA_FILTER_BW_194HZ_A		((0<<7) | (1<<6))
#define REG2_AA_FILTER_BW_362HZ_A		((1<<7) | (0<<6))
#define REG2_AA_FILTER_BW_50HZ_A		((1<<7) | (1<<6))

#define REG2_FULL_SCALE_BITS_A	((1<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_2G_A	((0<<5) | (0<<4) | (0<<3))
#define REG2_FULL_SCALE_4G_A	((0<<5) | (0<<4) | (1<<3))
#define REG2_FULL_SCALE_6G_A	((0<<5) | (1<<4) | (0<<3))
#define REG2_FULL_SCALE_8G_A	((0<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_16G_A	((1<<5) | (0<<4) | (0<<3))

#define REG5_ENABLE_T			(1<<7)

#define REG5_RES_HIGH_M			((1<<6) | (1<<5))
#define REG5_RES_LOW_M			((0<<6) | (0<<5))

#define REG5_RATE_BITS_M		((1<<4) | (1<<3) | (1<<2))
#define REG5_RATE_3_125HZ_M		((0<<4) | (0<<3) | (0<<2))
#define REG5_RATE_6_25HZ_M		((0<<4) | (0<<3) | (1<<2))
#define REG5_RATE_12_5HZ_M		((0<<4) | (1<<3) | (0<<2))
#define REG5_RATE_25HZ_M		((0<<4) | (1<<3) | (1<<2))
#define REG5_RATE_50HZ_M		((1<<4) | (0<<3) | (0<<2))
#define REG5_RATE_100HZ_M		((1<<4) | (0<<3) | (1<<2))
#define REG5_RATE_DO_NOT_USE_M	((1<<4) | (1<<3) | (0<<2))

#define REG6_FULL_SCALE_BITS_M	((1<<6) | (1<<5))
#define REG6_FULL_SCALE_2GA_M	((0<<6) | (0<<5))
#define REG6_FULL_SCALE_4GA_M	((0<<6) | (1<<5))
#define REG6_FULL_SCALE_8GA_M	((1<<6) | (0<<5))
#define REG6_FULL_SCALE_12GA_M	((1<<6) | (1<<5))

#define REG7_CONT_MODE_M		((0<<1) | (0<<0))

#define INT_CTRL_M              0x12
#define INT_SRC_M               0x13

/* default values for this device */
#define LSM303D_ACCEL_DEFAULT_RANGE_G		2
#define LSM303D_ACCEL_DEFAULT_RATE			200
#define LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ	50
#define LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	40    // 30 dans l'original

#define LSM303D_MAG_DEFAULT_RANGE_GA		2
#define LSM303D_MAG_DEFAULT_RATE			7

//////////////////////////////////////////////////////////////////////////////////////////////////////

#define EINVAL 9
#define OK 1
#define M_PI_F 3.14159265

#define RAD_TO_DEG ((double) (180.0/M_PI_F)) 

#define ZXYMDA 3
#define ZYXADA 3

// expceted values of reg1 and reg7 to catch in-flight
// brownouts of the sensor
uint8_t	_reg1_expected;
uint8_t	_reg7_expected;

unsigned _accel_range_m_s2;
float _accel_range_scale;

// lowpass filter
double _cutoff_freq;
double _a1;
double _a2;
double _b0;
double _b1;
double _b2;
double x_delay_element_1 = 0.0;        // x-axis buffered sample -1
double x_delay_element_2 = 0.0;        // x-axis buffered sample -2
double y_delay_element_1 = 0.0;        // y-axis buffered sample -1
double y_delay_element_2 = 0.0;        // y-axis buffered sample -2
double z_delay_element_1 = 0.0;        // z-axis buffered sample -1
double z_delay_element_2 = 0.0;        // z-axis buffered sample -2

uint8_t mode;  // 0:debug    1:accelerometer calib  2:magnetometer calib  4:nmea0183   5: raw values
volatile unsigned char sw1, pressed1;
uint8_t ncalib;
double ax, ay, az;
double mx, my, mz;

volatile uint8_t iflag;
uint8_t istate;
uint8_t who_am_i;
volatile uint16_t compteur;

volatile uint16_t ms_count;
uint8_t i, reg;

// Buffer for accelerometer readings.
int16_t reads[3];

// Buffer for magnetometer readings.
int16_t readsm[3];

int32_t axreads;
int32_t ayreads;
int32_t azreads;

double xc, yc, y_ax_ay;

// Magnetometer calibration data
double m_xBias, m_sens_x[3];
double m_yBias, m_sens_y[3];
double m_zBias, m_sens_z[3];

// Accelerometer calibration data
double a_xBias, sens_x[3];
double a_yBias, sens_y[3];
double a_zBias, sens_z[3];

// Heel and pitch angles
double rho, phi;

int8_t first_acc = 1;
double x_filt, y_filt, z_filt;

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

static char buf[500];
uint8_t	click;

unsigned char kbhit(void)
{
	// return nonzero if char waiting
	unsigned char b;
	b = 0;
	if(UCSR0A & (1<<RXC0))
		b = 1;
	return b;
}

void USART_Flush( void )
{
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}

static int uart_putchar0(char c)
{
   loop_until_bit_is_set(UCSR0A, UDRE0);
   UDR0 = c;
   return 0;
}

/*
 * This interrupt routine is called when the DRDY line
 * of the LSM303D goes high
 */
ISR(INT0_vect)
{
   iflag = 1;
   compteur++;
}

void SPI_MasterInit(void)
{
   // SS(PB2), MOSI(PB3), MISO(PB4), SCK(PB5)
   // Enable pull-up on PB2(SS)
   PORTB = _BV(PB2);
   // Set PB2(SS) as output high, PB3(MOSI) and PB5(SCK) as output low
   DDRB = _BV(DDB2) | _BV(DDB3) |_BV(DDB5);
	
   // Enable pull-up on PD2 (INT0)
   PORTD = _BV(PD2);
	
   // Enable SPI, Master, set clock rate fck/16
   SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); // 460.8 kHz

   uint8_t i;

   for(i = 1; i < 100; i++)  // 1 s delay
      _delay_loop_2(20000);
}

void ReadAccelerometer(void)
{
   uint8_t axl, axh, ayl, ayh, azl, azh;  
   
   PORTB &= ~(_BV(PB2)); // select LSM303D

   SPDR = DIR_READ | ADDR_INCREMENT | ADDR_OUT_X_L_A;
   while(!(SPSR & _BV(SPIF)));
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   axl = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   axh = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   ayl = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   ayh = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   azl = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   azh = SPDR;
 
   PORTB |= _BV(PB2); // deselect LSM303D
 
   reads[0] = ((((int16_t)axh) << 8) + axl);
   reads[1] = ((((int16_t)ayh) << 8) + ayl);
   reads[2] = ((((int16_t)azh) << 8) + azl);
}

void ReadMagnetometer(void)
{
   uint8_t mxl, mxh, myl, myh, mzl, mzh;  
   
   PORTB &= ~(_BV(PB2)); // select LSM303D
   SPDR = DIR_READ | ADDR_INCREMENT | ADDR_OUT_X_L_M;
   
   while(!(SPSR & _BV(SPIF)));
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   mxl = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   mxh = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   myl = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   myh = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   mzl = SPDR;
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   mzh = SPDR;
 
   PORTB |= _BV(PB2); // deselect LSM303D
 
   readsm[0] = ((((int16_t)mxh) << 8) + mxl);
   readsm[1] = ((((int16_t)myh) << 8) + myl);
   readsm[2] = ((((int16_t)mzh) << 8) + mzl);
}

uint8_t read_reg(uint8_t Address)
{
   uint8_t result = 0;
   PORTB &= ~(_BV(PB2)); // select LSM303D 
   SPDR = DIR_READ | Address;
   while(!(SPSR & _BV(SPIF)));
   SPDR = 0x00;
   while(!(SPSR & _BV(SPIF)));
   result = SPDR;
   PORTB |= _BV(PB2); // deselect LSM303D
   return(result);  
}

void write_reg(uint8_t Address, uint8_t Value)
{
   PORTB &= ~(_BV(PB2)); // select LSM303D
   SPDR = DIR_WRITE | Address;
   while(!(SPSR & _BV(SPIF)));
   SPDR = Value;
   while(!(SPSR & _BV(SPIF)));
   PORTB |= _BV(PB2); // deselect LSM303D
}

void disable_i2c(void)
{
	uint8_t a = read_reg(0x02);
	write_reg(0x02, (0x10 | a));
	a = read_reg(0x02);
	write_reg(0x02, (0xF7 & a));	
	a = read_reg(0x15);
	write_reg(0x15, (0x80 | a));
	a = read_reg(0x02);
	write_reg(0x02, (0xE7 & a));
}

void modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

int accel_set_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG2_FULL_SCALE_BITS_A;

	if (max_g == 0)
		max_g = 16;

	if (max_g <= 2) 
	{
		setbits |= REG2_FULL_SCALE_2G_A;
	}
    else if (max_g <= 4) 
	{
		setbits |= REG2_FULL_SCALE_4G_A;
	} 
	else if (max_g <= 6) 
	{
		setbits |= REG2_FULL_SCALE_6G_A;
	} 
	else if (max_g <= 8) 
	{
		setbits |= REG2_FULL_SCALE_8G_A;
	} 
	else if (max_g <= 16) 
	{
		setbits |= REG2_FULL_SCALE_16G_A;
	} 
	else 
	{
		return -EINVAL;
	}

	modify_reg(ADDR_CTRL_REG2, clearbits, setbits);

	return OK;
}

int accel_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG1_RATE_BITS_A;

	if (frequency == 0)
		frequency = 1600;

	if (frequency <= 100) 
	{
		setbits |= REG1_RATE_100HZ_A;
	} 
	else if (frequency <= 200) 
	{
		setbits |= REG1_RATE_200HZ_A;
	} 
	else if (frequency <= 400) 
	{
		setbits |= REG1_RATE_400HZ_A;
	} 
	else if (frequency <= 800) 
	{
		setbits |= REG1_RATE_800HZ_A;
	} 
	else if (frequency <= 1600) 
	{
		setbits |= REG1_RATE_1600HZ_A;
	} 
	else 
	{
		return -EINVAL;
	}

	modify_reg(ADDR_CTRL_REG1, clearbits, setbits);
	_reg1_expected = (_reg1_expected & ~clearbits) | setbits;

	return OK;
}

void set_cutoff_frequency(double sample_freq, double cutoff_freq)
{
	_cutoff_freq = cutoff_freq;
    if (_cutoff_freq <= 0.0) 
	{
        // no filtering
        return;
    }
    double fr = sample_freq /_cutoff_freq;
    double ohm = tan(M_PI_F / fr);
    double c = 1.0 + 2.0 * cos(M_PI_F / 4.0) * ohm + ohm * ohm;
    _b0 = ohm * ohm / c;
    _b1 = 2.0 * _b0;
    _b2 = _b0;
    _a1 = 2.0 * (ohm * ohm - 1.0) / c;
    _a2 = (1.0 - 2.0 * cos(M_PI_F / 4.0) * ohm + ohm * ohm) / c;
}


int accel_set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	set_cutoff_frequency(samplerate, bandwidth);

	return OK;
}

int accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG2_ANTIALIAS_FILTER_BW_BITS_A;

	if(bandwidth == 0)
		bandwidth = 773;

	if(bandwidth <= 50) 
	{
		setbits |= REG2_AA_FILTER_BW_50HZ_A;
	} 
	else if(bandwidth <= 194) 
	{
		setbits |= REG2_AA_FILTER_BW_194HZ_A;
	} 
	else if(bandwidth <= 362) 
	{
		setbits |= REG2_AA_FILTER_BW_362HZ_A;
	} 
	else if(bandwidth <= 773) 
	{
		setbits |= REG2_AA_FILTER_BW_773HZ_A;
	} 
	else 
	{
		return -EINVAL;
	}

	modify_reg(ADDR_CTRL_REG2, clearbits, setbits);

	return OK;
}

int mag_set_range(unsigned max_ga)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG6_FULL_SCALE_BITS_M;

	if(max_ga == 0)
		max_ga = 12;

	if(max_ga <= 2) 
	{
		setbits |= REG6_FULL_SCALE_2GA_M;
	} 
	else if(max_ga <= 4) 
	{
		setbits |= REG6_FULL_SCALE_4GA_M;
	} 
	else if(max_ga <= 8) 
	{
		setbits |= REG6_FULL_SCALE_8GA_M;
	} 
	else if(max_ga <= 12) 
	{
		setbits |= REG6_FULL_SCALE_12GA_M;
	} 
	else 
	{
		return -EINVAL;
	}

	modify_reg(ADDR_CTRL_REG6, clearbits, setbits);

	return OK;
}

int mag_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG5_RATE_BITS_M;


	if(frequency == 0)
		frequency = 100;

	if(frequency <= 4) 
	{
		setbits |= REG5_RATE_3_125HZ_M;
	}
	else if(frequency <= 7) 
	{
		setbits |= REG5_RATE_6_25HZ_M;
	}
	else if(frequency <= 13) 
	{
		setbits |= REG5_RATE_12_5HZ_M;
	}  
	else if(frequency <= 25) 
	{
		setbits |= REG5_RATE_25HZ_M;
	} 
	else if(frequency <= 50) 
	{
		setbits |= REG5_RATE_50HZ_M;
	} 
	else if(frequency <= 100) 
	{
		setbits |= REG5_RATE_100HZ_M;
	} 
	else 
	{
		return -EINVAL;
	}

	modify_reg(ADDR_CTRL_REG5, clearbits, setbits);

	return OK;
}


void reset(void)
{
	// ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();
	
	/* enable accel*/
	//_reg1_expected = REG1_X_ENABLE_A | REG1_Y_ENABLE_A | REG1_Z_ENABLE_A | REG1_BDU_UPDATE | REG1_RATE_800HZ_A;
	_reg1_expected = REG1_X_ENABLE_A | REG1_Y_ENABLE_A | REG1_Z_ENABLE_A | REG1_BDU_UPDATE | REG1_RATE_200HZ_A;
	write_reg(ADDR_CTRL_REG1, _reg1_expected);
	
	/* enable mag */
	_reg7_expected = REG7_CONT_MODE_M;
	write_reg(ADDR_CTRL_REG7, _reg7_expected);
	write_reg(ADDR_CTRL_REG5, REG5_RES_HIGH_M);
	write_reg(ADDR_CTRL_REG4, 0x04); // DRDY on MAG on INT2
	
	accel_set_range(LSM303D_ACCEL_DEFAULT_RANGE_G);
	accel_set_samplerate(LSM303D_ACCEL_DEFAULT_RATE);
	accel_set_driver_lowpass_filter((double)LSM303D_ACCEL_DEFAULT_RATE, (double)LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);

	// we setup the anti-alias on-chip filter as 50Hz. We believe
	// this operates in the analog domain, and is critical for
	// anti-aliasing. The 2 pole software filter is designed to
	// operate in conjunction with this on-chip filter
	accel_set_onchip_lowpass_filter_bandwidth(LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ);

	mag_set_range(LSM303D_MAG_DEFAULT_RANGE_GA);
	mag_set_samplerate(LSM303D_MAG_DEFAULT_RATE);
}

double x_apply(double sample)
{
    if (_cutoff_freq <= 0.0) {
        // no filtering
        return sample;
    }
    // do the filtering
    double delay_element_0 = sample - x_delay_element_1 * _a1 - x_delay_element_2 * _a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        // don't allow bad values to propogate via the filter
        delay_element_0 = sample;
    }
    double output = delay_element_0 * _b0 + x_delay_element_1 * _b1 + x_delay_element_2 * _b2;
    
    x_delay_element_2 = x_delay_element_1;
    x_delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}

double y_apply(double sample)
{
    if (_cutoff_freq <= 0.0) {
        // no filtering
        return sample;
    }
    // do the filtering
    double delay_element_0 = sample - y_delay_element_1 * _a1 - y_delay_element_2 * _a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        // don't allow bad values to propogate via the filter
        delay_element_0 = sample;
    }
    double output = delay_element_0 * _b0 + y_delay_element_1 * _b1 + y_delay_element_2 * _b2;
    
    y_delay_element_2 = y_delay_element_1;
    y_delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}

double z_apply(double sample)
{
    if (_cutoff_freq <= 0.0) {
        // no filtering
        return sample;
    }
    // do the filtering
    double delay_element_0 = sample - z_delay_element_1 * _a1 - z_delay_element_2 * _a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        // don't allow bad values to propogate via the filter
        delay_element_0 = sample;
    }
    double output = delay_element_0 * _b0 + z_delay_element_1 * _b1 + z_delay_element_2 * _b2;
    
    z_delay_element_2 = z_delay_element_1;
    z_delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}



int main(void)
{
   char buffer[64];
   iflag = 0;
   click = 0;
   
   mode = 4;
   
   double x_filter, y_filter, z_filter;
   x_filter = 0.0;
   y_filter = 0.0;
   z_filter = 0.0;
   
   stdout = stdin = &uart_str;
   stderr = &uart_str;
   
   /* Set baud rate : 57600 bps @ 8.0 MHz */   // Note : results to 115200 (to be verified)
   UBRR0L = (unsigned char)(8);
   /* Enable transmitter */
   UCSR0B = _BV(TXEN0) | _BV(RXEN0);
   
   SPI_MasterInit();
   
   sw1 = 0;
   pressed1 = 0;
   ncalib = 0;
   
   // enable external interrupt on DRDY line  (rising edge)
   EICRA = _BV(ISC00) | _BV(ISC01);
   EIMSK |= _BV(INT0);  // enable interrupt on DRDY line

   sei();
   
   a_xBias = 157.409051;
   a_yBias = -193.594777;
   a_zBias = -114.775853;

   sens_x[0] = 1.021264;
   sens_x[1] = -0.009813;
   sens_x[2] = -0.012086;
   
   sens_y[0] = -0.009813;
   sens_y[1] = 0.970292;
   sens_y[2] = -0.008260;
   
   sens_z[0] = -0.012086;
   sens_z[1] = -0.008260;
   sens_z[2] = 0.978060;

   m_xBias = 168.021769;
   m_yBias = -224.670512;
   m_zBias = -229.768268;

   m_sens_x[0] = 0.999116;
   m_sens_x[1] = 0.002565;
   m_sens_x[2] = 0.003536;
   
   m_sens_y[0] = 0.002565;
   m_sens_y[1] = 0.997115;
   m_sens_y[2] = 0.021222;
   
   m_sens_z[0] = 0.003536;
   m_sens_z[1] = 0.021222;
   m_sens_z[2] = 1.012723;
   
   uint8_t count;
   count = 0;
   
   reset();
      
   for(i = 1; i < 100; i++)  // 1 s delay
      _delay_loop_2(20000);	
   
   while(!(read_reg(ADDR_STATUS_M) & _BV(ZXYMDA)));
   ReadMagnetometer();
   
   uint16_t n_acc = 0;
   axreads = 0;
   ayreads = 0;
   azreads = 0;
  
   while (1) 
   {
      if(kbhit())     // data are available from USB-to-serial
	  {
		/* List of valid commands received
			
			STOP<ENTER> : stop all transmission from system
			
			M0<ENTER> : put in debug mode
			M1<ENTER> : put in accelerometer calibration mode
			M2<ENTER> : put in magnetometer calibration mode
			M4<ENTER> : put in NMEA mode (default)
			M5<ENTER> : put in raw mode
			
			x<ENTER> : print set of raw values when in mode 1 or 2
		*/
		
		if(fgets(buf, sizeof buf - 1, stdin) == NULL)
		{
			USART_Flush();
			continue;
		}
		else
		{
			USART_Flush();
			printf("%s\r\n", buf);
		}

		if(strstr(buf, "x") != NULL)
		{
			click = 1;
		}
		else if(strstr(buf, "STOP") != NULL)
		{
			mode = 9;
		}
		else if(strstr(buf, "M0") != NULL)
		{
			mode = 0;
		}
		else if(strstr(buf, "M1") != NULL)
		{
			mode = 1;
		}
		else if(strstr(buf, "M2") != NULL)
		{
			mode = 2;
		}
		else if(strstr(buf, "M4") != NULL)
		{
			mode = 4;
		}
	  }
	  
	  if(iflag  > 0)
      {
		 ReadMagnetometer();
		 
		 if(mode == 2)
		 {
			if(click == 1)
			{
				if(ncalib < 16)
				{
					sprintf(buffer, "%i %i %i", readsm[0], readsm[1], readsm[2]);
					for(i = 0; i < strlen(buffer); i++)
						uart_putchar0((unsigned char)(buffer[i]));
					uart_putchar0('\r');
					uart_putchar0('\n');
					ncalib++;
				}
				else    // ncalib = 16
				{
					ncalib = 0;
					click = 0;
				}
			}
		 }
		 
		 double mrx = readsm[0] - m_xBias;
		 double mry = readsm[1] - m_yBias;
		 double mrz = readsm[2] - m_zBias;
			
		 mx = m_sens_x[0] * mrx + m_sens_x[1] * mry + m_sens_x[2] * mrz;
		 my = m_sens_y[0] * mrx + m_sens_y[1] * mry + m_sens_y[2] * mrz;
		 mz = m_sens_z[0] * mrx + m_sens_z[1] * mry + m_sens_z[2] * mrz;
					
		 if(mode == 0)
			uart_putchar0('n');
		 
		 if(mode == 0)
		 {
		   // calculate and print the uncorrected heading
     	   double headbrut = atan2(my, mx) * RAD_TO_DEG;
           if(headbrut < 0.0)
              headbrut += 360.0;
		   sprintf(buffer, " %5.1f", headbrut);
           for(i = 0; i < strlen(buffer); i++)
             uart_putchar0((unsigned char)(buffer[i]));
		 }
		 
		 // calculate accelerometer average
         reads[0] = axreads / n_acc;
         reads[1] = ayreads / n_acc;
         reads[2] = azreads / n_acc;
		 		 
		 double rx = reads[0] - a_xBias;
		 double ry = reads[1] - a_yBias;
		 double rz = reads[2] - a_zBias;
	 
		 			
		 ax = sens_x[0] * rx + sens_x[1] * ry + sens_x[2] * rz;
		 ay = sens_y[0] * rx + sens_y[1] * ry + sens_y[2] * rz;
		 az = sens_z[0] * rx + sens_z[1] * ry + sens_z[2] * rz;
   
         // calculate heel(rho) and pitch(phi)
         rho = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
         phi = atan(ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
		 
		 // normalize accelerometer readings
         double norm = sqrt(ax * ax + ay * ay + az * az);
         ax /= norm;
         ay /= norm;
         az /= norm;
		 
   		 // tilt compensation
         double one_ax2 = 1.0 - ax * ax;
         y_ax_ay = my * ax * ay;
         xc = mx * one_ax2 - y_ax_ay - mz * ax * az;
         yc = my * az - mz * ay;
		 
		 if(mode == 0)
		 {
		   // calculate and print corrected heading, heel, pitch
		   double head_corr = atan2(yc, xc) * RAD_TO_DEG;
           if(head_corr < 0.0)
              head_corr += 360.0;
           sprintf(buffer, "  %5.1f  %5.1f  %5.1f  %5i", head_corr, rho, phi, n_acc);
           for(i = 0; i < strlen(buffer); i++)
              uart_putchar0((unsigned char)(buffer[i]));
           uart_putchar0('\r');
           uart_putchar0('\n');
		}
		else if(mode == 4)
		{
		   double head_corr = atan2(yc, xc) * RAD_TO_DEG;
           if(head_corr < 0.0)
              head_corr += 360.0;
           // $HCHDG,55.6,0.0,E,,*1F
		   sprintf(buffer, "$HCHDG,%5.1f", head_corr);
           for(i = 0; i < strlen(buffer); i++)
              uart_putchar0((unsigned char)(buffer[i]));
           uart_putchar0('\r');
           uart_putchar0('\n');
		   
		   //$PFEC,GPatt,,-8.7,+4.8*63   (pitch angle : -8.7 deg,  heel angle: 4.8 deg)
		   sprintf(buffer, "$PFEC,GPatt,,%5.1f,%5.1f", phi, rho);
		   for(i = 0; i < strlen(buffer); i++)
              uart_putchar0((unsigned char)(buffer[i]));
           uart_putchar0('\r');
           uart_putchar0('\n');
		}
					
         if(mode == 5)
         {
			sprintf(buffer, "%d  %d   %d    %d    %d   %d   %d", readsm[0], readsm[1], readsm[2], reads[0], reads[1], reads[2], n_acc);
			   for(i = 0; i < strlen(buffer); i++)
			uart_putchar0((unsigned char)(buffer[i]));
			uart_putchar0('\r');
			uart_putchar0('\n');
         }
	        
         iflag =  0;
			
         n_acc = 0;
         axreads = 0;
         ayreads = 0;
         azreads = 0;
      }
		 
      while(!(read_reg(ADDR_STATUS_A) & _BV(ZYXADA)));
	  ReadAccelerometer();
	  
	  x_filter = x_apply(reads[0]);
	  y_filter = y_apply(reads[1]);
	  z_filter = z_apply(reads[2]);
	  
		 
	  if(mode == 1)
	  {
		 if(click == 1)
		 {
		    if(ncalib < 64)
			{
			   //sprintf(buffer, "%5i %5i %5i", reads[0], reads[1], reads[2]);
			   sprintf(buffer, "%5i %5i %5i",(int)x_filter, (int)y_filter, (int)z_filter);
			   for(i = 0; i < strlen(buffer); i++)
			      uart_putchar0((unsigned char)(buffer[i]));
			   uart_putchar0('\r');
			   uart_putchar0('\n');
			   ncalib++;
			}
			else    // ncalib = 64
			{
			   ncalib = 0;
			   click = 0;
			}
		 }
	  }
		 
	  axreads += x_filter;
      ayreads += y_filter;
	  azreads += z_filter;
  
      n_acc++;
	  
   }
}

