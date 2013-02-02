/******************************************************************************
 * SG-1 Gripper MCB
 * 
 * Copyright (c) 2011-2012, Michael E. Ferguson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of the copyright holders nor the names of
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <avr/io.h>
#include <stdint.h>
#include "ax_device.h"

/* Encoder Specs -- this should eventually move to the register table */
#define ENCODER_OFFSET  3974
#define LOWER_LIMIT     5
#define UPPER_LIMIT     1172

/* LED */
#define DDR_LED         DDRD
#define PORT_LED        PORTD
#define PIN_LED         3
void led_init()
{
    DDR_LED |= (1<<PIN_LED);
    PORT_LED |= (1<<PIN_LED);
}
void write_led(const uint8_t address, const uint8_t value)
{
    if(value > 0){
        shared_table[address] = 1;
        PORT_LED &= 0xff-(1<<PIN_LED);
    }else{
        shared_table[address] = 0;
        PORT_LED |= 1<<PIN_LED;
    }
}

/* Control Update Clock */
volatile unsigned long systick = 0;
// timer0 prescalar set to 1024 = 15 counts/ms
// 100hz = 150 counts on timer0
// 255 + 1 - 150 = 
#define TCNT0_BOTTOM 106
void systick_init()
{
    // normal mode
    TCCR0A &= ~((1<<WGM01) | (1<<WGM00));  
    TCCR0B &= ~(1<<WGM02);
    // prescalar to 1024 (15 counts/ms)
    TCCR0B |= (1<<CS02) | (1<<CS00);
    TCCR0B &= ~(1<<CS01);
    // enable systick
    TIMSK0 |= (1<<TOIE0);
}
ISR(TIMER0_OVF_vect)
{
    TCNT0 = TCNT0_BOTTOM;
    systick++;
}

/* Motor on OC1A/OC1B */
#define DDR_MOTOR       DDRB
#define PIN_OC1A        1
#define PIN_OC1B        2
void motor_init()
{
    // set fast-PWM, 10-bit
    TCCR1A |= (1<<WGM11) | (1<<WGM10);
    TCCR1B |= (1<<WGM12);
    TCCR1B &= ~(1<<WGM13);
    // set pwm to 15Khz (no clock divider)
    TCCR1B |= (1<<CS10);
    TCCR1B &= ~(1<<CS11); 
    TCCR1B &= ~(1<<CS12);
    // initialize
    OCR1A = OCR1B = 0;
    // clear on compare
    TCCR1A |= (1<<COM1A1) | (1<<COM1B1);
    TCCR1A &= ~(1<<COM1A0) & ~(1<<COM1B0);
    // enable outputs
    DDR_MOTOR |= (1<<PIN_OC1A) | (1<<PIN_OC1B);
}

void motor_set(int pwm)
{
    if(pwm == 0)
    {
        // set a/b low, coast
        OCR1A = OCR1B = 0;  
    }
    else if(pwm > 0)
    {
        if(pwm > 512) pwm = 512;
        // a high, pulse b low
        OCR1A = 1023;
        OCR1B = 1023-pwm;
    }
    else
    {
        if(pwm < -512) pwm = -512;
        // b high, pulse a low
        OCR1A = 1023+pwm;
        OCR1B = 1023;
    }
}

void motor_brake()
{
    // set a/b high, applies brake
    OCR1A = OCR1B = 1023;
}

/* AS5045 */
#define DDR_SPI         DDRB
#define PORT_SPI        PORTB
#define PIN_CS          0
#define PIN_MISO        4
#define PIN_SCK         5
void as5045_init()
{
    DDR_SPI |= (1<<PIN_CS)|(1<<PIN_SCK);
    PORT_SPI |= (1<<PIN_CS);
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<SPR1)|(1<<SPR0);
}
unsigned char as5045_data[3];
int read_as5045()
{
    PORT_SPI &= 0xff-(1<<PIN_CS);
    delayus(1);

    SPDR = 0x00;    // dummy tranmission
    while(!(SPSR & (1<<SPIF)));
    as5045_data[0] = SPDR;

    SPDR = 0x00;
    while(!(SPSR & (1<<SPIF)));
    as5045_data[1] = SPDR;

    SPDR = 0x00;
    while(!(SPSR & (1<<SPIF)));
    as5045_data[2] = SPDR;

    PORT_SPI |= (1<<PIN_CS);
    /* this is a huge hack...
       the as5045 doesn't output data until after the first clock fall
       however, the atmega samples, so we have to toss the first bit */
    return (((int)as5045_data[0]<<5) + ((int)as5045_data[1] >> 3) - ENCODER_OFFSET) & 0xfff;
}

/* Analog */
#define PIN_VOLTAGE_SENSE   0
#define PIN_TEMP_SENSE      1
#define PIN_CURRENT_SENSE   3
void analog_init()
{
    ADMUX = 0x00;     // Aref(5V)
    ADCSRA = 0x86;    // enable, no auto trigger, no interrupt, clk/64
}
int read_analog(const uint8_t channel)
{
    ADMUX &= ~0x1F;                 // clear channel selection (low 5 bits)
    ADMUX |= (1<<channel);          // select specified channel
    
    delayus(1); //00);
    ADCSRA |= (1<<ADSC);            // ADC start conversion
    while ( ADCSRA&(1<<ADIF) )
        ;

    //delayus(100);
    ADCSRA |= (1<<ADSC);            // first is crap!
    while ( ADCSRA&(1<<ADIF) )
        ;

    return ADC;
}

/* if we disable torque, turn off the motor */
void write_enable(const uint8_t address, const uint8_t value)
{
    //shared_table[address] = value;
    if(value == 0)
        motor_set(0);
}

/* when second half of goal arrives, need to enable torque */
void write_goal(const uint8_t address, const uint8_t value)
{
    //shared_table[address] = value;
    shared_table[AX_TORQUE_ENABLE] = 1;
}

// TODO: add scaling
int read_voltage(const uint8_t address){ return read_analog(PIN_VOLTAGE_SENSE)/4; }
int read_temp(const uint8_t address){ return read_analog(PIN_TEMP_SENSE)/4; }

int read_systick(const uint8_t address){
    int shift = address - 71;
    return (systick>>(8*shift))&0xff;
}

int main()
{
    // initialize
    led_init();
    analog_init();
    motor_init();
    as5045_init();
    ax_device_init();
    systick_init();
    sei();

    // register callbacks
    ax_device_register_callbacks(AX_TORQUE_ENABLE, 0, &write_enable);
    ax_device_register_callbacks(AX_LED, 0, &write_led);
    ax_device_register_callbacks(AX_GOAL_POSITION_H, 0, &write_goal);
    ax_device_register_callbacks(AX_PRESENT_VOLTAGE, &read_voltage, 0);
    ax_device_register_callbacks(AX_PRESENT_TEMPERATURE, &read_temp, 0);
    ax_device_register_callbacks(71, &read_systick, 0);
    ax_device_register_callbacks(72, &read_systick, 0);
    ax_device_register_callbacks(73, &read_systick, 0);
    ax_device_register_callbacks(74, &read_systick, 0);
    
    // process
    unsigned long tick = 1;
    int pwm = 0;
    while(1)
    {    
        if(systick > tick)
        {
            // 
            int position = read_as5045();
            // TODO: add offset
            shared_table[AX_PRESENT_POSITION_L] = position&0xff;
            shared_table[AX_PRESENT_POSITION_H] = (position>>8)&0x0f;
            position &= 0xfff;

            // current sense is ADC3 (PC3)
            int current = read_analog(PIN_CURRENT_SENSE);
            // TODO: scaling, offset?
            shared_table[AX_PRESENT_LOAD_L] = current&0xff;
            shared_table[AX_PRESENT_LOAD_H] = (current>>8)&0x03;

            // do control loop
            if(shared_table[AX_TORQUE_ENABLE] > 0)
            {            
                // try to approach target
                int goal = (shared_table[AX_GOAL_POSITION_L] + (shared_table[AX_GOAL_POSITION_H]<<8))&0xfff;
                int err = goal - position;
                pwm = err*3;
                motor_set(pwm);
            }else{
                if(shared_table[AX_LED] > 0)
                    write_led(AX_LED,0);
                else
                    write_led(AX_LED,1);
            }

            tick++;
        }
        ax_device_process();
    }

    return 0;
}

