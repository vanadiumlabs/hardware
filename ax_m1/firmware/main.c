/******************************************************************************
 * AX-M1: a motor driver for the Bioloid Bus
 * 
 * Copyright (c) 2011, Michael E. Ferguson
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

#define DDR_SSR         DDRD
#define DDR_LED         DDRD
#define DDR_DIGITAL     DDRC
#define PORT_SSR        PORTD
#define PORT_LED        PORTD
#define PORT_DIGITAL    PORTC
#define PIN_SSR         PIND
#define PIN_DIGITAL     PINC
#define SSR_PIN         4
#define LED_PIN         5

// SSR is activated through GOAL_POSITION. > 512 is on
// Analog pins can be read, each port is an 8-bit register
#define REG_ANALOG      50
// Digital pins are controlled by direction and value registers
#define REG_DIGITAL_DIR 56
#define REG_DIGITAL_VAL 57

uint16_t ssr;
uint8_t ssr2;
int read_ssr(const uint8_t address)
{
    if(address == AX_PRESENT_POSITION_L){
        return ssr&0xff;
    }else{
        return ssr>>8;
    }
}
void write_ssr(const uint8_t address, const uint8_t value)
{
    if(address == AX_GOAL_POSITION_L){
        ssr2 = value;
    }else{
        ssr = (value<<8)+ssr2;
        if(ssr > 512){
            PORT_SSR |= (1<<SSR_PIN);
        }else{
            PORT_SSR &= 0xFF-(1<<SSR_PIN);
        }
    }
}

int read_analog(const uint8_t address)
{
    ADMUX &= ~0x1F;                 // clear channel selection (low 5 bits)
    ADMUX |= address-REG_ANALOG;    // select specified channel
    
    delayus(100);
    ADCSRA |= (1<<ADSC);            // ADC start conversion
    while ( ADCSRA&(1<<ADIF) )
        ;

    delayus(100);
    ADCSRA |= (1<<ADSC);            // first is crap!
    while ( ADCSRA&(1<<ADIF) )
        ;

    return ADC/4;
}

int read_digital_dir(const uint8_t address)
{
    return DDR_DIGITAL;
}

uint8_t direction;
void write_digital_dir(const uint8_t address, const uint8_t value)
{
    direction = value;
}

int read_digital(const uint8_t address)
{
    return PIN_DIGITAL;
}

void write_digital(const uint8_t address, const uint8_t value){
    DDR_DIGITAL = direction;
    PORT_DIGITAL = value;
}

void write_led(const uint8_t address, const uint8_t value){
    if(value > 0)
        PORT_LED &= 0xff-(1<<LED_PIN);
    else
        PORT_LED |= 1<<LED_PIN;
}

int main(){
    // initialize ADC    
    ADMUX = 0x00;     // Aref(5V)
    ADCSRA = 0x86;    // enable, no auto trigger, no interrupt, clk/64

    // initialize bus
    ax_device_init();
    sei();

    // setup ports
    DDR_SSR |= (1<<SSR_PIN);
    DDR_LED |= (1<<LED_PIN);
    PORT_LED |= (1<<LED_PIN);

    // register callbacks
    ax_device_register_callbacks(AX_GOAL_POSITION_L, 0, &write_ssr);
    ax_device_register_callbacks(AX_GOAL_POSITION_L+1, 0, &write_ssr);
    ax_device_register_callbacks(AX_PRESENT_POSITION_L, &read_ssr, 0);
    ax_device_register_callbacks(AX_PRESENT_POSITION_L+1, &read_ssr, 0);
    ax_device_register_callbacks(AX_LED, 0, &write_led);
    ax_device_register_callbacks(REG_ANALOG, &read_analog, 0);
    ax_device_register_callbacks(REG_ANALOG+1, &read_analog, 0);
    ax_device_register_callbacks(REG_ANALOG+2, &read_analog, 0);
    ax_device_register_callbacks(REG_ANALOG+3, &read_analog, 0);
    ax_device_register_callbacks(REG_ANALOG+4, &read_analog, 0);
    ax_device_register_callbacks(REG_ANALOG+5, &read_analog, 0);
    ax_device_register_callbacks(REG_DIGITAL_DIR, &read_digital_dir, &write_digital_dir);
    ax_device_register_callbacks(REG_DIGITAL_VAL, &read_digital, &write_digital);
    
    // process
    while(1){
        ax_device_process();
    }

    return 0;
}

