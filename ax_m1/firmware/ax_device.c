/******************************************************************************
 * ax_device.c - Funcitons for creating a device on the Bioloid Bus
 * 
 * Copyright (c) 2010-2011, Michael E. Ferguson
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

#include "ax_device.h"

/** Register Table */
uint8_t shared_table[] = {
/* EEPROM (24 bytes) */
  12,   /* AX_MODEL_NUMBER_L */
   0,   /* AX_MODEL_NUMBER_H */
   0,   /* AX_VERSION */
  42,   /* AX_ID */                 /* TODO: SET ID HERE */
   1,   /* AX_BAUD_RATE */
 250,   /* AX_RETURN_DELAY_TIME */
   0,   /* AX_CW_ANGLE_LIMIT_L */
   0,   /* AX_CW_ANGLE_LIMIT_H */
 255,   /* AX_CCW_ANGLE_LIMIT_L */
   3,   /* AX_CCW_ANGLE_LIMIT_H */
   0,   /* AX_SYSTEM_DATA2 */
  80,   /* AX_LIMIT_TEMPERATURE */
  60,   /* AX_DOWN_LIMIT_VOLTAGE */
 190,   /* AX_UP_LIMIT_VOLTAGE */
 255,   /* AX_MAX_TORQUE_L */
   3,   /* AX_MAX_TORQUE_H */
   2,   /* AX_RETURN_LEVEL */
  16,   /* AX_ALARM_LED */
  16,   /* AX_ALARM_SHUTDOWN */
   0,   /* AX_OPERATING_MODE */
   0,   /* AX_DOWN_CALIBRATION_L */
   0,   /* AX_DOWN_CALIBRATION_H */
   0,   /* AX_UP_CALIBRATION_L */
   0,   /* AX_UP_CALIBRATION_H */
/* RAM */
   0,   /* AX_TORQUE_ENABLE */
   0,   /* AX_LED */
   0,   /* AX_CW_COMPLIANCE_MARGIN */
   0,   /* AX_CCW_COMPLIANCE_MARGIN */
  32,   /* AX_CW_COMPLIANCE_SLOPE */
  32,   /* AX_CCW_COMPLIANCE_SLOPE */
   0,   /* AX_GOAL_POSITION_L */
   0,   /* AX_GOAL_POSITION_H */
   0,   /* AX_GOAL_SPEED_L */
   0,   /* AX_GOAL_SPEED_H */
 255,   /* AX_TORQUE_LIMIT_L */
   3,   /* AX_TORQUE_LIMIT_H */
   0,   /* AX_PRESENT_POSITION_L */
   2,   /* AX_PRESENT_POSITION_H */
   0,   /* AX_PRESENT_SPEED_L */
   0,   /* AX_PRESENT_SPEED_H */
   0,   /* AX_PRESENT_LOAD_L */
   0,   /* AX_PRESENT_LOAD_H */
   0,   /* AX_PRESENT_VOLTAGE */
  50,   /* AX_PRESENT_TEMPERATURE */
   0,   /* AX_REGISTERED_INSTRUCTION */
   0,   /* AX_PAUSE_TIME */
   0,   /* AX_MOVING */
   0,   /* AX_LOCK */
  32,   /* AX_PUNCH_L */
   0    /* AX_PUNCH_H */
};


/******************************************************************************
 * Helpers
 */

/** waits (pauses) for us microseconds */
void delayus(unsigned int us){
    // Each cyle of the loop = 8 instr = 8 clock cycles = 1us @8Mhz
    while (us-- > 0){
        asm("nop");
        asm("nop");
    }
}

/** fills shared_table from eeprom, if eeprom has data */
int read_table_from_eeprom(void){
    int i;
    i = eeprom_read_byte( (uint8_t*) 9 );
    if(i == 128){               // if table is valid
        for(i=0; i<24; i++){    // read our 24 bytes
            shared_table[i] = eeprom_read_byte( (uint8_t*) (i+10) );
        }
        return 0;
    }else{                      // else, new chip, fill eeprom
        for(i=0; i<24; i++){    // write our 24 bytes
            eeprom_write_byte( (uint8_t*) (i+10), shared_table[i] );
        }
        eeprom_write_byte( (uint8_t*) 9, 128 );
    }
    return -1;
}

/** updates eeprom value from shared_table */
void write_addr_to_eeprom(const uint8_t addr){
    eeprom_write_byte( (uint8_t*) (addr+10), shared_table[addr]);
}

/******************************************************************************
 * Buffer and serial interactions
 */
unsigned char rx_buffer[64];
volatile int rx_head;
volatile int rx_tail;

/** Read aspects* */
ISR(USART_RX_vect){
    rx_buffer[rx_head] = UDR0;
    rx_head = (rx_head+1); //%RX_BUFFER_SIZE;
}
int read(){   
    if(rx_tail == rx_head)
        return -1;
    int x = (int) rx_buffer[rx_tail];
    rx_tail = (rx_tail+1); //%RX_BUFFER_SIZE;
    return x;    
}
/** Sends a character out the serial port. */
void write(unsigned char data){
    while (bit_is_clear(UCSR0A, UDRE0));
    UDR0 = data;
}

/* Set the baud rate, this depends on us having a 16Mhz clock */
int setBaud(uint8_t baud){
    UCSR0A |= (1<<U2X0);
    if((baud == 1) || (baud == 3) || (baud==4) || (baud==7) ||
       (baud == 9) || (baud == 16) || (baud==50) || (baud ==103) || (baud == 207)){
        shared_table[AX_BAUD_RATE] = baud;  // 50 is our special 38.4k
        UBRR0L = baud;  
        return 1;
    }else{
        return -1;
    }
}
/** helper functions to emulate half-duplex */
void setTX(){
    UCSR0B &= ~(1<<RXCIE0);
    UCSR0B &= ~(1<<RXEN0);    
    UCSR0B |= (1<<TXEN0);
}
void setRX(){ 
    rx_head = rx_tail = 0;
    UCSR0B &= ~(1<<TXEN0);
    UCSR0B |= (1<<RXEN0);   
    UCSR0B |= (1<<RXCIE0);
}

/******************************************************************************
 * Packet Level
 */

void statusPacket(int id, int error, unsigned char * params, int p_cnt){
    int chk = id+2+p_cnt+error;
    // TODO: delay enough time
    delayus(25 /*RETURN_DELAY*/);
    // send data
    setTX();    
    write(0xFF);
    write(0xFF);
    write(id);
    write(2+p_cnt);
    write(error);
    while(p_cnt > 0){
        write(*params);
        chk += *params;
        params++;
        p_cnt--;
    } 
    write(~((unsigned char)chk%256));
    setRX();
}

/******************************************************************************
 * Packet Instructions
 */

int16_t p_id, p_len, p_ins;         // recieved packet id, length, instruction
int16_t p_params[RX_BUFFER_SIZE];   // recieved packet parameters
int16_t p_idx;                      // recieved packet parameter index count
int16_t p_chk;                      // recieved packet checksum
int16_t idx;

uint8_t p_output[RX_BUFFER_SIZE];   // sent packet output parameter

void baud_write_cb(const uint8_t addr, const uint8_t value){
    setBaud(value);
}

/** Initialize serial port, read variables from EEPROM */
int ax_device_init(void){
    //read_table_from_eeprom();
    setBaud(shared_table[AX_BAUD_RATE]);
    ax_device_register_callbacks(AX_BAUD_RATE, 0, baud_write_cb);
    setRX();
    idx = -1;
    return 0;
}

write_callback_t write_cb[MAX_REGISTER];
read_callback_t read_cb[MAX_REGISTER];

int ax_device_register_callbacks(uint8_t id, read_callback_t read, write_callback_t write){
    if(id < 100){
        read_cb[id] = read;
        write_cb[id] = write;
        return 0;
    }
    return -1;
}

int ax_device_process(void){
    int x = read();
    while(x != -1){
        if(idx == -1){          // waiting for new packet
            if(x == 0xff)
                idx++;
        }else if(idx == 0){     // waiting for second 0xFF        
            if(x == 0xff)
                idx++;
            else
               idx = -1;
        }else if(idx == 1){     // waiting for ID
            if( x!= 0xff ){        
                p_id = x;
                p_chk = x;
                idx++;
            }
        }else if(idx == 2){     // waiting for length
            p_len = x-2;        // = num of params + 2
            p_chk += x;
            idx++;  
        }else if(idx == 3){     // waiting for instruction
            p_ins = x;
            p_chk += x;
            p_idx = 0;
            idx++;  
        }else if(p_len > 0){    // waiting for params
            p_params[p_idx++] = x;
            p_chk += x;
            p_len--;
        }else{                  // waiting for checksum
            p_chk += x;
            if(p_id==shared_table[AX_ID]){
                if(p_chk%256 == 255){   // process packet
                    if(p_ins == INSTR_PING){
                        statusPacket(p_id,0,NULL,0);
                    }else if(p_ins == INSTR_READ_DATA){
                        if(shared_table[AX_RETURN_LEVEL] != RETURN_NONE){
                            int i = 0;
                            for( ; i < p_params[1]; i++){
                                int addr = i+p_params[0];
                                if(read_cb[addr] != NULL){
                                    p_output[i] = (*(read_cb[addr]))(addr);
                                }else if(addr < 50){
                                    p_output[i] =  shared_table[addr];
                                }else{
                                    p_output[i] = 0;
                                }
                            }
                            statusPacket(p_id,ERR_NONE,p_output,p_params[1]);
                        }
                    }else if(p_ins == INSTR_WRITE_DATA){
                        int addr = p_params[0];
                        int i=1;
                        for(; i<p_idx; i++){
                            if(addr < 50) shared_table[addr] = p_params[i];
                            if(write_cb[addr] != NULL){
                                (*(write_cb[addr]))(addr, p_params[i]);
                            }
                            //if(addr < 24) write_addr_to_eeprom(addr);
                            addr++;
                        } 
                        if(shared_table[AX_RETURN_LEVEL] > RETURN_READ) 
                            statusPacket(p_id,ERR_NONE,0,0);
                    }else if(shared_table[AX_RETURN_LEVEL] > RETURN_NONE){
                        statusPacket(p_id,ERR_INSTRUCTION,NULL,0);
                    }
                }else{                  // report checksum error
                    statusPacket(p_id,ERR_CHECKSUM,NULL,0);
                }
            }else if((p_id==254) && (p_chk%256 == 255)){ 
                int addr = p_params[0]; 
                int i, j;
                for( i=2; i < p_len - 4; i+=p_params[1]){
                    if( p_params[i] == shared_table[AX_ID] ){
                        for( j=0; j<p_params[1]; j++){
                            if(addr < 50) shared_table[addr] = p_params[j];
                            if(write_cb[addr] != NULL){
                                (*write_cb[addr])(addr, p_params[j]);
                            }
                            addr++;
                        }
                        break;
                    }
                }
            }
            idx = -1;
        }
        x = read();
    }
    return 0;
}

