/* 
 * FM tuner for PIC16F1503
 *    indicates a frequency by BCD(Binary-coded decimal)
 *
 *      JA1YTS:Toshiba Amature Radio Station
 *      JK1MLY:Hidekazu Inaba
 *
 *  (C)2022 JA1YTS,JK1MLY All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation and/or 
 * other materials provided with the distribution.
 *
 *
 * data sheet
 *  https://datasheet.lcsc.com/szlcsc/RDA5807M_C82537.pdf
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <xc.h>
#include <pic.h>
#include <pic16F1503.h>

#define _XTAL_FREQ 4000000
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection
#pragma config WDTE = OFF       // Watchdog Timer
#pragma config PWRTE = OFF      // Power-up Timer
#pragma config MCLRE = OFF      // MCLR Pin Function Select
#pragma config CP = OFF         // Flash Program Memory Code Protection
#pragma config BOREN = ON       // Brown-out Reset Enable
#pragma config CLKOUTEN = OFF   // Clock Out Enable
// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection
// #pragma config PLLEN = OFF      // 4x PLL OFF
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable
#pragma config BORV = LO        // Brown-out Reset Voltage Selection
// #pragma config LPBOREN = OFF    // Low Power Brown-out Reset Enable
#pragma config LVP = OFF        // Low-Voltage Programming Enable

#define	I2C_SDA_LOW     LATA5 = 0;TRISA5 = 0    // 0
#define I2C_SDA_HIGH	LATA5 = 1;TRISA5 = 1    // Z(1)
#define	I2C_SDA_READ    LATA5 = 0;TRISA5 = 1    // Input
#define	I2C_SDA_DATA    RA5
#define	I2C_SCK_LOW     LATA4 = 0
#define	I2C_SCK_HIGH	LATA4 = 1
#define	LED_WAIT        asm ("NOP");asm ("NOP");asm ("NOP")

#define	LED1_ON      	LATA0 = 1; LATA1 = 0
#define	LED1_OFF     	LATA0 = 0; LATA1 = 0
#define	LED2_ON      	LATA1 = 1; LATA2 = 0
#define	LED2_OFF     	LATA1 = 0; LATA2 = 0
#define	LED3_ON      	LATA2 = 1; LATC0 = 0
#define	LED3_OFF     	LATA2 = 0; LATC0 = 0
#define	LED4_ON      	LATC0 = 1; LATC1 = 0
#define	LED4_OFF     	LATC0 = 0; LATC1 = 0
#define	LED5_ON      	LATC1 = 1; LATC2 = 0
#define	LED5_OFF     	LATC1 = 0; LATC2 = 0

#define	SW_UP       	RC5
#define	SW_DN       	RC4
#define	SW_FN       	RC3
#define	PUSH_ON         0

#define	M_FRQ_SET       1      
#define	M_SEQ_DN        2
#define	M_SEQ_UP        3
#define	M_VOL_SET       4 
#define	M_TUN_CHK       8 
#define VOL_LMT_H       12
#define VOL_LMT_L       1
#define SET_TFM         100
#define SET_NAK         95
#define SET_JWV         113
#define SET_NHK         125

// I2C mode CHAN[9:0] BAND=2 CHAN = (Freq - 76.0M) / Ch_s
// TFM Freq=80.0 BAND=2 Ch_s=100k CHAN=40 0x28 00 0010 1000
#define	TUN_ADR         0x20
#define	REG02_H         0xC0
#define	MON02_H         0xE0
#define	FUP02_H         0xC3
#define	FDN02_H         0xC1
#define	REG02_L         0x01
#define	REG03_H         0x0A
#define	TFM03_H         0x0A
#define	REG03_L         0x08
#define	TUN03_L         0x18
#define	TFM03_L         0x18
#define	REG04_H         0x08
#define	AFC04_H         0x09
#define	REG04_L         0x80
#define	REG05_H         0x88
#define	TUN05_H         0x8C
#define	REG05_L         0x86

#define BRIT1           15
#define BRIT0           45
#define DARK1           1
#define DARK0           59

void port_init(void) {
    OSCCON = 0b01101000; //4MHz
    TRISA = 0b00101000; //Input(1)
    TRISC = 0b00111000; //Input(1)
    OPTION_REG = 0b00000000; //MSB WPUENn
    WPUA = 0b00101000; //PupOn(1)
    INTCON = 0b00000000;
    LATA = 0b00010000;
    LATC = 0b00000000;
    ANSELA = 0b00000000;
    ANSELC = 0b00000000;
}

void blk_led1(void) {
    for (uint8_t lp = 0; lp < 100; lp++) {
        LED1_ON;
        LED_WAIT;
        LED1_OFF;
        LED_WAIT;
    }
}

void blk_led2(void) {
    for (uint8_t lp = 0; lp < 100; lp++) {
        LED2_ON;
        LED_WAIT;
        LED2_OFF;
        LED_WAIT;
    }
}

void blk_led3(void) {
    for (uint8_t lp = 0; lp < 100; lp++) {
        LED3_ON;
        LED_WAIT;
        LED3_OFF;
        LED_WAIT;
    }
}

void blk_led4(void) {
    for (uint8_t lp = 0; lp < 100; lp++) {
        LED4_ON;
        LED_WAIT;
        LED4_OFF;
        LED_WAIT;
    }
}

void blk_led5(void) {
    for (uint8_t lp = 0; lp < 100; lp++) {
        LED5_ON;
        LED_WAIT;
        LED5_OFF;
        LED_WAIT;
    }
}

void led_chk(void) {
    blk_led1();
    __delay_ms(200);

    blk_led2();
    __delay_ms(200);

    blk_led3();
    __delay_ms(200);

    blk_led4();
    __delay_ms(200);

    blk_led5();
    __delay_ms(200);
}

void i2c_st(void) {
    I2C_SCK_HIGH;
    I2C_SDA_HIGH;
    __delay_us(10);
    I2C_SDA_LOW;
    __delay_us(6);
    I2C_SCK_LOW;
    __delay_us(1);
}

void i2c_sck(void) {
    I2C_SCK_HIGH;
    __delay_us(4);
    I2C_SCK_LOW;
    __delay_us(1);
}

void i2c_end(void) {
    // Stop            
    I2C_SDA_LOW;
    __delay_us(2);
    I2C_SCK_HIGH;
    __delay_us(6);
    // Wait
    I2C_SDA_HIGH;
    __delay_us(20);
}

void i2c_rtn(void) {
    I2C_SDA_HIGH;
    __delay_us(6);
    I2C_SCK_HIGH;
    __delay_us(4);
    I2C_SCK_LOW;
    I2C_SDA_LOW;
    __delay_us(20);
}

void i2c_ack(void) {
    __delay_us(1);
    I2C_SDA_LOW;
    __delay_us(5);
    I2C_SCK_HIGH;
    __delay_us(4);
    I2C_SCK_LOW;
    I2C_SDA_LOW;
    __delay_us(20);
}

void i2c_nak(void) {
    __delay_us(1);
    I2C_SDA_HIGH;
    __delay_us(5);
    I2C_SCK_HIGH;
    __delay_us(4);
    I2C_SCK_LOW;
    I2C_SDA_LOW;
    __delay_us(20);
}

void i2c_snd(uint8_t data) {
    for (uint8_t lp = 0; lp < 8; lp++) {
        if ((data & 0x80) == 0) {
            I2C_SDA_LOW;
        } else {
            I2C_SDA_HIGH;
        }
        data = (data << 1) & 0xff;
        __delay_us(2);
        i2c_sck();
    }
    i2c_rtn();
}

uint8_t i2c_rcv_cnt() {
    uint8_t data = 0;
    I2C_SDA_READ;
    for (uint8_t lp = 0; lp < 8; lp++) {
        data = (data << 1) & 0xff;
        __delay_us(2);
        if (I2C_SDA_DATA == 1) {
            data = data | 0x01;
        }
        i2c_sck();
    }
    i2c_ack();

    return data;
}

uint8_t i2c_rcv_end() {
    uint8_t data = 0;
    I2C_SDA_READ;
    for (uint8_t lp = 0; lp < 8; lp++) {
        data = (data << 1) & 0xff;
        __delay_us(2);
        if (I2C_SDA_DATA == 1) {
            data = data | 0x01;
        }
        i2c_sck();
    }
    i2c_nak();

    return data;
}

uint8_t tun_rcv(void) {
    uint8_t data;
    uint8_t freq;
    uint8_t seek;
    uint8_t rssi;

    i2c_st();
    // read
    data = (TUN_ADR & 0xFE) | 0x01;
    i2c_snd(data);

    // reg0A
    data = i2c_rcv_cnt();
    seek = (data & 0x40) >> 6;
    // 	
    data = i2c_rcv_cnt();
    freq = data;

    // reg0B
    data = i2c_rcv_cnt();
    rssi = data;
    // 	
    data = i2c_rcv_end();

    i2c_end();

    return freq;
}

void reg_set(uint8_t freq, uint8_t vol, uint8_t mode) {
    uint8_t frq_h;
    uint8_t frq_l;
    uint8_t frq_r;
    uint8_t data;

    frq_r = (freq - 60);
    frq_h = (frq_r >> 2);
    frq_l = ((frq_r & 0x03) << 6) & 0xff;

    i2c_st();
    // write
    data = (TUN_ADR & 0xFE);
    i2c_snd(data);

    // reg02
    if (mode == M_SEQ_UP) {
        data = FUP02_H;
    } else if (mode == M_SEQ_DN) {
        data = FDN02_H;
    } else {
        data = REG02_H;
    }
    i2c_snd(data);
    // 	
    data = REG02_L;
    i2c_snd(data);

    // reg03
    if (mode == M_TUN_CHK) {
        data = TFM03_H;
    } else {
        data = frq_h;
    }
    i2c_snd(data);
    // 	
    if (mode == M_TUN_CHK) {
        data = TFM03_L;
    } else if (mode == M_VOL_SET) {
        data = (frq_l & 0xC0) | (REG03_L & 0x3F);
    } else if (mode == M_SEQ_UP) {
        data = (frq_l & 0xC0) | (REG03_L & 0x3F);
    } else if (mode == M_SEQ_DN) {
        data = (frq_l & 0xC0) | (REG03_L & 0x3F);
    } else {
        data = (frq_l & 0xC0) | (TUN03_L & 0x3F);
    }
    i2c_snd(data);

    // reg04
    data = REG04_H;
    i2c_snd(data);
    // 	
    data = REG04_L;
    i2c_snd(data);

    // reg05
    if (mode == M_SEQ_UP) {
        data = TUN05_H;
    } else if (mode == M_SEQ_DN) {
        data = TUN05_H;
    } else {
        data = REG05_H;
    }
    i2c_snd(data);
    // 	
    if (mode == M_TUN_CHK) {
        data = REG05_L;
    } else {
        data = (REG05_L & 0xF0) | (vol & 0x0F);
    }
    i2c_snd(data);

    i2c_end();
}

void bcd_num(uint8_t num) {
    uint8_t tim0[5], tim1[5];

    if (num > 15) {
        blk_led1();
        blk_led5();
        return;
    }

    for (uint8_t lp = 0; lp < 4; lp++) {
        if (num & 0x08) {
            tim0[lp] = BRIT0;
            tim1[lp] = BRIT1;
        } else {
            tim0[lp] = DARK0;
            tim1[lp] = DARK1;
        }
        num = (uint8_t) (num << 1);
    }

    for (uint8_t lp = 0; lp < 60; lp++) {
        if ((num == 0) || (tim1[0] == BRIT1)) {
            LED2_ON;
        }
        if (num != 0) {
            for (uint8_t us = 0; us < tim1[0]; us++) {
                LED_WAIT;
            }
        }
        LED2_OFF;
        for (uint8_t us = 0; us < tim0[0]; us++) {
            LED_WAIT;
        }

        if ((num == 0) || (tim1[1] == BRIT1)) {
            LED3_ON;
        }
        if (num != 0) {
            for (uint8_t us = 0; us < tim1[1]; us++) {
                LED_WAIT;
            }
        }
        LED3_OFF;
        for (uint8_t us = 0; us < tim0[1]; us++) {
            LED_WAIT;
        }

        if ((num == 0) || (tim1[2] == BRIT1)) {
            LED4_ON;
        }
        if (num != 0) {
            for (uint8_t us = 0; us < tim1[2]; us++) {
                LED_WAIT;
            }
        }
        LED4_OFF;
        for (uint8_t us = 0; us < tim0[2]; us++) {
            LED_WAIT;
        }

        if ((num == 0) || (tim1[3] == BRIT1)) {
            LED5_ON;
        }
        if (num != 0) {
            for (uint8_t us = 0; us < tim1[3]; us++) {
                LED_WAIT;
            }
        }
        LED5_OFF;
        for (uint8_t us = 0; us < tim0[3]; us++) {
            LED_WAIT;
        }
    }
}

void frq_bcd_disp(uint8_t freq) {
    uint8_t frq3 = 7;
    uint8_t frq2 = 0;
    uint8_t frq1 = 0;

    while (freq > 99) {
        frq3++;
        freq = freq - 100;
    }
    if (frq3 > 9) {
        frq3 = 5;
    }
    while (freq > 9) {
        frq2++;
        freq = freq - 10;
    }
    if (frq2 > 9) {
        frq3 = 5;
    }
    while (freq > 0) {
        frq1++;
        freq = freq - 1;
    }
    if (frq1 > 9) {
        frq3 = 5;
    }

    bcd_num(frq3);
    //Wait        
    for (uint8_t lp = 0; lp < 4; lp++) {
        if ((SW_UP == PUSH_ON) || (SW_DN == PUSH_ON) || (SW_FN == PUSH_ON)) {
            return;
        } else {
            __delay_ms(50);
        }
    }

    bcd_num(frq2);
    //Wait        
    for (uint8_t lp = 0; lp < 4; lp++) {
        if ((SW_UP == PUSH_ON) || (SW_DN == PUSH_ON) || (SW_FN == PUSH_ON)) {
            return;
        } else {
            __delay_ms(50);
        }
    }

    bcd_num(frq1);
    //Wait        
    for (uint8_t lp = 0; lp < 6; lp++) {
        if ((SW_UP == PUSH_ON) || (SW_DN == PUSH_ON) || (SW_FN == PUSH_ON)) {
            return;
        } else {
            __delay_ms(50);
        }
    }
}

void main(void) {
    uint8_t freq = SET_TFM;
    uint8_t frq_seq = 0;
    uint8_t vol = 5;
    uint8_t disp_c = 3;

    //Initialize
    port_init();

    //Hard check    
    led_chk();

    //Test mode
    while (SW_FN == PUSH_ON) {
        reg_set(0, 0, M_TUN_CHK);
        for (uint8_t lp = 0; lp < 3; lp++) {
            blk_led1();
            blk_led3();
            blk_led5();
            __delay_ms(100);
        }
        __delay_ms(250);
        freq = SET_NHK;
        for (uint8_t lp = 0; lp < 16; lp++) {
            bcd_num(lp);
            __delay_ms(250);
        }
        while (SW_DN == PUSH_ON) {
            freq = SET_NAK;
            blk_led2();
        }
        while (SW_UP == PUSH_ON) {
            freq = SET_JWV;
            blk_led4();
        }
    }

    //Start
    __delay_ms(50);
    blk_led3();
    reg_set(freq, vol, M_FRQ_SET);
    __delay_ms(250);
    __delay_ms(250);

    //Loop    
    while (1) {
        uint8_t frq_rd;
        //Volume        
        if (SW_FN == PUSH_ON) {
            frq_seq = 0;
            disp_c = 0;
            __delay_ms(100);
            for (uint8_t lp = 0; lp < 10; lp++) {
                blk_led3();
                if (SW_UP == PUSH_ON) {
                    blk_led4();
                    if (vol < VOL_LMT_H) {
                        vol++;
                        lp = 0;
                        blk_led5();
                    }
                    reg_set(freq, vol, M_VOL_SET);
                }
                if (SW_DN == PUSH_ON) {
                    blk_led2();
                    if (vol > VOL_LMT_L) {
                        vol--;
                        lp = 0;
                        blk_led1();
                    }
                    reg_set(freq, vol, M_VOL_SET);
                }
                __delay_ms(200);
            }
            // exit vol set func 
            __delay_ms(250);
            __delay_ms(250);
            continue;
        }
        //Tuning
        if ((frq_seq == 0) && (disp_c < 8) && (disp_c > 1)) {
            //Frequency Up
            if (SW_UP == PUSH_ON) {
                disp_c = 2;
                freq++;
                if (freq > 250) {
                    blk_led1();
                    freq = 70;
                }

                for (uint8_t lp = 0; lp < 3; lp++) {
                    __delay_ms(200);
                    blk_led4();
                }
                //Step
                if (SW_UP != PUSH_ON) {
                    reg_set(freq, vol, M_FRQ_SET);
                    for (uint8_t lp = 0; lp < 2; lp++) {
                        __delay_ms(200);
                        blk_led4();
                    }
                    //Seek
                } else {
                    reg_set(freq, vol, M_SEQ_UP);
                    frq_seq = 1;
                    for (uint8_t lp = 0; lp < 5; lp++) {
                        __delay_ms(200);
                        blk_led5();
                    }
                }
                continue;
            }

            //Frequency Down
            if (SW_DN == PUSH_ON) {
                disp_c = 2;
                freq--;
                if (freq < 70) {
                    blk_led5();
                    freq = 250;
                }

                for (uint8_t lp = 0; lp < 3; lp++) {
                    __delay_ms(200);
                    blk_led2();
                }
                //Step
                blk_led2();
                if (SW_DN != PUSH_ON) {
                    reg_set(freq, vol, M_FRQ_SET);
                    for (uint8_t lp = 0; lp < 2; lp++) {
                        __delay_ms(200);
                        blk_led2();
                    }
                    //Seek
                } else {
                    reg_set(freq, vol, M_SEQ_DN);
                    frq_seq = 1;
                    for (uint8_t lp = 0; lp < 5; lp++) {
                        __delay_ms(200);
                        blk_led1();
                    }
                }
                continue;
            }
        }

        //Wait        
        for (uint8_t lp = 0; lp < 10; lp++) {
            __delay_ms(50);
            if ((SW_UP == PUSH_ON) || (SW_DN == PUSH_ON) || (SW_FN == PUSH_ON)) {
                if (disp_c > 7) {
                    blk_led1();
                    blk_led5();
                    disp_c = 0;
                }
                break;
            }
        }
        frq_seq = 0;

        //Display
        disp_c++;
        if (disp_c > 120) {
            disp_c = 6;
        } else if (disp_c > 7) {
            __delay_ms(100);
            continue;
        } else if (disp_c < 3) {
            __delay_ms(100);
            continue;
        }
        //active counter 3,4,5,6,7
        frq_rd = tun_rcv();
        if (frq_rd > 190) {
            blk_led2();
            blk_led4();
            freq = 40;
        } else {
            freq = (frq_rd + 60);
        }
        //sleep break
        frq_bcd_disp(freq);
        if ((SW_UP == PUSH_ON) || (SW_DN == PUSH_ON)) {
            if (disp_c > 7) {
                blk_led1();
                blk_led5();
                disp_c = 0;
            }
            continue;
        } else {
            __delay_ms(100);
        }
    }
}
