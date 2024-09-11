#include "config.h"
#ifdef PIC
#include <18F4550.h>
#fuses HS,NOWDT,NOPROTECT,NOLVP, CPUDIV1
#device adc=10         // ResoluciÃ³n del convertidor anÃ¡logo de 10 bits
#use delay(clock=20000000)
#define OUTPUT              0
#define INPUT_BSP           1
#define INPUT_PULL_UP       2
#define PORTA  0
#define PORTB  1
#define PORTC  2
#define PORTD  3
#define PORTE  4

// Declaracion de variables
#define LCD_RS_PIN      PIN_D0
#define LCD_RW_PIN      PIN_D1
#define LCD_ENABLE_PIN  PIN_D2
#define LCD_DATA4       PIN_D4                                    
#define LCD_DATA5       PIN_D5                                   
#define LCD_DATA6       PIN_D6                                   
#define LCD_DATA7       PIN_D7 
#include <lcd.c>         // Libreria del LCD

#endif

#ifdef ESP_32
#include <Arduino.h> 
#define int8  int8_t
#define int16 int16_t
#define int32 int32_t
#endif

void modoPin(int8 _pin, int8 _dir = 0, int8 _port= 0) {
#ifdef PIC
    int8 mask = 0x01 << _pin;
    int8 port_tris;

    if (_dir == 1 || _dir == 2) {  // Entrada
        if (_port == PORTA) {
            port_tris = get_tris_a();
            set_tris_a(port_tris | mask);
        } else if (_port == PORTB) {
            port_tris = get_tris_b();
            set_tris_b(port_tris | mask);
            if (_dir == 2) port_b_pullups(TRUE);
        } else if (_port == PORTC) {
            port_tris = get_tris_c();
            set_tris_c(port_tris | mask);
        } else if (_port == PORTD) {
            port_tris = get_tris_d();
            set_tris_d(port_tris | mask);
            if (_dir == 2) port_d_pullups(TRUE);
        } else if (_port == PORTE) {
            port_tris = get_tris_e();
            set_tris_e(port_tris | mask);
        }
    } else {  // Salidas
        mask = ~mask;
        if (_port == PORTA) {
            port_tris = get_tris_a();
            set_tris_a(port_tris & mask);
        } else if (_port == PORTB) {
            port_tris = get_tris_b();
            set_tris_b(port_tris & mask);
        } else if (_port == PORTC) {
            port_tris = get_tris_c();
            set_tris_c(port_tris & mask);
        } else if (_port == PORTD) {
            port_tris = get_tris_d();
            set_tris_d(port_tris & mask);
        } else if (_port == PORTE) {
            port_tris = get_tris_e();
            set_tris_e(port_tris & mask);
        }
    }
#endif

#ifdef ESP_32
    pinMode(_pin, _dir);
#endif
}

void initadc(int16 Clock_Div = 0x01, int8 _channel = 0, int8 _pin = 0) {
#ifdef PIC  
    setup_adc_ports(_pin);
    setup_adc(Clock_Div);
    set_adc_channel(_channel);
#endif
#ifdef ESP_32
    analogSetClockDiv(Clock_Div);
#endif
}

int16 getadc(int8 _pin = 0) {
    int16 read = 0;
#ifdef PIC
    read = read_adc();
#endif
#ifdef ESP_32
    read = analogRead(_pin);
#endif
    return read;
}

void imprimir(char* cadena, int8 x=1, int8 y=1) {
#ifdef PIC
    lcd_gotoxy(x, y);
    printf(lcd_putc, cadena);
#endif
#ifdef ESP_32
    Serial.println(cadena);
#endif
}

void writepin(int8 _pin, int8 _valor = 0) {
#ifdef PIC
    if (_valor == 1) {
        output_high(_pin);
    } else {
        output_low(_pin);
    }
#endif
#ifdef ESP_32
    digitalWrite(_pin, _valor);
#endif
}

void delaybsp(int16 _ms) {
#ifdef PIC
    delay_ms(_ms);
#endif
#ifdef ESP_32
    vTaskDelay(pdMS_TO_TICKS(_ms));
#endif
}

void initcom(int32 boud = 9600) {
#ifdef PIC
    lcd_init();
#endif
#ifdef ESP_32
    Serial.begin(boud);
#endif
}

void activ_interrupt_EXT(int8 _pin= 0){

#ifdef PIC
 enable_interrupts(GLOBAL |INT_EXT );
#endif
#ifdef ESP_32
 // attachInterrupt(23, isrBoton, FALLING);
#endif

}

int readpin(int32 _pin){
int read;
#ifdef PIC
read = input(_pin);
return read;
#endif

#ifdef ESP_32
read = digitalRead(_pin);
return read;
#endif

}