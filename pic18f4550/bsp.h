#include "config.h"

#ifdef PIC

#include <18F4550.h>                                 
#fuses HS,NOWDT,NOPROTECT,NOLVP, CPUDIV1
#device adc=10                //resolucion del adc       
#use delay(clock=20000000)    //frecuencia de reloj

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
//redefiniciones pra tipos de datos
#define int8  int8_t 
#define int16 int16_t
#define int32 int32_t

#endif

/*****************************************************************************
Función: modoPin
Objetivo: Definir el pín de interés como entrada o como salida
Descripción: La misma función da solucióin a PIC o a ESP por medio de directi-
vas
******************************************************************************/

void modoPin(int8 _pin, int8 _dir, int8 _port= 0) {
#ifdef PIC
   int8 mask;      //mascara para afectar solo a el bit necesario
   int8 port_tris; //es necesario conocer el estado del registro a afectar 
   if(_dir == 1 || _dir ==2){   //entrada
   
   mask = 0x01<<_pin;  //corrimientos el numero del pin a configurar
   
         if(_port == PORTA){ //PORTA 
            port_tris = get_tris_a();
            port_tris |= mask; 
            set_tris_a(port_tris);
         } else if(_port == PORTB){ //PORTB
         
            port_tris = get_tris_b();
            port_tris |= mask; 
            set_tris_b(port_tris);
            if(_dir == 2){
             port_b_pullups(TRUE);
            }
                 
         }else if(_port == PORTC){//PORTC
         
            port_tris = get_tris_c();
            port_tris |= mask; 
            set_tris_c(port_tris);
         
         }else if(_port == PORTD){ //PORTD
         
            port_tris = get_tris_d();
            port_tris |= mask; 
            set_tris_d(port_tris);
            if(_dir == 2){
             port_d_pullups(TRUE);
            }
         
         }else if(_port == PORTE){ //PORTE
         
            port_tris = get_tris_e();
            port_tris |= mask; 
            set_tris_e(port_tris);
         
         }
   } else {              //SALIDAS
      mask = 0x01<<_pin;
      mask = ~mask;
         if(_port == PORTA){ //PORTA 
            port_tris = get_tris_a();
            port_tris &= mask; 
            set_tris_a(port_tris);
         } else if(_port == PORTB){ //PORTB
         
            port_tris = get_tris_b();
            port_tris &= mask; 
            set_tris_b(port_tris);
         
         }else if(_port == PORTC){//PORTC
         
            port_tris = get_tris_c();
            port_tris &= mask; 
            set_tris_c(port_tris);
         
         }else if(_port == PORTD){ //PORTD
         
            port_tris = get_tris_d();
            port_tris &= mask; 
            set_tris_d(port_tris);
         
         }else if(_port == PORTE){ //PORTE
         
            port_tris = get_tris_e();
            port_tris &= mask; 
            set_tris_e(port_tris);
         
         }
  
   }

#endif

#ifdef ESP_32
    pinMode(_pin, _dir);
#endif
}

/*****************************************************************************
Función: initadc
Objetivo: Configurar adc para su lectura
Descripción: La misma función da solucióin a PIC o a ESP por medio de directi-
vas, sin embargo para la ESP 32 solo se usa un parametro de la función, por eso
es necesario darlke valores a los demas, y en caso de que solo se llame a 
initadc(); en PIC se establecera con el canal  y el pin A0 y en el caso de la 
ESP 32 solo ser le asignara que la frecuencia se toma directa sin ninguna divición

******************************************************************************/

void initadc(int16 Clock_Div = 0x01, int8 _channel = 0, int8 _pin =  0x0E) { //funcion para inisializar 
#ifdef PIC  
    setup_adc_ports(_pin);
    setup_adc(Clock_Div);
    set_adc_channel(_channel);
#endif
#ifdef ESP_32
    analogSetClockDiv(Clock_Div);
#endif
}
/*****************************************************************************
Función: getadc
Objetivo: realizar la lectura del adc
Descripción: La misma función da solucióin a PIC o a ESP por medio de directi-
vas, sin embargo para la ESP 32 es necesario saber el pin que deseas leer, y 
para el PIC solo leera el canal 0
Funciones previas necesarias: initadc()
******************************************************************************/ 
int16 getadc(int8 _pin = 0) { //leer el adc
    int16 read = 0;
#ifdef PIC
    read = read_adc();
#endif
#ifdef ESP_32
    read = analogRead(_pin);
#endif
    return read;
}


/*****************************************************************************
Función: initcom
Objetivo: inicializa la comunicación y configura los buses
Descripción: en el caso del PIC inicializa la lcd y en el caso de la ESP 
establece la velocidad de la transmisión de datos
Funciones previas necesarias:
******************************************************************************/ 

void initcom(int32 boud = 9600) {
#ifdef PIC
    lcd_init();
#endif
#ifdef ESP_32
    Serial.begin(boud);
#endif
}

/*****************************************************************************
Función: imprimir
Objetivo: manda las cadenas a mostrar
Descripción: en el caso del PIC manda la cadena a la pantalla lcd por lo que 
es necesario proveerle las cordenadas del cursor y en la ESP solo manda la 
cadena a consola
Funciones previas necesarias: initcom()
******************************************************************************/ 

void imprimir(char* cadena, int8 x=1, int8 y=1) { //
#ifdef PIC
    lcd_gotoxy(x, y);
    printf(lcd_putc, cadena);
#endif
#ifdef ESP_32
    Serial.println(cadena);
#endif
}
/*****************************************************************************
Función: writepin
Objetivo: escribir en un pin especififo
Descripción: 
Funciones previas necesarias: modoPin()
******************************************************************************/ 
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

/*****************************************************************************
Función: delaybsp
Objetivo: realizar un retardo
Descripción: el retardo sera en ms y corresponderá al parametro que se le pase 
a la función
Funciones previas necesarias:
******************************************************************************/
void delaybsp(int16 _ms) {
#ifdef PIC
    delay_ms(_ms);
#endif
#ifdef ESP_32
    vTaskDelay(pdMS_TO_TICKS(_ms));
#endif
}
/*****************************************************************************
Función: activ_interrupt_EXT
Objetivo: inicializa la interrupçión externa en el pin BO 
Descripción: solo funciona para PIC
Funciones previas necesarias: modoPin()
******************************************************************************/

void activ_interrupt_EXT(int8 _pin= 0){

#ifdef PIC
 enable_interrupts(GLOBAL |INT_EXT );
#endif
#ifdef ESP_32
 // attachInterrupt(23, isrBoton, FALLING);
#endif

}
/*****************************************************************************
Función: readpin
Objetivo: conocer el valor digital de un pin  
Descripción: el pin debe ser configurado como entrada y en ocaciones es 
recomendable agregarle PULL-UP
Funciones previas necesarias: modoPin()
******************************************************************************/
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

