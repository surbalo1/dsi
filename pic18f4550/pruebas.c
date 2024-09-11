#include "bsp.h"

int flag = 1;               //indica si el sistema esta prendido
long int longdato;          // Variable de 2 bytes para guardar un dato de 10 bits
float volt;

#INT_EXT      //Interrupcion por el Puerto B
void intext_isr() { 

  if(flag==1){         //checa si PORTC_0 = 1
    flag=0;            //Si es verdadero cambia a PORTC_0 = 0
  }
  else {
    flag=1;              //Si no es verdadero cambia a PORTC_0 = 1

  }

}



void main() { 

 // initgpio();
  //pinmodo(pin,direccion,port)
  modoPin(0,INPUT_BSP,PORTA);    //PINA0 ENTRADA DEL ADC 
  modoPin(0,INPUT_PULL_UP,PORTB);//PINB0 ENTRADA CON PULLUP encender y apagar sistema 
  modoPin(3,INPUT_PULL_UP,PORTB);//PINB3 ENTRADA CON PULLUP desabilitar la lectura del adc
  
  modopin(0,OUTPUT,PORTD);
  modopin(1,OUTPUT,PORTD);
  modopin(2,OUTPUT,PORTD);
  modopin(4,OUTPUT,PORTD);
  modopin(5,OUTPUT,PORTD);
  modopin(6,OUTPUT,PORTD);
  modopin(7,OUTPUT,PORTD);
  
  initadc(ADC_CLOCK_DIV_8, 0,AN0); //PIN A0 Y CANAL 0
  

  activ_interrupt_EXT();// b7 ;Habilita las interrupciones globalmente
                              // b4 ;Habilita la interrupción por el bit RB0
   // ENABLE_INTERRUPTS(GLOBAL | INT_RB);
                              
   initcom();
   delaybsp(6);            // Retardo de 6 ms
   
  char cadena[];
  
    while(1){

      if(flag==1){     //sistema encendido
         while(!(readpin(PIN_B3))){
        
           sprintf(cadena,"No_Disponible     "); 
           imprimir(cadena,1,1);
         }
            longdato= getadc();      // Se lee el canal 0 y se guarda en la variable longdato(10 bits)
            volt = (longdato*5)/1024.00;
      
      
           sprintf(cadena,"VOLTAJE: %f",volt); 
           imprimir(cadena,1,1);
             delaybsp(  250 );
      
      } else {   //sistema apagado
   
         sprintf(cadena,"OFF                    "); 
         imprimir(cadena,1,1);
      }
  }
}


