
#include "bsp.h"

// Definición de pines
const int botonPin = 23;         // Pin del botón
const int ledPrendidoPin = 25;   // LED para indicar que el sistema está encendido
const int ledApagadoPin = 26;    // LED para indicar que el sistema está apagado
const int adcPin = 34;           // Pin para lectura del ADC

// Función para inicializar hardware
void inicializarHardware() {
    modoPin(botonPin, INPUT_PULLUP);  // Configurar botón como entrada con resistencia pull-up
    modoPin(ledPrendidoPin, OUTPUT);  // Configurar LED de sistema encendido como salida
    modoPin(ledApagadoPin, OUTPUT);   // Configurar LED de sistema apagado como salida
    modoPin(adcPin, INPUT);           // Configurar pin del ADC como entrada
}

// Variables globales
volatile bool botonPresionado = false;
volatile unsigned long tiempoUltimaPresion = 0;
const unsigned long debounceDelay = 10;
bool sistemaActivo = false;
const float VREF = 3.3;
const float MAX_VOLTAGE = 5.0;
TaskHandle_t tareaADC;
TaskHandle_t tareaControl;

void IRAM_ATTR isrBoton() {
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoUltimaPresion > debounceDelay) {
        botonPresionado = true;
        tiempoUltimaPresion = tiempoActual;
    }
}

void tareaLeerADC(void *parameter) {
    char cadena[50];  // Buffer para sprintf
    
    while (true) {
        if (sistemaActivo) {
            int valorADC = getadc(adcPin);
            float voltaje = (valorADC / 4095.0) * VREF;
            voltaje = (voltaje / VREF) * MAX_VOLTAGE;

            sprintf(cadena, "Valor ADC en 0-5V: %.2f", voltaje);  // Formato flotante con 2 decimales
            imprimir(cadena);  // Reemplazo de Serial.println
        } else {
            sprintf(cadena, "No_Disponible");
            imprimir(cadena);  // Reemplazo de Serial.println
        }
        delaybsp(pdMS_TO_TICKS(1000));
    }
}

void tareaControlSistema(void *parameter) {
    char cadena[30];  // Buffer para sprintf
    int contadorPresiones = 0;
    
    while (true) {
        if (botonPresionado) {
            botonPresionado = false;
            contadorPresiones++;
            
            if (contadorPresiones % 2 != 0) {
                sistemaActivo = true;
                sprintf(cadena, "Sistema prendido");
                imprimir(cadena);  // Reemplazo de Serial.println
                writepin(ledPrendidoPin, HIGH);
                writepin(ledApagadoPin, LOW);
            } else {
                sistemaActivo = false;
                sprintf(cadena, "Sistema apagado");
                imprimir(cadena);  // Reemplazo de Serial.println
                writepin(ledPrendidoPin, LOW);
                writepin(ledApagadoPin, HIGH);
            }
        }
        delaybsp(pdMS_TO_TICKS(50));
    }
}

void setup() {
    inicializarHardware();
    initcom(9600);  // Inicializa la comunicación serial (modificada)

    
    #ifdef ESP_32
    attachInterrupt(digitalPinToInterrupt(botonPin), isrBoton, FALLING);
    xTaskCreatePinnedToCore(tareaControlSistema, "ControlSistema", 1024, NULL, 1, &tareaControl, 1);
    xTaskCreatePinnedToCore(tareaLeerADC, "LeerADC", 2048, NULL, 1, &tareaADC, 1);
    #endif

}

void loop() {
    // El loop está vacío ya que FreeRTOS maneja las tareas en ESP32
}
