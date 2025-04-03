#include <stdio.h>
#include "stm32f7xx.h"

// Define correct pin mapping based on test results
#define DIGIT_1_PIN 0  // Extremo izquierdo (PD0) - 0xFE activa este dígito
#define DIGIT_2_PIN 1  // Segundo desde izquierda (PD1) - 0xFD activa este dígito
#define DIGIT_3_PIN 2  // Tercero desde izquierda (PD2) - 0xFB activa este dígito
#define DIGIT_4_PIN 3  // Extremo derecho (PD3) - 0xF7 activa este dígito

// Variable para el contador (ahora hasta 9999)
volatile uint16_t counter = 0;

// Variable para controlar qué dígito se muestra (para multiplexación)
volatile uint8_t current_digit = 0;

// Variables para controlar los tiempos con SysTick
volatile uint32_t tick_counter = 0;     // Contador general de ticks (1ms cada uno)
volatile uint16_t display_update = 0;   // Flag para actualizar display en bucle principal





//----------------------------------------------ADC-----------------------------------//
// Variable para el voltaje leído
volatile uint16_t voltage = 0; // Corregir el tipo de dato a uint16_t
volatile double volts = 0; // Variable para almacenar el voltaje convertido



void adc(){

    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3); //enable clock for port A,B,C ,D

    RCC->APB2ENR|=(1<<8)|(1<<9)|(1<<10); //ENABLE ADC 1,2 AND 3 CLOCK

    // A PORTS AS ADC
    /*MODER 0 FOR 8 BITS ,4 MHZ, 112 CYCLES , ALLINGMENT RIGHT 
    0,3-  1V----->TURN ON BLUE LED
    1,3-  2V----->TURN ON GREEN LED
    2-    3V----->TURN ON RED LED
    */
    GPIOA->MODER|=(1<<1)|(1<<0); //PA0 as analog mode
    GPIOA->PUPDR|=(1<<1); //PA0 as pull down mode

// PORTS FOR A0 ADC CONFIGURATION
    ADC->CCR&= ~(0b11<<16); //F= 8MHZ
    ADC1->CR1&= ~(0b11<<24);//12 bit resolution
    ADC1->CR2|=(1<<0)|(1<<10); // turn on ADC & set end of conversion
    ADC1->CR2 &= ~(1<<11); //right alignment
    
    ADC1->SMPR2|=(0b111<<0); //480 cycles
    
    ADC1->SQR3 &= ~(0b11111<<0); //clear channel
    ADC1->SQR3|=(0b00000<<0); //channel 0


}

void adc_main(){
    ADC1->CR2|=(1<<30); //start conversion

    while(((ADC1->SR & (1<<1)) >> 1) == 0){

        ADC2->SR &= ~(1<<1);

    } //wait for conversion to complete (EOC flag)
        
        voltage = ADC1->DR; //read value
        
        volts = (voltage * 3.3) / 4095; //convert to volts
        
        








}


//----------------------------------------------end ADC-----------------------------------//



// Valores para mostrar cada número (0-9) para display CÁTODO COMÚN
const uint8_t seven_segment_digits[10] = {
    0b00111111,  // 0: A, B, C, D, E, F 
    0b00000110,  // 1: B, C
    0b01011011,  // 2: A, B, D, E, G
    0b01001111,  // 3: A, B, C, D, G
    0b01100110,  // 4: B, C, F, G
    0b01101101,  // 5: A, C, D, F, G
    0b01111101,  // 6: A, C, D, E, F, G
    0b00000111,  // 7: A, B, C
    0b01111111,  // 8: A, B, C, D, E, F, G
    0b01101111   // 9: A, B, C, D, F, G
};

// Función para inicializar el sistema
void MySystemInit(void) {
    // Habilitar el reloj para GPIOD y GPIOE
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    
    // Configurar GPIOE (segmentos) como salidas
    GPIOE->MODER &= ~(0xFFFF);  // Limpiar bits para PE0-PE7
    GPIOE->MODER |= 0x5555;     // Establecer modo salida (01) para PE0-PE7
    
    // Configurar GPIOD (dígitos) como salidas
    GPIOD->MODER &= ~(0xFF);    // Limpiar bits para PD0-PD3
    GPIOD->MODER |= 0x55;       // Establecer modo salida (01) para PD0-PD3

    // Configurar SysTick para interrumpir cada 1ms (1000Hz)
    // SystemCoreClock para STM32F7 es típicamente 216MHz
    SysTick_Config(SystemCoreClock / 1000);
}

// Función para actualizar el display según el dígito actual
void UpdateDisplay(void) {
    // Apagar todos los dígitos para evitar fantasmas
    GPIOD->ODR = 0xFF;  // Para cátodo común, 1 = apagado
    
    // Calcular qué dígito y valor mostrar
    uint16_t value = counter;
    uint8_t digit_value = 0;
    
    // Extraer el dígito correspondiente
    switch (current_digit) {
        case 0:  // Dígito de unidades (extremo derecho)
            digit_value = value % 10;
            GPIOE->ODR = seven_segment_digits[digit_value];
            GPIOD->ODR = 0xF7;  // Activar PD3 (extremo derecho)
            break;
            
        case 1:  // Dígito de decenas
            value /= 10;
            digit_value = value % 10;
            // Solo mostrar si el número es >= 10
            if (counter >= 10) {
                GPIOE->ODR = seven_segment_digits[digit_value];
                GPIOD->ODR = 0xFB;  // Activar PD2 (tercero desde izquierda)
            }
            break;
            
        case 2:  // Dígito de centenas
            value /= 100;
            digit_value = value % 10;
            // Solo mostrar si el número es >= 100
            if (counter >= 100) {
                GPIOE->ODR = seven_segment_digits[digit_value];
                GPIOD->ODR = 0xFD;  // Activar PD1 (segundo desde izquierda)
            }
            break;
            
        case 3:  // Dígito de millares (extremo izquierdo)
            value /= 1000;
            digit_value = value % 10;
            // Solo mostrar si el número es >= 1000
            if (counter >= 1000) {
                GPIOE->ODR = seven_segment_digits[digit_value];
                GPIOD->ODR = 0xFE;  // Activar PD0 (extremo izquierdo)
            }
            break;
    }
    
    // Avanzar al siguiente dígito para la próxima actualización
    current_digit = (current_digit + 1) % 4;
}

// Manejador de interrupción para SysTick (cada 1ms)
extern "C" void SysTick_Handler(void) {
    // Incrementar contador general de ticks
    tick_counter++;
    
    // Actualizar display con una alta frecuencia (cada 2ms)
    // Esto da un tiempo de refresco de aproximadamente 500Hz
    if (tick_counter % 2 == 0) {
        display_update = 1; // Flag para actualizar el display en el bucle principal
    }
    
    // Actualizar el contador del display (cada 100ms = 10 veces por segundo)
    if (tick_counter % 100 == 0) {
        // Incrementar contador de 0 a 9999
        counter++;
        if (counter > 9999) {
            counter = 0;
        }
    }
}

int main(void) {
    // Inicializar el sistema
    MySystemInit();
    
    // Iniciar con valor 0
    counter = 0;
    current_digit = 0;
    
    // Inicializar estados de salida
    GPIOE->ODR = 0;       // Todos segmentos apagados
    GPIOD->ODR = 0xFF;    // Todos dígitos apagados (para cátodo común, 1 = apagado)
    
    // Bucle principal simplificado con SysTick
    while (1) {

     //---------------------------------------------DANIEL´S CODE-------------------------------------------------------------------//

     adc(); // Inicializar ADC 
     adc_main(); // Llamar a la función principal del ADC

    //---------------------------------------------END DANIEL´S CODE-------------------------------------------------------------------//


        // Actualizar el display cuando lo indique la interrupción de SysTick
        if (display_update) {
            display_update = 0;
            UpdateDisplay();
        }
    }
}
