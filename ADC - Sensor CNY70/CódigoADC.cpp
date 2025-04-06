#include <stdio.h>
#include "stm32f7xx.h"

// Define correct pin mapping based on test results
#define DIGIT_1_PIN 0  // Extremo izquierdo (PD0) - 0xFE activa este dígito
#define DIGIT_2_PIN 1  // Segundo desde izquierda (PD1) - 0xFD activa este dígito
#define DIGIT_3_PIN 2  // Tercero desde izquierda (PD2) - 0xFB activa este dígito
#define DIGIT_4_PIN 3  // Extremo derecho (PD3) - 0xF7 activa este dígito

// Valores para mostrar cada número (0-9) para display CÁTODO COMÚN
// IMPORTANTE: Movido al inicio para que esté disponible en todas las funciones
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

// Patrones para letras (A-z, g, r, J, N, R) para display CÁTODO COMÚN
// Índices: 0=A, 1=z, 2=g, 3=r, 4=J, 5=N, 6=R
const uint8_t seven_segment_letters[16] = {
    0b01110111,  // A: Segmentos A, B, C, E, F, G
    0b01011011,  // z: Similar a 2 pero adaptado para la letra z
    0b01111101,  // g: Similar a 6 pero adaptado para la letra g
    0b01010000,  // r: Segmentos E, G (r minúscula)
    0b00001110,  // J: Segmentos B, C, D (J mayúscula)
    0b01110110,  // N: Segmentos A, B, C, E, F y adaptado
    0b01010101,  // R: Segmentos A, B, E, F, G adaptado
    0b01010100,  // n: Segmentos C, E, G (n minúscula)
    0b01111100,  // b: Segmentos C, D, E, F, G (b minúscula)
    0b01111001,  // E: Segmentos A, D, E, F, G
    0b01110001,  // F: Segmentos A, E, F, G
    0b01110011,  // P: Segmentos A, B, E, F, G
    0b01011110,  // d: Segmentos B, C, D, E, G (d minúscula)
    0b01011100,  // o: Segmentos C, D, E, G (o minúscula)
    0b01111001,  // E: Segmentos A, D, E, F, G
    0b01110001   // F: Segmentos A, E, F, G
};

// Códigos de identificación para los colores
// Cada color tiene 4 caracteres: 2 letras y 2 números
const uint8_t color_codes[4][4] = {
    {10, 1, 0, 1},    // Azul: "Az01" (A=10, z=1, 0, 1)
    {2, 3, 3, 2},     // Gris: "gr32" (g=2, r=3, 3, 2)
    {5, 3, 4, 5},     // Negro: "Nr45" (N=5, r=3, 4, 5)
    {6, 4, 9, 7}      // Rojo: "RJ97" (R=6, J=4, 9, 7)
};

// Variable para el contador (ahora hasta 9999)
volatile uint16_t counter = 0;

// Variable para controlar qué dígito se muestra (para multiplexación)
volatile uint8_t current_digit = 0;

// Variables para controlar los tiempos con SysTick
volatile uint32_t tick_counter = 0;     // Contador general de ticks (1ms cada uno)
volatile uint16_t display_update = 0;   // Flag para actualizar display en bucle principal

// Variables para control de modo
volatile uint8_t operation_mode = 0;    // 0: Contador normal, 1: Identificación de color
volatile uint32_t color_mode_start = 0; // Almacena el tiempo de inicio del modo color
volatile uint8_t color_detected = 0;    // Código de color detectado
volatile uint8_t color_display[4] = {0}; // Valores a mostrar para el color detectado

// Forward declarations (declaraciones anticipadas)
void UpdateDisplay(void);
void MySystemInit(void);

//----------------------------------------------ADC-----------------------------------//
// Variable para el voltaje leído
volatile uint16_t voltage = 0;
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


// Función para inicializar la interrupción PB2
void Init_PB2_Interrupt(void) {
    // 1. Habilitar el reloj para GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    // 2. Configurar PB2 como entrada con pull-up
    GPIOB->MODER &= ~(0x3 << (2*2));      // Limpiar bits (entrada = 00)
    GPIOB->PUPDR &= ~(0x3 << (2*2));      // Limpiar bits de pull-up/down
    GPIOB->PUPDR |= (0x1 << (2*2));       // Activar pull-up (01)
    
    // 3. Configurar interrupción externa para PB2
    // Habilitar el reloj para SYSCFG
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    // Conectar PB2 a EXTI2 (línea 2)
    SYSCFG->EXTICR[0] &= ~(0xF << 8);     // Limpiar bits EXTI2[3:0]
    SYSCFG->EXTICR[0] |= (0x1 << 8);      // Conectar puerto B a EXTI2 (0001)
    
    // Configurar EXTI2 (flanco descendente)
    EXTI->RTSR &= ~(1 << 2);              // Desactivar trigger en flanco ascendente
    EXTI->FTSR |= (1 << 2);               // Activar trigger en flanco descendente
    EXTI->IMR |= (1 << 2);                // Desenmascarar interrupción para EXTI2
    
    // 4. Configurar NVIC para EXTI2
    NVIC_SetPriority(EXTI2_IRQn, 3);      // Prioridad media-baja
    NVIC_EnableIRQ(EXTI2_IRQn);           // Habilitar interrupción
}

// Función para determinar el color basado en la lectura ADC
void DetectColor(void) {
    // Realizar la lectura ADC
    adc_main();
    
    // Determinar el color basado en el rango de voltaje
    if (volts >= 0.19 && volts < 0.21) {
        // Azul - "Az01"
        color_display[0] = 10 + 0;  // 'A' (índice 0)
        color_display[1] = 10 + 1;  // 'z' (índice 1)
        color_display[2] = 0;       // '0'
        color_display[3] = 1;       // '1'
        color_detected = 1;
    }
    else if (volts >= 0.29 && volts < 0.34) {
        // Gris - "gr32"
        for (int i = 0; i < 4; i++) {
            color_display[i] = color_codes[1][i];
        }
        color_detected = 2;
    }
    else if (volts >= 0.1 && volts < 0.7) {
        // Negro - "Nr45"
        for (int i = 0; i < 4; i++) {
            color_display[i] = color_codes[2][i];
        }
        color_detected = 3;
    }
    else if (volts >= 0.84 && volts < 0.89) {
        // Rojo - "RJ97"
        for (int i = 0; i < 4; i++) {
            color_display[i] = color_codes[3][i];
        }
        color_detected = 4;
    }
    else {
        // Fuera de rango - "ndEF"
        color_display[0] = 10 + 7;  // 'n' (índice 7)
        color_display[1] = 10 + 12; // 'd' (índice 12)
        color_display[2] = 10 + 9;  // 'E' (índice 9)
        color_display[3] = 10 + 10; // 'F' (índice 10)
        color_detected = 5;
    }
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

// Función para mostrar en display según modo
void UpdateDisplayWithMode(void) {
    // Apagar todos los dígitos para evitar fantasmas
    GPIOD->ODR = 0xFF;  // Para cátodo común, 1 = apagado
    
    if (operation_mode == 0) {
        // Modo contador - usar función existente
        UpdateDisplay();
    } 
    else {
        // Modo identificación de color - mostrar resultado
        uint8_t digit_value = 0;
        
        // Extraer el dígito correspondiente para el modo color
        switch (current_digit) {
            case 0:  // Dígito extremo derecho - último carácter (0)
                digit_value = color_display[3];
                if (digit_value < 10) {
                    // Es un dígito numérico
                    GPIOE->ODR = seven_segment_digits[digit_value];
                } else {
                    // Es una letra
                    uint8_t letter_index = digit_value - 10;
                    if (letter_index < 16) {
                        GPIOE->ODR = seven_segment_letters[letter_index];
                    } else {
                        GPIOE->ODR = 0; // Apagar segmentos si índice inválido
                    }
                }
                GPIOD->ODR = 0xF7;  // Activar extremo derecho (PD3)
                break;
                
            case 1:  // Segundo dígito desde la derecha - penúltimo carácter (r)
                digit_value = color_display[2];
                if (digit_value < 10) {
                    // Es un dígito numérico
                    GPIOE->ODR = seven_segment_digits[digit_value];
                } else {
                    uint8_t letter_index = digit_value - 10;
                    if (letter_index < 16) {
                        GPIOE->ODR = seven_segment_letters[letter_index];
                    } else {
                        GPIOE->ODR = 0; // Apagar segmentos si índice inválido
                    }
                }
                GPIOD->ODR = 0xFB;  // Activar segundo desde derecha (PD2)
                break;
                
            case 2:  // Tercer dígito - segundo carácter (r)
                digit_value = color_display[1];
                if (digit_value < 10) {
                    // Es un dígito numérico
                    GPIOE->ODR = seven_segment_digits[digit_value];
                } else {
                    uint8_t letter_index = digit_value - 10;
                    if (letter_index < 16) {
                        GPIOE->ODR = seven_segment_letters[letter_index];
                    } else {
                        GPIOE->ODR = 0; // Apagar segmentos si índice inválido
                    }
                }
                GPIOD->ODR = 0xFD;  // Activar tercer dígito (PD1)
                break;
                
            case 3:  // Cuarto dígito - primer carácter (E)
                digit_value = color_display[0];
                if (digit_value < 10) {
                    // Es un dígito numérico
                    GPIOE->ODR = seven_segment_digits[digit_value];
                } else {
                    uint8_t letter_index = digit_value - 10;
                    if (letter_index < 16) {
                        GPIOE->ODR = seven_segment_letters[letter_index];
                    } else {
                        GPIOE->ODR = 0; // Apagar segmentos si índice inválido
                    }
                }
                GPIOD->ODR = 0xFE;  // Activar cuarto dígito (PD0)
                break;
        }
        
        // Avanzar al siguiente dígito para la próxima actualización
        current_digit = (current_digit + 1) % 4;
    }
}

// Manejador de interrupción para EXTI2 (PB2)
extern "C" void EXTI2_IRQHandler(void) {
    if (EXTI->PR & (1 << 2)) {
        // Limpiar flag de interrupción pendiente
        EXTI->PR |= (1 << 2);
        
        // Cambiar al modo identificación de color
        operation_mode = 1;
        
        // Guardar tiempo actual como inicio del modo color
        color_mode_start = tick_counter;
        
        // Detectar color
        DetectColor();
    }
}

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

// Manejador de interrupción para SysTick (cada 1ms)
extern "C" void SysTick_Handler(void) {
    // Incrementar contador general de ticks
    tick_counter++;
    
    // Verificar si debemos volver al modo contador (después de 30 segundos = 30000ms)
    if (operation_mode == 1 && (tick_counter - color_mode_start) >= 30000) {
        operation_mode = 0; // Volver al modo contador normal
    }
    
    // Actualizar display con una alta frecuencia (cada 2ms)
    if (tick_counter % 2 == 0) {
        display_update = 1; // Flag para actualizar el display en el bucle principal
    }
    
    // Detectar color constantemente en modo color (cada 100ms)
    if (operation_mode == 1 && tick_counter % 100 == 0) {
        DetectColor(); // Actualizar la detección de color periódicamente
    }
    
    // Actualizar el contador del display (cada 500ms = 0.5 segundos)
    if (tick_counter % 500 == 0 && operation_mode == 0) {
        // Incrementar contador solo en modo contador
        counter++;
        if (counter > 9999) {
            counter = 0;
        }
    }
}

int main(void) {
    // Inicializar el sistema
    MySystemInit();
    
    // Inicializar interrupción PB2
    Init_PB2_Interrupt();
    
    // Inicializar ADC
    adc();
    
    // Iniciar con valor 0
    counter = 0;
    current_digit = 0;
    operation_mode = 0;
    
    // Asegurarse de que color_display tenga valores predeterminados válidos
    for (int i = 0; i < 4; i++) {
        color_display[i] = i; // Valores predeterminados simples (0,1,2,3)
    }
    
    // Inicializar estados de salida
    GPIOE->ODR = 0;       // Todos segmentos apagados
    GPIOD->ODR = 0xFF;    // Todos dígitos apagados (para cátodo común, 1 = apagado)
    
    // Bucle principal simplificado con SysTick
    while (1) {
        // Actualizar el display cuando lo indique la interrupción de SysTick
        if (display_update) {
            display_update = 0;
            UpdateDisplayWithMode(); // Función modificada para mostrar según el modo
        }
    }
}