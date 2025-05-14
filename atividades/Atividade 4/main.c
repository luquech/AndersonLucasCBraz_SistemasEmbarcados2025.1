#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED0 20
#define LED1 45
#define LED2 37
#define LED3 42

#define BOTAO1 4
#define BOTAO2 3

uint8_t contador = 0;  
uint8_t soma = 1;

unsigned long contadordebounce = 0;
unsigned long contadordebounce2 = 0;

void atualizar_leds() {
    gpio_set_level(LED0, contador & 1);
    gpio_set_level(LED1, (contador >> 1) & 1);
    gpio_set_level(LED2, (contador >> 2) & 1);
    gpio_set_level(LED3, (contador >> 3) & 1);
}

void configurar_gpio() {
    
    gpio_set_direction(LED0, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);

    gpio_set_direction(BOTAO1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTAO1, GPIO_PULLUP_ONLY);

    gpio_set_direction(BOTAO2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTAO2, GPIO_PULLUP_ONLY);
}

void app_main() {
    configurar_gpio();
    atualizar_leds();
    
    while (1) {
        if (gpio_get_level(BOTAO1) == 0) {
          while(1){
            contadordebounce++;
            if (contadordebounce == 100000){
            break;}
            }
            if (contadordebounce == 100000){
              if (contador < 15){ 
              if ((contador + soma) <= 15){contador = contador +soma;}
              else if ((contador + soma) >15){
                contador = (contador+soma) - 16;
              }
            contadordebounce = 0;
            atualizar_leds();
            printf("Contador: %d\n", contador);}
            
            else if(contador == 15) {contador = -1 + soma;
            contadordebounce = 0;
            atualizar_leds();
            printf("Contador: %d\n", contador);}
            }
              
        }

        
        if (gpio_get_level(BOTAO2) == 0) {
          while(1){
            contadordebounce2++;
            if (contadordebounce2 == 100000){
            break;}
            }
            if (contadordebounce2 == 100000){
            if (soma == 1) soma = 2;
            else if(soma == 2) soma =1;
            contadordebounce2 = 0;
            printf("Soma atualizada: %d\n", soma);
            }  
        }

          
    }
}
