#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED0 20
#define LED1 45
#define LED2 37
#define LED3 42
#define BOTAO1 4
#define BOTAO2 3

uint8_t contador = 0;
uint8_t soma = 1;
uint64_t ultimo_tempo_botao1 = 0;
uint64_t ultimo_tempo_botao2 = 0;
uint8_t ultimo_contador = 0;
uint8_t ultima_soma = 1;
const uint64_t debounce_time = 100000;

void atualizar_leds() {
    gpio_set_level(LED0, contador & 1);
    gpio_set_level(LED1, (contador >> 1) & 1);
    gpio_set_level(LED2, (contador >> 2) & 1);
    gpio_set_level(LED3, (contador >> 3) & 1);
}

void configurar_gpio() {
    gpio_reset_pin(LED0);
    gpio_set_direction(LED0, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED1);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED2);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED3);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);

    gpio_reset_pin(BOTAO1);
    gpio_set_direction(BOTAO1, GPIO_MODE_INPUT);
    gpio_pullup_en(BOTAO1);
    gpio_set_intr_type(BOTAO1, GPIO_INTR_NEGEDGE);

    gpio_reset_pin(BOTAO2);
    gpio_set_direction(BOTAO2, GPIO_MODE_INPUT);
    gpio_pullup_en(BOTAO2);
    gpio_set_intr_type(BOTAO2, GPIO_INTR_NEGEDGE);
}

static void IRAM_ATTR botao1_handler(void* arg) {
    uint64_t agora = esp_timer_get_time();
    if ((agora - ultimo_tempo_botao1) > debounce_time) {
        ultimo_tempo_botao1 = agora;
        if (contador < 15) {
            contador = contador + soma;
            if (contador > 15) {
                contador = contador - 16;
            }
        } else {
            contador = soma - 1;
        }
    }
}

static void IRAM_ATTR botao2_handler(void* arg) {
    uint64_t agora = esp_timer_get_time();
    if ((agora - ultimo_tempo_botao2) > debounce_time) {
        ultimo_tempo_botao2 = agora;
        if (soma == 1) {
            soma = 2;
        } else {
            soma = 1;
        }
    }
}

void app_main() {
    configurar_gpio();
    atualizar_leds();
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO1, botao1_handler, NULL);
    gpio_isr_handler_add(BOTAO2, botao2_handler, NULL);

    printf("Contador: %d, Soma: %d\n", contador, soma);
    
    while(1) {
        atualizar_leds();
        
        if (contador != ultimo_contador || soma != ultima_soma) {
            printf("Contador: %d, Soma: %d\n", contador, soma);
            ultimo_contador = contador;
            ultima_soma = soma;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
