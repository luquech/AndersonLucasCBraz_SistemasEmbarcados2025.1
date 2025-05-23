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

volatile uint8_t contador = 0;
volatile uint8_t soma = 1;
volatile uint64_t ultimo_tempo_botao1 = 0;
volatile uint64_t ultimo_tempo_botao2 = 0;
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

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    uint64_t agora = esp_timer_get_time();
    
    if (gpio_num == BOTAO1) {
        if ((agora - ultimo_tempo_botao1) > debounce_time) {
            ultimo_tempo_botao1 = agora;
            contador = (contador < 15) ? contador + soma : soma - 1;
            if (contador > 15) contador -= 16;
        }
    } 
    else if (gpio_num == BOTAO2) {
        if ((agora - ultimo_tempo_botao2) > debounce_time) {
            ultimo_tempo_botao2 = agora;
            soma = (soma == 1) ? 2 : 1;
        }
    }
}

void app_main() {
    configurar_gpio();
    atualizar_leds();
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO1, gpio_isr_handler, (void*) BOTAO1);
    gpio_isr_handler_add(BOTAO2, gpio_isr_handler, (void*) BOTAO2);

    while(1) {
        atualizar_leds();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
