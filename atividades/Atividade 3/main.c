#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1 15
#define LED2 21

void app_main() {
  esp_rom_gpio_pad_select_gpio(LED1);
  gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
  esp_rom_gpio_pad_select_gpio(LED2);
  gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
  int LIGAR = 0;
  int LIGAR2 = 0;
  int contador = 0;

  while (true) {

    contador +=1;
    if(contador % 1 == 0){
      LIGAR2 = !LIGAR2;
      gpio_set_level(LED2, LIGAR2);
      printf("Piscar LED2\n");
    }

    if (contador % 5 == 0){
      LIGAR = !LIGAR;
      gpio_set_level(LED1, LIGAR);
      printf("Piscar LED1\n");
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);

  }
}
