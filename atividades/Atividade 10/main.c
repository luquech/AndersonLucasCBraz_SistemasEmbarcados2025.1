#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "math.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "esp_vfs_fat.h"
#include <string.h>
#include <time.h>
#include <sys/unistd.h>
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"

// Pinos
#define A 2
#define B 1
#define C 48
#define D 45
#define E 37
#define F 41
#define G 40
#define DP 47

#define LED_PWM 6
#define BOTAO1 4
#define BOTAO2 3
#define SENSOR 8

// I2C para LCD
#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_SDA_IO 19
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT_NUM 0
#define LCD_ADDR 0x27 

// PWM
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 5000

// Comandos LCD
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_DDRAM_ADDR 0x80

// Flags LCD
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00

#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// SD Card
#define MOUNT_POINT "/sdcard"
#define LOG_FILE "/sdcard/ntc_log.csv"
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 12
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

// Variáveis globais
double ValorNTC = 0;
uint8_t alarme = 25;
uint64_t ultimo_tempo_botao1 = 0;
uint64_t ultimo_tempo_botao2 = 0;
const uint64_t debounce_time = 100000;
static sdmmc_card_t* card;
static bool sd_card_initialized = false;

// Variáveis RTOS
QueueHandle_t xTempQueue = NULL;
SemaphoreHandle_t xSDCardMutex = NULL;

// Funções LCD
void lcd_send_nibble(uint8_t data, uint8_t rs) {
    uint8_t nibble = (data & 0xF0) | rs | LCD_BACKLIGHT | 0x04;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, nibble, true);
    i2c_master_write_byte(cmd, nibble & ~0x04, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_PORT_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

void lcd_send_byte(uint8_t data, uint8_t rs) {
    lcd_send_nibble(data, rs);
    lcd_send_nibble(data << 4, rs);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void lcd_send_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, 0x00);
}

void lcd_send_data(uint8_t data) {
    lcd_send_byte(data, 0x01);
}

void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_nibble(0x20, 0); 
    vTaskDelay(pdMS_TO_TICKS(1));

    lcd_send_cmd(LCD_FUNCTION_SET | LCD_2LINE | LCD_5x8DOTS);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd(LCD_CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_cmd(LCD_ENTRY_MODE_SET | 0x02); 
    vTaskDelay(pdMS_TO_TICKS(1));
}

void lcd_print_str(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_clear() {
    lcd_send_cmd(LCD_CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void atualizar_lcd() {
    char line1[17], line2[17];
    snprintf(line1, sizeof(line1), "NTC: %.2f C", ValorNTC);
    snprintf(line2, sizeof(line2), "Alarme: %d", alarme);
    
    lcd_send_cmd(LCD_SET_DDRAM_ADDR | 0x00);
    lcd_print_str(line1);
    
    lcd_send_cmd(LCD_SET_DDRAM_ADDR | 0x40);
    lcd_print_str(line2);
}

void atualizar_leds() {
    static bool estado_pisca = false;
    static uint64_t ultimo_tempo_pisca = 0;
    int diferenca = alarme - ValorNTC;

    // Mapeamento correto dos segmentos para um display de 7 segmentos comum
    // Segmentos: A (topo), B (superior direito), C (inferior direito), D (base)
    //            E (inferior esquerdo), F (superior esquerdo), G (meio)
    void set_display(uint8_t a, uint8_t b, uint8_t c, uint8_t d, 
                    uint8_t e, uint8_t f, uint8_t g, uint8_t dp) {
        gpio_set_level(A, a);
        gpio_set_level(B, b);
        gpio_set_level(C, c);
        gpio_set_level(D, d);
        gpio_set_level(E, e);  // Segmento inferior esquerdo
        gpio_set_level(F, f);
        gpio_set_level(G, g);
        gpio_set_level(DP, dp);
    }

    // Verifica se está no modo de alarme
    if (ValorNTC >= alarme) {
        uint64_t agora = esp_timer_get_time();
        if (agora - ultimo_tempo_pisca > 500000) {
            estado_pisca = !estado_pisca;
            ultimo_tempo_pisca = agora;
        }

        // Dígito "F" (segmentos A, F, G, E ativos)
        if (estado_pisca) {
            set_display(1, 0, 0, 0, 1, 1, 1, 0); // A, F, G, E ligados
        } else {
            set_display(0, 0, 0, 0, 0, 0, 0, 0);
        }

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 255);
    } else {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);

        // Mostra dígitos conforme a diferença de temperatura
        if (diferenca <= 2) {
            // Dígito "D" (segmentos B, C, D, E, G ativos)
            set_display(0, 1, 1, 1, 1, 0, 1, 0);
        } else if (diferenca <= 10) {
            // Dígito "7" (segmentos A, B, C ativos)
            set_display(1, 1, 1, 0, 0, 0, 0, 0);
        } else if (diferenca <= 15) {
            // Dígito "3" (segmentos A, B, C, D, G ativos)
            set_display(1, 1, 1, 1, 0, 0, 1, 0);
        } else if (diferenca <= 20) {
            // Dígito "0" (todos exceto G)
            set_display(1, 1, 1, 1, 1, 1, 0, 0); // E deve estar ativo aqui
        } else {
            set_display(0, 0, 0, 0, 0, 0, 0, 0);
        }
    }
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Funções de configuração
void configurar_gpio() {
    // Configura display de 7 segmentos
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << A) | (1ULL << B) | (1ULL << C) | (1ULL << D) |
                       (1ULL << E) | (1ULL << F) | (1ULL << G) | (1ULL << DP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configura LED PWM
    gpio_reset_pin(LED_PWM);
    gpio_set_direction(LED_PWM, GPIO_MODE_OUTPUT);

    // Configura botões
    gpio_reset_pin(BOTAO1);
    gpio_set_direction(BOTAO1, GPIO_MODE_INPUT);
    gpio_pullup_en(BOTAO1);

    gpio_reset_pin(BOTAO2);
    gpio_set_direction(BOTAO2, GPIO_MODE_INPUT);
    gpio_pullup_en(BOTAO2);
}

void configurar_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LED_PWM,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void configurar_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT_NUM, conf.mode, 0, 0, 0));
}

// Funções SD Card
bool init_sd_card() {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = SPI2_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .max_transfer_sz = 4000
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA));
    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        printf("Falha ao montar o cartão SD\n");
        return false;
    }
    return true;
}

void write_to_sd(double temperature, uint8_t alarm) {
    if (!sd_card_initialized) return;
    
    FILE* f = fopen(LOG_FILE, "a");
    if (f == NULL) return;
    
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    fprintf(f, "%04d-%02d-%02d %02d:%02d:%02d,%.2f,%d\n",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
           temperature, alarm);
    fclose(f);
}

// Handlers de botão
static void IRAM_ATTR botao1_handler(void* arg) {
    uint64_t agora = esp_timer_get_time();
    if ((agora - ultimo_tempo_botao1) > debounce_time) {
        ultimo_tempo_botao1 = agora;
        alarme += 5;
    }
}

static void IRAM_ATTR botao2_handler(void* arg) {
    uint64_t agora = esp_timer_get_time();
    if ((agora - ultimo_tempo_botao2) > debounce_time) {
        ultimo_tempo_botao2 = agora;
        alarme -= 5;
    }
}

void vTaskTempRead(void *pvParameters) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
    
    const float BETA = 3950;
    uint32_t log_counter = 0;
    
    while(1) {
        int adc_raw = adc1_get_raw(ADC1_CHANNEL_7);
        double temp = 1 / (log(1 / (4095.0 / adc_raw - 1)) / BETA + 1.0 / 298.15) - 273.15;
        
        ValorNTC = temp;
        xQueueSend(xTempQueue, &temp, 0);
        
        if (sd_card_initialized && (++log_counter % 10 == 0)) {
            if(xSemaphoreTake(xSDCardMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                write_to_sd(temp, alarme);
                xSemaphoreGive(xSDCardMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskDisplayUpdate(void *pvParameters) {
    while(1) {
        double temp;
        if(xQueueReceive(xTempQueue, &temp, 0) == pdTRUE) {
            ValorNTC = temp;
        }
        
        atualizar_lcd();
        atualizar_leds();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskButtonMonitor(void *pvParameters) {
    uint64_t agora;
    
    while(1) {
        agora = esp_timer_get_time();
        
        if(!gpio_get_level(BOTAO1) && (agora - ultimo_tempo_botao1 > debounce_time)) {
            ultimo_tempo_botao1 = agora;
            alarme += 5;
        }
        
        if(!gpio_get_level(BOTAO2) && (agora - ultimo_tempo_botao2 > debounce_time)) {
            ultimo_tempo_botao2 = agora;
            alarme -= 5;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main() {

    configurar_gpio();
    configurar_pwm();
    configurar_i2c();
    lcd_init();

    // Inicializa RTOS
    xTempQueue = xQueueCreate(5, sizeof(double));
    xSDCardMutex = xSemaphoreCreateMutex();

    // Inicializa SD Card
    if(xSemaphoreTake(xSDCardMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        sd_card_initialized = init_sd_card();
        if(sd_card_initialized) {
            FILE* f = fopen(LOG_FILE, "r");
            if(f == NULL) {
                f = fopen(LOG_FILE, "w");
                if(f != NULL) {
                    fprintf(f, "Data,Hora,Temperatura(C),Alarme\n");
                    fclose(f);
                }
            } else {
                fclose(f);
            }
        }
        xSemaphoreGive(xSDCardMutex);
    }

    // Configura interrupções
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO1, botao1_handler, NULL);
    gpio_isr_handler_add(BOTAO2, botao2_handler, NULL);

    // Cria tasks
    xTaskCreate(vTaskTempRead, "TempRead", 2048, NULL, 2, NULL);
    xTaskCreate(vTaskDisplayUpdate, "DisplayUpdate", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskButtonMonitor, "ButtonMonitor", 2048, NULL, 3, NULL);

    vTaskDelete(NULL);
}
