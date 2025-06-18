#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#define LED0 20
#define LED1 45
#define LED2 37
#define LED3 42
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

// LCD
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_DDRAM_ADDR 0x80

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

// SD Card (SPI)
#define MOUNT_POINT "/sdcard"
#define LOG_FILE "/sdcard/ntc_log.csv"
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 12
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

double ValorNTC = 0;
uint8_t alarme = 25;
uint64_t ultimo_tempo_botao1 = 0;
uint64_t ultimo_tempo_botao2 = 0;
const uint64_t debounce_time = 100000;
static sdmmc_card_t* card;
static bool sd_card_initialized = false;

// Funções LCD
void lcd_send_nibble(uint8_t data, uint8_t rs) {
    uint8_t nibble = (data & 0xF0) | rs | LCD_BACKLIGHT | 0x04;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, nibble, true);
    i2c_master_write_byte(cmd, nibble & ~0x04, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT_NUM, cmd, pdMS_TO_TICKS(100));
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
    char line1[17];  
    char line2[17];  

    snprintf(line1, sizeof(line1), "NTC: %.2f C", ValorNTC);
    snprintf(line2, sizeof(line2), "Alarme: %d", alarme);
    
    lcd_send_cmd(LCD_SET_DDRAM_ADDR | 0x00);  
    lcd_print_str(line1);
    
    lcd_send_cmd(LCD_SET_DDRAM_ADDR | 0x40);  
    lcd_print_str(line2);
}

void atualizar_leds() {
    static bool estado_pisca = false;
    int diferenca = alarme - ValorNTC;

    if (ValorNTC > alarme) {
        estado_pisca = !estado_pisca;
        gpio_set_level(LED0, estado_pisca);
        gpio_set_level(LED1, estado_pisca);
        gpio_set_level(LED2, estado_pisca);
        gpio_set_level(LED3, estado_pisca);

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 255);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        return;
    }

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    gpio_set_level(LED0, 0);
    gpio_set_level(LED1, 0);
    gpio_set_level(LED2, 0);
    gpio_set_level(LED3, 0);

    if (diferenca <= 2)  gpio_set_level(LED3, 1);
    if (diferenca <= 10) gpio_set_level(LED2, 1);
    if (diferenca <= 15) gpio_set_level(LED1, 1);
    if (diferenca <= 20) gpio_set_level(LED0, 1);
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
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT_NUM, conf.mode, 0, 0, 0));
}

bool init_sd_card() {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // Configuração do host SPI
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    
    // Configuração do SPI
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = SPI2_HOST;  
    
    // Configuração dos pinos SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    // Inicializa o barramento SPI
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA));
    
    // Monta o sistema de arquivos
    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        printf("Falha ao montar o cartão SD (0x%x)\n", ret);
        return false;
    }

    printf("Cartão SD inicializado via SPI!\n");
    return true;
}

void write_to_sd(double temperature, uint8_t alarm) {
    // Verifica se o SD card está inicializado
    if (!sd_card_initialized) {
        printf("[SD] Erro: Cartão SD não inicializado!\n");
        return;
    }

    printf("[SD] Tentando abrir arquivo %s para escrita...\n", LOG_FILE);
    
    // Abre o arquivo em modo append
    FILE* f = fopen(LOG_FILE, "a");
    if (f == NULL) {
        printf("[SD] Erro: Falha ao abrir o arquivo para escrita!\n");
        return;
    }
    printf("[SD] Arquivo aberto com sucesso!\n");

    // Obtém o timestamp atual
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    // Formata a mensagem de log
    char log_entry[100];
    snprintf(log_entry, sizeof(log_entry), 
             "%04d-%02d-%02d %02d:%02d:%02d,%.2f,%d\n",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
             temperature, alarm);

    printf("[SD] Gravando dados: %s", log_entry);
    
    // Escreve no arquivo
    if (fprintf(f, "%s", log_entry) < 0) {
        printf("[SD] Erro: Falha ao escrever no arquivo!\n");
    } else {
        printf("[SD] Dados escritos com sucesso!\n");
    }

    // Fecha o arquivo
    if (fclose(f) != 0) {
        printf("[SD] Erro: Falha ao fechar o arquivo!\n");
    } else {
        printf("[SD] Arquivo fechado corretamente.\n");
    }
}

static void IRAM_ATTR botao1_handler(void* arg) {
    uint64_t agora = esp_timer_get_time();
    if ((agora - ultimo_tempo_botao1) > debounce_time) {
        ultimo_tempo_botao1 = agora;
        alarme = alarme + 5;
    }
}

static void IRAM_ATTR botao2_handler(void* arg) {
    uint64_t agora = esp_timer_get_time();
    if ((agora - ultimo_tempo_botao2) > debounce_time) {
        ultimo_tempo_botao2 = agora;
        alarme = alarme - 5;
    }
}

void app_main() {
    configurar_gpio();
    configurar_pwm();
    configurar_i2c();
    lcd_init();

    // Inicializa o cartão SD via SPI
    sd_card_initialized = init_sd_card();
    if (sd_card_initialized) {

        FILE* f = fopen(LOG_FILE, "r");
        if (f == NULL) {
            f = fopen(LOG_FILE, "w");
            if (f != NULL) {
                fprintf(f, "Data,Hora,Temperatura(C),Alarme\n");
                fclose(f);
            }
        } else {
            fclose(f);
        }
    }

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO1, botao1_handler, NULL);
    gpio_isr_handler_add(BOTAO2, botao2_handler, NULL);

    atualizar_leds();
    atualizar_lcd();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12); // 0-3.3V
    adc1_get_raw(SENSOR);

    uint32_t log_counter = 0;
    const uint32_t log_interval = 10; // Log a cada 10 iterações (1 segundo)

    while(1) {
        int adc_raw = adc1_get_raw(ADC1_CHANNEL_7); 
        const float BETA = 3950;
        ValorNTC = 1 / (log(1 / (4095.0 / adc_raw - 1)) / BETA + 1.0 / 298.15) - 273.15;

        atualizar_lcd();
        atualizar_leds();

        if (sd_card_initialized && (++log_counter % log_interval == 0)) {
            write_to_sd(ValorNTC, alarme);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
