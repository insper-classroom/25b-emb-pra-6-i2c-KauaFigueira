#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "mpu6050.h"
#include "mouse.h"
#include "Fusion.h"

// ===================== Config =====================
#define SAMPLE_PERIOD       (0.01f)   // 100 Hz
#define DEADZONE_DPS        (4.0f)    // °/s
#define GAIN_DPS2COUNT      (2.5f)    // °/s -> counts
#define TAP_JERK_G          (0.60f)   // Δacc em g para "tap"
#define TAP_COOLDOWN_MS     300
#define GYRO_SENS           (131.0f)  // LSB -> °/s (±250 dps)
#define ACCEL_SENS          (16384.0f)// LSB -> g (±2 g)

// GPIO Pins
#define BTN_SHOOT_GPIO      14        // Botão de disparo
#define BTN_TOGGLE_GPIO     17        // Botão liga/desliga
#define BUZZER_GPIO         15        // Buzzer para som de tiro
#define DEBOUNCE_MS         80        // Debounce aumentado para evitar múltiplos disparos
#define HOLD_THRESHOLD_MS   150       // Tempo para considerar "segurar" o botão

#ifndef USE_PICO_DOCK
#  define I2C_SDA_GPIO 4
#  define I2C_SCL_GPIO 5
#else
#  define I2C_SDA_GPIO 17
#  define I2C_SCL_GPIO 16
#endif

#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define MPU_ADDRESS 0x68

// ===================== CONFIGURAÇÃO DE EIXOS =====================
// Sensor EM PÉ, apontado para DIREITA
// Ajuste DIR_X e DIR_Y se os eixos estiverem invertidos
#define DIR_X  (1.0f)      // use -1.0f para inverter horizontal
#define DIR_Y  (-1.0f)     // use 1.0f para inverter vertical

// ===================== Tipos/CTX ==================
typedef struct {
    QueueHandle_t q;
    volatile bool mouse_enabled;  // Flag para habilitar/desabilitar mouse
    SemaphoreHandle_t toggle_sem; // Semáforo para botão toggle
    SemaphoreHandle_t shoot_sem;  // Semáforo para botão shoot
    volatile bool button_held;    // Flag para detectar botão segurado
} app_ctx_t;

typedef struct {
    float bx; // bias X
    float by; // bias Y
} gyro_bias_t;

// Contexto global para callbacks (necessário para ISR)
static app_ctx_t *g_ctx = NULL;
static volatile uint32_t last_toggle_time = 0;
static volatile uint32_t last_shoot_press_time = 0;
static volatile uint32_t last_shoot_release_time = 0;
static volatile bool shoot_button_pressed = false;

// ===================== Protos =====================
static void mpu6050_reset(void);
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
static inline int16_t sat_i16_from_float(float x);
static gyro_bias_t calibrate_gyro_bias(void);
static void gpio_callback(uint gpio, uint32_t events);
static void setup_buttons(app_ctx_t *ctx);
static void funcao(int freq, double tempo, int pin);
static void gunshot_sound(void);
static void rapid_gunshot_sound(void);
static void mpu6050_task(void *p);
static void uart_task(void *p);

// ===================== MPU6050 LL =================
static void mpu6050_reset(void) {
    uint8_t buf[] = {0x6B, 0x00}; // sai de sleep
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;

    // Accel 0x3B..0x40
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[(i * 2) + 1]);
    }

    // Gyro 0x43..0x48
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (int16_t)((buffer[i * 2] << 8) | buffer[(i * 2) + 1]);
    }

    // Temp 0x41..0x42
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
}

// ===================== Util =======================
static inline int16_t sat_i16_from_float(float x) {
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)x;
}

static gyro_bias_t calibrate_gyro_bias(void) {
    const int N = 500; // ~5 s a 100 Hz
    int16_t accel[3], gyro[3], temp;
    float sx = 0.0f, sy = 0.0f;

    printf("=== CALIBRACAO: Mantenha o sensor PARADO e EM PE! ===\n");
    for (int i = 0; i < N; ++i) {
        mpu6050_read_raw(accel, gyro, &temp);
        // SENSOR EM PÉ: Y do sensor = X do mouse, Z do sensor = Y do mouse
        sx += (gyro[1] / GYRO_SENS);  // gyro[1] = Y do sensor
        sy += (gyro[2] / GYRO_SENS);  // gyro[2] = Z do sensor
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    gyro_bias_t b = { .bx = sx / (float)N, .by = sy / (float)N };
    printf("=== CALIBRACAO COMPLETA: bias_x=%.3f, bias_y=%.3f ===\n", b.bx, b.by);
    return b;
}

// ===================== Som Buzzer =================
// Função base para gerar tom no buzzer
static void funcao(int freq, double tempo, int pin) {
    double T = 1.0 / freq;
    int delay = (int)((T * 1000000) / 2);
    int ciclo = (int)(tempo / T);
    
    for (int i = 0; i < ciclo; i++) {
        gpio_put(pin, 1);
        sleep_us(delay);
        gpio_put(pin, 0);
        sleep_us(delay);
    }
}

// Som de tiro único (AK-47 style)
static void gunshot_sound(void) {
    // Explosão inicial (bang!) - frequências altas e rápidas
    funcao(2500, 0.008, BUZZER_GPIO);
    funcao(2000, 0.008, BUZZER_GPIO);
    funcao(1500, 0.010, BUZZER_GPIO);
    
    // Eco/reverberação - sweep rápido descendente
    for (int f = 1200; f >= 300; f -= 80) {
        funcao(f, 0.004, BUZZER_GPIO);
    }
    
    // Ruído final (casca caindo)
    funcao(200, 0.015, BUZZER_GPIO);
    sleep_ms(5);
    funcao(180, 0.010, BUZZER_GPIO);
}

// Som de rajada (tiro rápido repetitivo)
static void rapid_gunshot_sound(void) {
    // Som mais curto para rajada
    funcao(2200, 0.006, BUZZER_GPIO);
    funcao(1800, 0.005, BUZZER_GPIO);
    funcao(1200, 0.006, BUZZER_GPIO);
    
    // Sweep curto
    for (int f = 900; f >= 400; f -= 120) {
        funcao(f, 0.003, BUZZER_GPIO);
    }
    
    funcao(220, 0.008, BUZZER_GPIO);
}

// ===================== GPIO Callback ==============
static void gpio_callback(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (gpio == BTN_TOGGLE_GPIO && (events & GPIO_IRQ_EDGE_FALL)) {
        // Debounce
        if ((now - last_toggle_time) > DEBOUNCE_MS) {
            last_toggle_time = now;
            xSemaphoreGiveFromISR(g_ctx->toggle_sem, &xHigherPriorityTaskWoken);
        }
    }
    else if (gpio == BTN_SHOOT_GPIO) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Botão pressionado
            if ((now - last_shoot_press_time) > DEBOUNCE_MS) {
                last_shoot_press_time = now;
                shoot_button_pressed = true;
                xSemaphoreGiveFromISR(g_ctx->shoot_sem, &xHigherPriorityTaskWoken);
            }
        }
        else if (events & GPIO_IRQ_EDGE_RISE) {
            // Botão solto
            if ((now - last_shoot_release_time) > DEBOUNCE_MS) {
                last_shoot_release_time = now;
                shoot_button_pressed = false;
                g_ctx->button_held = false;
            }
        }
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ===================== Setup Botões ===============
static void setup_buttons(app_ctx_t *ctx) {
    // Configura GPIO dos botões como entrada com pull-up
    gpio_init(BTN_SHOOT_GPIO);
    gpio_set_dir(BTN_SHOOT_GPIO, GPIO_IN);
    gpio_pull_up(BTN_SHOOT_GPIO);
    
    gpio_init(BTN_TOGGLE_GPIO);
    gpio_set_dir(BTN_TOGGLE_GPIO, GPIO_IN);
    gpio_pull_up(BTN_TOGGLE_GPIO);
    
    // Configura GPIO do buzzer como saída
    gpio_init(BUZZER_GPIO);
    gpio_set_dir(BUZZER_GPIO, GPIO_OUT);
    gpio_put(BUZZER_GPIO, 0);
    
    // Configura interrupções (falling e rising para detectar pressionar e soltar)
    gpio_set_irq_enabled_with_callback(BTN_SHOOT_GPIO, 
                                       GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
                                       true, &gpio_callback);
    gpio_set_irq_enabled(BTN_TOGGLE_GPIO, GPIO_IRQ_EDGE_FALL, true);
    
    // Cria semáforos binários
    ctx->toggle_sem = xSemaphoreCreateBinary();
    ctx->shoot_sem = xSemaphoreCreateBinary();
    
    // Inicia com mouse habilitado
    ctx->mouse_enabled = true;
    ctx->button_held = false;
}

// ===================== Tasks ======================
static void mpu6050_task(void *p) {
    app_ctx_t *ctx = (app_ctx_t *)p;

    printf("=== SENSOR EM PE, APONTADO PARA DIREITA ===\n");

    // I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    // Calibração local (sem globais)
    gyro_bias_t bias = calibrate_gyro_bias();

    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    mouse_t mouse;

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_tap  = 0;
    TickType_t shoot_press_time = 0;
    bool was_holding = false;

    float prev_ax_g = 0.0f;

    while (true) {
        // Verifica botão toggle (não bloqueante)
        if (xSemaphoreTake(ctx->toggle_sem, 0) == pdTRUE) {
            ctx->mouse_enabled = !ctx->mouse_enabled;
            printf("Mouse %s\n", ctx->mouse_enabled ? "HABILITADO" : "DESABILITADO");
            
            // Feedback sonoro ao ligar/desligar
            if (ctx->mouse_enabled) {
                funcao(1000, 0.05, BUZZER_GPIO);
                sleep_ms(30);
                funcao(1200, 0.05, BUZZER_GPIO);
            } else {
                funcao(800, 0.05, BUZZER_GPIO);
                sleep_ms(30);
                funcao(600, 0.05, BUZZER_GPIO);
            }
        }
        
        // Verifica botão shoot (não bloqueante)
        if (xSemaphoreTake(ctx->shoot_sem, 0) == pdTRUE) {
            if (ctx->mouse_enabled) {
                shoot_press_time = xTaskGetTickCount();
            }
        }
        
        // Verifica se está segurando o botão
        if (shoot_button_pressed && ctx->mouse_enabled) {
            TickType_t now = xTaskGetTickCount();
            uint32_t hold_duration = pdTICKS_TO_MS(now - shoot_press_time);
            
            if (hold_duration > HOLD_THRESHOLD_MS && !ctx->button_held) {
                // Começou a segurar - envia comando de pressionar
                ctx->button_held = true;
                was_holding = true;
                mouse.axis = 4;  // 4 = hold start
                mouse.val  = 1;
                xQueueSend(ctx->q, &mouse, 0);
                printf("HOLD START!\n");
            }
            
            // Toca som de rajada enquanto segura
            if (ctx->button_held) {
                rapid_gunshot_sound();
                sleep_ms(80);  // Cadência de tiro (750 RPM)
            }
        }
        else if (!shoot_button_pressed && was_holding) {
            // Soltou o botão após segurar - envia comando de soltar
            mouse.axis = 5;  // 5 = hold release
            mouse.val  = 0;
            xQueueSend(ctx->q, &mouse, 0);
            was_holding = false;
            printf("HOLD RELEASE!\n");
        }
        else if (!shoot_button_pressed && shoot_press_time != 0) {
            // Clique rápido (não segurou)
            TickType_t now = xTaskGetTickCount();
            uint32_t press_duration = pdTICKS_TO_MS(now - shoot_press_time);
            
            if (press_duration <= HOLD_THRESHOLD_MS && ctx->mouse_enabled) {
                mouse.axis = 3;  // 3 = single shot
                mouse.val  = 1;
                xQueueSend(ctx->q, &mouse, 0);
                gunshot_sound();
                printf("SINGLE SHOT!\n");
            }
            shoot_press_time = 0;
        }
        
        // Só processa sensor se mouse estiver habilitado
        if (ctx->mouse_enabled) {
            // Leitura crua
            mpu6050_read_raw(acceleration, gyro, &temp);

            // *** REMAPEAMENTO DE EIXOS: SENSOR EM PÉ ***
            // Y do sensor → X do mouse (esquerda/direita)
            // Z do sensor → Y do mouse (cima/baixo)
            float gx = (gyro[1] / GYRO_SENS) - bias.bx; // gyro[1] = Y do sensor
            float gy = (gyro[2] / GYRO_SENS) - bias.by; // gyro[2] = Z do sensor

            FusionVector gyroscope = {
                .axis.x = gyro[1] / GYRO_SENS,  // Y do sensor
                .axis.y = gyro[2] / GYRO_SENS,  // Z do sensor
                .axis.z = gyro[0] / GYRO_SENS,  // X do sensor
            };
            FusionVector accelerometer = {
                .axis.x = acceleration[1] / ACCEL_SENS,  // Y do sensor
                .axis.y = acceleration[2] / ACCEL_SENS,  // Z do sensor
                .axis.z = acceleration[0] / ACCEL_SENS,  // X do sensor
            };

            // Fusão (mantém orientação disponível)
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

            // Deadzone e ganho
            gx = (fabsf(gx) < DEADZONE_DPS) ? 0.0f : gx;
            gy = (fabsf(gy) < DEADZONE_DPS) ? 0.0f : gy;

            float vx = DIR_X * gx * GAIN_DPS2COUNT;
            float vy = DIR_Y * gy * GAIN_DPS2COUNT;

            // Eixo X
            mouse.axis = 0;
            mouse.val  = sat_i16_from_float(vx);
            xQueueSend(ctx->q, &mouse, 0);

            // Eixo Y
            mouse.axis = 1;
            mouse.val  = sat_i16_from_float(vy);
            xQueueSend(ctx->q, &mouse, 0);

            // Clique por "tap" (ajustado para o novo eixo)
            float ax_g = accelerometer.axis.x;  // Agora é Y do sensor
            float jerk = ax_g - prev_ax_g;
            prev_ax_g = ax_g;

            if (jerk > TAP_JERK_G) {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_tap) > pdMS_TO_TICKS(TAP_COOLDOWN_MS)) {
                    last_tap = now;
                    mouse.axis = 2;   // 2 = click
                    mouse.val  = 1;   // pulso
                    xQueueSend(ctx->q, &mouse, 0);
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10)); // 100 Hz
    }
}

static void uart_task(void *p){
    app_ctx_t *ctx = (app_ctx_t *)p;
    mouse_t mouse;

    uart_init(uart0, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    while (1) {
        if (xQueueReceive(ctx->q, &mouse, portMAX_DELAY)) {
            // Sync
            uart_putc_raw(uart0, 0xFF);
            // Eixo
            uart_putc_raw(uart0, (uint8_t)mouse.axis);
            // Valor (LSB, MSB) como int16_t little-endian
            uint16_t u = (uint16_t)mouse.val;
            uart_putc_raw(uart0, (uint8_t)(u & 0xFF));
            uart_putc_raw(uart0, (uint8_t)((u >> 8) & 0xFF));
        }
    }
}

int main(void) {
    stdio_init_all();

    app_ctx_t *ctx = (app_ctx_t *)pvPortMalloc(sizeof(app_ctx_t));
    ctx->q = xQueueCreate(10, sizeof(mouse_t));
    ctx->mouse_enabled = true; 
    ctx->button_held = false;
    
    // Armazena contexto global para callbacks
    g_ctx = ctx;
    
    // Configura botões com callbacks
    setup_buttons(ctx);

    xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, ctx, 1, NULL);
    xTaskCreate(uart_task,    "uart_Task",    1024, ctx, 1, NULL);

    vTaskStartScheduler();
    while (true) { }
}