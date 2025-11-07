#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <task.h>

#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048

// Morseviestiin liittyvät muuttujat
char currentSymbol;
char morseMessage[256];
size_t morseIndex = 0;
bool symbolDetected = false;
bool letterFinalized = false;
bool readyForNextSymbol = true;

// Yksi yksinkertainen tila: äänitetäänkö vai ei
bool isRecording = false;

// Prototyypit
static void init_task(void *arg);
static void morse_task(void *arg);
static void status_task(void *arg);
void buttonFxn(uint gpio, uint32_t eventMask);
void add_symbol_to_message(char symbol);
void send_morse_message(const char *message);

int main() {
    stdio_init_all();

    TaskHandle_t initTaskHandle = NULL;
    xTaskCreate(init_task, "init", DEFAULT_STACK_SIZE, NULL, 3, &initTaskHandle);
    vTaskStartScheduler();
    return 0;
}

static void init_task(void *arg) {
    (void)arg;
    init_hat_sdk();
    init_i2c_default();
    sleep_ms(200);
    ICM42670_start_with_default_values();

    gpio_init(RGB_LED_R); gpio_set_dir(RGB_LED_R, GPIO_OUT);
    gpio_init(RGB_LED_G); gpio_set_dir(RGB_LED_G, GPIO_OUT);
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, &buttonFxn);

    TaskHandle_t morseTaskHandle = NULL;
    xTaskCreate(morse_task, "morse", DEFAULT_STACK_SIZE, NULL, 2, &morseTaskHandle);

    TaskHandle_t statusTaskHandle = NULL;
    xTaskCreate(status_task, "status", DEFAULT_STACK_SIZE, NULL, 1, &statusTaskHandle);

    vTaskDelete(NULL);
}

// Painike vaihtaa äänitystilan
void buttonFxn(uint gpio, uint32_t eventMask) {
    if (gpio == BUTTON1) {
        isRecording = !isRecording;
        if (!isRecording) {
            // Lähetetään viesti heti kun tallennus lopetetaan
            printf("Lähetetään viesti: %s\n", morseMessage);
            send_morse_message(morseMessage);
            morseIndex = 0;
            morseMessage[0] = '\0';
        } else {
            printf("Tallennus alkaa\n");
        }
    }
}

static void morse_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temp;
    TickType_t symbolStartTick = 0, lastSymbolAcceptedTick = 0;

    for (;;) {
        if (isRecording && ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
            TickType_t now = xTaskGetTickCount();
            if (readyForNextSymbol) {
                if (fabs(az) > 0.8) { currentSymbol = '.'; symbolStartTick = now; symbolDetected = true; readyForNextSymbol = false; }
                else if (fabs(ax) > 0.8) { currentSymbol = '-'; symbolStartTick = now; symbolDetected = true; readyForNextSymbol = false; }
                else if (fabs(ay) > 0.8) { currentSymbol = ' '; symbolStartTick = now; symbolDetected = true; readyForNextSymbol = false; }
            } else if (symbolDetected && (now - symbolStartTick) > pdMS_TO_TICKS(500)) {
                add_symbol_to_message(currentSymbol);
                printf("Hyväksytty symboli: %c\n", currentSymbol);
                symbolDetected = false;
                lastSymbolAcceptedTick = now;
                letterFinalized = false;
            } else if (!symbolDetected && (now - lastSymbolAcceptedTick) > pdMS_TO_TICKS(700)) {
                readyForNextSymbol = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void status_task(void *arg) {
    (void)arg;
    for (;;) {
        if (isRecording) {
            gpio_put(RGB_LED_G, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_put(RGB_LED_G, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            gpio_put(RGB_LED_R, 1);
            vTaskDelay(pdMS_TO_TICKS(600));
            gpio_put(RGB_LED_R, 0);
            vTaskDelay(pdMS_TO_TICKS(600));
        }
    }
}

void add_symbol_to_message(char symbol) {
    if (morseIndex < sizeof(morseMessage) - 1) {
        morseMessage[morseIndex++] = symbol;
        morseMessage[morseIndex] = '\0';
    }
}

void send_morse_message(const char *message) {
    if (message == NULL || message[0] == '\0') {
        printf("Ei lähetettävää viestiä\n");
        return;
    }
    uart_puts(uart0, message);
    uart_puts(uart0, "\n");
}
