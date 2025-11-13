#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// Default stack size for the tasks
#define DEFAULT_STACK_SIZE 2048 

//Tilakone
enum state { IDLE=1, RECORDING, SEND };
enum state programState = IDLE;

//morseviestiin liittyvät globaalit muuttujat
char morseMessage[256]; 
size_t morseIndex = 0;
bool letterFinalized = false; //onko kirjain valmis

//onko uart alustettu, tämän avulla yritetty parantaa sitä, että saisi picon toimimaan
bool uart_initialized = false;

//prototyypit
static void morse_task(void *arg);
static void status_task(void *arg);
void add_symbol_to_message(char symbol);
void buttonFxn(uint gpio, uint32_t eventMask);
void send_morse_message(const char* message);
void change_state(enum state newState);
void send_debug_message(const char* debug);
    
int main() {
    stdio_init_all();
    
    // Odotetaan sarjaliikenneyhteyttä (tämä voidaan myös poistaa jos ei tarvita)
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    init_hat_sdk();    
    sleep_ms(300);

    // Alustetaan LEDit, buzzer ja napit SDK:n kautta
    init_led();
    init_buzzer();
    init_button1();
    init_button2();

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &buttonFxn);

    // UART0 alustus
    uart_init(uart0, 9600);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    uart_initialized = true;

    // Luodaan morse- ja status-taskit
    TaskHandle_t morseTaskHandle = NULL;
    TaskHandle_t statusTaskHandle = NULL;

    BaseType_t morse = xTaskCreate(morse_task, "morse", DEFAULT_STACK_SIZE, NULL, 2, &morseTaskHandle);
    BaseType_t status = xTaskCreate(status_task, "status", DEFAULT_STACK_SIZE, NULL, 1, &statusTaskHandle);

    if (morse != pdPASS) {
        send_debug_message("Morse task creation failed");
    }
    if (status != pdPASS) {
        send_debug_message("Status task creation failed");
    }

    printf("Creating tasks now...\n");
    printf("Starting scheduler...\n");
    // Käynnistetään RTOS (ei palaa)
    vTaskStartScheduler();

    return 0;
}

void buttonFxn(uint gpio, uint32_t eventMask) {
    switch (programState) {
        case IDLE:
            change_state(RECORDING);
            break;

        case RECORDING:
            change_state(SEND);
            break;

        case SEND:
            send_morse_message(morseMessage);

            // viesti nollataan
            morseIndex = 0;
            morseMessage[0] = '\0';
            letterFinalized = false;
            change_state(IDLE);
            break;
    }
}

static void morse_task(void *arg) {
    (void)arg;

    if (init_ICM42670() == 0) {
        send_debug_message("IMU initialized successfully");
        if (ICM42670_start_with_default_values() != 0) {
            send_debug_message("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        send_debug_message("IMU init failed");
    }

    send_debug_message("Morse task initialized");

    float ax, ay, az, gx, gy, gz, temp;
    char currentSymbol; //nykyinen symboli
    bool symbolDetected = false; //onko symbooli havaittu
    bool readyForNextSymbol = true; // mahdollistaa useamman saman symbolin lukemisen liikuttamatta laitetta
    //aikaperusteisen symbolien hyväksymiseen tarvittavat muuttujat
    TickType_t symbolStartTick = 0;
    TickType_t lastSymbolAcceptedTick = 0;

    for(;;) {
        // lukee IMUN:n sensoridatan -> palauttaa 0 jos lukeminen onnistuu
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
            if(programState == RECORDING)   {
                TickType_t now = xTaskGetTickCount();
                if (readyForNextSymbol) {
                    if (fabs(az) > 0.8 && fabs(ax) < 0.3 && fabs(ay) < 0.3) {
                        //laite vaakatasossa = piste
                        currentSymbol = '.';
                        symbolStartTick = now;
                        symbolDetected = true;
                        readyForNextSymbol = false;
                        send_debug_message("Detected potential DOT");
                    } else if (fabs(ax) > 0.8 && fabs(az) < 0.3 && fabs(ay) < 0.3) {
                        //laite pystyasennossa = viiva
                        currentSymbol = '-';
                        symbolStartTick = now;
                        symbolDetected = true;
                        readyForNextSymbol = false;
                        send_debug_message("Detected potential DASH");
                    } else if (fabs(ay) > 0.8 && fabs(ax) < 0.3 && fabs(az) < 0.3) {
                        //kallistuu ylöspäin = välilyönti
                        currentSymbol = ' ';
                        symbolStartTick = now;
                        symbolDetected = true;
                        readyForNextSymbol = false;
                        send_debug_message("Detected SPACE");
                    }
                } else if (symbolDetected) {  
                    //symboli hyväksytään, jos asento pysyy 500ms
                    if ((now - symbolStartTick) > pdMS_TO_TICKS(500)) {
                        add_symbol_to_message(currentSymbol);
                        //printf("Hyväksytty symboli: %c\n", currentSymbol);
                        send_debug_message("Symbol confirmed");
                        symbolDetected = false;
                        lastSymbolAcceptedTick = now;
                        letterFinalized = false;
                    }                    
                } else {
                    if ((now - lastSymbolAcceptedTick) > pdMS_TO_TICKS(700)) {
                        readyForNextSymbol = true;
                    }
                }
                if ((now - lastSymbolAcceptedTick) > pdMS_TO_TICKS(1500) && 
                    !letterFinalized && morseIndex > 0) {
                    //printf("Kirjain valmis\n");
                    send_debug_message("Letter finalized");
                    letterFinalized = true;
                }
            }
        } else {
            send_debug_message("Failed to read IMU data");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Päivittää LED-tilat ohjelman päätilan mukaan
static void status_task(void *arg) {
    (void)arg;

    send_debug_message("Status task initialized");

    const TickType_t blinkSlow = pdMS_TO_TICKS(500);
    const TickType_t blinkFast = pdMS_TO_TICKS(200);

    for (;;) {
        switch (programState) {
            case IDLE:
                init_rgb_led();
                rgb_led_write(0, 255, 255);   // punainen päälle (0 = ON)
                vTaskDelay(blinkSlow);
                rgb_led_write(255, 255, 255); // kaikki pois päältä
                vTaskDelay(blinkSlow);
                break;

            case RECORDING:
                init_rgb_led();
                rgb_led_write(255, 255, 0);   // sininen päälle
                vTaskDelay(blinkFast);
                rgb_led_write(255, 255, 255); // pois
                vTaskDelay(blinkFast);
                break;

            case SEND:
                init_rgb_led();
                rgb_led_write(255, 0, 255);   // vihreä jatkuvasti
                vTaskDelay(pdMS_TO_TICKS(800));
                break;
        }
    }
}

//Viestin lähetys kerralla
void send_morse_message(const char* message) {
    if (message == NULL || message[0] == '\0') {
        //printf("Viestissä ei ole sisältöä");
        send_debug_message("Empty message - not sent");
        return;
    }
    
    uart_puts(uart0, message);
    uart_puts(uart0, "  \n"); // kaksi välilyöntiä ja rivinvaihto loppuun
    buzzer_play_tone(1000, 200);  // Piippaa 200 ms
}

void change_state(enum state newState) {
    programState = newState;
    switch (newState) {
        case IDLE:
            send_debug_message("State changed to IDLE");
            break;
        case RECORDING:
            send_debug_message("State changed to RECORDING");
            break;
        case SEND:
            send_debug_message("State changed to SEND");
            break;
    }
}

//lisätään IMU-sensorilla saatu merkki viestiin, tarkistetaan että puskuriin mahtuu
void add_symbol_to_message(char symbol) {
    if (morseIndex < sizeof(morseMessage) -1) {
        morseMessage[morseIndex++] = symbol;
        morseMessage[morseIndex] = '\0';
    }
}

void send_debug_message(const char* debug) {
    if (uart_initialized) {
        uart_puts(uart0, "__");
        uart_puts(uart0, debug);
        uart_puts(uart0, "__\n");
    }
    printf("DEBUG: %s\n", debug);
}
