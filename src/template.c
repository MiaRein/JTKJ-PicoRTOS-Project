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

//Tilakone ei pakollinen Taso 1:ssä
enum state { IDLE=1, RECORDING, READY_TO_SEND };
enum state programState = IDLE;

//morseviestiin liittyvät globaalit muuttujat
char currentSymbol; 
char morseMessage[256]; 
size_t morseIndex = 0;
bool symbolDetected = false; //onko symbooli havaittu
bool letterFinalized = false; //onko kirjain valmis
bool readyForNextSymbol = true; // mahdollistaa useamman saman symbolin lukemisen liikuttamatta laitetta

//prototyypit
static void init_task(void *arg);
static void morse_task(void *arg);
static void status_task(void *arg);
void add_symbol_to_message(char symbol);
void buttonFxn(uint gpio, uint32_t eventMask);
void send_morse_message(const char* message);
void change_state(enum state newState);
void display_message(const char* message);
    
int main() {
    stdio_init_all();

    //Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/

    TaskHandle_t initTaskHandle = NULL;
    xTaskCreate(init_task, "init_task", DEFAULT_STACK_SIZE, NULL, 3, &initTaskHandle);
    
    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

static void init_task(void *arg) {
    (void)arg;

    // HATin ja I2C:n alustus
    init_hat_sdk();
    init_i2c_default();
    sleep_ms(300);

    if (init_ICM42670() != 0) {
        gpio_put(RGB_LED_R, 1);
        printf("IMU init failed\n");
    } else {
        ICM42670_start_with_default_values();
    }

    // LEDien alustus
    gpio_init(RGB_LED_B); gpio_set_dir(RGB_LED_B, GPIO_OUT);
    gpio_init(RGB_LED_G); gpio_set_dir(RGB_LED_G, GPIO_OUT);
    gpio_init(RGB_LED_R); gpio_set_dir(RGB_LED_R, GPIO_OUT);

    // Painike
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, &buttonFxn);

    // Summeri
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);

    // Näyttö
    init_display();

    // Luodaan morse_task (viestien muodostamiseen)
    TaskHandle_t morseTaskHandle = NULL;
    BaseType_t morse = xTaskCreate(morse_task, "morse", DEFAULT_STACK_SIZE, NULL, 2, &morseTaskHandle);
    if (morse != pdPASS) {
        printf("Morse Task creation failed\n");
    }

    // Luodaan status_task (vilkuttaa LEDiä tai näyttää tilaa)
    TaskHandle_t statusTaskHandle = NULL;
    BaseType_t status = xTaskCreate(status_task, "status", DEFAULT_STACK_SIZE, NULL, 1, &statusTaskHandle);
    if (status != pdPASS) {
        printf("Status Task creation failed\n");
}

    // Init task voi lopettaa itsensä, koska sen tehtävä on ohi
    vTaskDelete(NULL);
}


//lisätään IMU-sensorilla saatu merkki viestiin, tarkistetaan että puskuriin mahtuu
void add_symbol_to_message(char symbol) {
    if (morseIndex < sizeof(morseMessage) -1) {
        morseMessage[morseIndex++] = symbol;
        morseMessage[morseIndex] = '\0';
    }
}

void buttonFxn(uint gpio, uint32_t eventMask) {
    if (gpio == BUTTON1) {
        switch (programState) {
            case IDLE:
                change_state(RECORDING);
                break;

            case RECORDING:
                change_state(READY_TO_SEND);
                break;

            case READY_TO_SEND:
                printf("Lähetetään viesti: %s\n", morseMessage);
                send_morse_message(morseMessage);
                
                // Vihreä LED ja summeri päälle merkiksi onnistuneesta lähetyksestä
                gpio_put(RGB_LED_G, 1);
                gpio_put(BUZZER_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(300)); //summeri soi 300ms
                gpio_put(BUZZER_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(200));// Näytä LED hetken
                gpio_put(RGB_LED_G, 0);

                // viesti nollataan
                morseIndex = 0;
                morseMessage[0] = '\0';
                letterFinalized = false;
                change_state(IDLE);
                break;
        }
    }
}

static void morse_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temp;
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
                    } else if (fabs(ax) > 0.8 && fabs(az) < 0.3 && fabs(ay) < 0.3) {
                        //laite pystyasennossa = viiva
                        currentSymbol = '-';
                        symbolStartTick = now;
                        symbolDetected = true;
                        readyForNextSymbol = false;
                    } else if (fabs(ay) > 0.8 && fabs(ax) < 0.3 && fabs(az) < 0.3) {
                        //kallistuu ylöspäin = välilyönti
                        currentSymbol = ' ';
                        symbolStartTick = now;
                        symbolDetected = true;
                        readyForNextSymbol = false;
                    }
                } else if (symbolDetected) {  
                    //symboli hyväksytään, jos asento pysyy 500ms
                    if ((now - symbolStartTick) > pdMS_TO_TICKS(500)) {
                        add_symbol_to_message(currentSymbol);
                        printf("Hyväksytty symboli: %c\n", currentSymbol);
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
                    printf("Kirjain valmis\n");
                    letterFinalized = true;

                    // Sininen LED merkiksi valmiista kirjaimesta
                    gpio_put(RGB_LED_B, 1);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    gpio_put(RGB_LED_B, 0);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Päivittää LED-tilat ohjelman päätilan mukaan
static void status_task(void *arg) {
    (void)arg;
    const TickType_t blinkSlow = pdMS_TO_TICKS(500);
    const TickType_t blinkFast = pdMS_TO_TICKS(200);

    for (;;) {
        switch (programState) {
            case IDLE:
                gpio_put(RGB_LED_R, 1);
                vTaskDelay(blinkSlow);
                gpio_put(RGB_LED_R, 0);
                vTaskDelay(blinkSlow);
                break;

            case RECORDING:
                gpio_put(RGB_LED_B, 1);
                vTaskDelay(blinkFast);
                gpio_put(RGB_LED_B, 0);
                vTaskDelay(blinkFast);
                break;

            case READY_TO_SEND:
                gpio_put(RGB_LED_G, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_put(RGB_LED_G, 0);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
    }
}


//Viestin lähetys kerralla
void send_morse_message(const char* message) {
    if (message == NULL || message[0] == '\0') {
        printf("Viestissä ei ole sisältöä");
        return;
    }
    //näyttö ei ole pakollinen taso 1ssä, poista jos haluat
    display_message(message);
    
    uart_puts(uart0, message);
    uart_puts(uart0, "  \n"); // kaksi välilyöntiä ja rivinvaihto loppuun
}

void change_state(enum state newState) {
    programState = newState;
    switch (newState) {
        case IDLE:
            display_message("Tila: IDLE");
            break;
        case RECORDING:
            display_message("Tila: RECORDING");
            break;
        case READY_TO_SEND:
            display_message("Tila: READY_TO_SEND");
            break;
    }
}

//näyttö ei ole pakollinen Taso 1:ssä
void display_message(const char* message) {
    clear_display(); //tyhjentää näytön ennen uutta viestiä
    set_text_cursor(0, 0); //asettaa tekstin aloituskohdan vasempaan yläkulmaan
    write_text(message); //kirjoittaa viestin näytölle
}