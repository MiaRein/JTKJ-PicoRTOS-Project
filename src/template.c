
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <math.h>

#include "tkjhat/sdk.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 

//Add here necessary states
enum state { IDLE=1, RECORDING, READY_TO_SEND };
enum state programState = IDLE;

char currentSymbol; 
char morseMessage[256]; 
size_t morseIndex = 0;
absolute_time_t symbolStartTime; //kuinka kauan symbolin havainnosta
bool symbolDetected = false; //onko symbooli havaittu
bool letterFinalized = false; //onko kirjain valmis
bool readyForNextSymbol = true; // mahdollistaa useamman saman symbolin lukemisen liikuttamatta laitetta
absolute_time_t lastSymbolAcceptedTime; //edellisen symbolin havainno

static void morse_task(void *arg);
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

    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

     //IMU:n alustaminen
    init_i2c_default();
    if (init_ICM42670() != 0) {
        gpio_put(RGB_LED_R, 1); //punainen led palaa virheestä
        sleep_ms(1000);
        gpio_put(RGB_LED_R, 0);
        printf("IMU init failed\n");
    }
    ICM42670_start_with_default_values();

    //LEDin eri värien alustaminen
    gpio_init(RGB_LED_B); //sininen
    gpio_set_dir(RGB_LED_B, GPIO_OUT);

    gpio_init(RGB_LED_G); //vihreä
    gpio_set_dir(RGB_LED_G, GPIO_OUT);

    gpio_init(RGB_LED_R); //punainen
    gpio_set_dir(RGB_LED_R, GPIO_OUT);

    //painikkeiden alustaminen
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, &buttonFxn);

    //äänimerkin alustus
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0); //aluksi pois päältä

    //näytön alustus
    init_display();

    TaskHandle_t morseTaskHandle = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t morse = xTaskCreate(morse_task, "morse", DEFAULT_STACK_SIZE, NULL, 2, &morseTaskHandle);    

    if (morse != pdPASS) {
        printf("Morse Task creation failed\n");
        return 0;
    }
    
    TaskHandle_t uartTaskHandle = NULL;
    BaseType_t uart = xTaskCreate(uart_task, "uart", DEFAULT_STACK_SIZE, NULL, 2, &uartTaskHandle);

    if (uart != pdPASS) {
        printf("Uart Task creation failed\n");
        return 0;
    }
    
    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
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
                sleep_ms(300); //summeri soi 300ms
                gpio_put(BUZZER_PIN, 0);

                sleep_ms(200); // Näytä LED hetken
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

    for(;;) {
        // lukee IMUN:n sensoridatan -> palauttaa 0 jos lukeminen onnistuu
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
            if(programState == RECORDING)   {
               if (readyForNextSymbol) {
                    if (fabs(az) > 0.8 && fabs(ax) < 0.3 && fabs(ay) < 0.3) {
                        //laite vaakatasossa
                        currentSymbol = '.';
                        symbolStartTime = get_absolute_time();
                        symbolDetected = true;
                        readyForNextSymbol = false;
                    } else if (fabs(ax) > 0.8 && fabs(az) < 0.3 && fabs(ay) < 0.3) {
                        //laite pystyasennossa
                        currentSymbol = '-';
                        symbolStartTime = get_absolute_time();
                        symbolDetected = true;
                        readyForNextSymbol = false;
                    } else if ((fabs(ay) > 0.8 || fabs(ay) < -0.8) && fabs(ax) < 0.3 && fabs(az) < 0.3) {
                        //kallistuu ylöspäin
                        currentSymbol = ' ';
                        symbolStartTime = get_absolute_time();
                        symbolDetected = true;
                        readyForNextSymbol = false;
                    }
                } else if (symbolDetected) {  
                    //symboli hyväksytään, jos asento pysyy 500ms
                    if (absolute_time_diff_us(symbolStartTime, get_absolute_time()) > 500000) {
                        add_symbol_to_message(currentSymbol);
                        printf("Hyväksytty symboli: %c\n", currentSymbol);
                        symbolDetected = false;
                        lastSymbolAcceptedTime = get_absolute_time();
                        letterFinalized = false;
                    }                    
                } else {
                    if (absolute_time_diff_us(lastSymbolAcceptedTime, get_absolute_time()) > 700000) {
                        readyForNextSymbol = true;
                    }
                }
                if (absolute_time_diff_us(lastSymbolAcceptedTime, get_absolute_time()) > 1500000 && !letterFinalized && morseIndex > 0) {
                    printf("Kirjain valmis\n");
                    letterFinalized = true;


                    // Sininen LED merkiksi valmiista kirjaimesta
                    gpio_put(RGB_LED_B, 1);
                    sleep_ms(500); 
                    gpio_put(RGB_LED_B, 0);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//Viestin lähetys kerralla
void send_morse_message(const char* message) {
    if (message == NULL || message[0] == '\0') {
        printf("Viestissä ei ole sisältöä");
        return;
    }
    display_message(message);
    
    uart_puts(uart0, message);
    uart_putc_raw(uart0, '\n'); //rivinvaihto loppuun
}

void change_state(enum state newState) {
    programState = newState;
    const char* stateText = "";
    switch (newState) {
        case IDLE: stateText = "Tila: IDLE"; break;
        case RECORDING: stateText = "Tila: RECORDING"; break;
        case READY_TO_SEND: stateText = "Tila: READY_TO_SEND"; break;
    }
    printf("s\n", stateText);
    display_message(stateText);
}

void display_message(const char* message) {
    clear_display(); //tyhjentää näytön ennen uutta viestiä
    set_text_cursor(0, 0); //asettaa tekstin aloituskohdan vasempaan yläkulmaan
    //jos viesti on pitkä, harkitse rivinvaihtoa
    //write_text_xy(0, 0, "Viesti:");
    //write_text_xy(0, 10, message); // toinen rivi
    write_text(message); //kirjoittaa viestin näytölle
}
