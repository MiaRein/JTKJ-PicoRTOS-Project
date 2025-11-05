
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

//nykyinen symboli
char currentSymbol; 

//Morse-viestin puskuri
char morseMessage[256]; 
size_t morseIndex   = 0;

//aikaperusteiseen kirjainten ja viestin hyväksyntään globaalit muuttujat
bool letterFinalized;
absolute_time_t lastSymbolTime; //edellisen symbolin havainnosta

//lisätään IMU-sensorilla saatu merkki viestiin, tarkistetaan että puskuriin mahtuu
void add_symbol_to_message(char symbol) {
    if (morseIndex < sizeof(morseMessage) -1) {
        morseMessage[morseIndex++] = symbol;
        morseMessage[morseIndex] = '\0';
    }
}


void buttonFxn(uint gpio, uint32_t eventMask) {
    if (gpio == BUTTON1) {
        printf("Lähetetään viesti: %s\n", morseMessage)
        
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
    }
}


// lukee IMU-dataa ja tunnistaa liikkeet/asennot
static void morse_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temp;

    absolute_time_t symbolStartTime; //kuinka kauan symbolin havainnosta
    bool symbolDetected = false; //onko symbooli havaittu

    for(;;) {
        // lukee IMUN:n sensoridatan -> palauttaa 0 jos lukeminen onnistuu
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
            if(programState == RECORDING)   {
               if (!symbolDetected) {
                    if (fabs(az) > 0.8 && fabs(ax) < 0.3 && fabs(ay) < 0.3) {
                        //laite vaakatasossa
                        currentSymbol = '.';
                        symbolStartTime = get_absolute_time();
                        symbolDetected = true;
                    } else if (fabs(ax) > 0.8 && fabs(az) < 0.3 && fabs(ay) < 0.3) {
                        //laite pystyasennossa
                        currentSymbol = '-';
                        symbolStartTime = get_absolute_time();
                        symbolDetected = true;
                    } else if ((fabs(ay) > 0.8 || fabs(ay) < -0.8) && fabs(ax) < 0.3 && fabs(az) < 0.3) {
                        //kallistuu ylöspäin
                        currentSymbol = ' ';
                        symbolStartTime = get_absolute_time();
                        symbolDetected = true;
                    }
                } else {  
                    //symboli hyväksytään, jos asento pysyy 500ms
                    if (absolute_time_diff_us(symbolStartTime, get_absolute_time()) > 500000) {
                        add_symbol_to_message(currentSymbol);
                        printf("Hyväksytty symboli: %c\n", currentSymbol);
                        symbolDetected = false;
                        lastSymbolTime = get_absolute_time();
                        letterFinalized = false;
                    }                    
                }
                if (absolute_time_diff_us(lastSymbolTime, get_absolute_time()) > 1500000 && !letterFinalized && morseIndex > 0) {
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

// lähettää tunnistetut symbolit UART:n kautta
static void uart_task(void *arg) {
    (void)arg;

    for(;;) {
        tight_loop_contents(); // Modify with application code here.
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

//testataakseen asentoja
/*
static void imu_test_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temp;

    for (;;) {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
            printf("ax: %.2f, ay: %.2f, az: %.2f\n", ax, ay, az);

            if (fabs(az) > 0.85 && fabs(ax) < 0.3 && fabs(ay) < 0.3) {
                printf("Asento: Vaakatasossa (piste)\n");
            } else if (fabs(ax) > 0.85 && fabs(ay) < 0.3 && fabs(az) < 0.3) {
                printf("Asento: Pystyasennossa (viiva)\n");
            } else if (fabs(ay) > 0.85 && fabs(ax) < 0.3 && fabs(az) < 0.3) {
                printf("Asento: Kallistettu eteenpäin (välilyönti)\n");
            } else if (fabs(ay) < -0.85 && fabs(ax) < 0.3 && fabs(az) < 0.3) {
                printf("Asento: Kallistettu taaksepäin (välilyönti)\n");
            } else {
                printf("Asento: Ei tunnistettu\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/

int main() {
    stdio_init_all();
    //Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }

    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

     //IMU:n alustaminen
    init_i2c_default();
    if (init_ICM42670() != 0) {
        printf("IMU init failed\n");
    }
    ICM42670_start_with_default_values();

    //LEDin eri värien alustaminen
    gpio_init(RGB_LED_B);
    gpio_set_dir(RGB_LED_B, GPIO_OUT);

    gpio_init(RGB_LED_G);
    gpio_set_dir(RGB_LED_G, GPIO_OUT);

    //painikkeiden alustaminen
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, &buttonFxn);

    //äänimerkin alustus
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0); //aluksi pois päältä

    //asennon testauksiin
    //xTaskCreate(imu_test_task, "IMU Test", DEFAULT_STACK_SIZE, NULL, 1, NULL);

    TaskHandle_t morseTaskHandle = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t morse = xTaskCreate(morse_task,       // (en) Task function
                "morse",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &morseTaskHandle);    // (en) A handle to control the execution of this task

    TaskHandle_t uartTaskHandle = NULL;
    BaseType_t uart = xTaskCreate(uart_task, "uart", DEFAULT_STACK_SIZE, NULL, 2, &uartTaskHandle);

    /* TÄMÄ KOMMENTEISSA KUN pdPASS herjaa ettei ole alustettu?
    if(result != pdPASS) {
        printf("Morse Task creation failed\n");
        return 0;
    }*/
    

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

