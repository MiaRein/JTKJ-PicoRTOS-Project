
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
//#include <pins.h>

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
absolute_time_t lastSymbolTime; //kuinka kauan edellisen symbolin havainnosta
absolute_time_t lastMessageActivityTime; //kuinka kauan minkään toiminnan havainnosta
bool letterFinalized = false; //onko kirjain valmis


/* valmiiksi annettu esimerkki
static void example_task(void *arg){
    (void)arg;

    for(;;){
        tight_loop_contents(); // Modify with application code here.
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}*/

// lukee IMU-dataa ja tunnistaa liikkeet/asennot
static void morse_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temp;

    

    for(;;) {
        // lukee IMUN:n sensoridatan -> palauttaa 0 jos lukeminen onnistuu
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
            if(programState == RECORDING)   {
                if (fabs(az) > 0.8 && fabs(ax) < 0.3) {
                    //jos Z-akselin kiihtyvyys on yli 0.8g (laite vaakatasossa) ja X-akseli on lähes nolla, tulkitaan pisteeksi
                    currentSymbol = '.';
                    add_symbol_to_message(currentSymbol);
                    printf("Havaittu piste\n");
                } else if (fabs(ax) > 0.8 && fabs(az) < 0.3) {
                    //jos X-akselin kiihtyvyys on yli 0.8 g (laite pystyasennossa) ja Z-akseli on lähes nolla, tulkitaan viivaksi
                    currentSymbol = '-';
                    add_symbol_to_message(currentSymbol);
                    printf("Havaittu viiva\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//lisätään IMU-sensorilla saatu merkki viestiin, tarkistetaan että puskuriin mahtuu
void add_symbol_to_message(char symbol) {
    if (morseIndex < sizeof(morseMessage) -1) {
        morseMessage[morseIndex++] = symbol;
        morseMessage[morseIndex] = '\0';
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

int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/ 

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

    gpio_init(RGB_LED_R);
    gpio_set_dir(RGB_LED_R, GPIO_OUT);

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

