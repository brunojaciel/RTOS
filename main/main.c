#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "freertos/semphr.h"

#define TOUCH_THRESH_NO_USE   (0)

bool car_system_sensors[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool car_system_actuator[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//0: electronic injection | 1: internal temperature | 
//2: ABS right whell | 3: ABS left whell |
//4: airbag | 5: seat belt | 

//6: front headlight light right | 7: front headlight light left | 
//8: power window system right | 9: power window system left
//10: two door lock right | 11: two door lock left 

SemaphoreHandle_t mutex; // Declare a mutex handle

void read_sensor(int index){
    xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
    car_system_sensors[index] = !car_system_sensors[index];
    xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
}

void execute_actuator(int index){
    xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
    car_system_actuator[index] = !car_system_actuator[index];
    xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
    //printf("\nActive the actuator!");
}

/*
  Read values sensed at all available touch pads.
 Print out values in a loop on a serial monitor.
 */
static void tp_read_task(void *pvParameter)
{
    uint16_t touch_value_0, touch_value_3, touch_value_4, touch_value_5;
    uint16_t touch_value_6, touch_value_7, touch_value_8, touch_value_9;

    //printf("Touch Sensor normal mode read, the output format is: \nTouchpad num:[raw data]\n\n");
    while(1) {

        touch_pad_read(0, &touch_value_0);
        //touch_pad_read(2, &touch_value_2);
        touch_pad_read(3, &touch_value_3);
        touch_pad_read(4, &touch_value_4);
        touch_pad_read(5, &touch_value_5);
        touch_pad_read(6, &touch_value_6);
        touch_pad_read(7, &touch_value_7);
        touch_pad_read(8, &touch_value_8);
        touch_pad_read(9, &touch_value_9);
        
        //printf("T%d:[%4d] \n", 2, touch_value_2);

        if(touch_value_0 < (uint16_t)100){
            printf("\nelectronic injection");
            xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
            car_system_sensors[0] = 1;
            xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }

        if(touch_value_3 < (uint16_t)100){
            printf("\ninternal temperature");
            xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
            car_system_sensors[1] = 1;
            xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }

        if(touch_value_4 < (uint16_t)100){
            printf("\nABS");
            xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
            car_system_sensors[2] = 1;
            car_system_sensors[3] = 1;
            xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }

        if(touch_value_5 < (uint16_t)100){
            printf("\nairbag");
            xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
            car_system_sensors[4] = 1;
            xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }

        if(touch_value_6 < (uint16_t)100){
            printf("\nseat belt");
            xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
            car_system_sensors[5] = 1;
            xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }

        if(touch_value_7 < (uint16_t)100){
            printf("\nfront headlight light");
            xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
            car_system_sensors[6] = 1;
            car_system_sensors[7] = 1;
            xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }

        if(touch_value_8 < (uint16_t)100){
            printf("\npower window system");
            xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
            car_system_sensors[8] = 1;
            car_system_sensors[9] = 1;
            xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }

        if(touch_value_9 < (uint16_t)100){
            printf("\ntwo door lock");
            xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
            car_system_sensors[10] = 1;
            car_system_sensors[11] = 1;
            xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        //printf("Engine %d\n", car_system_sensors[0]);
    }
}

void thread_display(void) {
	while(1){

        printf("\nENGINE: Actuator electronic injection: %d | Actuator internal temperature: %d\n", car_system_sensors[0], car_system_sensors[1]);
        printf("BRAKE: ABS right: %d | ABS left: %d\n", car_system_sensors[2], car_system_sensors[3]);
        printf("LSE: Airbag: %d | Seat belt: %d\n", car_system_sensors[4], car_system_sensors[5]);
        printf("LVT: FHL right: %d | FHL left: %d | PWS right: %d | PWS left: %d | TDL right: %d | TDL left: %d\n", car_system_sensors[6], car_system_sensors[7], car_system_sensors[8], car_system_sensors[9], car_system_sensors[10], car_system_sensors[11]);
        
        vTaskDelay(500/portTICK_PERIOD_MS);
	}
}

static void tp_execute_task(void *pvParameter){
    while(1)
    {
        for (int i = 0; i < 12; i++) {
            if (car_system_sensors[i] == 1) {
                read_sensor(i);
                execute_actuator(i);
            }
        }

        thread_display();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void tp_touch_pad_init(void)
{
    
    for (int i = 0;i< TOUCH_PAD_MAX;i++) {
        touch_pad_config(i, TOUCH_THRESH_NO_USE);
    }
    
}

void app_main(void)
{
    // Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
    touch_pad_init();

    // Set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    tp_touch_pad_init();

    // Create a mutex
    mutex = xSemaphoreCreateMutex();

    // Start task to read values sensed by pads
    xTaskCreate(&tp_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
    xTaskCreate(&tp_execute_task, "execute_task", 2048, NULL, 5, NULL);
}
