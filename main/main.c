#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "freertos/queue.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define TOUCH_THRESH_NO_USE (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)
#define THRESHOLD_OFF 200 // Valor do limiar para desativar

static const char *TAG = "Touch pad";

static volatile bool s_pad_activated[TOUCH_PAD_MAX];
static uint32_t s_pad_threshold_off[TOUCH_PAD_MAX];
static uint32_t s_pad_init_val[TOUCH_PAD_MAX];

volatile bool car_system_actuator[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// 0: electronic injection | 3: internal temperature |
// 4: ABS | 5: airbag | 6: seat belt |
// 7: front headlight light | 8: power window system | 9: two door lock

uint64_t beginEL, endTimeEL, beginIT, endTimeIT;
uint64_t beginABS, endTimeABS, beginAIR, endTimeAIR;
uint64_t beginSB, endTimeSB, beginLVT, endTimeLVT;
uint64_t beginFHL, endTimeFHL, beginPWS, endTimePWS;
uint64_t beginTDL, endTimeTDL, beginDisplay, endTimeDisplay;

SemaphoreHandle_t mutex; // Declare a mutex handle

void read_sensor(int index)
{
    uint16_t touch_value;
    touch_pad_read_filtered(index, &touch_value);
    xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
    if (touch_value > s_pad_threshold_off[index])
    {
        s_pad_activated[index] = false;
    }
    xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
}

void execute_actuator(int index)
{
    xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
    car_system_actuator[index] = true;
    xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
}
void colorful_print(const char *name, bool binary)
{
    printf("%s", name);

    if (binary == 0)
    {
        printf("\033[31mDESATIVADO \033[0m"); // Vermelho para desativado
    }
    else
    {
        printf("\033[32mATIVADO \033[0m"); // Verde para ativado
    }

    printf("\n");
}

static void thread_display(void *pvParameter)
{
    while (1)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        beginDisplay = esp_timer_get_time();
        printf("\033[2J\033[1;1H");
        colorful_print("ENGINE: Actuator electronic injection: ", car_system_actuator[0]);
        colorful_print("Actuator internal temperature: ", car_system_actuator[3]);
        colorful_print("BRAKE (ABS): ", car_system_actuator[4]);
        colorful_print("Airbag: ", car_system_actuator[5]);
        colorful_print("Seat belt: ", car_system_actuator[6]);
        colorful_print("Front Headlight Light: ", car_system_actuator[7]);
        colorful_print("Power Window System: ", car_system_actuator[8]);
        colorful_print("Two Door Lock: ", car_system_actuator[9]);
        
        printf("\nDeadline electronic injection: %llu microseconds\n", (endTimeEL - beginEL));
        printf("\nDeadline internal temperature: %llu microseconds\n", (endTimeIT - beginIT));
        printf("\nDeadline ABS: Spent cycles: %llu microseconds\n", (endTimeABS - beginABS));
        printf("\nDeadline AIRBAG: %llu microseconds\n", (endTimeAIR - beginAIR));
        printf("\nDeadline seat belt: %llu microseconds\n", (endTimeSB - beginSB));
        beginLVT = beginPWS + beginSB + beginTDL;
        endTimeLVT = endTimePWS + endTimeSB + endTimeTDL;
        printf("\nDeadline LVT: %llu microseconds\n", (endTimeLVT - beginLVT));
        endTimeDisplay = esp_timer_get_time();
        printf("\nDeadline Display: %llu microseconds\n", (endTimeDisplay - beginDisplay));
        xSemaphoreGive(mutex);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void eletronic_injection(void *pvParameter)
{
    while (1)
    {
        if (s_pad_activated[0] == 1)
        {
            beginEL = esp_timer_get_time();
            read_sensor(0);
            execute_actuator(0);
            endTimeEL = esp_timer_get_time();
        }
        else
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[0] = false;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(0.5 / portTICK_PERIOD_MS);
        //Time of cable
        vTaskDelay(0.016 / portTICK_PERIOD_MS);
    }
}

void internal_temperature(void *pvParameter)
{
    while (1)
    {
        
        if (s_pad_activated[3] == 1)
        {
            beginIT = esp_timer_get_time();
            read_sensor(3);
            execute_actuator(3);
            endTimeIT = esp_timer_get_time();
        }
        else
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[3] = 0;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
        //Time of cable
        vTaskDelay(0.016 / portTICK_PERIOD_MS);
    }
}

void abs_brake(void *pvParameter)
{
    while (1)
    {
        if (s_pad_activated[4] == 1)
        {
            beginABS = esp_timer_get_time();
            read_sensor(4);
            execute_actuator(4);
            endTimeABS = esp_timer_get_time();
        }
        else
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[4] = 0;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        //Time of cable
        vTaskDelay(0.016 / portTICK_PERIOD_MS);
    }
}

void airbag(void *pvParameter)
{
    while (1)
    {
        if (s_pad_activated[5] == 1)
        {
            beginAIR = esp_timer_get_time();
            read_sensor(5);
            execute_actuator(5);
            endTimeAIR = esp_timer_get_time();
        }
        else
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[5] = 0;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        //Time of cable
        vTaskDelay(0.016 / portTICK_PERIOD_MS);
    }
}

void seat_belt(void *pvParameter)
{
    while (1)
    {
        if (s_pad_activated[6] == 1)
        {
            beginSB = esp_timer_get_time();
            read_sensor(6);
            execute_actuator(6);
            endTimeSB = esp_timer_get_time();
        }
        else
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[6] = 0;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //Time of cable
        vTaskDelay(0.016 / portTICK_PERIOD_MS);
    }
}

void front_headlight_light(void *pvParameter)
{
    while (1)
    {
        if (s_pad_activated[7] == 1)
        {
            beginFHL = esp_timer_get_time();
            read_sensor(7);
            execute_actuator(7);
            endTimeFHL = esp_timer_get_time();
        }
        else
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[7] = 0;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //Time of cable
        vTaskDelay(0.016 / portTICK_PERIOD_MS);
    }
}

void power_window_system(void *pvParameter)
{
    while (1)
    {
        if (s_pad_activated[8] == 1)
        {
            beginPWS = esp_timer_get_time();
            read_sensor(8);
            execute_actuator(8);
            endTimePWS = esp_timer_get_time();
        }
        else
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[8] = 0;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //Time of cable
        vTaskDelay(0.016 / portTICK_PERIOD_MS);
    }
}

void two_door_lock(void *pvParameter)
{
    while (1)
    {
        if (s_pad_activated[9] == 1)
        {
            beginTDL = esp_timer_get_time();
            read_sensor(9);
            execute_actuator(9);
            endTimeTDL = esp_timer_get_time();
        }
        else
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[9] = 0;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //Time of cable
        vTaskDelay(0.016 / portTICK_PERIOD_MS);
    }
}

static void tp_touch_pad_init(void)
{
    for (int i = 0; i < TOUCH_PAD_MAX; i++)
    {
        // init RTC IO and mode for touch pad.
        touch_pad_config(i, TOUCH_THRESH_NO_USE);
    }
}

/*
  Read values sensed at all available touch pads.
  Use 2 / 3 of read value as the threshold
  to trigger interrupt when the pad is touched.
  Note: this routine demonstrates a simple way
  to configure activation threshold for the touch pads.
  Do not touch any pads when this routine
  is running (on application start).
 */
static void tp_example_set_thresholds(void)
{
    uint16_t touch_value;
    for (int i = 0; i < TOUCH_PAD_MAX; i++)
    {
        // read filtered value
        touch_pad_read_filtered(i, &touch_value);
        s_pad_init_val[i] = touch_value;
        ESP_LOGI(TAG, "test init: touch pad [%d] val is %d", i, touch_value);
        // set interrupt threshold.
        ESP_ERROR_CHECK(touch_pad_set_thresh(i, touch_value * 2 / 3));
        s_pad_threshold_off[i] = touch_value - 500;
    }
}

/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void tp_example_rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    // clear interrupt
    touch_pad_clear_status();
    for (int i = 0; i < TOUCH_PAD_MAX; i++)
    {
        if ((pad_intr >> i) & 0x01)
        {
            s_pad_activated[i] = true;
        }
    }
}

void app_main(void)
{
    //  Initialize touch pad peripheral.
    //  The default fsm mode is software trigger mode.
    touch_pad_init();
    // If use interrupt trigger mode, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    tp_touch_pad_init();
    // Initialize and start a software filter to detect slight change of capacitance.
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    // Set thresh hold
    tp_example_set_thresholds();
    // Register touch interrupt ISR
    touch_pad_isr_register(tp_example_rtc_intr, NULL);
    // interrupt mode, enable touch interrupt
    touch_pad_intr_enable();
    // Create a mutex
    mutex = xSemaphoreCreateMutex();

    // Start tasks to read values sensed by pads
    xTaskCreate(&eletronic_injection, "eletronic_injection_task", 2048, NULL, 7, NULL);
    xTaskCreate(&internal_temperature, "internal_temperature_task", 2048, NULL, 6, NULL);
    xTaskCreate(&abs_brake, "abs_brake_task", 2048, NULL, 5, NULL);
    xTaskCreate(&airbag, "airbag_task", 2048, NULL, 4, NULL);
    xTaskCreate(&seat_belt, "seat_belt_task", 2048, NULL, 3, NULL);
    xTaskCreate(&two_door_lock, "two_door_lock_task", 2048, NULL, 2, NULL);
    xTaskCreate(&power_window_system, "power_window_system_task", 2048, NULL, 2, NULL);
    xTaskCreate(&front_headlight_light, "front_headlight_light_task", 2048, NULL, 2, NULL);
    // Start tasks to print values sensed by pads
    xTaskCreate(&thread_display, "display_task", 2048, NULL, 1, NULL);
}
