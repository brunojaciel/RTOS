#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "freertos/queue.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"
#include "esp_log.h"
#include "freertos/semphr.h"

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

static const char *TAG = "Touch pad";

static bool s_pad_activated[TOUCH_PAD_MAX];
static uint32_t s_pad_init_val[TOUCH_PAD_MAX];

bool car_system_actuator[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//0: electronic injection | 3: internal temperature | 
//4: ABS | 5: airbag | 6: seat belt | 
//7: front headlight light | 8: power window system | 9: two door lock

SemaphoreHandle_t mutex; // Declare a mutex handle

void read_sensor(int index){
    xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
    s_pad_activated[index] = false;
    xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
}

void execute_actuator(int index){
    xSemaphoreTake(mutex, portMAX_DELAY); // Take the mutex before accessing the global variable
    car_system_actuator[index] = true;
    xSemaphoreGive(mutex); // Give back the mutex after modifying the global variable
}

static void thread_display(void *pvParameter) {
    while(1){
        xSemaphoreTake(mutex, portMAX_DELAY);
        printf("\033[2J\033[1;1H");
        printf("\nENGINE: Actuator electronic injection: %d | Actuator internal temperature: %d\n", car_system_actuator[0], car_system_actuator[3]);
        printf("BRAKE: ABS %d\n", car_system_actuator[4]);
        printf("LSE: Airbag: %d | Seat belt: %d\n", car_system_actuator[5], car_system_actuator[6]);
        printf("LVT: Front Headlight Light: %d | Power Window System: %d | Two Door Lock: %d\n", car_system_actuator[7], car_system_actuator[8], car_system_actuator[9]);
        xSemaphoreGive(mutex);

        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

static void tp_execute_task(void *pvParameter){
    while(1)
    {
        for (int i = 0; i < 12; i++) {
            if (s_pad_activated[i] == 1) {
                //printf("aq");
                read_sensor(i);
                execute_actuator(i);
            } else {
                xSemaphoreTake(mutex, portMAX_DELAY);
                car_system_actuator[i] = 0;
                xSemaphoreGive(mutex);
            }
        }
        //thread_display();
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void eletronic_injection(void *pvParameter){
    while(1){
        if(s_pad_activated[0] == 1){
            read_sensor(0);
            execute_actuator(0);
        } else {
            xSemaphoreTake(mutex, portMAX_DELAY);
            car_system_actuator[0] = 0;
            xSemaphoreGive(mutex);
        }
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
}

static void tp_touch_pad_init(void)
{
    for (int i = 0;i< TOUCH_PAD_MAX;i++) {
        //init RTC IO and mode for touch pad.
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
    for (int i = 0; i < TOUCH_PAD_MAX; i++) {
        //read filtered value
        touch_pad_read_filtered(i, &touch_value);
        s_pad_init_val[i] = touch_value;
        ESP_LOGI(TAG, "test init: touch pad [%d] val is %d", i, touch_value);
        //set interrupt threshold.
        ESP_ERROR_CHECK(touch_pad_set_thresh(i, touch_value * 2 / 3));

    }
}

/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void tp_example_rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    for (int i = 0; i < TOUCH_PAD_MAX; i++) {
        if ((pad_intr >> i) & 0x01) {
            s_pad_activated[i] = true;
        }
    }
}

void app_main(void)
{
    //uint64_t eus, eus2;

    //eus = esp_timer_get_time();
    // Initialize touch pad peripheral, it will start a timer to run a filter
    //ESP_LOGI(TAG, "Initializing touch pad");
    // Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
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
    //interrupt mode, enable touch interrupt
    touch_pad_intr_enable();
    // Create a mutex
    mutex = xSemaphoreCreateMutex();

    // Start task to read values sensed by pads
    xTaskCreate(&tp_execute_task, "execute_task", 2048, NULL, 5, NULL);
    xTaskCreate(&thread_display, "display_task", 2048,  NULL, 5, NULL);
    //xTaskCreate(&eletronic_injection, "eletronic_injection_task", 2048, NULL, 5, NULL);
}
