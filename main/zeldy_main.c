/* Zeldy project gestion capteur

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "math.h"
#include <driver/adc.h>
#include <driver/gpio.h>
#include <driver/timer.h>
#include "nvs_flash.h"
#include "esp_log.h"

/*
// pin setup
const int pinLed = 26;
const int pinZCVoltage = 0 ; // pin zero crossing voltage TODO change for true value
const int pinADCVoltage = 0; // analog pin for voltage TODO change for true value
const int pinADCCurrent = 0; // analog pin for current TODO change for true value
*/

#define ADC_VOLTAGE ADC1_CHANNEL_4 //CH4 = GPIO32 ... CH7 =GPIO37. WARNING DO NOT, repeat NOT use others WARNING
#define ADC_CURRENT ADC1_CHANNEL_5

// TIMER SETUP
#define TIMER_PERIODE_US 1000000



// ADC setup
const int iADCVoltageZeroValue = 2048;      //TODO adjust value of 0V mesured by ADC
const float fVoltageMultiplier= 0.15;       //TODO adjust value
const int iADCCurrentZeroValue = 2048;      //TODO adjust value
const float fCurrentMultiplier= 0.015;      //TODO adjust value



// flag setup
/*volatile int iZCVoltageFlag =0; // flag raised on zero crossing voltage
volatile int iReadADCFlag =0;
volatile int iTimerFlag = 0;
*/
volatile  EventGroupHandle_t EventGroup;
#define FLAG_iZCVoltage (1<<0) // flag raised on zero crossing voltage
#define FLAG_iReadADC   (1<<1) // flag raised when reading ADC phase and lowered otherwise when computing data
#define FLAG_iTimer     (1<<2) //flag raised by timer

// mutex for critical flags
portMUX_TYPE muxZCVoltageFlag = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxTimerFlag = portMUX_INITIALIZER_UNLOCKED;


// synchro values
int iCountZCVoltage=0;
int iCurrentADCPos=0;
int iAction =0; // 0 read voltage, 1 read current, 2 compute


// Data
float fCosPhi = 1; 
int * piADCVoltageRead;
int * piADCCurrentRead;
int iNbMeas=0; //nb of measures
float fMeanActivePower=0.0;
float fMeanApparentPower=0.0;
float fVoltageRMS = 0.0;
float fCurrentRMS = 0.0;

//Timers
static esp_timer_handle_t periodic_timer;



// Convertion and computation rms
float fnADC2Voltage (int ADCVoltageValue){
    return ( fVoltageMultiplier * (ADCVoltageValue-iADCVoltageZeroValue) );}

float fnADC2Current (int ADCCurrentValue){
    return ( fCurrentMultiplier * (ADCCurrentValue-iADCCurrentZeroValue) );}

void fnComputeRMS(){
    ESP_LOGI("ComputeRMS::", "Computing for iNbMeas=%d", iNbMeas);
    for (int i=0;i<iNbMeas ;i++) {
        float tempV = fnADC2Voltage(piADCVoltageRead[i]);
        float tempI = fnADC2Current(piADCCurrentRead[i]);
        fMeanActivePower+= tempV * tempI;
        fVoltageRMS += tempV * tempV;
        fCurrentRMS += tempI * tempI;
    }
    fMeanActivePower = fMeanActivePower/iNbMeas;
    fVoltageRMS = sqrt(fVoltageRMS)/iNbMeas;
    fCurrentRMS = sqrt(fCurrentRMS)/iCurrentADCPos;
    fMeanApparentPower = fVoltageRMS * fCurrentRMS;
    fCosPhi = fMeanActivePower / fMeanApparentPower;
    ESP_LOGI("ComputeRMS::", "Results CosPhi=%f", fCosPhi);

}



// interupt handlers
static void IRAM_ATTR ISRZCVoltage(void * args){
    BaseType_t xHigherPriorityTaskWoken;
    ESP_LOGI("ISR ZC Voltage::", "Called");

    // Rst timer
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_PERIODE_US));
    ESP_LOGI("ISR ZC Voltage::","Timer reset");

    /*
     *  used as a light weight and faster binary or counting semaphore alternative.
     */
    xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(
            EventGroup,   /* The event group being updated. */
           FLAG_iZCVoltage, /* The bits being set. */
            &xHigherPriorityTaskWoken );

    ESP_LOGI("ISR ZC Voltage::" ,"Exited");
}

static void ISRTimer(void * stuff){
    BaseType_t xHigherPriorityTaskWoken;
    ESP_LOGI("ISR Timer::","called");
    xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(
            EventGroup,   /* The event group being updated. */
            FLAG_iTimer, /* The bits being set. */
            &xHigherPriorityTaskWoken );

    ESP_LOGI("ISR Timer::","exit");
}


// Process flags
void fnProcessZCVoltageFlag(){
    iCountZCVoltage++;

    if (iCountZCVoltage%2 == 0){
        iAction = (iAction +1)%3;

    }

    ESP_LOGI("Process ZC Voltage::", "iCountZCVoltage %d, iAction %d", iCountZCVoltage,iAction);

    if (iAction == 2) {

        xEventGroupClearBits(
                EventGroup,  /* The event group being updated. */
                FLAG_iReadADC );/* The bits being cleared. */
        //iReadADCFlag = 0;
        fnComputeRMS();
        iNbMeas = 0;
        iCurrentADCPos = 0;

        xEventGroupSetBits(
                EventGroup,  /* The event group being updated. */
                FLAG_iReadADC );/* The bits being cleared. */
       // iReadADCFlag = 1;
    }
    else{
    }
    xEventGroupClearBits(
            EventGroup,  /* The event group being updated. */
            FLAG_iZCVoltage );/* The bits being cleared. */
  /*  portENTER_CRITICAL(&muxZCVoltageFlag);
    iZCVoltageFlag = 0;
    portEXIT_CRITICAL(&muxZCVoltageFlag);*/
}

void fnProcessTimerFlag(){
    ESP_LOGI("Process timer::", "Process timer called action %d", iAction);
    int tmp;
    if (iAction ==0){
         tmp = adc1_get_raw(ADC_VOLTAGE);
        ESP_LOGI("Process timer::", "Read ADC voltage returns %d", tmp);
        if (tmp>=0){
            piADCVoltageRead[iNbMeas] = tmp;
            iNbMeas++;
            ESP_LOGI("Process timer::", "Increment iNbMeas new val %d", iNbMeas);
        }
    }
    if (iAction ==1){
        tmp = adc1_get_raw(ADC_CURRENT);
        ESP_LOGI("Process timer::", "Read ADC current returns %d", tmp);
        if (tmp>=0){
            piADCCurrentRead[iCurrentADCPos] = tmp;
            ESP_LOGI("Process timer::", "Increment iCurrentADCPos new val %d", iCurrentADCPos);

            iCurrentADCPos++;
        }
    }
    xEventGroupClearBits(
            EventGroup,  /* The event group being updated. */
            FLAG_iTimer);/* The bits being cleared. */
  /* portENTER_CRITICAL(&muxTimerFlag);
    iTimerFlag = 0;
     portEXIT_CRITICAL(&muxTimerFlag);
     */
    
}




void app_main()
{
    BaseType_t uxBits;
    esp_err_t err;
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");


    //Event signaling
    EventGroup = xEventGroupCreate();
    if( EventGroup == NULL )
    {
        ESP_LOGE("App main", "Event group creation failed");
    }




// ISR settings
    gpio_pad_select_gpio(GPIO_NUM_5);
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_intr_type(GPIO_NUM_5, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_intr_enable(GPIO_NUM_5));
    ESP_ERROR_CHECK(gpio_isr_register(ISRZCVoltage, NULL, 0, 0));


    printf("Start config\n");

// timer setting
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &ISRTimer,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_PERIODE_US));



    piADCVoltageRead = malloc(1000*sizeof(int));
    piADCCurrentRead = malloc(1000*sizeof(int));


    //setup ADC
    adc_power_on();
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_VOLTAGE,ADC_ATTEN_DB_0));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CURRENT,ADC_ATTEN_DB_0));


    while (1){

        uxBits = xEventGroupWaitBits(
                EventGroup,   /* The event group being tested. */
                FLAG_iZCVoltage  | FLAG_iTimer, /* The bits within the event group to wait for. */
                pdFALSE,        /* Flag bits should not be cleared before returning. */
                pdFALSE,       /* Don't wait for both bits, either bit will do. */
                portMAX_DELAY  );/* Wait a maximum */
        if( ( uxBits & FLAG_iZCVoltage ) != 0 ){
            fnProcessZCVoltageFlag();
        }else if (  ( uxBits & FLAG_iTimer ) != 0 ){
            fnProcessTimerFlag();
        }
        ESP_LOGI("Main Loop::", "Event has been triggered");
    }

}
