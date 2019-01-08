/* Zeldy project gestion capteur

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#define TIMER_PERIODE_MS 1000000



// ADC setup
const int iADCVoltageZeroValue = 2048;      //TODO adjust value of 0V mesured by ADC
const float fVoltageMultiplier= 0.15;       //TODO adjust value
const int iADCCurrentZeroValue = 2048;      //TODO adjust value
const float fCurrentMultiplier= 0.015;      //TODO adjust value



// flag setup
volatile int iZCVoltageFlag =0; // flag raised on zero crossing voltage
volatile int iReadADCFlag =0;   // flag raised when reading ADC phase and lowered otherwise when computing data
volatile int iTimerFlag = 0; //flag raised by timer

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
}



// interupt handlers
static void IRAM_ATTR ISRZCVoltage(void * args){
    ESP_LOGI("ISR ZC Voltage::", "Called");
    // Rst timer
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_PERIODE_MS));
    ESP_LOGI("ISR ZC Voltage::","Timer reset");


    portENTER_CRITICAL_ISR(&muxZCVoltageFlag);
    iZCVoltageFlag =1;
    portEXIT_CRITICAL_ISR(&muxZCVoltageFlag);
    ESP_LOGI("ISR ZC Voltage::" ,"Exited");
}

static void ISRTimer(void * stuff){
    ESP_LOGI("ISR Timer::","called");
    portENTER_CRITICAL_ISR(&muxTimerFlag);
    iTimerFlag =1;
    portEXIT_CRITICAL_ISR(&muxTimerFlag);
    /* timer_pause(TIMER_GROUP_0, TIMER_0);
     timer_set_counter_value(TIMER_GROUP_0, TIMER_GROUP_0, 0x00000000ULL);
     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0,1000);
     timer_set_alarm(TIMER_GROUP_0, TIMER_0,TIMER_ALARM_EN );
     timer_start(TIMER_GROUP_0, TIMER_0);
     */
    ESP_LOGI("ISR Timer::","exit");
}


// Process flags
void fnProcessZCVoltageFlag(){
    iCountZCVoltage++;
    if (iCountZCVoltage%2 == 0){
        iAction = (iAction +1)%3;
    }
    if (iAction == 3) {
        iReadADCFlag = 0; 
        fnComputeRMS();
        iNbMeas = 0;
        iCurrentADCPos = 0;
        iReadADCFlag = 1;
    }
    else{
    }
    portENTER_CRITICAL(&muxZCVoltageFlag);
    iZCVoltageFlag = 0;
    portEXIT_CRITICAL(&muxZCVoltageFlag);
}

void fnProcessTimerFlag(){
    if (iAction ==0){
        int tmp = adc1_get_raw(ADC_VOLTAGE);
        if (tmp>=0){
            piADCVoltageRead[iNbMeas] = tmp;
            iNbMeas++;
        }
    }
    if (iAction ==1){
        int tmp = adc1_get_raw(ADC_CURRENT);
        if (tmp>=0){
            piADCCurrentRead[iCurrentADCPos] = tmp;
            iCurrentADCPos++;
        }
    }
   portENTER_CRITICAL(&muxTimerFlag);
    iTimerFlag = 0;
     portEXIT_CRITICAL(&muxTimerFlag);
    
}




void app_main()
{
    esp_err_t err;
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);
    uint64_t tval;
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


    //fflush(stdout);




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
            .name = "periodic"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_PERIODE_MS));



    piADCVoltageRead = malloc(1000*sizeof(int));
    piADCCurrentRead = malloc(1000*sizeof(int));


    //setup ADC
    adc_power_on();
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_VOLTAGE,ADC_ATTEN_DB_0));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CURRENT,ADC_ATTEN_DB_0));


    //xTaskCreate(&fnProcessTimerFlag, "fnProcessTimerFlag", 4096, NULL , 5, NULL);

    while (1){
       // timer_get_counter_value(TIMER_GROUP_0, TIMER_0 , &tval);
    printf("t:%lld [%d]\n",esp_timer_get_time() , iTimerFlag);
        if (iZCVoltageFlag) {fnProcessZCVoltageFlag();}
        if (iTimerFlag) {fnProcessTimerFlag();}
    vTaskDelay(50);
    }


    //esp_restart();
}
