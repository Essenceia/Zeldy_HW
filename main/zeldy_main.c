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

/*
// pin setup
const int pinLed = 26;
const int pinZCVoltage = 0 ; // pin zero crossing voltage TODO change for true value
const int pinADCVoltage = 0; // analog pin for voltage TODO change for true value
const int pinADCCurrent = 0; // analog pin for current TODO change for true value
*/

#define ADC_VOLTAGE ADC1_CHANNEL_4 //CH4 = GPIO32 ... CH7 =GPIO37. WARNING DO NOT, repeat NOT use others WARNING
#define ADC_CURRENT ADC1_CHANNEL_5




// ADC setup
const int iADCVoltageZeroValue = 2048;      //TODO adjust value of 0V mesured by ADC
const float fVoltageMultiplier= 0.15;       //TODO adjust value
const int iADCCurrentZeroValue = 2048;      //TODO adjust value
const float fCurrentMultiplier= 0.015;      //TODO adjust value



// flag setup
volatile int iZCVoltageFlag =0; // flag raised on zero crossing voltage
volatile int iReadADCFlag =0;   // flag raised when reading ADC phase and lowered otherwise when computing data
volatile int iTimerFlag = 1; //flag raised by timer

// mutex for critical flags
portMUX_TYPE muxZCVoltageFlag = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxReadADCFlag = portMUX_INITIALIZER_UNLOCKED;


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
void IRAM_ATTR ISRZCVoltage(){
    // TODO rst timer
    portENTER_CRITICAL_ISR(&muxZCVoltageFlag);
    iZCVoltageFlag =1;
    portEXIT_CRITICAL_ISR(&muxZCVoltageFlag);
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
        iNbMeas = 0; iCurrentADCPos = 0;
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
}




void app_main()
{
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


    fflush(stdout);




// ISR settings
   // attachInterrupt(digitalPinToInterrupt(pinZCVoltage), ISRZCVoltage, FALLING);


    piADCVoltageRead = malloc(1000*sizeof(int));
    piADCCurrentRead = malloc(1000*sizeof(int));
    
    //setup ADC
    adc_power_on();
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_VOLTAGE,ADC_ATTEN_DB_0);
    adc1_config_channel_atten(ADC_CURRENT,ADC_ATTEN_DB_0);
    
    

    while (1){

    if (iZCVoltageFlag) {fnProcessZCVoltageFlag();}
    if (iTimerFlag) {fnProcessTimerFlag();}

    }


    //esp_restart();
}
