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

// pin setup
const int pinLed = 26;
const int pinZCVoltage = 0 ; // pin zero crossing voltage TODO change for true value
const int pinADCVoltage = 0; // analog pin for voltage TODO change for true value
const int pinADCCurrent = 0; // analog pin for current TODO change for true value



// ADC setup
const int iADCVoltageZeroValue = 2048;      //TODO adjust value of 0V mesured by ADC
const float fVoltageMultiplier= 0.15;       //TODO adjust value
const int iADCCurrentZeroValue = 2048;      //TODO adjust value
const float fCurrentMultiplier= 0.015;      //TODO adjust value



// flag setup
volatile int iZCVoltageFlag =0; // flag raised on zero crossing voltage
volatile int iSetupFlag =1;     // flag used for synchro on voltage zero crossing
volatile long lTimeAtZCVoltage;
volatile int iReadDACFlag =0;   // flag raised when reading ADC phase and lowered otherwise

// mutex for critical flags
portMUX_TYPE muxZCVoltageFlag = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxReadADCFlag = portMUX_INITIALIZER_UNLOCKED;


// synchro values
int iCountZCVoltage=0;
long lHalfPeriod =0;
long lElapsedTimeSetup=0;



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
    return ( fVoltageMultiplier * (ADCVoltageValue-iADCVoltageZeroValue) );

float fnADC2Current (int ADCCurrentValue){
    return ( fCurrentMultiplier * (ADCCurrentValue-iADCCurrentZeroValue) );

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
    fCurrentRMS = sqrt(fCurrentRMS)/iNbMeas;
    fMeanApparentPower = fVoltageRMS * fCurrentRMS;
    fCosPhi = fMeanActivePower / fMeanApparentPower;
}



// interupt handlers
void IRAM_ATTR ISRZCVoltage(){
    lTimeAtZCVoltage = 0 // TODO timer value
    // TODO rst timer
    if (iSetupFlag) {
        portENTER_CRITICAL_ISR(&muxZCVoltageFlag);
        iZCVoltageFlag =1;
        portEXIT_CRITICAL_ISR(&muxZCVoltageFlag);
    }
}




// Process flags
void fnProcessZCVoltageFlag(){
    iCountZCVoltage++;
    if (iCountZCVoltage!=1){
        lElapsedTimeSetup += lTimeAtZCVoltage;
    }
    if (iCountZCVoltage == 31) { //30 periods elapsed, 60 halfPeriods
        lHalfPeriod = lElapsedTimeSetup /60;
        //TODO setup timer w/ halfperiod
        iSetupFlag = 0;
    }
    portENTER_CRITICAL(&muxZCVoltageFlag);
    iZCVoltageFlag = 0;
    portEXIT_CRITICAL(&muxZCVoltageFlag);
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
    attachInterrupt(digitalPinToInterrupt(pinZCVoltage), ISRZCVoltage, FALLING);




    while (1){

    if (iZCVoltageFlag) {fnProcessZCVoltageFlag();}

    }


    //esp_restart();
}
