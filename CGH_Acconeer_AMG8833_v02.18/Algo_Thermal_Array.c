/*
 * Algo_Thermal_Array.c
 *
 *  Created on: Dec 14, 2020
 *      Author: Roman
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

// SimpleLink includes
#include "simplelink.h"

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "utils.h"
#include "interrupt.h"
#include "adc.h"
#include "pin.h"

// common interface includes
#include "uart_if.h"
#include "uart.h"
#include "common.h"
#include "pinmux.h"
#include "i2c_if.h"
#include "hw_memmap.h"
#include "gpio.h"

#include "gpio_if.h"

// HTTP Client lib
#include <http/client/httpcli.h>
#include <http/client/common.h>


#define MAXTEMP 36.0
//#define EVERYTHING
#define STOVERWEIGHT

//Intercept = LogReg intercept, MAXTEMP acts as a cutoff and works to calculate deltas
//long double INTERCEPT = -80.86761318;
//long double COEFS[] = {3.70798620304983, -0.9909279577895282, -0.29446573476006294, -0.47535733655841617, -1.743256564262238, -2.270018940490561, -1.3614577395210572, -0.16999450472493485, 2.2538077489696, 0.21647093251127822, 0.7334064478583081, -0.43100941536401866, -2.923254095015214, -3.322876304049127, -2.635861255884273, 1.6102437626666433, 2.138917056826817, 0.1950754339923257, 0.06291173578497972, -0.3659952322481449, -1.0129668234237577, -0.9007056982387306, 0.15169699095263917, 1.8909560416493307, 1.8374089778091336, -0.7241630040516533, -0.6618189634084953, -1.9157158391935492, -0.8849642206344733, -1.0136730266200222, -0.48007418045779526, -0.3289364390781734, -0.46852638141174785, 0.008031081787522298, -1.7451713543519392, -2.152498962877684, -0.6073349249935394, -0.6296114645869864, -0.8884442268166084, -0.9419531633587428, -1.6984084451056678, -0.4377215157838058, -2.09568134604332, -2.741895331249335, -0.5898194599278813, 0.024282150290170567, -0.1554224806623687, -2.114797613053203, -1.6623556032612279, -0.06727792590257231, -1.8282166650648968, -1.7948102534275103, -0.1221976380260383, 0.8465847594417248, -0.7000254704309686, -1.7808869699155667, -0.3023787803038754, 0.0008396415883426886, -1.1508896488362004, -0.47312734943518714, 0.24250103963511963, 2.231718783638034, 1.8177799638907133, -0.8689204928629898, -0.1519084157399389};
float MINTEMP = 0.0;
float AVTEMP = 0.0;
float BASELINE[64];
float STDEV = 0.0;
float PIXELS[64];
float UPDATED[65];
float SENSITIVITYCOEFFICIENT = 0.45;
float BASESTANDEV = 0.0;
bool updThisPer = false;
bool overThisPer = false;
unsigned char sinceLastFrame = 0;
bool TRAVELING[] = {false,false,false,false,false,false};
bool FP[] = {true,false,false,false,false,false};
unsigned char tsize = 6;
//COEFS is the LogReg regression coefficients, BASELINE is updated later

//UTILITY/MATH FUNCS----------------


float calculateSD(float data[], unsigned int length) {
    float sum = 0.0, mean, SD = 0.0;
    int i;
    for (i = 0; i < length; ++i) {
        sum += data[i];
    }
    mean = sum / length;
    for (i = 0; i < length; ++i)
        SD += pow(data[i] - mean, 2);
    return sqrt(SD / length);
}

void convertListToPercentsForLogisticsUse(float *inputPixelList, unsigned int size){
    unsigned int i = 0;
    //float* percentageStorer = (float*)malloc(size * sizeof(float));
    for (i = 0; i < size; i++){
        PIXELS[i] = ((AVTEMP - inputPixelList[i])/inputPixelList[i]);
    }
    //The LogReg was built on percentage differences --> All must be converted to %
}
bool areEqual(bool arr1[], bool arr2[], int n, int m)
{
    int i;
    // If lengths of array are not equal means
    // array are not equal
    if (n != m)
        return false;


    // Linearly compare elements
    for (i = 0; i < n; i++){
        if (arr1[i] != arr2[i]){
           return false;
        }
    }



    // If all elements were same.
    return true;
}
bool shiftTravelingArrayAndCheck(bool result){
    unsigned char i;
    bool temp[] = {false,false,false,false,false,false};
    for (i = 0; i < (tsize - 1); i++){
        temp[i+1] = TRAVELING[i];
    }
    temp[0] = result;
    for (i = 0; i < tsize; i++){
        TRAVELING[i] = temp[i];
    }
    if (areEqual(TRAVELING,FP,6,6)){
        //DBG_PRINT("OVERRIDE");
        overThisPer = true;
        return true;
    }
    overThisPer = false;
    return false;
}

double average(float *list, unsigned int size){
    if (size > 0){
        double averageCounter = 0;
        unsigned int i;
        for (i = 0; i < size; i++){
            averageCounter+=list[i];
        }
        return averageCounter/size;
    }
    return 0;

}

void updateSensitivityCoefficient(){
    SENSITIVITYCOEFFICIENT = 0.45 + 0.05 * (AVTEMP - 22.0);
    if (SENSITIVITYCOEFFICIENT < 0.45){
        SENSITIVITYCOEFFICIENT = 0.45;
    }
}

void updateMinimumTemperature(float *listToDo, unsigned int size){
    MINTEMP = 100;
    unsigned int i;
    for (i = 0; i < size; i++){
        if (listToDo[i] < MINTEMP){
            MINTEMP = listToDo[i];
        }
    }
}



void addElementToBeginningOfFloatArray(float *listToAppendTo, float itemToAppend, unsigned int size){
    unsigned int newSize = size+1;
    //float newArr[newSize];
    //float* newArr = (float*)malloc(newSize * sizeof(float));
    //newArr[0] = itemToAppend;
    UPDATED[0] = itemToAppend;
    unsigned int i;
    for (i = 1; i < newSize; i++){
        UPDATED[i] = listToAppendTo[i-1];
        //newArr[i] = listToAppendTo[i-1];
    }
}

/*
void updateBaseline(bool result1, bool result2, bool result3, float probability, float *inputPixelList, unsigned int size){
    //This will update the baseline if there is a nearly 100% chance that no human is there
    if (!result1 && !result2 && !result3 && probability < 0.3){
        unsigned int i;
        for (i = 0; i < size; i++){
            BASELINE[i] = inputPixelList[i];
        }
    }
}

*/
unsigned int boolToInt(bool a){
    if (a){
        return 1;
    }
    return 0;
}

void updateBaseline(bool r1, bool r2, bool r3, float *inputPixelList, unsigned int size){
    if (boolToInt(r1) + boolToInt(r2) + boolToInt(r3) == 0){
        unsigned int i;
        for (i = 0; i < size; i++){
           BASELINE[i] = inputPixelList[i];
        }
        float temp = calculateSD(BASELINE,64);
        if (BASESTANDEV == 0){
            BASESTANDEV = temp;
        }
        else if (temp >= (BASESTANDEV - 0.05) && (temp <= BASESTANDEV + 0.05)){
            DBG_PRINT("Updated");
            BASESTANDEV = temp;
        }
    }
}

void updateBaselineSTOverweight(bool r1, float *inputPixelList, unsigned int size){
    if (!r1){
        unsigned int i;
        sinceLastFrame += 1;
        if (sinceLastFrame >= 10){
            for (i = 0; i < size; i++){
               BASELINE[i] = inputPixelList[i];
            }
        }
        float temp = calculateSD(BASELINE,64);
        //DBG_PRINT("TEMP:%f",temp);
        //DBG_PRINT("BASESTANDEV%f",BASESTANDEV);
        if (BASESTANDEV == 0){
            BASESTANDEV = temp;
            updThisPer = true;
        }
        else if ((temp >= (BASESTANDEV - 0.05) && (temp <= BASESTANDEV + 0.05) && sinceLastFrame >=10 && temp <= 0.7 && temp >= 0.3) || sinceLastFrame >= 50){
            //DBG_PRINT("Updated");
            updThisPer = true;
            BASESTANDEV = temp;
            sinceLastFrame = 0;
        } else {
            updThisPer = false;
        }
    } else {
        updThisPer = false;
    }
}

//-----------------------------------

//ALGOS------------------------------
bool medianTemperatureAlgo(float *inputPixelList, unsigned int size){
    float delta = (MAXTEMP - AVTEMP)/7.0 * SENSITIVITYCOEFFICIENT; //Calculate a dynamic delta; if AVTEMP = 20 --> Required observed change = 8/3 degrees C
    unsigned int count = 0;
    unsigned int i;
    for (i = 0; i<size; i++){
        if (inputPixelList[i] - delta > AVTEMP){
            count+=1;
        }
    }
    if (count > 5){ //If more than 3 pixels are present with a temp greater than AV + Delta --> Assume human
        return true;
    }
    return false;
}

bool standardDeviationAlgo(float stdev){
    if (BASESTANDEV != 0){
        if ((stdev >= (BASESTANDEV + 0.15) && stdev >= 0.65) || stdev >= 0.85){
            return true;
        }
    } else {
        if (stdev >= 0.8){
            return true;
        }
    }
    /*
    if (stdev >= 0.8){
        return true;
    }
    */
    return false;
}

bool minimumTemperatureAlgo(float *inputPixelList, unsigned int size){
    float delta = (MAXTEMP - MINTEMP)/2.5 * SENSITIVITYCOEFFICIENT; //Same idea as Avg Temperature, but with Mintemp of all pixels instead of Avtemp
    unsigned int count = 0;
    unsigned int i;
    for (i = 0; i < size; i++){
        if (inputPixelList[i] - MINTEMP > delta){
            count+=1;
        }
    }
    if (count > 6){
        return true;
    }
    return false;
}
bool countHotPixels(float *inputPixelList, unsigned int size){
    unsigned int i;
    unsigned int count = 0;
    for (i = 0; i < size; i++){
        if (inputPixelList[i] > 28.0){
            count+=1;
        }
    }
    if (count > 3){
        return true;
    }
    return false;
}

bool checkAgainstBaseline(float *inputPixelList, unsigned int size){
    if (BASELINE[0] != 0){ //Basically checks if the baseline was created, otherwise b[0] = 0
        float averageOfBaseline = average(BASELINE, 64);
        float delta = (MAXTEMP - averageOfBaseline)/8.0 * SENSITIVITYCOEFFICIENT; //DELTA required here is bigger than rest (/4 instead of /6) since baseline is 100% empty
        unsigned int count = 0;
        unsigned int i;
        for (i = 0; i < size; i++){
            if (BASELINE[i] < inputPixelList[i] - delta){
                count+=1;
            }
        }
        if (count > 6){
            return true;
        }
        return false;
    }
    return true; //Until baseline is built, return true
}
/*

float getProbability(float *inputPixelList, unsigned int size){
    long double numerator = INTERCEPT; // B0 in b0 + b1X1 + b2X2...
    unsigned int i = 0;
    for (i = 0; i < size; i++){
        numerator += COEFS[i] * inputPixelList[i];  // b1X1 + b2X2 + ...
    }
    return  (float) (1/(1+exp(-numerator))); //Logistic regression form; also is numerator/1+exp(numerator) which is why the variable is called numerator

}
*/
//-----------------------------------------------
//DECISION FUNCTIONS------
/*
bool noBaselineDecision(bool logRegRes, bool medTempRes, bool minTempRes, float probability){

    if (probability > 0.2 && probability < 0.8){ //If confidence is low
        if (logRegRes == medTempRes){ //Check for two equal results
            return logRegRes;
        }
        else{
            if (logRegRes == minTempRes){
                return logRegRes;
            }
            return medTempRes;
        }
    }
    return logRegRes; //If confidence is high return regression result

}
*/
bool noBaselineDecision(bool medTempRes, bool minTempRes, bool stdevres){

    if (boolToInt(medTempRes) || boolToInt(minTempRes) || boolToInt(stdevres)){
        return true;
    }
    return false;

}
/*
bool baselineWeightedDecision(bool r1, bool r2, bool r3, float probability){
    if (probability < 0.2 || probability > 0.8){ //If logreg is sure weigh it fully
        if ((probability * 0.33 + boolToInt(r1) * 0.33 + boolToInt(r2) * 0.33 + boolToInt(r3) * 0.33) > 0.5){
            return true;
        }
        return false;
    } else { //Otherwise, prob is weighted much less
        if ((probability * 0.15 + boolToInt(r1) * 0.39 + boolToInt(r2) * 0.39 + boolToInt(r3) * 0.39) > 0.5){
            return true;
        }
        return false;
    }
}
*/
bool baselineWeightedDecision(bool r1, bool r2, bool r3, bool r4){
   if ((boolToInt(r4) + boolToInt(r1)  + boolToInt(r2)  + boolToInt(r3)) >= 2){
        return true;
    }
    return false;
}

//---------
//OVERARCH

bool calculateHumanPresence(float *inputPixelList, unsigned int size){
    //unsigned char i;

    //for(i = 0; i < 64; i++)
        //DBG_PRINT("%.2f \n", inputPixelList[i]);
    AVTEMP = average(inputPixelList,size);
    updateSensitivityCoefficient();
    STDEV = calculateSD(inputPixelList,size);
    //DBG_PRINT("AVTEMP: %f_",AVTEMP);

    //DBG_PRINT("SENSCOEF: %f_",SENSITIVITYCOEFFICIENT);
    //DBG_PRINT("STDEV: %f_", STDEV);
    updateMinimumTemperature(inputPixelList,size);
    bool resultOne = medianTemperatureAlgo(inputPixelList,size);
    bool resultTwo = minimumTemperatureAlgo(inputPixelList,size);
    bool resultFive = standardDeviationAlgo(STDEV);
    //convertListToPercentsForLogisticsUse(inputPixelList,size);
    addElementToBeginningOfFloatArray(PIXELS,AVTEMP,size);
    /*

    float probability = getProbability(UPDATED,size + 1);
    //DBG_PRINT("%f",probability);
    //The above code first creates a list with AVTEMP at the beginning followed by the % differences from the extraction code, then gets the logreg
    bool resultThree;
    if (probability >= 0.5){
        resultThree = true;
    } else {
        resultThree = false;
    }
    */
    //bool resultThree = countHotPixels(inputPixelList,size);

    //updateBaseline(resultOne,resultTwo,resultThree,probability,inputPixelList,size);
#ifdef EVERYTHING
    updateBaseline(resultOne,resultTwo,resultFive,inputPixelList,size);
    if (BASELINE[0] != 0){
        bool resultFour = checkAgainstBaseline(inputPixelList,size);
        //bool finres = baselineWeightedDecision(resultOne,resultTwo,resultFour,probability);
        bool finres = baselineWeightedDecision(resultOne,resultTwo,resultFour,resultFive);
        if (shiftTravelingArrayAndCheck(finres)){
            finres = false;
        }

        if (resultOne){
            DBG_PRINT("MEDIANTEMP:HUMAN_");
        } else {
            DBG_PRINT("MEDIANTEMP:NOHUMAN_");
        }
        if (resultTwo){
           DBG_PRINT("MINTEMP:HUMAN_");
        } else {
           DBG_PRINT("MINTEMP:NOHUMAN_");
        }
        if (resultFour){
           DBG_PRINT("BASELINE:HUMAN_");
        } else {
           DBG_PRINT("BASELINE:NOHUMAN_");
        }
        if (resultFive){
           DBG_PRINT("STDEV:HUMAN_");
        } else {
           DBG_PRINT("STDEV:NOHUMAN_");
        }

  //      TRAVELING[travelingInteger] = finres;
    //    travelingInteger+=1;
        return finres;
    }
    else{
        //bool finres = noBaselineDecision(resultThree,resultOne,resultTwo,probability);
        bool finres = noBaselineDecision(resultOne,resultTwo,resultFive);
        if (shiftTravelingArrayAndCheck(finres)){
            finres = false;
        }

        if (resultOne){
            DBG_PRINT("MEDIANTEMP:HUMAN_");
        } else {
            DBG_PRINT("MEDIANTEMP:NOHUMAN_");
        }
        if (resultTwo){
           DBG_PRINT("MINTEMP:HUMAN_");
        } else {
           DBG_PRINT("MINTEMP:NOHUMAN_");
        }
        if (resultFive){
           DBG_PRINT("STDEV:HUMAN_");
        } else {
           DBG_PRINT("STDEV:NOHUMAN_");
        }


      //  TRAVELING[travelingInteger] = finres;
        //travelingInteger+=1;
        return finres;
    }
#endif
#ifdef STOVERWEIGHT
    //updateBaseline(resultOne,resultTwo,resultFive,inputPixelList,size);
    updateBaselineSTOverweight(resultFive,inputPixelList,size);
    if (resultFive || AVTEMP >= 29.5){
        //DBG_PRINT("R5");
        if (!shiftTravelingArrayAndCheck(resultFive)){
            //DBG_PRINT("%f,%f,%f,%d,%d,",AVTEMP,SENSITIVITYCOEFFICIENT,STDEV,boolToInt(updThisPer),boolToInt(overThisPer));
            return true;
        }
        //DBG_PRINT("%f,%f,%f,%d,%d,",AVTEMP,SENSITIVITYCOEFFICIENT,STDEV,boolToInt(updThisPer),boolToInt(overThisPer));
        return false;
    } else {
        if (BASELINE[0] != 0){
            bool resultFour = checkAgainstBaseline(inputPixelList,size);
            //bool finres = baselineWeightedDecision(resultOne,resultTwo,resultFour,probability);
            bool finres = baselineWeightedDecision(resultOne,resultTwo,resultFour,resultFive);
            if (shiftTravelingArrayAndCheck(finres)){
                finres = false;
            }
            //DBG_PRINT("%f,%f,%f,%d,%d,",AVTEMP,SENSITIVITYCOEFFICIENT,STDEV,boolToInt(updThisPer),boolToInt(overThisPer));
            /*
            if (resultOne){
                DBG_PRINT("MEDIANTEMP:HUMAN_");
            } else {
                DBG_PRINT("MEDIANTEMP:NOHUMAN_");
            }
            if (resultTwo){
               DBG_PRINT("MINTEMP:HUMAN_");
            } else {
               DBG_PRINT("MINTEMP:NOHUMAN_");
            }
            if (resultFour){
               DBG_PRINT("BASELINE:HUMAN_");
            } else {
               DBG_PRINT("BASELINE:NOHUMAN_");
            }
            if (resultFive){
               DBG_PRINT("STDEV:HUMAN_");
            } else {
               DBG_PRINT("STDEV:NOHUMAN_");
            }
            */

      //      TRAVELING[travelingInteger] = finres;
        //    travelingInteger+=1;
            return finres;
        }
        else{
            //bool finres = noBaselineDecision(resultThree,resultOne,resultTwo,probability);
            bool finres = noBaselineDecision(resultOne,resultTwo,resultFive);
            if (shiftTravelingArrayAndCheck(finres)){
                finres = false;
            }
            /*
            if (resultOne){
                DBG_PRINT("MEDIANTEMP:HUMAN_");
            } else {
                DBG_PRINT("MEDIANTEMP:NOHUMAN_");
            }
            if (resultTwo){
               DBG_PRINT("MINTEMP:HUMAN_");
            } else {
               DBG_PRINT("MINTEMP:NOHUMAN_");
            }
            if (resultFive){
               DBG_PRINT("STDEV:HUMAN_");
            } else {
               DBG_PRINT("STDEV:NOHUMAN_");
            }
            */

          //  TRAVELING[travelingInteger] = finres;
            //travelingInteger+=1;
            return finres;
        }
    }
#endif
#ifdef STANDEVONLY
    if (shiftTravelingArrayAndCheck(resultFive)){
       return false;
    }
    return resultFive;
#endif
}

