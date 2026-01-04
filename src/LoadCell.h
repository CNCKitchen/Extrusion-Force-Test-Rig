#ifndef LoadCell_h
#define LoadCell_h
#include <Arduino.h>
#include "HX711.h"
#include <stdint.h>

class LoadCell{

    public:
         //========== Konstruktor ==========//
        LoadCell(const uint8_t dataPin, const uint8_t clockPin);

        //========== Funktions-Prototypen  ==========//
        double getMeanWheight(const uint8_t NUM_SAMPLES);
        double getRawWheight();
        float getForce();
        bool tare(uint8_t samples = 10);    
        void clearTare();                    

    private:

        //========== Funktions-Prototypen  ==========//
        double calcWeight(long analogVal);

        //========== Variablen  ==========//
        const uint8_t _dataPin;
        const uint8_t _clockPin;
        HX711 _scale;

        bool   _tareActive = false;
        double _tareOffset_g = 0.0;   // gespeicherter Nullpunkt in Gramm           

};

#endif