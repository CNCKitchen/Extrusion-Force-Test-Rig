#ifndef HotEnd_h
#define HotEnd_h
#include <Arduino.h>
#include <stdint.h>

class HotEnd{

    public:
        //========== Konstruktor ==========//   
        HotEnd(const uint8_t gatePin, const uint8_t NTC_Pin, const uint8_t fanPin);

        //========== Funktions-Prototypen  ==========//

        // Heizer ansteuern (0..255)
        void setHeaterPwm(uint8_t pwmValue);

        // Lüfter ansteuern (0..255)
        void setFanPwm(uint8_t pwmValue);

        // NTC-Widerstand in Ohm (gemittelt)
        double getNtcVoltage(void);
        
        // Temperatur in °C (aus Tabelle + Interpolation)
        float getTemperature();

        // PI-Regelung für Hot-End
        void piController(float temp, float dt, const float setPoint);

    private:

        //========== Funktions-Prototypen  ==========//

        // aus Widerstand (kΩ) Temperatur interpolieren
        static float temperatureFromResistance(float rKOhm); 

        //========== Variablen  ==========//

        // Lüfter
        const uint8_t _fanPin;

        // Heizelement
        const uint8_t _gatePin;
        
        // Thermistor
        const uint8_t _NTCPin;
    
        static constexpr double _ADC_VREF   = 3.3;      // V
        static constexpr uint16_t _ADC_MAX  = 4095;     // 12-bit Auflösung

        static constexpr double _K_CORR = -0.04346;
        static constexpr double _D_CORR =  1.1333;
        static constexpr double _R_FIXED = 2500.0;      // Ohm, Festwiderstand
        
        static constexpr uint16_t _SAMPLE_COUNT = 10;
        static constexpr uint16_t _SAMPLE_DELAY_MS = 50;

        struct _NtcPoint {
            float voltageV;    // Volt
            float tempC;  // °C
        };

        static constexpr size_t _NTC_TABLE_SIZE = 15;
        static const _NtcPoint _ntcTable[_NTC_TABLE_SIZE];

        // Regler
        static constexpr float _OUT_MIN = 0.0f;
        static constexpr float _OUT_MAX = 255.0f;
        static constexpr float _T_MAX_SAFE = 290.0f;   // Sicherheitsgrenze

        static constexpr float _Kp = 9.3010f;
        static constexpr float _Ki = 0.9310f;

        float _integralTerm = 0.0f;        // integrierter I-Anteil
        float _controlOutput = 0.0f;       // Stellgröße (0..255)

};

#endif