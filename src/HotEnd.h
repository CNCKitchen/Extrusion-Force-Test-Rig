#ifndef HotEnd_h
#define HotEnd_h
#include <Arduino.h>
#include <stdint.h>

class HotEnd{

    public:
        //========== Konstruktor ==========//   
        HotEnd(const uint8_t gatePin, const uint8_t NTC_Pin, const uint8_t fanPin);

        //========== Funktions-Prototypen Public ==========//

        // Heizer ansteuern (0..255)
        void setHeaterPwm(uint8_t pwmValue);

        // Lüfter ansteuern (0..255)
        void setFanPwm(uint8_t pwmValue);
        
        // Temperatur in °C (aus Tabelle + Interpolation)
        float getTemperature();

        // Leistung in W am Hot End
        float getPower(uint8_t pwmVal);

        // PID-Regelung für Hot-End
        void pidController(float temp, float dt, const float setPoint);

        // Auto-Tuning der PID-Parameter (Ziegler-Nichols Relay Methode)
        bool autoTunePID(float targetTemp, float dt);

        // Auto-Tuning zurücksetzen
        void resetAutoTune();

        // Status des Auto-Tunings abfragen
        bool isAutoTuning() const { return _isAutoTuning; }

        // PID-Parameter manuell setzen
        void setPIDParameters(float kp, float ki, float kd);

        // Aktuelle PID-Parameter abrufen
        void getPIDParameters(float& kp, float& ki, float& kd) const;

    private:

        //========== Funktions-Prototypen Privat ==========//

        // aus Widerstand (kΩ) Temperatur interpolieren
        static float temperatureFromResistance(float rKOhm); 

        // NTC-Spannung in Volt (gemittelt)
        double getNtcVoltage(void);

        //========== Variablen Privat ==========//

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

        // PID Regler
        static constexpr float _OUT_MIN = 0.0f;
        static constexpr float _OUT_MAX = 255.0f;
        static constexpr float _T_MAX_SAFE = 290.0f;   // Sicherheitsgrenze

        // PID-Parameter (werden durch Auto-Tuning optimiert)
        float _Kp = 8.0f;
        float _Ki = 0.5f;
        float _Kd = 2.0f;

        float _integralTerm = 0.0f;        // integrierter I-Anteil
        float _lastError = 0.0f;           // vorheriger Fehler für D-Anteil
        float _controlOutput = 0.0f;       // Stellgröße (0..255)

        // Auto-Tuning Variablen (Relay-Methode)
        bool _isAutoTuning = false;
        float _tuneSetpoint = 0.0f;
        float _tuneOutputStep = 100.0f;    // Relay-Amplitude (PWM)
        unsigned long _tuneStartTime = 0;
        
        // Für Oszillationserkennung
        static constexpr uint8_t _MAX_PEAKS = 10;
        float _peakHigh[_MAX_PEAKS];
        float _peakLow[_MAX_PEAKS];
        unsigned long _peakTime[_MAX_PEAKS];
        uint8_t _peakCount = 0;
        bool _relayState = false;          // false = low, true = high
        float _lastTemp = 0.0f;
        bool _wasRising = false;

        // Heizelement
        static constexpr float _powerHeater = 40; // Leistung Hot End in Watt
};

#endif