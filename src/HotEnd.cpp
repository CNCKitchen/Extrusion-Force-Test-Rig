#include "HotEnd.h"

//========== Konstruktor ==========//   

HotEnd::HotEnd(const uint8_t gatePin, const uint8_t NTC_Pin, const uint8_t fanPin): _gatePin(gatePin), _NTCPin(NTC_Pin), _fanPin(fanPin){
    pinMode(_gatePin,OUTPUT);
    pinMode(_fanPin,OUTPUT);
    pinMode(_NTCPin,INPUT);
}

//========== Öffentliche Funktions-Implementierungen  ==========//

void HotEnd::setHeaterPwm(uint8_t pwmValue) {
    analogWrite(_gatePin, pwmValue);   
}
   
void HotEnd::setFanPwm(uint8_t pwmValue){
    analogWrite(_fanPin, pwmValue); 
}

float HotEnd::getTemperature() {
    double esp_voltage = getNtcVoltage();
    float v1; //unterer Stützwert, Spannung
    float v2; //oberer Stützwert, Spannung
    
    float t1; //unterer Stützwert, Temperatur
    float t2; //oberer Stützwert, Temperatur

    // Unterhalb/oberhalb des Tabellenbereichs clampen
    if (esp_voltage >= _ntcTable[0].voltageV) {
        //interpoliere mit Steigung zwischen 0ten und 1ten Element
        v1=_ntcTable[0].voltageV;
        v2=_ntcTable[1].voltageV;
        t1=_ntcTable[0].tempC;
        t2=_ntcTable[1].tempC;
    }
    else if (esp_voltage <= _ntcTable[_NTC_TABLE_SIZE - 1].voltageV) {
        //interpoliere mit Steigung zwischen vorletzten und letzten Element
        v1=_ntcTable[_NTC_TABLE_SIZE - 2].voltageV;
        v2=_ntcTable[_NTC_TABLE_SIZE - 1].voltageV;
        t1=_ntcTable[_NTC_TABLE_SIZE - 2].tempC;
        t2=_ntcTable[_NTC_TABLE_SIZE - 1].tempC;
    }
    else{
        bool noch_nicht_gefunden=true;
        for (size_t i = 0; i < _NTC_TABLE_SIZE - 1 &&noch_nicht_gefunden; ++i) {
            v1 = _ntcTable[i].voltageV;
            v2 = _ntcTable[i + 1].voltageV;

            if (esp_voltage <= v1 && esp_voltage >= v2) {
                t1 = _ntcTable[i].tempC;
                t2 = _ntcTable[i + 1].tempC;
                noch_nicht_gefunden=false;
            }
        }
    }

    //lineare Interpolation
    float k=(t1-t2)/(v1-v2);
    float d=t2;
    return k*(esp_voltage-v2)+d;
}

float HotEnd::getPower(uint8_t pwmVal){
    float D = static_cast<float>(pwmVal) / 255.0f;  // oder pwmVal / 255.0f;
    float power = D * _powerHeater;                 // z.B. _powerHeater = 40.0f;
    return power;
}

void HotEnd::pidController(float temp, float dt, const float setPoint){
    // Sicherheitsprüfungen
    if (temp > _T_MAX_SAFE) {
        Serial.println("Maximale Temperatur von 290°C überschritten! Heizer AUS.");
        _integralTerm   = 0.0f;
        _lastError      = 0.0f;
        _controlOutput  = 0.0f;
        setHeaterPwm(0);       
        return;
    }

    if(temp < 0){
        Serial.println("Minimale Temperatur von 0°C darf nicht unterschritten werden! Heizer AUS.");
        _integralTerm   = 0.0f;
        _lastError      = 0.0f;
        _controlOutput  = 0.0f;
        setHeaterPwm(0);
        return;
    }

    // Fehlerberechnung
    float error = setPoint - temp;

    // P-Anteil
    float pTerm = _Kp * error;

    // I-Anteil mit Anti-Windup
    _integralTerm += error * dt;
    
    // Anti-Windup: Integral nur begrenzen, wenn Ausgang nicht gesättigt ist
    float preOutput = pTerm + _Ki * _integralTerm;
    if (preOutput > _OUT_MAX) {
        _integralTerm = (_OUT_MAX - pTerm) / _Ki;
    } else if (preOutput < _OUT_MIN) {
        _integralTerm = (_OUT_MIN - pTerm) / _Ki;
    }
    float iTerm = _Ki * _integralTerm;

    // D-Anteil (mit Filterung über Fehleränderung)
    float dTerm = 0.0f;
    if (dt > 0.0001f) { // Vermeidung von Division durch Null
        float errorRate = (error - _lastError) / dt;
        dTerm = _Kd * errorRate;
    }
    _lastError = error;

    // PID-Ausgang
    float u = pTerm + iTerm + dTerm;

    // Stellgröße begrenzen
    if (u > _OUT_MAX) u = _OUT_MAX;
    if (u < _OUT_MIN) u = _OUT_MIN;
    _controlOutput = u;

    // PWM-Wert (0..255) schreiben
    uint8_t pwmValue = static_cast<uint8_t>(_controlOutput);
    setHeaterPwm(pwmValue);
    
    // Debug-Ausgabe
    Serial.print(">Temp:");
    Serial.println(temp);
    Serial.print(">Setpoint:");
    Serial.println(setPoint);
    Serial.print(">PWM:");
    Serial.println(pwmValue);
    Serial.print(">Power:");
    Serial.println(getPower(pwmValue));
}

bool HotEnd::autoTunePID(float targetTemp, float dt) {
    // Sicherheitsprüfungen
    if (targetTemp > _T_MAX_SAFE) {
        Serial.println("Zieltemperatur zu hoch für Auto-Tuning!");
        return false;
    }

    float temp = getTemperature();
    
    if (temp < 0 || temp > _T_MAX_SAFE) {
        Serial.println("Temperatur außerhalb sicherer Grenzen!");
        resetAutoTune();
        return false;
    }

    // Initialisierung beim ersten Aufruf
    if (!_isAutoTuning) {
        Serial.println("=== Auto-Tuning gestartet ===");
        Serial.print("Zieltemperatur: ");
        Serial.println(targetTemp);
        
        _isAutoTuning = true;
        _tuneSetpoint = targetTemp;
        _tuneStartTime = millis();
        _peakCount = 0;
        _relayState = true; // Start mit hoher Leistung
        _lastTemp = temp;
        _wasRising = false;
        
        // Reset PID state
        _integralTerm = 0.0f;
        _lastError = 0.0f;
        
        return false; // Noch nicht fertig
    }

    // Timeout-Check (max. 10 Minuten)
    if ((millis() - _tuneStartTime) > 600000) {
        Serial.println("Auto-Tuning Timeout!");
        resetAutoTune();
        return false;
    }

    // Relay-Kontrolle: Schalte zwischen hoher und niedriger Leistung
    if (_relayState) {
        // Hohe Leistung bis Zieltemperatur überschritten
        setHeaterPwm(static_cast<uint8_t>(_tuneOutputStep + 100));
        if (temp > _tuneSetpoint) {
            _relayState = false;
        }
    } else {
        // Niedrige/keine Leistung bis unter Zieltemperatur
        setHeaterPwm(50);
        if (temp < _tuneSetpoint) {
            _relayState = true;
        }
    }

    // Erkennung von Temperatur-Peaks (Maxima und Minima)
    bool isRising = (temp > _lastTemp);
    
    // Peak erkannt wenn Richtungswechsel
    if (_wasRising && !isRising && _peakCount < _MAX_PEAKS) {
        // Maximum erkannt
        _peakHigh[_peakCount] = _lastTemp;
        _peakTime[_peakCount] = millis();
        _peakCount++;
        
        Serial.print("Peak HIGH #");
        Serial.print(_peakCount);
        Serial.print(": ");
        Serial.println(_lastTemp);
    } else if (!_wasRising && isRising && _peakCount < _MAX_PEAKS) {
        // Minimum erkannt
        _peakLow[_peakCount] = _lastTemp;
        // Zeit wird bei HIGH gesetzt
    }
    
    _lastTemp = temp;
    _wasRising = isRising;

    // Wenn genug Oszillationen erfasst wurden, berechne PID-Parameter
    if (_peakCount >= 4) {
        Serial.println("=== Oszillationsdaten erfasst ===");
        
        // Berechne durchschnittliche Amplitude
        float amplitudeSum = 0.0f;
        uint8_t amplitudeCount = 0;
        for (uint8_t i = 1; i < _peakCount; i++) {
            if (_peakHigh[i] > _peakLow[i]) {
                amplitudeSum += (_peakHigh[i] - _peakLow[i]) / 2.0f;
                amplitudeCount++;
            }
        }
        float amplitude = (amplitudeCount > 0) ? (amplitudeSum / amplitudeCount) : 1.0f;
        
        // Berechne durchschnittliche Periode (Zeit zwischen Peaks)
        float periodSum = 0.0f;
        for (uint8_t i = 1; i < _peakCount - 1; i++) {
            periodSum += (_peakTime[i + 1] - _peakTime[i]);
        }
        float period_ms = periodSum / (_peakCount - 2);
        float period_s = period_ms / 1000.0f;
        
        // Ultimate Gain (Ku) und Ultimate Period (Tu)
        float Ku = (4.0f * _tuneOutputStep) / (3.14159f * amplitude);
        float Tu = period_s;
        
        Serial.print("Amplitude: ");
        Serial.println(amplitude);
        Serial.print("Periode (s): ");
        Serial.println(Tu);
        Serial.print("Ku: ");
        Serial.println(Ku);
        
        // Ziegler-Nichols PID-Regeln
        _Kp = 0.6f * Ku;
        _Ki = 1.2f * Ku / Tu;
        _Kd = 0.075f * Ku * Tu;
        
        Serial.println("=== Neue PID-Parameter ===");
        Serial.print("Kp: ");
        Serial.println(_Kp);
        Serial.print("Ki: ");
        Serial.println(_Ki);
        Serial.print("Kd: ");
        Serial.println(_Kd);
        Serial.println("=== Auto-Tuning abgeschlossen ===");
        
        _isAutoTuning = false;
        
        // Normalen Betrieb fortsetzen
        _integralTerm = 0.0f;
        _lastError = 0.0f;
        
        return true; // Tuning erfolgreich abgeschlossen
    }
    
    return false; // Noch nicht fertig
}

void HotEnd::resetAutoTune() {
    _isAutoTuning = false;
    _peakCount = 0;
    _integralTerm = 0.0f;
    _lastError = 0.0f;
    setHeaterPwm(0);
    Serial.println("Auto-Tuning zurückgesetzt");
}

void HotEnd::setPIDParameters(float kp, float ki, float kd) {
    _Kp = kp;
    _Ki = ki;
    _Kd = kd;
    
    // Reset integrator when changing parameters
    _integralTerm = 0.0f;
    _lastError = 0.0f;
    
    Serial.println("PID-Parameter gesetzt:");
    Serial.print("Kp: ");
    Serial.println(_Kp);
    Serial.print("Ki: ");
    Serial.println(_Ki);
    Serial.print("Kd: ");
    Serial.println(_Kd);
}

void HotEnd::getPIDParameters(float& kp, float& ki, float& kd) const {
    kp = _Kp;
    ki = _Ki;
    kd = _Kd;
}


//========== Private Funktions-Implementierungen  ==========//

//Tabelle mit Stützwerten, direkt {voltage [V], temperature[°C]}
const HotEnd::_NtcPoint HotEnd::_ntcTable[HotEnd::_NTC_TABLE_SIZE] = {
    {2.046000, 102.923943},
    {1.653000, 120.719604},
    {1.240000, 140.233643},
    {0.838000, 163.093643},
    {0.525000, 188.993500},
    {0.414000, 201.788239},
    {0.370000, 208.010605},
    {0.293000, 220.299622},
    {0.245000, 230.233643},
    {0.193000, 243.005783},
    {0.173000, 249.198715},
    {0.139000, 260.301727},
    {0.119000, 269.375000},
    {0.095000, 279.709290},
    {0.084000, 286.803040}
};

double HotEnd::getNtcVoltage() {
    double sum_voltage = 0.0;

    for (uint16_t i = 0; i < _SAMPLE_COUNT; ++i) {
        uint16_t val_raw = analogRead(_NTCPin);

        // ADC-Spannung laut ESP
        double U_out_esp = (static_cast<double>(val_raw) * _ADC_VREF) / _ADC_MAX;

        sum_voltage+=U_out_esp;
    }
    return sum_voltage / _SAMPLE_COUNT; // Mittelwert in Volt
}