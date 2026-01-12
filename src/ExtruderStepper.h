#ifndef ExtruderStepper_h
#define ExtruderStepper_h
#include <Arduino.h>
#include <AccelStepper.h>

class ExtruderStepper {

  public:

    //========== Konstruktor ==========//
    ExtruderStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin);


    //========== Funktions-Prototypen Public ==========//

    // maxSpeedStepsPerS = maximale Schrittfrequenz (Steps/s)
    void begin(float maxSpeedStepsPerS = 3000.0f, float accelStepsPerS2 = 3000.0f);

    // Konstantgeschwindigkeits Betrieb 
    void setFilamentSpeedMmS(float mm_s);   // Vorschub in mm/s (Filament)
    void runSpeed();                        // muss sehr oft aufgerufen werden
    void stop();                            // Geschwindigkeit auf 0

    void enable(bool on = true);

    float stepsPerMM() const { return _stepsPerMM; }
    float mmPerStep() const { return 1.0f / _stepsPerMM; }   // “mm/step” ist abgeleitet

      // Gesamt extrudierte Strecke seit Start (mm)
    float getExtrudedMmSinceStart() const;
    long getTimeMsSinceStart() const;
    void  resetExtrudedMm();

  private:

    //========== Variablen Privat ==========//

    // Hemera Datenblatt: nominal steps/mm bei 1/16 = 397
    static constexpr float _STEPS_PER_MM_X16 = 397.0f;

    // TMC2209 
    static constexpr int   _MICROSTEPPING = 8;

    static constexpr bool  _ENABLE_ACTIVE_LOW = true; 

    int32_t _lastStepPos = 0;     // letzter gemessener Stepper-Positionswert (Steps)
    int64_t _extrudedSteps = 0;   // aufsummierte Steps seit Start
    long _timeStamp = 0;

    uint8_t _stepPin, _dirPin, _enPin;
    float   _stepsPerMM;
    AccelStepper _stepper;
};

#endif