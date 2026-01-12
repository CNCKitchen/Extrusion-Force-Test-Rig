#include "LoadCell.h"

//========== Konstruktor ==========//   
LoadCell::LoadCell(const uint8_t dataPin, const uint8_t clockPin): _dataPin(dataPin), _clockPin(clockPin) {
    _scale.begin(_dataPin, _clockPin);
}

//========== Öffentliche Funktions-Implementierungen  ==========//
double LoadCell::getMeanWheight(const uint8_t NUM_SAMPLES){
    static int64_t sumAdc = 0;
    static uint8_t count  = 0;

    static double lastWeight = 0.0;
    static bool hasMean = false;

    if (_scale.is_ready()) {
        long raw = _scale.read();
        sumAdc += raw;
        count++;

        if (count >= NUM_SAMPLES) {
            int64_t meanAdc = sumAdc / count;          
            lastWeight = calcWeight((long)meanAdc);
            hasMean = true;

            sumAdc = 0;
            count  = 0;
            return lastWeight;
        }
    }
    // Falls noch kein neuer Mittelwert fertig ist:
    return hasMean ? lastWeight : 0.0;
}

double LoadCell::getRawWheight(){
    static double lastWeight = 0.0;  // letzter gültiger Gewichtswert
    static bool hasRaw = false;      // gibt es schon einen gültigen Rohwert?

    if (_scale.is_ready()) { 
        long raw = _scale.read();    // HX711-Lib liefert long (signed)
        double weight = calcWeight(raw);

        lastWeight = weight;         
        hasRaw = true;

        return weight;               // neuer gültiger Messwert
    }
    // Hier landen wir, wenn gerade kein neuer Wert gelesen werden konnte
    if (hasRaw) {
        // letzten gültigen Wert zurückgeben
        return lastWeight;
    } else {
        // Ganz am Anfang: noch nie ein gültiger Wert -> Default
        return 0.0;
    }
}

float LoadCell::getForce(){
    return 9.81f * ( (float)getMeanWheight(2) / 1000.0f ); // Kraft in N 
}

bool LoadCell::tare(uint8_t numSamples) {
  int64_t sum = 0;
  uint8_t cnt = 0;

  // Wichtig: während tare() sollte kein anderer Task parallel _scale.read() machen!

  unsigned long t0 = millis();
  while (cnt < numSamples && (millis() - t0) < 1000) { // max 1s warten
    if (_scale.is_ready()) {
      sum += _scale.read();
      cnt++;
    }
    delay(1); // auf ESP32 ok (yield)
  }

  if (cnt == 0) return false;

  long meanAdc = (long)(sum / cnt);

  // Baseline aus aktueller Regression berechnen
  bool oldActive = _tareActive;
  _tareActive = false;                 // damit calcWeight "roh" ist
  _tareOffset_g = calcWeight(meanAdc); // Baseline in g
  _tareActive = oldActive;

  _tareActive = true;
  return true;
}

void LoadCell::clearTare() {
  _tareActive = false;
  _tareOffset_g = 0.0;
}

//========== Private Funktions-Implementierungen  ==========//

double LoadCell::calcWeight(long analogVal){
  double w = 0.009557 * analogVal - 1892.82; // g
  if (_tareActive) w -= _tareOffset_g;       // Tare abziehen
  return w;
}