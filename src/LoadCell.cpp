#include "LoadCell.h"

//========== Constructor ==========//   
LoadCell::LoadCell(const uint8_t dataPin, const uint8_t clockPin): _dataPin(dataPin), _clockPin(clockPin) {
    _scale.begin(_dataPin, _clockPin);
}

//========== Public Function Implementations  ==========//
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
    static double lastWeight = 0.0;  // last valid weight value
    static bool hasRaw = false;      // is there already a valid raw value?

    if (_scale.is_ready()) { 
        long raw = _scale.read();    // HX711 lib returns long (signed)
        double weight = calcWeight(raw);

        lastWeight = weight;         
        hasRaw = true;

        return weight;               // new valid measured value
    }
    // We end up here if no new value could be read at the moment
    if (hasRaw) {
        // return last valid value
        return lastWeight;
    } else {
        // At the very beginning: never a valid value yet -> Default
        return 0.0;
    }
}

float LoadCell::getForce(){
    return 9.81f * ( (float)getMeanWheight(2) / 1000.0f ); // Force in N 
}

bool LoadCell::tare(uint8_t numSamples) {
  int64_t sum = 0;
  uint8_t cnt = 0;

  // Important: during tare() no other task should do _scale.read() in parallel!

  unsigned long t0 = millis();
  while (cnt < numSamples && (millis() - t0) < 1000) { // max 1s wait
    if (_scale.is_ready()) {
      sum += _scale.read();
      cnt++;
    }
    delay(1); // ok on ESP32 (yield)
  }

  if (cnt == 0) return false;

  long meanAdc = (long)(sum / cnt);

  // Calculate baseline from current regression
  bool oldActive = _tareActive;
  _tareActive = false;                 // so calcWeight is "raw"
  _tareOffset_g = calcWeight(meanAdc); // Baseline in g
  _tareActive = oldActive;

  _tareActive = true;
  return true;
}

void LoadCell::clearTare() {
  _tareActive = false;
  _tareOffset_g = 0.0;
}

//========== Private Function Implementations  ==========//

double LoadCell::calcWeight(long analogVal){
  double w = 0.009557 * analogVal - 1892.82; // g
  if (_tareActive) w -= _tareOffset_g;       // Subtract tare
  return w;
}