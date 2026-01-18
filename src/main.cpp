#include <Arduino.h>
#include <stdint.h>
#include "HotEnd.h"
#include "LoadCell.h"
#include "ExtruderStepper.h"
#include "Rotary_Encoder.h"
#include "GUICommunication.h"

/*
 * PID TEMPERATURE CONTROLLER
 * 
 * The system uses a PID controller with auto-tuning for temperature control.
 * 
 * Serial commands:
 * - AUTOTUNE:<temp>      - Starts auto-tuning for given temperature (e.g. AUTOTUNE:200)
 * - SETPID:<kp>,<ki>,<kd> - Sets PID parameters manually (e.g. SETPID:8.0,0.5,2.0)
 * - GETPID               - Shows current PID parameters
 * 
 * See PID_README.md for details.
 */

//========== Stepper ==========//

#define EN_PIN     11
#define STEP_PIN    10
#define DIR_PIN     9

// Parameters come from GUI
float feed_rate_per_s_in_mm;
float feed_length_in_mm;

// Measurement time in ms (calculated by GUI parameters)
unsigned long measureTimeMS = 0;

//========== Hot-End ==========//

#define NTC_PIN 4
#define HEATER_PIN 6
#define FAN_PIN 7
#define HEATER_DELAY 100

// Parameters come from GUI
float heater_temp_target = 180.; 
uint8_t hot_end_turn_off=0; 

//==========  Load Cell ==========//

#define LOADCELL_DOUT_PIN 8
#define LOADCELL_SCK_PIN 3
uint8_t tare=0; // 1 if should be zeroed


//==========  Rotary Encoder ==========//

#define ROTARY_ENCODER_PIN 5

//========== Objects ==========//
HotEnd myHotEnd(HEATER_PIN, NTC_PIN, FAN_PIN);
LoadCell myLoadCell(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
ExtruderStepper extruder(STEP_PIN, DIR_PIN, EN_PIN);
Encoder myEncoder(ROTARY_ENCODER_PIN);
GUICom my_GUI;

//========== Tasks ==========//
TaskHandle_t loadCellTaskHandle = NULL;
TaskHandle_t NTCTaskHandle = NULL;
TaskHandle_t stepperTaskHandle =  NULL;
TaskHandle_t hotEndTaskHandle = NULL;
TaskHandle_t serialTaskHandle = NULL;
TaskHandle_t RotEncoderTaskHandle = NULL;
TaskHandle_t ControllerTaskHandle = NULL;
TaskHandle_t TelemetryTaskHandle = NULL;

//========== Queues ==========//

// Queue for temperature values (NTC Task -> Hot End Task)
QueueHandle_t tempHotEndQueueHandle = NULL; 
#define TEMP_QUEUE_LENGTH 10
#define TEMP_QUEUE_SIZE sizeof(float)
// Queue for temperature values (NTC Task -> Telemetry Task) --> uses the same macros as tempHotEndQueue
QueueHandle_t tempTelemetryQueueHandle = NULL; 


// Queue for force values (LoadCell Task -> Telemetry Task)
QueueHandle_t ForceQueueHandle = NULL; 
#define FORCE_QUEUE_LENGTH 10
#define FORCE_QUEUE_SIZE sizeof(float)

// Queue for slip (RotEncoder Task -> Telemetry Task)
QueueHandle_t SlipTelemetryQueueHandle = NULL; 
#define SLIP_QUEUE_LENGTH 10
#define SLIP_QUEUE_SIZE sizeof(float)
// Queue for slip (RotEncoder Task -> Controller Task) --> uses the same macros as SlipTelemetryQueue
QueueHandle_t SlipControllerQueueHandle = NULL; 

//========== Controller Task ==========//

// Timestamp for Controller Task
unsigned long timeStamp = 0;

// Flag if defined temperature not yet reached
bool tempReached = false; 

// Flag that measurement is running
bool isMeasuring = false;

// States for measurement modes
enum MeasureMode: uint8_t{
  MODE_TIME = 0,
  MODE_MAX_FORCE = 1
};
volatile MeasureMode gMode = MODE_TIME;

// Stepper skip/slip detection
static constexpr float SLIP_TRIGGER_PERCENT = 5.0f;     // Slip in % at which it should be turned off
static constexpr uint32_t SLIP_HOLD_MS = 2000;            // Slip must occur for this long
static constexpr uint32_t MAX_FORCE_TIMEOUT_MS = 120000; // Safety: 2min max-force mode

bool heaterLockedOff = false;                   // in Max-Force mode after T_target = true

// ====================== Funktionen-Definitionen ======================//
// Task Funktionen
void loadCell_task(void* parameters);
void NTC_task(void* parameters);
void stepper_task(void* parameters);
void hotEnd_task(void* parameters);
void serial_task(void* parameters);
void rotEncoder_task(void* parameters);
void controller_task(void* parameters);
void Telemetry_Task(void* parameters);

// Helfer Funktionen 
void stopAllActuators(void);
bool computeMeasureTime();
void tareLoadCell();

void setup(){
  Serial.begin(115200);
  extruder.begin(3000, 3000);

  // Queues
  tempHotEndQueueHandle = xQueueCreate(TEMP_QUEUE_LENGTH, TEMP_QUEUE_SIZE);
  if(tempHotEndQueueHandle == NULL) Serial.println("Error creating temperature queue for hotend");

  tempTelemetryQueueHandle = xQueueCreate(TEMP_QUEUE_LENGTH, TEMP_QUEUE_SIZE);
  if(tempTelemetryQueueHandle == NULL) Serial.println("Error creating temperature queue for telemetry");

  ForceQueueHandle = xQueueCreate(FORCE_QUEUE_LENGTH, FORCE_QUEUE_SIZE);
  if(ForceQueueHandle == NULL) Serial.println("Error creating force queue for telemetry");

  SlipTelemetryQueueHandle = xQueueCreate(SLIP_QUEUE_LENGTH, SLIP_QUEUE_SIZE);
  if(SlipTelemetryQueueHandle == NULL) Serial.println("Error creating slip queue for telemetry");

  SlipControllerQueueHandle = xQueueCreate(SLIP_QUEUE_LENGTH, SLIP_QUEUE_SIZE);
  if(SlipControllerQueueHandle == NULL) Serial.println("Error creating slip queue for controller");

  // Tasks
  if (xTaskCreatePinnedToCore (loadCell_task, "Load Cell Task", 6144, nullptr, 1, &loadCellTaskHandle, 0) != pdPASS) {
    Serial.println("Error creating Load Cell Task");
  }
  vTaskSuspend(loadCellTaskHandle); // Pause task for now

  if (xTaskCreatePinnedToCore (NTC_task, "NTC Task", 6144, nullptr, 1, &NTCTaskHandle, 0) != pdPASS) {
    Serial.println("Error creating NTC Task");
  }
  vTaskSuspend(NTCTaskHandle); // Pause task for now

  if (xTaskCreatePinnedToCore (stepper_task, "Stepper Task", 6144, nullptr, 1, &stepperTaskHandle, 1) != pdPASS) {
    Serial.println("Error creating Stepper Task");
  }
  vTaskSuspend(stepperTaskHandle); // Pause task for now

  if (xTaskCreatePinnedToCore (hotEnd_task, "Hot End Task", 6144, nullptr, 1, &hotEndTaskHandle, 0) != pdPASS) {
    Serial.println("Error creating Hot End Task");
  }
  vTaskSuspend(hotEndTaskHandle); // Pause task for now

  if (xTaskCreatePinnedToCore (rotEncoder_task, "Rotary Encoder Task", 6144, nullptr, 1, &RotEncoderTaskHandle, 0) != pdPASS) {
    Serial.println("Error creating Rotary Encoder Task");
  }
  vTaskSuspend(RotEncoderTaskHandle); // Pause task for now

  if (xTaskCreatePinnedToCore (controller_task, "Controller Task", 6144, nullptr, 1, &ControllerTaskHandle, 0) != pdPASS) {
    Serial.println("Error creating Controller Task");
  }
  vTaskSuspend(ControllerTaskHandle); // Pause task for now

  if (xTaskCreatePinnedToCore (Telemetry_Task, "Telemetry Task", 6144, nullptr, 1, &TelemetryTaskHandle, 0) != pdPASS) {
    Serial.println("Error creating Telemetry Task");
  }
  vTaskSuspend(TelemetryTaskHandle);

  if (xTaskCreatePinnedToCore (serial_task, "Serial Task", 6144, nullptr, 1, &serialTaskHandle, 0) != pdPASS) {
    Serial.println("Error creating Serial Task");
  }
  
}

void loop(){

  
  vTaskDelay(pdMS_TO_TICKS(50));
}

// ====================== Function Implementations ======================//

// Task functions
void loadCell_task(void* parameters){
  for(;;){
    float force = myLoadCell.getForce();
    if(isMeasuring){
      float dummy;
      if(xQueueSend(ForceQueueHandle, (void*)&force, 0) != pdPASS){
        xQueueReceive(ForceQueueHandle, (void*)&dummy,0);
        xQueueSend(ForceQueueHandle, (void*)&force, 0);
      }   
    }     
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void NTC_task(void* parameters){
  for(;;){
    float temp = myHotEnd.getTemperature(); 
    float dummy;
    if(xQueueSend(tempHotEndQueueHandle, (void*)&temp, 0) != pdPASS){
        xQueueReceive(tempHotEndQueueHandle, (void*)&dummy,0);
        xQueueSend(tempHotEndQueueHandle, (void*)&temp, 0);
      }

      if(xQueueSend(tempTelemetryQueueHandle, (void*)&temp, 0) != pdPASS){
        xQueueReceive(tempTelemetryQueueHandle, (void*)&dummy,0);
        xQueueSend(tempTelemetryQueueHandle, (void*)&temp, 0);
      }    
    vTaskDelay(pdMS_TO_TICKS(100));           
  }
}

void stepper_task(void*){
  for(;;){
    extruder.runSpeed();
    taskYIELD();  // release very briefly without 1ms sleep
  }
}

void rotEncoder_task(void* parameters){
  for(;;){
    
    float ist = myEncoder.get_length();
    float soll = extruder.getExtrudedMmSinceStart();
    float dummy;
    if(isMeasuring){
       if (soll > 0.001f) {

        float slip = (1.0f - (ist / soll)) * 100.0f; // slip in %
        
        if(xQueueSend(SlipTelemetryQueueHandle, (void*)&schlupf, 0) != pdPASS){
          xQueueReceive(SlipTelemetryQueueHandle, (void*)&dummy,0);
          xQueueSend(SlipTelemetryQueueHandle, (void*)&schlupf, 0);
        }
        if(xQueueSend(SlipControllerQueueHandle, (void*)&schlupf, 0) != pdPASS){
          xQueueReceive(SlipControllerQueueHandle, (void*)&dummy,0);
          xQueueSend(SlipControllerQueueHandle, (void*)&schlupf, 0);
        }
      } 
      else {

        float schlupf_0 = 0;
        if(xQueueSend(SlipTelemetryQueueHandle, (void*)&schlupf_0, 0) != pdPASS){
        xQueueReceive(SlipTelemetryQueueHandle, (void*)&dummy,0);
        xQueueSend(SlipTelemetryQueueHandle, (void*)&schlupf_0, 0);
        }
        if(xQueueSend(SlipControllerQueueHandle, (void*)&schlupf_0, 0) != pdPASS){
        xQueueReceive(SlipControllerQueueHandle, (void*)&dummy,0);
        xQueueSend(SlipControllerQueueHandle, (void*)&schlupf_0, 0);
        }

      }
    } 
    vTaskDelay(pdMS_TO_TICKS(100));
  }
   
}

void hotEnd_task(void* parameters){
  myHotEnd.setFanPwm(180);
  const float dt_s = HEATER_DELAY / 1000.0f;  // Time step for PID controller in seconds
  
  for(;;){
    static float temp = 0;
    while(xQueueReceive(tempHotEndQueueHandle, &temp, 0) == pdPASS){
      // Empty queue up to the newest element
    }
    
    if (heaterLockedOff) {
      // In MAX_FORCE mode: heater completely off
      myHotEnd.setHeaterPwm(0);
      myHotEnd.setFanPwm(180);
    } 
    else{
      // Normal temperature control with PID controller
      myHotEnd.pidController(temp, dt_s, heater_temp_target);
      myHotEnd.setFanPwm(180);
    }

    // Check if target temperature has been reached (with tolerance band)
    if (!tempReached && temp >= (heater_temp_target - 2.0f) && temp <= (heater_temp_target + 2.0f)) {
      timeStamp = millis(); // Set timestamp when measurement begins
      tempReached = true;                 
      isMeasuring = true;

      if (gMode == MODE_MAX_FORCE) {
        heaterLockedOff = true;  // <- from now on the heater stays off!
        myHotEnd.setHeaterPwm(0);
      }

      // Stepper
      extruder.resetExtrudedMm();
      extruder.enable(true);
      extruder.setFilamentSpeedMmS(feed_rate_per_s_in_mm);
      vTaskResume(stepperTaskHandle);

      // Encoder
      if(!myEncoder.reset()) Serial.println("Fehler beim reseten des Encoders");
      if(!myEncoder.start_counter()) Serial.print("Fehler beim Encoderstart");

      vTaskResume(RotEncoderTaskHandle);
      vTaskResume(loadCellTaskHandle);
      vTaskResume(ControllerTaskHandle);
      Serial.println("begin"); // tells GUI that heating is finished and measurement begins
    }
    
    vTaskDelay(pdMS_TO_TICKS(HEATER_DELAY));
  }
}

void serial_task(void* parameters){
  for(;;){

    // Check for auto-tuning command (send "AUTOTUNE:<temperature>" via Serial)
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      if (input.startsWith("AUTOTUNE:")) {
        // Extract target temperature
        String tempStr = input.substring(9);
        float tuneTemp = tempStr.toFloat();
        
        if (tuneTemp > 50.0f && tuneTemp < 290.0f) {
          Serial.print("Starting auto-tuning for ");
          Serial.print(tuneTemp);
          Serial.println("°C...");
          Serial.println("IMPORTANT: Measurement must not be running!");
          
          // Ensure measurement is not running
          if (!isMeasuring && !tempReached) {
            heater_temp_target = tuneTemp;
            
            // Start auto-tuning process
            vTaskResume(NTCTaskHandle);
            vTaskResume(hotEndTaskHandle);
            
            // Wait for auto-tuning to complete
            const float dt_s = HEATER_DELAY / 1000.0f;
            while (!myHotEnd.autoTunePID(tuneTemp, dt_s)) {
              vTaskDelay(pdMS_TO_TICKS(HEATER_DELAY));
            }
            
            Serial.println("Auto-tuning completed!");
            
            // Get and display new parameters
            float kp, ki, kd;
            myHotEnd.getPIDParameters(kp, ki, kd);
            Serial.println("=== Optimized PID Parameters ===");
            Serial.print("Kp: ");
            Serial.println(kp, 4);
            Serial.print("Ki: ");
            Serial.println(ki, 4);
            Serial.print("Kd: ");
            Serial.println(kd, 4);
            
            vTaskSuspend(hotEndTaskHandle);
            vTaskSuspend(NTCTaskHandle);
          } else {
            Serial.println("ERROR: Cannot start auto-tuning - measurement already running!");
          }
        } else {
          Serial.println("ERROR: Invalid temperature (50-290°C)");
        }
        continue;
      }
      
      // Check for manual PID parameter setting (send "SETPID:<kp>,<ki>,<kd>")
      if (input.startsWith("SETPID:")) {
        String params = input.substring(7);
        int comma1 = params.indexOf(',');
        int comma2 = params.lastIndexOf(',');
        
        if (comma1 > 0 && comma2 > comma1) {
          float kp = params.substring(0, comma1).toFloat();
          float ki = params.substring(comma1 + 1, comma2).toFloat();
          float kd = params.substring(comma2 + 1).toFloat();
          
          myHotEnd.setPIDParameters(kp, ki, kd);
          Serial.println("PID parameters set manually");
        } else {
          Serial.println("ERROR: Format: SETPID:<kp>,<ki>,<kd>");
        }
        continue;
      }
      
      // Check for PID parameter query
      if (input == "GETPID") {
        float kp, ki, kd;
        myHotEnd.getPIDParameters(kp, ki, kd);
        Serial.println("=== Current PID Parameters ===");
        Serial.print("Kp: ");
        Serial.println(kp, 4);
        Serial.print("Ki: ");
        Serial.println(ki, 4);
        Serial.print("Kd: ");
        Serial.println(kd, 4);
        continue;
      }
    }

    bool gotCmd = my_GUI.get_serial_input(&heater_temp_target, &feed_rate_per_s_in_mm, &feed_length_in_mm, &hot_end_turn_off, &tare);

    // Always prioritize tare as soon as flag is set
    if (tare == 1) {
      tareLoadCell();     
      tare = 0;           // IMPORTANT: otherwise it will tare endlessly
    }

    if(gotCmd){

      // print data 
      Serial.println("Received data:");
      Serial.println("Temperature");
      Serial.println(heater_temp_target);
      Serial.println("FeedRate");
      Serial.println(feed_rate_per_s_in_mm);
      Serial.println("FeedLength");
      Serial.println(feed_length_in_mm);
      Serial.println("Turn off?");
      Serial.println(hot_end_turn_off);

      // Select mode based on GUI flag
      gMode = (hot_end_turn_off == 1) ? MODE_MAX_FORCE : MODE_TIME;


      // Messung vorbereiten
      tempReached = false;
      isMeasuring = false;
      heaterLockedOff = false;
      timeStamp = 0;              // wichtig: damit Controller nicht sofort stoppt

      xQueueReset(tempTelemetryQueueHandle);
      xQueueReset(ForceQueueHandle);
      xQueueReset(SlipTelemetryQueueHandle);
      xQueueReset(tempHotEndQueueHandle);        
      xQueueReset(SlipControllerQueueHandle);

      // Calculate time only for MODE_TIME
      if (gMode == MODE_TIME) {
        if (!computeMeasureTime()) {
          Serial.println("Error calculating measurement time");
          continue;
        }
      } 
      else {
        measureTimeMS = 0; // in Max-Force mode, not stopped by time
      }
      // Start heating
      vTaskResume(TelemetryTaskHandle);
      vTaskResume(NTCTaskHandle);
      vTaskResume(hotEndTaskHandle);
      vTaskSuspend(NULL);   // Pause serial task
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void controller_task(void* parameters){
  static uint32_t slipAboveSince = 0; // Time since exceeding SLIP_TRIGGER_PERCENT
  static uint32_t maxForceStart = 0; // Max Force Mode start time in ms

  for (;;) {

    // Safety start time for Max-Force mode
    if (isMeasuring && gMode == MODE_MAX_FORCE && maxForceStart == 0) {
      maxForceStart = millis();
      slipAboveSince = 0;
    }

    bool stopNow = false; // Flag to end measurement on timeout or too much slip

    if (isMeasuring && timeStamp != 0) {

      if (gMode == MODE_TIME) {
        if (measureTimeMS != 0 && (millis() - timeStamp >= measureTimeMS)) {
          stopNow = true;
        }
      } 
      else { // MODE_MAX_FORCE

        // Slip "newest" values fetch (empty queue to last)
        float slip = NAN;
        bool gotSlip = false;
        while (xQueueReceive(SlipControllerQueueHandle, &slip, 0) == pdPASS) {
          gotSlip = true;
        }

        if (gotSlip) {
          if (slip >= SLIP_TRIGGER_PERCENT) {
            if (slipAboveSince == 0) slipAboveSince = millis();
            if (millis() - slipAboveSince >= SLIP_HOLD_MS) stopNow = true;
          } 
          else {
            slipAboveSince = 0;
          }
        }

        // Safety Timeout
        if (!stopNow && maxForceStart != 0 && (millis() - maxForceStart >= MAX_FORCE_TIMEOUT_MS)) {
          Serial.println("Max-Force Timeout -> Stop");
          stopNow = true;
        }
      }
    }

    if (stopNow) {
      timeStamp = 0;
      measureTimeMS = 0;
      tempReached = false;
      isMeasuring = false;
      heaterLockedOff = false;
      maxForceStart = 0;
      slipAboveSince = 0;

      stopAllActuators();
      myEncoder.reset();
      Serial.println("end");

      vTaskSuspend(stepperTaskHandle);
      vTaskSuspend(hotEndTaskHandle);
      vTaskSuspend(loadCellTaskHandle);
      vTaskSuspend(NTCTaskHandle);
      vTaskSuspend(RotEncoderTaskHandle);
      vTaskSuspend(TelemetryTaskHandle);

      vTaskResume(serialTaskHandle);
      vTaskSuspend(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
  
void Telemetry_Task(void* parameters){
  for(;;){
    float temp = NAN, force = NAN, slip = NAN;
    unsigned long time = millis();
    while(xQueueReceive(tempTelemetryQueueHandle, &temp, 0) == pdPASS){
      // Empty queue up to newest element
    }
    while(xQueueReceive(ForceQueueHandle, &force, 0) == pdPASS){
      // Empty queue up to newest element
    }
    while(xQueueReceive(SlipTelemetryQueueHandle, &slip, 0) == pdPASS){
      // Empty queue up to newest element
    }

  
    Serial.print("f");
    Serial.println(force, 3);
    Serial.print("s");
    Serial.println(slip, 2);
    Serial.print("c");
    Serial.println(temp, 2);  
    Serial.print("t");
    Serial.println(time);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Helper functions
void stopAllActuators(void){
  myHotEnd.setFanPwm(0);
  myHotEnd.setHeaterPwm(0);
  extruder.stop();
  extruder.enable(false);
}

bool computeMeasureTime(){ 
  float t_s = feed_length_in_mm / feed_rate_per_s_in_mm;  // Seconds
  measureTimeMS = (unsigned long)(t_s * 1000.0f);    // Milliseconds
  Serial.print("MeasureTimeMS=");
  Serial.println(measureTimeMS);
  return true;
}

void tareLoadCell(){
  
  eTaskState st = eTaskGetState(loadCellTaskHandle);
  bool wasRunning = (st != eSuspended);
  if (wasRunning) vTaskSuspend(loadCellTaskHandle);

  if (!myLoadCell.tare(20)) {
    Serial.println("Tare failed (HX711 not ready?)");
  } else {
    Serial.println("Tare OK -> Force/Weight now relative to zero point");
  }

  if (wasRunning) vTaskResume(loadCellTaskHandle);

  tare = 0; 

}