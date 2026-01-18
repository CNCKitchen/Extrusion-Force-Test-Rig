#include "GUICommunication.h"

//========== Constructor ==========//

GUICom::GUICom(){

};

//========== Public Function Implementations  ==========//

bool GUICom::get_serial_input(float* temp_data, float* feedrate, float* feedlength, uint8_t* shut_off, uint8_t* tare){
    if(Serial.available()>0){ // if input from GUI available
        String receivedData = Serial.readStringUntil('\n');
        if(receivedData=="tare"){
          *tare=1;
          return false; // tare does not start measurement
        }else{
          measurement_parameter_string my_parameters;
          split_string(receivedData, &meine_parameter);
          
          *temp_data=string_to_float(meine_parameter.temp_string);
          *feedrate=string_to_float(meine_parameter.feedrate_string);
          *feedlength=string_to_float(meine_parameter.feedlength_string);
          *shut_off=(uint8_t)string_to_float(meine_parameter.abschalten_string);
          return true;
        }
    }else{
        return false;
    }
};

//========== Private Function Implementations  ==========//

float GUICom::string_to_float(String text){
  int length=text.length();
  int point_pos=1;
  while(point_pos<length && text[point_pos]!='.'){
    point_pos++;
  }
  String before_decimal=text.substring(0,point_pos);
  int factor=1;
  float val=0.;
  for(int i=0; i<before_decimal.length(); i++){
    val+=(before_decimal[before_decimal.length()-i-1]-'0')*factor;
    factor*=10;
  }

  if(point_pos<length){
    String after_decimal=text.substring(point_pos+1);
    float factor=10.;
    for(int i=0; i<after_decimal.length(); i++){
      val+=(after_decimal[i]-'0')/factor;
      factor*=10.;
    }
  }
  return val;
}


void GUICom::split_string(String raw_string, measurement_parameter_string* pStruct){
  int index=raw_string.indexOf(' ');
  pStruct->temp_string=raw_string.substring(0,index);
  raw_string=raw_string.substring(index+1);

  index=raw_string.indexOf(' ');
  pStruct->feedrate_string=raw_string.substring(0,index);
  raw_string=raw_string.substring(index+1);

  index=raw_string.indexOf(' ');
  pStruct->feedlength_string=raw_string.substring(0,index);
  raw_string=raw_string.substring(index+1);

  index=raw_string.indexOf(' ');
  pStruct->turn_off_string=raw_string.substring(0,index);
  
}