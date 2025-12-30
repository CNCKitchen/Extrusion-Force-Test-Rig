#include "GUICommunication.h"

//========== Konstruktor ==========//

GUICom::GUICom(){

};

//========== Öffentliche Funktions-Implementierungen  ==========//

bool GUICom::get_serial_input(uint32_t* temp_data, uint32_t* vorschub_data, uint32_t* abschalten){
    if(Serial.available()>0){ //falls input von GUI vorhanden
        String receivedData = Serial.readStringUntil('\n');
    
        mess_parameter_string meine_parameter;
        split_string(receivedData, &meine_parameter);
        
        *temp_data=string_to_int(meine_parameter.temp_string);
        *vorschub_data=string_to_int(meine_parameter.vorschub_string);
        *abschalten=string_to_int(meine_parameter.abschalten_string);
        return true;
    }else{
        return false;
    }
};

//========== Private Funktions-Implementierungen  ==========//

uint32_t GUICom::string_to_int(String text){
  int length=text.length();
  int factor=1;
  int val=0;
  for(int i=0; i<length; i++){
    val+=(text[length-1-i]-'0')*factor;
    factor*=10;
  }
  return val;
}


void GUICom::split_string(String raw_string, mess_parameter_string* pStruct){
  int index=raw_string.indexOf(' ');
  pStruct->temp_string=raw_string.substring(0,index);
  raw_string=raw_string.substring(index+1);

  index=raw_string.indexOf(' ');
  pStruct->vorschub_string=raw_string.substring(0,index);
  raw_string=raw_string.substring(index+1);

  index=raw_string.indexOf(' ');
  pStruct->abschalten_string=raw_string.substring(0,index);
}