#ifndef GUICommunication_h
#define GUICommunication_h
#include <Arduino.h>


class GUICom{
    
    public:

    //========== Konstruktor ==========// 
    GUICom(); //keine Pins zu übergeben

    //========== Funktions-Prototypen  ==========//

    //überprüft, ob es input von der GUI gibt und schreibt diesen ggf. in die Variablen
    bool get_serial_input(uint32_t* temp, uint32_t* vorschub, uint32_t* abschalten);


    private:

    typedef struct{
        String temp_string;
        String vorschub_string;
        String abschalten_string;
    }mess_parameter_string;


    //========== Funktions-Prototypen  ==========//

    //konvertiert Zahlen-String zu integer
    uint32_t string_to_int(String text);

    //spaltet empfangenen String in Teilstrings aus Zahlen auf
    void split_string(String raw_string, mess_parameter_string* pStruct);

};
#endif