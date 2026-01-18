#ifndef GUICommunication_h
#define GUICommunication_h
#include <Arduino.h>


class GUICom{
    
    public:

        //========== Constructor ==========// 
        GUICom(); // no pins to pass

        //========== Public Function Prototypes ==========//

        // checks if there is input from GUI and writes it to variables if applicable
        bool get_serial_input(float* temp, float* feedrate, float* feedlength, uint8_t* shut_off, uint8_t* tare);


    private:

        typedef struct{
            String temp_string;
            String feedrate_string;
            String feedlength_string;
            String turn_off_string;
        }measurement_parameter_string;


        //========== Private Function Prototypes ==========//

        // converts number string to float
        float string_to_float(String text);

        // splits received string into substrings of numbers
        void split_string(String raw_string, measurement_parameter_string* pStruct);

};
#endif