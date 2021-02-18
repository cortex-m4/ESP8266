
#include "libLed.h"




LED userLed;

void Init_Led(void){
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO4);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO5);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO14);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO15);
}
int led_getLedWarm(void){

}
int led_getLedLight(void){

}
int led_getMode(void){

}

int led_setLedWarm(int argc){

}
int led_setLedLight(int argc){

}
int led_setMode(int argc){

}
