#include "ets_sys.h"
#include "osapi.h"

#include "user_interface.h"

#include "gpio.h"

#include "mem.h"
#include "libLed.h"


#define LED_R_ON    {GPIO_OUTPUT_SET(GPIO_ID_PIN(12),1);}
#define LED_G_ON    {GPIO_OUTPUT_SET(GPIO_ID_PIN(13),1);}
#define LED_B_ON    {GPIO_OUTPUT_SET(GPIO_ID_PIN(14),1);}
#define LED_W_ON    {GPIO_OUTPUT_SET(GPIO_ID_PIN(15),1);}

#define LED_R_OFF    {GPIO_OUTPUT_SET(GPIO_ID_PIN(12),0);}
#define LED_G_OFF    {GPIO_OUTPUT_SET(GPIO_ID_PIN(13),0);}
#define LED_B_OFF    {GPIO_OUTPUT_SET(GPIO_ID_PIN(14),0);}
#define LED_W_OFF    {GPIO_OUTPUT_SET(GPIO_ID_PIN(15),0);}

LED userLed;

void Init_Led(void){
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO12);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO14);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO15);
}
int led_getLedWarm(void){
    if(userLed.warm == LED_WARM_RED){

    }
}
int led_getLedLight(void){
    return userLed.light;
}
int led_getMode(void){

}

int led_setLedWarm(int argc){

}
int led_setLedLight(int argc){

}
int led_setMode(int argc){

}
