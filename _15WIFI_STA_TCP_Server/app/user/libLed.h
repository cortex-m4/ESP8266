#ifndef __LIB_LED_H__
#define __LIB_LED_H__


#include "ets_sys.h"
#include "osapi.h"

#include "user_interface.h"

#include "gpio.h"

#include "mem.h"

#define LED_WARM_RED        1
#define LED_WARM_GREEN      2
#define LED_WARM_BLUE       3
#define LED_WARM_CYAN       4
#define LED_WARM_PURPLE     5
#define LED_WARM_YELLOW     6
#define LED_WARM_ORANGE     7
#define LED_WARM_WHITE      8

#define LED_BLACK           0


#define LED_R_ON    {GPIO_OUTPUT_SET(GPIO_ID_PIN(4),1);}
#define LED_G_ON    {GPIO_OUTPUT_SET(GPIO_ID_PIN(5),1);}
#define LED_B_ON    {GPIO_OUTPUT_SET(GPIO_ID_PIN(14),1);}
#define LED_W_ON    {GPIO_OUTPUT_SET(GPIO_ID_PIN(15),1);}

#define LED_R_OFF    {GPIO_OUTPUT_SET(GPIO_ID_PIN(4),0);}
#define LED_G_OFF    {GPIO_OUTPUT_SET(GPIO_ID_PIN(5),0);}
#define LED_B_OFF    {GPIO_OUTPUT_SET(GPIO_ID_PIN(14),0);}
#define LED_W_OFF    {GPIO_OUTPUT_SET(GPIO_ID_PIN(15),0);}

typedef struct{
    int warm;
    int light;
    int mode;
}LED;

extern LED userLed;

void Init_Led(void);
int led_getLedWarm(void);
int led_getLedLight(void);
int led_getMode(void);

int led_setLedWarm(int argc);
int led_setLedLight(int argc);
int led_setMode(int argc);

#endif
