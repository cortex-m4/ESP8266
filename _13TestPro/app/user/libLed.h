#ifndef __LIB_LED_H__
#define __LIB_LED_H__

#define LED_WARM_RED        1
#define LED_WARM_GREEN      2
#define LED_WARM_BLUE       3
#define LED_WARM_CYAN       4
#define LED_WARM_PURPLE     5
#define LED_WARM_YELLOW     6
#define LED_WARM_ORANGE     7
#define LED_WARM_WHITE      8

#define LED_BLACK           0

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
