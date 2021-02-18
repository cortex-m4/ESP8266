#include "msgHandler.h"

//================event====================
 void eventLedColorWarm(int argc){
     os_printf("[event] user set color warm\n");
 }
 void eventLedLight(int argc){
     os_printf("[event] user set color light\n");
 }
 void eventLedMode(int argc){
     os_printf("[event] user set color mode\n");
 }
 void eventLedTimer(void* argc){
     os_printf("[event] user set color timer\n");
 }
//================msg basic================
 int userMsgRam[USERMASG_MAXLEN]={0};

 int userMsgAdd(int userMsg){
     int i;
     for(i=0;i<USERMASG_MAXLEN;i++){
         if(userMsgRam[i]==USERMSG_FREE){
             userMsgRam[i]=userMsg;
         }
         return 0;
     }
     return -1;
 }
 int userMsgGet(void){
     int temp;
     int i;
     for(i=0;i<USERMASG_MAXLEN;i++){
         if(userMsgRam[i]!=USERMSG_FREE){
             temp=userMsgRam[i];
             userMsgRam[i]=USERMSG_FREE;
             return temp;
         }
     }
     return USERMSG_FREE;
 }
 int userMsgInit(int userMsg){
     int i;
     for(i=0;i<USERMASG_MAXLEN;i++){
         userMsgRam[i]=USERMSG_FREE;
     }
     return 0;
 }

 void userHandler(void){
    int tempMsg=USERMSG_FREE;
    tempMsg=userMsg_get();
    switch(tempMsg){
        case USERMSG_USER_LED_COLOR_WARM:

        break;
        case USERMSG_USER_LED_COLOR_LIGHT:

        break;
        case USERMSG_USER_LED_MODE:

        break;
        case USERMSG_USER_LED_TIMER:

        break;
        
        default:break;
    }
 }
