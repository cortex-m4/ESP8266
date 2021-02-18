#ifndef __MSGHANDLER_H__
#define __MSGHANDLER_H__

#define USERMASG_MAXLEN     8

#define USERMSG_FREE        0
#define USERMSG_SYS         100
#define USERMSG_SYS_INIT    (USERMSG_SYS+1)

#define USERMSG_USER        1000
#define USERMSG_USER_LED_COLOR_WARM         (USERMSG_USER+1)
#define USERMSG_USER_LED_COLOR_LIGHT        (USERMSG_USER+2)
#define USERMSG_USER_LED_MODE               (USERMSG_USER+3)
#define USERMSG_USER_LED_TIMER              (USERMSG_USER+4)


int userMsgRam[USERMASG_MAXLEN];

//================event====================
  void eventLedColorWarm(int argc);
  void eventLedLight(int argc);
  void eventLedMode(int argc);
  void eventLedTimer(void* argc);
//================msg basic================

  int userMsgAdd(int userMsg);
  int userMsgGet(void);
  int userMsgInit(int userMsg);

  void userHandler(void);



#endif

