/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ets_sys.h"
#include "osapi.h"

#include "user_interface.h"

#include "gpio.h"

#include "driver/gpio16.h"

#include "driver/uart.h"


os_timer_t OS_Timer_0;		//定义一个软件定时器的结构体变量

uint8_t LED_STATE=1;

//if GPIO4 IS A LED

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR
user_rf_pre_init(void)
{
}
//===============================USER_CODE_START======================================

/*
	gpio中断回调函数
*/
void GPIO_EXIT(void){
	u32 gpio_status;
	u32 F_GPIO_0_INT;

	gpio_status=GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS,gpio_status);
	//清除中断状态

	F_GPIO_0_INT=gpio_status&(0x01<<0);
	//获取GPIO 0中断状态

	//判断是否来下降沿中断
	if(F_GPIO_0_INT){
		LED_STATE=!LED_STATE;
		gpio16_output_set(LED_STATE);
		os_printf("Enter GPIO IT!\n");
	}

}
/*
	软件定时器回调函数

*/
void SOFT_TIMER0_CB(void){

	LED_STATE=!LED_STATE;			//LED灯翻转
	gpio16_output_set(LED_STATE);	//设置LED灯
	os_printf("\r\n====Enter_SOFT_Timer_CB=========\r\n");
									//打印系统状态
}
/*
	硬件定时器中断回调函数
*/
void HARD_TIMER_CB(void){

	LED_STATE=!LED_STATE;			//LED灯翻转
	gpio16_output_set(LED_STATE);	//设置LED灯
	os_printf("\r\n====Enter_HARD_Timer_CB=========\r\n");
									//打印系统状态
}

/*
	GPIO中断使能
*/
void ICACHE_FLASH_ATTR 
GPIO_IT_INIT(void){
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U,FUNC_GPIO0);	//GPIO作为IO口
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(0));					//GPIO0失能输出
	ETS_GPIO_INTR_DISABLE();							//关闭中断使能
	ETS_GPIO_INTR_ATTACH(GPIO_EXIT,NULL);				//注册中断回调函数
	gpio_pin_intr_state_set(GPIO_ID_PIN(0),GPIO_PIN_INTR_NEGEDGE);
														//设置下降沿中断
	ETS_GPIO_INTR_ENABLE();								//开启中断
}


/*
	软件定时器初始化
	set_time:	设置软件定时器的时间	单位 ms
	mod:		软件定时器的模式 			0：一次模式
									1：重复模式
*/
void ICACHE_FLASH_ATTR 
SOFT_TIMER_INIT(uint32_t set_time,uint8_t mod){
	os_timer_disarm(&OS_Timer_0);		//关闭定时器
	os_timer_setfn(&OS_Timer_0,(os_timer_func_t *)SOFT_TIMER0_CB,NULL);
	os_timer_arm(&OS_Timer_0,set_time,mod);
}


/*
	硬件定时器初始化
	set_time: 设置硬件定时器的时间     单位us

	mod :		软件定时器的模式		0:一次模式
								1:重复模式
*/
void ICACHE_FLASH_ATTR 
HRAD_TIMER_INIT(uint32_t set_time,uint8_t mod){
	hw_timer_init(0,mod);		//初始化硬件定时器
	hw_timer_set_func(HARD_TIMER_CB);	//注册硬件定时器中断回调函数
	hw_timer_arm(set_time);
}


/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_init(void) 
{
	uart_init(9600,9600);
	HRAD_TIMER_INIT(1000000,1);			//初始化软件定时器		 1s		
	gpio16_output_conf();				//初始化GPIO16

	while(1) system_soft_wdt_feed();		//喂狗

}


