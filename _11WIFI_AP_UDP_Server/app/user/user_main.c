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

#include "mem.h"

#include "espconn.h"

#include "driver/gpio16.h"

#include "driver/uart.h"

#define FLASH_SEC				100				//扇区
#define FLASH_SEC_OFFSET		0				//扇区偏移位数，必须是4的倍数

#define LED_ON			gpio16_output_set(0)
#define LED_OFF			gpio16_output_set(1)

#define HARD_TIMER_USE  0			//硬件定时器函数使用
#define SOFT_TIMER_USE	0			//软件定时器函数使用
#define GPIO_INTR_USE	0			//GPIO中断的使用
#define TASK_USE_DEBUG	0			//TASK任务的使用测试
#define MY_FLASH_USE	0			//FLASH使用调试

#define ESP8266_AP_SSID		"MY_ESP8266"
#define ESP8266_AP_PASS		"123456789"

#define MSM_TASK_LEN	2			//系统任务接受的数量


uint8_t LED_STATE=1;				//led灯的值

os_timer_t OS_Timer_0;				//定义一个软件定时器的结构体变量

os_event_t *event_task_1;			//定义一个任务事件测试一下

uint16_t msm_type=0;				//事件类型
char msm_par='A';					//事件参数

struct espconn ST_NetCon;	//全局变量
							/*
								连接类型
								连接状态
								连接原型
								连接数量
								...
		
							*/
esp_udp ST_UDP;				//UDP通信结构体 远端端口 本地端口 本地ip 远端ip




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
	ms延时函数
*/
void ICACHE_FLASH_ATTR 
os_delay_ms(uint32_t n){
	uint32_t i;
	for(i=n;i>0;i--){
		os_delay_us(1000);
	}
}

void ESP8266_WIFI_Send_Cb(void *arg){
	os_printf("esp8266_wifi_send ok\n");
}

void ESP8266_WIFI_Recv_Cb(void *arg, char *pdata, unsigned short len){
	struct espconn * T_arg=arg;		//缓存网络连接结构体指针
	remot_info *P_port_info=NULL;	//远端连接信息结构体指针


	//根据数据设置LED的亮灭
	if(pdata[0]=='k')LED_ON;
	else if(pdata[0]=='g')LED_OFF;

	os_printf("\esp8266 rev data=%s\n",pdata);

	//获取远端信息，获取是谁发的
	if(espconn_get_connection_info(T_arg,&P_port_info,0)==ESPCONN_OK){
		
		T_arg->proto.udp->remote_port=P_port_info->remote_port;
		T_arg->proto.udp->remote_port=P_port_info->remote_ip[0];
		T_arg->proto.udp->remote_port=P_port_info->remote_ip[1];
		T_arg->proto.udp->remote_port=P_port_info->remote_ip[2];
		T_arg->proto.udp->remote_port=P_port_info->remote_ip[3];
		
	espconn_send(T_arg,"ESP8266_WIFI_Recv_OK",os_strlen("ESP8266_WIFI_Recv_OK"));
	//发送数据
	}

	
}



void ICACHE_FLASH_ATTR 
ESP8266_AP_INIT(void){

	struct softap_config AP_Config;	//AP参数结构体
	wifi_set_opmode(0x02);			//设置为AP模式，并且保存到flash
	/*
		0x01：Station 模式 
		0x02：SoftAP 模式 
		0x03：Station+SoftAP 模式
	*/
	os_memset(&AP_Config,0,sizeof(struct softap_config));
								//AP参数结构体归0
	os_strcpy(AP_Config.ssid,ESP8266_AP_SSID);
								//设置SSID
	os_strcpy(AP_Config.password,ESP8266_AP_PASS);
								//设置密码
	AP_Config.ssid_len=os_strlen(ESP8266_AP_SSID);
								//设置SSID长度
	AP_Config.channel=1;		//设置通道号
	AP_Config.authmode=AUTH_WPA2_PSK;
								//设置加密模式
	AP_Config.ssid_hidden=0;	//不隐藏SSID
	AP_Config.max_connection=4;	//设置最大连接数
	AP_Config.beacon_interval=100;	
								//信标间隔时槽100~60000ms
	wifi_softap_set_config(&AP_Config);
								//设置soft-AP 并且保存到FLASH
}
void ICACHE_FLASH_ATTR 
ESP8266_UDP_NetCon_init(void){
	ST_NetCon.type=ESPCONN_UDP;		//通信协议选择UDP
	ST_NetCon.proto.udp=&ST_UDP;	//申请内存

	ST_NetCon.proto.udp->local_port=114;
									//设置本地端口
	espconn_regist_sentcb(&ST_NetCon,ESP8266_WIFI_Send_Cb);
	espconn_regist_recvcb(&ST_NetCon,ESP8266_WIFI_Recv_Cb);
		//espconn_sent_callback
		//espconn_recv_callback
	espconn_create(&ST_NetCon);				//初始化UDP通信		
}


#if MY_FLASH_USE
/*
	简单的Flash储存函数
	addr	扇区号
	data 	要存储的数据
	len	 	数据的长度 数组长度

*/
void ICACHE_FLASH_ATTR 
FLASH_SAVE_DATA(uint32_t addr,uint32_t *data,uint16_t len){

	spi_flash_erase_sector(addr);			//擦除扇区
	
	spi_flash_write(addr*4*1024+FLASH_SEC_OFFSET,data,len*4);			//写入数据
	
}
/*
	简单的Flash读取函数
	addr 扇区号
	data 读出的数据
	len	 数据的长度

*/
void ICACHE_FLASH_ATTR 
FLASH_READ_DATA(uint32_t addr,uint32_t *data,uint16_t len){
	
	spi_flash_read(addr*4*1024+FLASH_SEC_OFFSET,data,len*4);
		
}

#endif


#if TASK_USE_DEBUG
/*
	任务执行函数测试
	
*/
void func_task_1(os_event_t *task_msm){
	os_printf("msm type=%d,msm par=%c\r\n", task_msm->sig, task_msm->par);
}
/*
	任务初始化函数
	
*/

void ICACHE_FLASH_ATTR 
TASK_TEST_INIT(void){
	event_task_1=(os_event_t *)os_malloc((sizeof(os_event_t))* MSM_TASK_LEN);
																	//计算内存并且为任务分配内存
	system_os_task(func_task_1,USER_TASK_PRIO_0, event_task_1,MSM_TASK_LEN);		
																	//创建任务 注册函数，分配优先级
		//func_task_1 		任务执行函数
		//USER_TASK_PRIO_0	优先级
		//event_task		任务指针
		//MSM_TASK_LEN		深度
		
}

#endif

#if GPIO_INTR_USE

/*
	gpio中断回调函数
*/
void GPIO_EXIT_CB(void){
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
	GPIO中断使能
*/
void ICACHE_FLASH_ATTR 
GPIO_IT_INIT(void){
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U,FUNC_GPIO0);	//GPIO作为IO口
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(0));					//GPIO0失能输出
	ETS_GPIO_INTR_DISABLE();							//关闭中断使能
	ETS_GPIO_INTR_ATTACH(GPIO_EXIT_CB,NULL);			//注册中断回调函数
	gpio_pin_intr_state_set(GPIO_ID_PIN(0),GPIO_PIN_INTR_NEGEDGE);
														//设置下降沿中断
	ETS_GPIO_INTR_ENABLE();								//开启中断
}
#endif


#if SOFT_TIMER_USE
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


#endif

#if HARD_TIMER_USE
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

#endif



/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_init(void) 
{

	uint8_t i;
	uart_init(9600,9600);
	gpio16_output_conf();				//初始化GPIO16
	os_printf("\n");							
	ESP8266_AP_INIT();					//初始化wifi
	ESP8266_UDP_NetCon_init();			//初始化UDP连接
	
}


