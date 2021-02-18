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

#include "cJSON.h"
#include "libLed.h"

#include "ip_addr.h"

//#define USE_STA_IP

#define FLASH_SEC				100				//扇区
#define FLASH_SEC_OFFSET		0				//扇区偏移位数，必须是4的倍数

#define LED_R_OFF		gpio16_output_set(0)
#define LED_R_ON		gpio16_output_set(1)

#define HARD_TIMER_USE  0			//硬件定时器函数使用
#define SOFT_TIMER_USE	1			//软件定时器函数使用
#define GPIO_INTR_USE	0			//GPIO中断的使用
#define TASK_USE_DEBUG	0			//TASK任务的使用测试
#define MY_FLASH_USE	0			//FLASH使用调试

#define ESP8266_AP_SSID		"muyuanZA"
#define ESP8266_AP_PASS		"muyuan2021"

#define ESP8266_STA_SSID	"Mi_11"
#define ESP8266_STA_PASS	"123456789"

#define MSM_TASK_LEN	2			//系统任务接受的数量

#define CMD_MODE_COLOR	"color"
#define CMD_MODE_VALUE	"value"

#define CMD_COLOR_RED	"red"
#define CMD_COLOR_GREEN	"green"
#define CMD_COLOR_BLUE	"blue"




uint8_t LED_STATE=1;				//led灯的值

os_timer_t OS_Timer_0;				//定义一个软件定时器的结构体变量
os_timer_t OS_Timer_1;				//定义一个软件定时器的结构体变量

os_event_t *event_task_1;			//定义一个任务事件测试一下

uint16_t msm_type=0;				//事件类型
char msm_par='A';					//事件参数

float HW_Timer_Counter=0;

unsigned char Red_PWM_Value=0;			//pwm 脉宽调制值
unsigned char Green_PWM_Value=0;		//pwm 脉宽调制值
unsigned char Blue_PWM_Value=0;			//pwm 脉宽调制值

struct espconn ST_NetCon;	//全局变量
							/*
								连接类型
								连接状态
								连接原型
								连接数量
								...
		
							*/
esp_udp ST_UDP;				//UDP通信结构体 远端端口 本地端口 本地ip 远端ip

struct ip_info ST_ESP8266_IP;		//IP信息结构体




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
/*
	网络数据成功发送
*/
void ESP8266_WIFI_Send_Cb(void *arg){
	os_printf("esp8266_wifi_send ok\n");
}
/*
	网络数据成功接收的回调函数
	arg:网络传输结构体指针
	pdata:网络传输数据指针
	len:数据长度
*/
void ESP8266_WIFI_Recv_Cb(void *arg, char *pdata, unsigned short len){
	struct espconn * T_arg=arg;		//缓存网络连接结构体指针
	remot_info *P_port_info=NULL;	//远端连接信息结构体指针


	//根据数据设置LED的亮灭
	/*
	if(pdata[0]=='R'||pdata[0]=='R'){
		LED_R_ON;
	}else if(pdata[0]=='g'||pdata[0]=='G'){
		LED_B_ON;
	}else if(pdata[0]=='B'||pdata[0]=='b'){
		LED_G_ON;
	}else if(pdata[0]=='X'||pdata[0]=='x'){
		LED_R_OFF;LED_G_OFF;LED_R_OFF;
	}
	*/
	cJSON *root;
	cJSON *item;
	char *cmdBuf;	//模式暂存
	char *colorBuf;	//color暂存

	cmdBuf=(char *)os_zalloc(6);
	colorBuf=(char *)os_zalloc(6);
	memset(cmdBuf,0,6);
	memset(colorBuf,0,6);
	root=(cJSON *)os_zalloc(sizeof(cJSON));
	item=(cJSON *)os_zalloc(sizeof(cJSON));
	root=cJSON_Parse(pdata);
	if(!root){
		os_printf("\nError Json!!\r\n");
		cJSON_Delete(root);
		os_free(cmdBuf);
		os_free(colorBuf);
	}else{
		item=cJSON_GetObjectItem(root,"cmd");	//解析指令
		memcpy(cmdBuf,item->valuestring,6);
		item=cJSON_GetObjectItem(root,"color");	//解析需要控制的颜色
		memcpy(colorBuf,item->valuestring,6);
		if(strcmp(cmdBuf,"color")==0){
			if(strcmp(colorBuf,CMD_COLOR_RED)==0){
				item=cJSON_GetObjectItem(root,"value");	//解析PWM值
				Red_PWM_Value=item->valueint;
			}
			if(strcmp(colorBuf,CMD_COLOR_BLUE)==0){
				item=cJSON_GetObjectItem(root,"value");	//解析PWM值
				//os_printf("\n[color]=blue\n");
				Blue_PWM_Value=item->valueint;
			}
			if(strcmp(colorBuf,CMD_COLOR_GREEN)==0){
				item=cJSON_GetObjectItem(root,"value");	//解析PWM值
				Green_PWM_Value=item->valueint;
			}

		}
		cJSON_Delete(item);
		
	}
	cJSON_Delete(root);

	os_printf("\nesp8266 rev data=%s\n",pdata);
	os_free(cmdBuf);
	os_free(colorBuf);

	/*
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
	*/
	//TCP是面向连接的，向远端主机回应可以直接使用T_arg结构体直接指向IP信息，无需调用espconn_get_connection_info
	//向对方发送应答
	espconn_send(T_arg,"ESP8266 WIFI Recv ok",os_strlen("ESP8266 WIFI Recv ok"));


	
}
/*
	ESP8266 STA 模式初始化
*/
void ICACHE_FLASH_ATTR 
ESP8266_STA_INIT(void){
	struct station_config STA_Config;	//STA参数结构体
	//struct ip_info ST_ESP8266_IP;		//STA信息结构体
	
	wifi_set_opmode(0x01);			//设置为sta模式，并且保存到flash
	/*
		0x01：Station 模式 
		0x02：SoftAP 模式 
		0x03：Station+SoftAP 模式
	*/
	os_memset(&STA_Config,0,sizeof(struct station_config));
								//AP参数结构体归0
	os_strcpy(STA_Config.ssid,ESP8266_STA_SSID);
								//设置SSID
	os_strcpy(STA_Config.password,ESP8266_STA_PASS);
								//设置密码
	wifi_station_set_config(&STA_Config);
								//设置sta 并且保存到FLASH
}

/*
	ESP8266 AP 模式初始化
*/
#if 0
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
#endif
/*
	TCP连接断开的回调
*/
void ICACHE_FLASH_ATTR 
ESP8266_TCP_Disconnect_Cb(void *arg){
	os_printf("\nESP8266 TPC Disconnect ok\n");
}


/*
	TCP连接建立成功的回调函数
*/
void ICACHE_FLASH_ATTR 
ESP8266_TCP_Connect_Cb(void *arg){
	//注册网络数据发送成功的回调函数
	espconn_regist_sentcb((struct espconn *)arg,ESP8266_WIFI_Send_Cb);
	//注册网络数据接收成功的回调函数
	espconn_regist_recvcb((struct espconn *)arg,ESP8266_WIFI_Recv_Cb);
	//注册成功断开TCP连接的回调函数
	espconn_regist_disconcb((struct espconn *)arg,ESP8266_TCP_Disconnect_Cb);

	os_printf("\nESP8266 TCP Connect ok\n");
}
/*
	TCP连接异常断开的回调函数
*/
void ICACHE_FLASH_ATTR 
ESP8266_TCP_Break_Cb(void *arg,sint8 err){
	os_printf("\nESP8266 TCP Break!\n");
}

/*
	初始化网络连接为TCP模式
*/
void ICACHE_FLASH_ATTR 
ESP8266_TCP_NetCon_init(void){
	ST_NetCon.type=ESPCONN_TCP;		//通讯协议选择TCP
	ST_NetCon.proto.tcp=(esp_tcp *)os_zalloc(sizeof(esp_tcp));
	//此处无需设置目标IP ESP8266作为Server 不需要知道Client的IP端口
	ST_NetCon.proto.tcp->local_port=8266;	//设置本地端口
	//ST_NetCon.proto.tcp->remote_port=888;	//目标端口
	//ST_NetCon.proto.tcp->remote_ip[0]=192;//目标ip
	//ST_NetCon.proto.tcp->remote_ip[1]=168;//目标ip
	//ST_NetCon.proto.tcp->remote_ip[2]=4;	//目标ip
	//ST_NetCon.proto.tcp->remote_ip[3]=2;	//目标ip

	//注册连接回调函数，异常断开回调
	espconn_regist_connectcb(&ST_NetCon,ESP8266_TCP_Connect_Cb);
	espconn_regist_reconcb(&ST_NetCon,ESP8266_TCP_Break_Cb);

	espconn_accept(&ST_NetCon);				//创建TCP_Server,建立监听

	espconn_regist_time(&ST_NetCon,300,0);	//设置超时时间，单位 s 最大7200 超时时间不能为0

}
/*
	初始化网络连接为UDP 模式
		Client模式 需要知道目标服务器的地址
	

*/
void ICACHE_FLASH_ATTR 
ESP8266_UDP_NetCon_init(void){
	ST_NetCon.type=ESPCONN_UDP; 	//通信协议选择UDP
	ST_NetCon.proto.udp=&ST_UDP;	
	/*
		申请内存
		也可以这样写：
		ST_NetCon.proto.udp=(esp_udp *)os_zalloc(sizeof(esp_udp));
		开辟内存
	*/
	ST_NetCon.proto.udp->local_port=114;		//设置本地端口
	ST_NetCon.proto.udp->remote_port=8080;		//设置目标服务器端口
	ST_NetCon.proto.udp->remote_ip[0]=192;		//设置目标IP地址
	ST_NetCon.proto.udp->remote_ip[1]=168;
	ST_NetCon.proto.udp->remote_ip[2]=4;
	ST_NetCon.proto.udp->remote_ip[3]=2;
	//u8 remote_ip[4]=[192,168,4,2];		
	//os_memcpy(ST_NetCon.proto.udp->remote_ip,remote_ip,4);		//拷贝内存
	espconn_regist_sentcb(&ST_NetCon,ESP8266_WIFI_Send_Cb);
	//网络发送成功回调函数
	espconn_regist_recvcb(&ST_NetCon,ESP8266_WIFI_Recv_Cb);
	//网络数据接收回调函数
	//espconn_sent_callback
	//espconn_recv_callback
	espconn_create(&ST_NetCon); 			//初始化UDP通信		
	espconn_send(&ST_NetCon,"Hello ,I am ESP8266",os_strlen("Hello ,I am ESP8266"));
											//出动向Server发起通信
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

#if HARD_TIMER_USE
/*
	硬件定时器中断回调函数
*/
void HARD_TIMER_CB(void){
	HW_Timer_Counter++;
	//PWM 1khz
	if(HW_Timer_Counter==100){
		HW_Timer_Counter=0;
	}
	//RED PWM CON
	if(HW_Timer_Counter>Red_PWM_Value){
		LED_R_OFF;
	}else if(HW_Timer_Counter<Red_PWM_Value){
		LED_R_ON;
	}
	//RED PWM CON
	if(HW_Timer_Counter>Green_PWM_Value){
		LED_G_OFF;
	}else if(HW_Timer_Counter<Green_PWM_Value){
		LED_G_ON;
	}
	//RED PWM CON
	if(HW_Timer_Counter>Blue_PWM_Value){
		LED_B_OFF;
	}else if(HW_Timer_Counter<Blue_PWM_Value){
		LED_B_ON;
	}
	//打印系统状态
}
/*
	硬件定时器初始化
	set_time: 设置硬件定时器的时间     单位us

	mod :	软件定时器的模式	0:一次模式
								1:重复模式
*/
void ICACHE_FLASH_ATTR 
HRAD_TIMER_INIT(uint32_t set_time,uint8_t mod){
	hw_timer_init(0,mod);		//初始化硬件定时器
	hw_timer_set_func(HARD_TIMER_CB);	//注册硬件定时器中断回调函数
	hw_timer_arm(set_time);
}

#endif

#if SOFT_TIMER_USE
/*
	软件定时器回调函数

*/
void SOFT_TIMER1_CB(void){
	float tempValue=0.0;
	//PWM 1khz
	HW_Timer_Counter+=1;
	if(HW_Timer_Counter==10){
		HW_Timer_Counter=0;
	}
	tempValue=(float)Red_PWM_Value/10;
	//RED PWM CON
	if(HW_Timer_Counter>tempValue){
		LED_R_OFF;
	}else if(HW_Timer_Counter<tempValue){
		LED_R_ON;
	}
	tempValue=(float)Green_PWM_Value/10;
	//RED PWM CON
	if(HW_Timer_Counter>tempValue){
		LED_G_OFF;
	}else if(HW_Timer_Counter<tempValue){
		LED_G_ON;
	}
	tempValue=(float)Blue_PWM_Value/10;
	//RED PWM CON
	if(HW_Timer_Counter>tempValue){
		LED_B_OFF;
	}else if(HW_Timer_Counter<tempValue){
		LED_B_ON;
	}
}
/*
	软件定时器初始化1
	set_time:	设置软件定时器的时间	单位 ms
	mod:		软件定时器的模式 			0：一次模式
									1：重复模式
*/
void ICACHE_FLASH_ATTR 
SOFT_TIMER_INIT_1(uint32_t set_time,uint8_t mod){
	os_timer_disarm(&OS_Timer_1);		//关闭定时器
	os_timer_setfn(&OS_Timer_1,(os_timer_func_t *)SOFT_TIMER1_CB,NULL);
	os_timer_arm(&OS_Timer_1,set_time,mod);
}
/*
	软件定时器回调函数

*/
void SOFT_TIMER0_CB(void){
	
	u8 ESP8266_IP[4];					//点分十进制形式保存IP
	/*
	wifi_get_ip_info(SOFTAP_IF,&ST_ESP8266_IP);
										//查询ESP8266的地址
	if(ST_ESP8266_IP.ip.addr!=0){
		ESP8266_IP[0]=ST_ESP8266_IP.ip.addr;
		ESP8266_IP[1]=ST_ESP8266_IP.ip.addr>>8;
		ESP8266_IP[2]=ST_ESP8266_IP.ip.addr>>16;
		ESP8266_IP[3]=ST_ESP8266_IP.ip.addr>>24;
		os_printf("ESP8266_IP=%d.%d.%d.%d\n",ESP8266_IP[0],ESP8266_IP[1],ESP8266_IP[2], ESP8266_IP[3]);
		//ESP8266_UDP_NetCon_init();
		//初始化UDP通信
		
	}
	*/
	//STA模式
	//判断是否获取IP
	if(wifi_station_get_connect_status()==STATION_GOT_IP){
		wifi_get_ip_info(SOFTAP_IF,&ST_ESP8266_IP);	//获取STA的IP信息
#ifdef USE_STA_IP
		wifi_station_dhcpc_stop();
		IP4_ADDR(&ST_ESP8266_IP.ip,192,168,235,57);
		IP4_ADDR(&ST_ESP8266_IP.gw,192,168,235,1);
		IP4_ADDR(&ST_ESP8266_IP.netmask,255,255,255,0);
		wifi_set_ip_info(STATION_IF,&ST_ESP8266_IP);
		//wifi_station_dhcpc_start();

#endif
		ESP8266_IP[0]=ST_ESP8266_IP.ip.addr;
		ESP8266_IP[1]=ST_ESP8266_IP.ip.addr>>8;
		ESP8266_IP[2]=ST_ESP8266_IP.ip.addr>>16;
		ESP8266_IP[3]=ST_ESP8266_IP.ip.addr>>24;
		//显示ESP8266 IP地址
		os_printf("ESP8266 IP =%d.%d.%d.%d\n",ESP8266_IP[0],ESP8266_IP[1],ESP8266_IP[2],ESP8266_IP[3]);
	}
	os_timer_disarm(&OS_Timer_0);
	//关闭定时器
	ESP8266_TCP_NetCon_init();
	//初始化TCP通讯
	//HRAD_TIMER_INIT(100,1);
	SOFT_TIMER_INIT_1(1,1);
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

	Init_Led();							//初始化LED
	//os_printf("okokokoko\n");							
	//ESP8266_AP_INIT();					//初始化wifi
	ESP8266_STA_INIT();
	
	LED_R_OFF;
	LED_G_OFF;
	LED_B_OFF;
	SOFT_TIMER_INIT(3000,0);
	
	
}


