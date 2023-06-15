#ifndef  __ESP8266_TEST_H
#define	 __ESP8266_TEST_H



#include "stm32f4xx.h"
/********************************** ͷ�ļ������� **********************************/

#include "esp8266.h"
#include <stdio.h>  
#include <string.h>  
#include <stdbool.h>
#include "usart.h"
/************************************* END *****************************************/


/********************************** �û���Ҫ���õĲ���**********************************/
#define      macUser_ESP8266_ApSsid                      "Yahboom2"        //Ҫ���ӵ��ȵ������
#define      macUser_ESP8266_ApPwd                       "yahboom890729"         //Ҫ���ӵ��ȵ����Կ

#define      macUser_ESP8266_TcpServer_IP                "192.168.2.101"    //Ҫ���ӵķ������� IP
#define      macUser_ESP8266_TcpServer_Port              "8086"              //Ҫ���ӵķ������Ķ˿�


/********************************** �ⲿȫ�ֱ��� ***************************************/
extern volatile uint8_t ucTcpClosedFlag;



/********************************** ���Ժ������� ***************************************/
void ESP8266_StaTcpClient_Unvarnish_ConfigTest(void);
//�˺����ŵ���ѭ����
void ESP8266_CheckRecvDataTest(void);

#endif

