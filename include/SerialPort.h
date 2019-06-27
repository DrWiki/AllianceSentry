#ifndef ALLIANCE_SENRTY_SERIAL_DEMO_SERIALPORT_H
#define ALLIANCE_SENRTY_SERIAL_DEMO_SERIALPORT_H

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <string.h>
#include <string>

using namespace std;

#define UART_FALSE  -1
#define UART_TRUE   0

#define GNR_COM  1
#define USB_COM  2
#define COM_TYPE USB_COM

unsigned char XorCode(unsigned char *buf,int len);
unsigned char XorCode(std::string buf,int len);

class SerialPort{
public:
    int isNormal = UART_FALSE;
    int SerialID = UART_FALSE;
    bool isDebug = true;
    SerialPort(); 	      		           					                  //构造函数
    ~SerialPort();			           					                      //析构函数

    int Open(const char* dev_name,int speed);                	   					          //打开串口
    void Close();                    	   					                  //关闭串口
    int readBuffer(char *rcv_buf,int data_len);                               //串口接受
    int writeBuffer(unsigned char *send_buf,int data_len);                    //串口发送
    int sendWheelSpd(float lspd,float rspd);                                  //发送轮子速度

private:
    int Set(int speed,int flow_ctrl,int databits,int stopbits,char parity);   //串口设置参数
};
#endif //ALLIANCE_SENRTY_SERIAL_DEMO_SERIALPORT_H
