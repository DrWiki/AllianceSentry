#include "SerialPort.h"
SerialPort::SerialPort(){
    isNormal = UART_FALSE;
    SerialID = UART_FALSE;
    isDebug = true;
}
SerialPort::~SerialPort(){
    Close();
}

int SerialPort::Open(const char* dev_name,int speed){
    SerialID = open(dev_name, O_RDWR|O_NOCTTY|O_NDELAY);
    if(isDebug){
        printf("\n---SerialPort---: You choose %d\n",SerialID);
    }
    if (SerialID == UART_FALSE){
        if(isDebug){
            perror("---SerialPort---: Can't Open Serial Port!");
        }
        return(UART_FALSE);
    }

    //恢复串口为阻塞状态 等待数据读入
    //非阻塞：fcntl(fd,F_SETFL,FNDELAY)
    bool temp = fcntl(SerialID, F_SETFL, 0);
    if(temp < 0){
        if(isDebug){
            printf("---SerialPort---: False on ""fcntl(ID, F_SETFL, 0) < 0"" \n");
        }
        return(UART_FALSE);
    }else{
        if(isDebug){
            printf("---SerialPort---:fcntl= %d \n",fcntl(SerialID, F_SETFL,0));
        }
    }
    //设置串口信息
    isNormal = Set(speed,0,8,1,'N');
    if(isNormal != UART_FALSE){
        if(isDebug){
            printf("---SerialPort---: Successfully Open ttyUSB%d\n",SerialID);
        }
    }
    return SerialID;
}
void SerialPort::Close(){
    close(SerialID);
}
int SerialPort::Set(int speed,int flow_ctrl,int databits,int stopbits,char parity){

    int   i;
    speed_t speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    //设置串口输入波特率和输出波特率
    /*
    for ( i= 0;  i < 7;  i++){
        if  (speed == name_arr[i]){
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }
    */
    cfmakeraw(&options);              //设置为原始模式,所有输入都按字节处理,实测没有这步配置,FF有时会读成7F甚至5F
    cfsetispeed(&options, B115200);   //输入波特率
    cfsetospeed(&options, B115200);   //输出波特率

    /*
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
    */
    options.c_cflag = 0x18b2;       //以上被注释语句等价于这步

    //设置数据流控制
    switch(flow_ctrl){
        case 0 ://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1 ://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2 ://使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
        default:
            break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits){
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            if(isDebug){
                fprintf(stderr,"Unsupported data size\n");
            }
            return (UART_FALSE);
    }
    //设置校验位
    switch (parity){
        case 'n':
        case 'N': //无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O'://设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E'://设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': //设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            if(isDebug){
                fprintf(stderr,"Unsupported parity\n");
            }
            return (UART_FALSE);
    }
    // 设置停止位

    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB; break;
        case 2:
            options.c_cflag |= CSTOPB; break;
        default:
            if(isDebug){
                fprintf(stderr,"Unsupported stop bits\n");
            }
            return (UART_FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
    //options.c_lflag &= ~(ISIG | ICANON);
    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(SerialID,TCIFLUSH);
    //激活配置 (将修改后的termios数据设置到串口中）

    if (tcsetattr(SerialID,TCSANOW,&options) != 0){
        if(isDebug){
            perror("com set error!\n");
        }
        return (UART_FALSE);
    }
    if(isDebug){
        printf("usart set normal\r\n");
    }
    return (UART_TRUE);
}

int SerialPort::readBuffer(char *rcv_buf,int data_len){
    ssize_t len;
//    int fs_sel =1 ;
    fd_set fs_read;
    struct timeval time;
    if(isNormal == UART_FALSE){
        return UART_FALSE;
    }
    FD_ZERO(&fs_read);
    FD_SET(SerialID,&fs_read);
    time.tv_sec = 0;
    time.tv_usec = 1000;  //1ms
    //使用select实现串口的多路通信
    //fs_sel = select(ID+1,&fs_read,NULL,NULL,&time);
    len = read(SerialID,rcv_buf, static_cast<size_t >(data_len));
    if(len>0){
        return static_cast<int>(len);
    }else{
        return UART_FALSE;
    }
}
/********************************************************************
* 名称：                  UART0_Send
* 功能：                发送数据
* 入口参数：        fd                  :文件描述符
*                              send_buf    :存放串口发送数据
*                              data_len    :一帧数据的个数
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int SerialPort::writeBuffer(unsigned char *send_buf,int data_len){
    ssize_t len = 0;
    if(isNormal == UART_FALSE){
        return UART_FALSE;
    }
    len = write(SerialID,(char *)send_buf, static_cast<size_t >(data_len));
    if (len == data_len ){
        return static_cast<int>(len);
    }else{
        tcflush(SerialID,TCOFLUSH);
        return UART_FALSE;
    }
}

/*
 *
 *
 *     //测试是否为终端设备  确认串口是否打开
    if(!isatty(STDIN_FILENO)){
        if(isDebug){
            printf("---SerialPort---: standard input is not a terminal device\n");
        }
        return(UART_FALSE);
    }else{
        if(isDebug){
            printf("---SerialPort---: isatty success!\n");
        }
    }*/
