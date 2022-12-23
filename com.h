/**
 * @name:COM LIBRARY
 * @author:Aokiji
 * @version:V1.0.0
 * @description:TODO
 */

#include <iostream>
#include <assert.h>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <sys/termios.h>
#include <cerrno>
#include <string>
#include <cstring>
#include <sys/ioctl.h>
#include <iomanip>
#include <termios.h>
#include <string>
#include <thread>
#include <functional>

/* 关闭assert */
//#define NDEBUG
/* whether to print some information */
#define IF_PRINT_INFORMATION  1
/* whether to print some error information */
#define IF_THROW_EXCEPTION    1

using ReceiveCallback = std::function<void (char*,int)>;

namespace com{

class USART{
private:
    /* 串口名 */
    const char *file_name = "/dev/ttyUSB0";
    /* 波特率 */
    int speed = B115200;
    /* 硬件控制 */
    int flow_ctrl = 0;
    /* 数据位 */
    int databits = 8;
    /* 奇偶校验 */
    int parity = 0;
    /* 停止位 */
    int stopbits = 1;
    /* 串口配置结构体 */
    struct termios options = {0};
    /* 文件 */
    int fd = -1;
    /* 判断串口是否成功打开 */
    bool if_success = 0;
    /* 是否开启接收 */
    bool receivable = true;
    /* 最大接收长度 */
    unsigned int receive_Maxlength = 2048;
    /* 接收回调函数 */
    ReceiveCallback receiveCallback;
public:
    /**
     * @brief 串口初始化
     */
    /* 串口初始化 */
    void USART_INIT();
    /* 发送信息 */
    /**
     * @brief 支持sting，unsigned char[],char[]进行发送信息，只发送一次
     * @param write_buffer 需要发送的值
     */
    void USART_SEND(std::string write_buffer);
    void USART_SEND(unsigned char write_buffer[]);
    void USART_SEND(char write_buffer[]);
    /* 设置回调函数 */
    /**
     * @brief 设置回调函数用于进行接收后的处理
     * @param receiveCallback 函数模板
     */
    void USART_setReceiveCallback(ReceiveCallback receiveCallback);
    void USART_OPEN();
    void USART_CLOSE();
    void USART_SET();
    void USART_INFORMATION();
    void USART_SET_SPEED(int set_speed);
    void USART_SET_FLOW_CTRL(int set_flow_ctrl);
    void USART_SET_DATABITS(int set_databits);
    void USART_SET_PARITY(int set_parity);
    void USART_SET_STOPBITS(int set_stopbits);

protected:
    /* 串口可设置对应值为如下内容，可参照INIT函数中的注释使用 */
    std::string files_name[3] ={
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "..."
    };
    int speed_arr[7] = {
            B115200,
            B19200,
            B9600,
            B4800,
            B2400,
            B1200,
            B300,
    };
    int flow_ctrl_arr[3] = {0, 1, 2};
    int databits_arr[4] = {5, 6, 7, 8};
    int parity_arr[4] = {0, 1, 2, 3};
    int stopbits_arr[2] = {1, 2};
};


}
