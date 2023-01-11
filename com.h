/**
 * @name:COM LIBRARY
 * @author:Aokiji
 * @version:V1.0.0
 * @description:TODO
 */
#pragma once

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
#include <map>
#include <vector>


#define is_char      0
#define is_int       1
#define is_float     2
#define is_double    3


#define COUT_RED_START      std::cout<<"\033[1;31m";
#define COUT_GREEN_START    std::cout<<"\033[1;32m";
#define COUT_YELLOW_START   std::cout<<"\033[1;33m";
#define COUT_BLUE_START     std::cout<<"\033[1;34m";
#define COUT_PURPLE_START   std::cout<<"\033[1;35m";
#define COUT_CYAN_START     std::cout<<"\033[1;36m";
#define COUT_WHITE_START    std::cout<<"\033[1;37m";

#define COUT_COLOR_END    std::cout <<"\033[0m";


/* 关闭assert */
//#define NDEBUG
/* whether to print some information */
#define IF_PRINT_INFORMATION  true
/* whether to print some error information */
#define IF_THROW_EXCEPTION    true

namespace com{


class USART;
class RECEIVE_DATA;


class FRAME{
    friend class USART;
    friend class RECEIVE_DATA;
private:
    int send_data_part = 0;
    int write_index = 0;
    char write_buffer[128];
public:
    void FRAME_ADD_ELEMENT(int type);
    void FRAME_PRINT_ELEMENTS();
    void FRAME_ADD_DATA(char data);
    void FRAME_ADD_DATA(std::string data);
    void FRAME_ADD_DATA(int data);
    void FRAME_ADD_DATA(float data);
    void FRAME_ADD_DATA(double data);
    void FRAME_SHOW_DATA();
    void FRAME_SEND_DATA(USART &usart, unsigned int times = 1);

    int send_data_size = 0;
    std::map<int, int> format;
};


class RECEIVE_DATA{
private:
    void * receive_data[128];
    int receive_data_size = 0;
public:
    void RECEIVE_DATA_PUSH_VALUE(char data);
    void RECEIVE_DATA_PUSH_VALUE(int data);
    void RECEIVE_DATA_PUSH_VALUE(float data);
    void RECEIVE_DATA_PUSH_VALUE(double data);
    void *RECEIVE_DATA_GET_VAlUE(int index);
};


using Normal_ReceiveCallback = std::function<void (char *, int)>;
using Frame_ReceiveCallback = std::function<void (char *, int, FRAME frame)>;


class USART{
    friend class FRAME;
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
    bool if_success = true;
    /* 是否开启接收 */
    bool receivable = true;
    /* 最大接收长度 */
    unsigned int receive_Maxlength = 2048;
    /* 普通接收回调函数 */
    Normal_ReceiveCallback normal_receiveCallback = nullptr;
    /* frame模式接收回调函数 */
    Frame_ReceiveCallback frame_ReceiveCallback = nullptr;
    /* 接收数据的数据格式 */
    FRAME receive_frame;
    /* 是否启用特殊接收模式 */
    bool if_use_frame_mode = false;
public:
    /**
     * @brief 串口初始化
     */
     void USART_INIT();
    /**
     * @brief 支持sting，unsigned char[],char[]进行发送信息，只发送一次
     * @param write_buffer 需要发送的值
     */
    void USART_SEND(std::string write_buffer);
    void USART_SEND(unsigned char write_buffer[]);
    void USART_SEND(char write_buffer[]);

    void USART_SEND_FRAME(char *write_buff ,int send_times = 1);
    /**
     * @brief 设置回调函数用于进行接收后的处理
     * @param receiveCallback 函数模板
     */
    void USART_SET_ReceiveCallback_Normal(Normal_ReceiveCallback normal_receiveCallback);
    void USART_SET_ReceiveCallback_Frame(Frame_ReceiveCallback frame_ReceiveCallback);
    void USART_OPEN();
    void USART_CLOSE();
    void USART_SET();
    void USART_SET_COM_NAME(std::string str);
    void USART_INFORMATION();
    void USART_SET_SPEED(int set_speed);
    void USART_SET_FLOW_CTRL(int set_flow_ctrl);
    void USART_SET_DATABITS(int set_databits);
    void USART_SET_PARITY(int set_parity);
    void USART_SET_STOPBITS(int set_stopbits);
    void USART_SET_IF_RECEIVE(int if_receive);
    void USART_GET_FRAME(FRAME &frame);
    void USART_SET_RECEIVE();
    void USART_SET_RECEIVER_MODE(bool if_use_frame_mode);

protected:
    /* 串口可设置对应值为如下内容，可参照INIT函数中的注释使用 */
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
