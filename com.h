/**
 * @name:COM LIBRARY
 * @author:Aokiji996
 * @version:V2.1.0
 * @description:the library to solve the communication between two devices
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

namespace com{

const int is_char   = 0;
const int is_int    = 1;
const int is_float  = 2;
const int is_double = 3;


#define COUT_RED_START      std::cout<<"\033[1;31m";
#define COUT_GREEN_START    std::cout<<"\033[1;32m";
#define COUT_YELLOW_START   std::cout<<"\033[1;33m";
#define COUT_BLUE_START     std::cout<<"\033[1;34m";
#define COUT_PURPLE_START   std::cout<<"\033[1;35m";
#define COUT_CYAN_START     std::cout<<"\033[1;36m";
#define COUT_WHITE_START    std::cout<<"\033[1;37m";
#define COUT_COLOR_END      std::cout <<"\033[0m";


/* 关闭assert */
//#define NDEBUG
/* whether to print some information */
#define IF_PRINT_INFORMATION  true
/* whether to print some error information */
#define IF_THROW_EXCEPTION    true


class USART;
class RECEIVE_DATA;


class FRAME{
    friend class USART;
    friend class RECEIVE_DATA;
private:
    /**
     * @param send_data_part the frame can be divided into several parts, show how many parts they have
     */
    int send_data_part = 0;
    /**
     * @param write_idnex a index to help developer to make sure the place to insert data
     */
    int write_index = 0;
    /**
     * @param where to save the data to send
     */
    char write_buffer[128];
public:
    /**
     * @brief add the type that you want to use to send or receive data
     * @param type
     */
    void FRAME_ADD_ELEMENT(int type);
    /**
     * @brief add concrete data in order for send
     * @param data
     */
    void FRAME_ADD_DATA(char data);
    void FRAME_ADD_DATA(std::string data);
    void FRAME_ADD_DATA(int data);
    void FRAME_ADD_DATA(float data);
    void FRAME_ADD_DATA(double data);
    /**
     * @brief show the frame type
     */
    void FRAME_PRINT_ELEMENTS();
    /**
     * @brief show the data prepare to send
     */
    void FRAME_SHOW_DATA();
    /**
     * @brief send data save in the write_buffer
     * @param usart the com you want to use to send the data
     * @param times the times you want to send, default equal to 1
     */
    void FRAME_SEND_DATA(USART &usart, unsigned int times = 1);
    /**
     * @param send_data_size show the size of send data to help to insert the '\0'
     */
    int send_data_size = 0;
    /**
     * @param format save the frame type
     */
    std::map<int, int> format;
};


class RECEIVE_DATA{
private:
    /**
     * @param receive_data an array where to save the data which received by com, but not this variable haven't been set successfully, so it won't be used now.
     */
    void *receive_data[128];
    /**
     * @param receive_data_size as the name you think
     */
    int receive_data_size = 0;
public:
    /**
     * @brief add the data to receive_data, don't use it now, it's being developed
     * @param data the data pushback to the receive_data
     */
    void RECEIVE_DATA_PUSH_VALUE(char data);
    void RECEIVE_DATA_PUSH_VALUE(int data);
    void RECEIVE_DATA_PUSH_VALUE(float data);
    void RECEIVE_DATA_PUSH_VALUE(double data);
    /**
     * @brief get data from receive_data, don't use it now, it's being developed
     * @param index the index of the num you want
     * @return a pointer you can transfer it into the type you need to read
     */
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
    /**
     * @brief new send function support to send data as the frame type
     * @param write_buff the content of the send data
     * @param send_times send times
     */
    void USART_SEND_FRAME(char *write_buff ,int send_size, int send_times = 1);
    /**
     * @brief 设置回调函数用于进行接收后的处理
     * @param receiveCallback 函数模板
     */
    void USART_SET_ReceiveCallback_Normal(Normal_ReceiveCallback normal_receiveCallback);
    void USART_SET_ReceiveCallback_Frame(Frame_ReceiveCallback frame_ReceiveCallback);
    /**
     * @brief open the com
     */
    void USART_OPEN();
    /**
     * @brief close the com
     */
    void USART_CLOSE();
    /**
     * @brief set all the settings of this com
     */
    void USART_SET();
    /**
     * @brief set the name of the com to open
     * @param str com's name
     */
    void USART_SET_COM_NAME(std::string str);
    /**
     * @brief print the com information
     */
    void USART_INFORMATION();
    /**
     * @brief set the baud rate
     * @param set_speed the rate you want
     */
    void USART_SET_SPEED(int set_speed);
    /**
     * @brief set the flow ctrl mode
     * @param set_flow_ctrl the mode you want
     */
    void USART_SET_FLOW_CTRL(int set_flow_ctrl);
    /**
     * @brief set the data bits
     * @param set_databits
     */
    void USART_SET_DATABITS(int set_databits);
    /**
     * @brief set the parity
     * @param set_parity
     */
    void USART_SET_PARITY(int set_parity);
    /**
     * @brief set the data bits
     * @param set_stopbits
     */
    void USART_SET_STOPBITS(int set_stopbits);
    /**
     * @brief set if you want open the receive mode
     * @param if_receive
     */
    void USART_SET_IF_RECEIVE(int if_receive);
    /**
     * @brief get the frame type to tell the com how to explain the data
     * @param frame
     */
    void USART_GET_FRAME(FRAME &frame);
    /**
     * @brief set all settings to set receive (for developer)
     */
    void USART_SET_RECEIVE();
    /**
     * @brief if use the special receive mode to receive data
     * @param if_use_frame_mode
     */
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
