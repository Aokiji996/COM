/*
 * @Author: Aokiji996 1300833135@qq.com
 * @Date: 2022-12-05 17:12:42
 * @LastEditors: Aokiji996 1300833135@qq.com
 * @LastEditTime: 2022-12-05 22:31:27
 * @FilePath: /COM/main.cpp
 * @Description: Using example for com library V1.0.0
 */
#include <iostream>
#include "com.h"
#include <functional>

com::USART u1;

/* set receive callback */
void get_receive(char* data, int length)
{
    printf("received: %s\n", data);

    std::string responsePrefix = "received: ";
    std::string response(data, length);
    response = responsePrefix + response;

    u1.USART_SEND((char*)response.c_str());
}

int main() {
    std::function<void(char *, int)> foo = get_receive;
    u1.USART_setReceiveCallback(foo);
    u1.USART_INIT();

    char buf1[] = "Hello1";
    unsigned char buf2[] = "Hello2";
    std::string buf3 = "Hello3";
    u1.USART_SEND(buf1);
    u1.USART_SEND(buf2);
    u1.USART_SEND(buf3);

    while (1){}

    u1.USART_CLOSE();
    return 0;
}
