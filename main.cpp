#include <iostream>
#include <functional>

#include "com.h"

com::USART u1;
com::USART u2;

/* set receive callback */
void get_receive(char *data, int length, com::FRAME frame)
{
    /* 展示如何从中获取对应的数据 */
    for(auto iter = frame.format.begin();iter != frame.format.end(); ++iter){
        switch (iter->second) {
            case is_char:
                std::cout << data[iter->first] << ' ';
                break;
            case is_int:
                int c;
                c = data[iter->first] - '0';
                std::cout << c << ' ';
                break;
            case is_float:
                float a;
                a = *(float *)(&data[iter->first]);
                std::cout << a << ' ';
                break;
            case is_double:
                double b;
                b = *(double *)(&data[iter->first]);
                std::cout << b << ' ';
                break;
        }
    }
    std::cout << std::endl;
}


int main() {
    /* 制作模板 */
    com::FRAME f;
    f.FRAME_ADD_ELEMENT(is_char);
    f.FRAME_ADD_ELEMENT(is_char);
    f.FRAME_ADD_ELEMENT(is_int);
    f.FRAME_ADD_ELEMENT(is_float);
    f.FRAME_ADD_ELEMENT(is_float);
    f.FRAME_ADD_ELEMENT(is_char);
    f.FRAME_PRINT_ELEMENTS();


    /* 将模板格式传递给串口 */
    u2.USART_GET_FRAME(f);

    f.FRAME_ADD_DATA('?');
    f.FRAME_ADD_DATA('B');
    f.FRAME_ADD_DATA(1);
    f.FRAME_ADD_DATA(1.234f);
    f.FRAME_ADD_DATA(2.45f);
    f.FRAME_ADD_DATA('?');
    f.FRAME_SEND_DATA(u1);

    /* 接收数据设置 */
    std::function<void(char *, int, com::FRAME)> foo = get_receive;
    u2.USART_SET_ReceiveCallback_Frame(foo);
    u2.USART_SET_COM_NAME("/dev/ttyUSB1");
    u2.USART_SET_RECEIVER_MODE(true);
    u2.USART_INIT();

    while(1){

    }

    u2.USART_CLOSE();
    return 0;
}
