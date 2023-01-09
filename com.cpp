#include "com.h"

namespace com{

void USART::USART_OPEN() {
    this->fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
#if IF_THROW_EXCEPTION
    if(this->fd == -1){
        this->if_success = false;
        COUT_RED_START
        printf(" Error! in Opening ttyUSB0 \n");
        COUT_COLOR_END
    } else{
#if IF_PRINT_INFORMATION
        COUT_GREEN_START
        printf(" ttyUSB0 Opened Successfully \n");
        COUT_COLOR_END
#endif
    }
#endif
}

void USART::USART_CLOSE() {
    if(fd != -1){
        close(this->fd);
#if IF_PRINT_INFORMATION
        COUT_BLUE_START
        printf(" +----------------------------------+\n");
        printf(" |        Serial Port Close         |\n");
        printf(" +----------------------------------+\n");
        COUT_COLOR_END
#endif
    }
}

void USART::USART_INFORMATION() {
    assert(this->if_success);
#if IF_PRINT_INFORMATION
    COUT_YELLOW_START
    switch (this->speed) {
        case B115200:
            printf(" BaudRate = 115200 \n");
            break;
        case B19200:
            printf(" BaudRate = 19200 \n");
            break;
        case B9600:
            printf(" BaudRate = 9600 \n");
            break;
        case B4800:
            printf(" BaudRate = 4800 \n");
            break;
        case B2400:
            printf(" BaudRate = 2400 \n");
            break;
        case B1200:
            printf(" BaudRate = 1200 \n");
            break;
        case B300:
            printf(" BaudRate = 300 \n");
            break;
    }
    printf(" StopBits = %d \n", this->stopbits);
    switch (this->parity) {
        case 0:
            printf(" Parity = none \n");
            break;
        case 1:
            printf(" Parity = odd \n");
            break;
        case 2:
            printf(" Parity = even \n");
            break;
        case 3:
            printf(" Parity = space \n");
            break;
    }
    if(this->receivable){
        printf(" Receive Data\n");
    }else{
        printf(" Refuse Data\n");
    }
    COUT_COLOR_END
#endif
}

void USART::USART_SET() {
    if(!this->if_success){
        return;
    }
    /* Error Checking */
    if (tcgetattr(fd, &this->options)){
        this->if_success = false;
#if IF_PRINT_INFORMATION
        COUT_RED_START
        printf(" Error in SetupsSerial !\n");
        COUT_COLOR_END
#endif
    }
    if(!this->if_success){
        return;
    }

    /* 设置串口输入波特率和输出波特率 */
    cfsetispeed(&this->options, this->speed);
    cfsetospeed(&this->options, this->speed);

    /* 修改控制模式，保证程序不会占用串口 */
    options.c_cflag |= CLOCAL;
    /* 修改控制模式，使得能够从串口中读取输入数据 */
    options.c_cflag |= CREAD;

    /* 设置数据流控制 */
    switch (this->flow_ctrl) {
        /* 不使用数据流控制 */
        case 0:
            this->options.c_cflag &= ~CRTSCTS;
            break;
        /* 使用硬件数据流控制 */
        case 1:
            this->options.c_cflag |= CRTSCTS;
            break;
        /* 使用软件数据流控制 */
        case 2:
            this->options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }

    /* 设置数据位，屏蔽其他标志位 */
    this->options.c_cflag &= ~CSIZE;
    switch (this->databits) {
        case 5:
            this->options.c_cflag |= CS5;
            break;
        case 6:
            this->options.c_cflag |= CS6;
            break;
        case 7:
            this->options.c_cflag |= CS7;
            break;
        case 8:
            this->options.c_cflag |= CS8;
            break;
    }

    /* 设置校验位 */
    switch (this->parity) {
        /* 无奇偶校验位 */
        case 0:
            this->options.c_cflag &= ~PARENB;
            this->options.c_iflag &= ~INPCK;
            break;
        /* 设置为奇校验 */
        case 1:
            this->options.c_cflag |= (PARODD | PARENB);
            this->options.c_iflag |= INPCK;
            break;
        /* 设置为偶校验 */
        case 2:
            this->options.c_cflag != PARENB;
            this->options.c_cflag &= ~PARODD;
            this->options.c_iflag |= INPCK;
            break;
        /* 设置为空格 */
        case 3:
            this->options.c_cflag &= ~PARENB;
            this->options.c_cflag &= ~CSTOPB;
            break;
    }

    /* 设置停止位 */
    switch (this->stopbits) {
        case 1:
            this->options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            this->options.c_cflag |= CSTOPB;
            break;
    }

    /* 设置操作模式 */
    this->options.c_oflag &= ~OPOST;
    this->options.c_oflag &= ~(ONLCR | OCRNL);

    this->options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    this->options.c_iflag &= ~(ICRNL | INLCR);
    this->options.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* 设置等待时间和最小接收字符 */
    /* 读取一个字符等待1*(1/10)s */
    this->options.c_cc[VTIME] = 1;
    /* 读取字符的最少个数为1 */
    this->options.c_cc[VMIN] = 1;

    /* 如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读 */
    tcflush(this->fd,TCIFLUSH);

    /* 激活配置 */
    if(tcsetattr(this->fd, TCSANOW, &this->options) != 0){
        this->if_success = false;
#if IF_THROW_EXCEPTION
        COUT_RED_START
        printf(" com set error!\n");
        COUT_COLOR_END
#endif
    }
    if(!this->if_success){
        return;
    }
    if(this->receivable){
        std::thread([&]{
            char* receiveData = new char[this->receive_Maxlength];
            int receivedLength = 0;
            int selectResult = -1;

            while (this->receivable){
                memset(receiveData,0,this->receive_Maxlength);
                receivedLength = read(this->fd, receiveData, this->receive_Maxlength); /* block util data received */
                if(receivedLength > 0){
                    if(nullptr != receiveCallback){
                        receiveCallback(receiveData,receivedLength);
                    }
                }
                receivedLength = 0;
            }
            delete[] receiveData;
            receiveData = nullptr;
        }).detach();
    }
}

void USART::USART_INIT() {
#if IF_PRINT_INFORMATION
    COUT_BLUE_START
    printf(" +----------------------------------+\n");
    printf(" |        Serial Port Init          |\n");
    printf(" +----------------------------------+\n");
    COUT_COLOR_END
#endif
    this->USART_OPEN();
    this->USART_SET();
#if IF_PRINT_INFORMATION
    if(this->if_success){
        this->USART_INFORMATION();
    }
    COUT_BLUE_START
    printf(" +----------------------------------+\n");
    printf(" |        Serial Port End Init      |\n");
    printf(" +----------------------------------+\n");
    COUT_COLOR_END
#endif
}

void USART::USART_SEND(std::string str) {
    const char* write_buffer = str.c_str();
    const unsigned char* u_write_buffer = (const unsigned char*)write_buffer;
    write(this->fd, write_buffer, strlen(write_buffer));
#if IF_PRINT_INFORMATION
    //TODO:add the print function
#endif
}

void USART::USART_SEND(unsigned char u_write_buffer[]) {
    std::string str(reinterpret_cast<char*>(u_write_buffer));
    write(this->fd, u_write_buffer, str.length());
#if IF_PRINT_INFORMATION
    //TODO:add the print function
#endif
}

void USART::USART_SEND(char write_buffer[]){
    const unsigned char* u_write_buffer = (const unsigned char*)write_buffer;
    write(this->fd, write_buffer, strlen(write_buffer));
#if IF_PRINT_INFORMATION
    //TODO:add the print function
#endif
}

void USART::USART_setReceiveCallback(ReceiveCallback receiveCallback) {
    this->receiveCallback = receiveCallback;
}

void USART::USART_SET_SPEED(int set_speed) {
    bool is_legal = false;
    for(size_t i = 0; i < sizeof(this->speed_arr)/sizeof(this->speed_arr[0]); i++){
        if(this->speed_arr[i] == set_speed){
            is_legal = true;
        }
    }
    if(is_legal){
        this->speed = set_speed;
#if IF_PRINT_INFORMATION
        COUT_GREEN_START
        printf("set variable success!\n");
        COUT_COLOR_END //TODO:make clearer the print information
#endif
    } else{
#if IF_THROW_EXCEPTION
        COUT_RED_START
        printf("illegal variable!\n");
        COUT_COLOR_END
#endif
    }
}

void USART::USART_SET_FLOW_CTRL(int set_flow_ctrl) {
    bool is_legal = false;
    for (size_t i = 0; i < sizeof(this->flow_ctrl_arr) / sizeof(this->flow_ctrl_arr[0]); i++) {
        if (this->flow_ctrl_arr[i] == set_flow_ctrl) {
            is_legal = true;
        }
    }
    if (is_legal) {
        this->flow_ctrl = set_flow_ctrl;
#if IF_PRINT_INFORMATION
        COUT_GREEN_START
        printf("set variable success!\n");
        COUT_COLOR_END
#endif
    } else {
#if IF_THROW_EXCEPTION
        COUT_RED_START
        printf("illegal variable!\n");
        COUT_COLOR_END
#endif
    }
}

void USART::USART_SET_DATABITS(int set_databits) {
    bool is_legal = false;
    for (size_t i = 0; i < sizeof(this->databits_arr) / sizeof(this->databits_arr[0]); i++) {
        if (this->databits_arr[i] == set_databits) {
            is_legal = true;
        }
    }
    if (is_legal) {
        this->databits = set_databits;
#if IF_PRINT_INFORMATION
        COUT_GREEN_START
        printf("set variable success!\n");
        COUT_COLOR_END
#endif
    } else {
#if IF_THROW_EXCEPTION
        COUT_RED_START
        printf("illegal variable!\n");
        COUT_COLOR_END
#endif
    }
}

void USART::USART_SET_PARITY(int set_parity) {
    bool is_legal = false;
    for (size_t i = 0; i < sizeof(this->parity_arr) / sizeof(this->parity_arr[0]); i++) {
        if (this->parity_arr[i] == set_parity) {
            is_legal = true;
        }
    }
    if (is_legal) {
        this->parity = set_parity;
#if IF_PRINT_INFORMATION
        COUT_GREEN_START
        printf("set variable success!\n");
        COUT_COLOR_END
#endif
    } else {
#if IF_THROW_EXCEPTION
        COUT_RED_START
        printf("illegal variable!\n");
        COUT_COLOR_END
#endif
    }
}

void USART::USART_SET_STOPBITS(int set_stopbits) {
    bool is_legal = false;
    for (size_t i = 0; i < sizeof(this->stopbits_arr) / sizeof(this->stopbits_arr[0]); i++) {
        if (this->stopbits_arr[i] == set_stopbits) {
            is_legal = true;
        }
    }
    if (is_legal) {
        this->stopbits = set_stopbits;
#if IF_PRINT_INFORMATION
        COUT_GREEN_START
        printf("set variable success!\n");
        COUT_COLOR_END
#endif
    } else {
#if IF_THROW_EXCEPTION
        COUT_RED_START
        printf("illegal variable!\n");
        COUT_COLOR_END
#endif
    }
}


void FRAME::FRAME_ADD_ELEMENT(int type) {
    switch(type){
        case is_char:
            this->format.emplace(this->size, is_char);
            break;
        case is_int:
            this->format.emplace(this->size, is_int);
            break;
        case is_float:
            this->format.emplace(this->size, is_float);
            break;
        case is_double:
            this->format.emplace(this->size, is_double);
            break;
        default:
#if IF_PRINT_INFORMATION
            COUT_RED_START
            printf("illegal variable!\n");
            COUT_COLOR_END
#endif
            return;
            break;
    }
    this->size += 1;
#if IF_PRINT_INFORMATION
    COUT_GREEN_START
    printf("set variable success!\n");
    COUT_COLOR_END
#endif
}


}
