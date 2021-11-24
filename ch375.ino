#include "ch375.h"

// 引脚连接
#define D0 GPIO_NUM_16
#define D1 GPIO_NUM_32
#define D2 GPIO_NUM_33
#define D3 GPIO_NUM_25
#define D4 GPIO_NUM_26
#define D5 GPIO_NUM_27
#define D6 GPIO_NUM_14
#define D7 GPIO_NUM_12

#define WR GPIO_NUM_17
#define RD GPIO_NUM_5
#define A0 GPIO_NUM_18
#define INT GPIO_NUM_19

unsigned char buffer[64]; /* 公用缓冲区 */

#define usb_delay_us(n) delayMicroseconds(n)
#define usb_delay_2us() usb_delay_us(2)

boolean is_output = false;

void write(uint8_t d)
{
  if (!is_output)
  {
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);
    pinMode(D7, OUTPUT);
    is_output = true;
  }

  digitalWrite(D0, (d & (0x01 << 0)) ? 1 : 0);
  digitalWrite(D1, (d & (0x01 << 1)) ? 1 : 0);
  digitalWrite(D2, (d & (0x01 << 2)) ? 1 : 0);
  digitalWrite(D3, (d & (0x01 << 3)) ? 1 : 0);
  digitalWrite(D4, (d & (0x01 << 4)) ? 1 : 0);
  digitalWrite(D5, (d & (0x01 << 5)) ? 1 : 0);
  digitalWrite(D6, (d & (0x01 << 6)) ? 1 : 0);
  digitalWrite(D7, (d & (0x01 << 7)) ? 1 : 0);
}

uint8_t read()
{
  if (is_output)
  {
    pinMode(D0, INPUT);
    pinMode(D1, INPUT);
    pinMode(D2, INPUT);
    pinMode(D3, INPUT);
    pinMode(D4, INPUT);
    pinMode(D5, INPUT);
    pinMode(D6, INPUT);
    pinMode(D7, INPUT);
    is_output = false;
  }
  uint8_t d = 0;
  if (digitalRead(D0))
    d |= (0x01 << 0);
  if (digitalRead(D1))
    d |= (0x01 << 1);
  if (digitalRead(D2))
    d |= (0x01 << 2);
  if (digitalRead(D3))
    d |= (0x01 << 3);
  if (digitalRead(D4))
    d |= (0x01 << 4);
  if (digitalRead(D5))
    d |= (0x01 << 5);
  if (digitalRead(D6))
    d |= (0x01 << 6);
  if (digitalRead(D7))
    d |= (0x01 << 7);
  return d;
}

void write_cmd(uint8_t d)
{
  printf("CMD: %02X\n", d);
  digitalWrite(A0, 1);
  write(d);
  digitalWrite(WR, 1);
  usb_delay_us(20);
  digitalWrite(WR, 0);
  usb_delay_us(20);
  digitalWrite(WR, 1);
}

void write_data(uint8_t d)
{
  printf("DAT: %02X\n", d);
  digitalWrite(A0, 0);
  write(d);
  digitalWrite(WR, 1);
  usb_delay_us(20);
  digitalWrite(WR, 0);
  usb_delay_us(20);
  digitalWrite(WR, 1);
}

uint8_t read_data()
{
  digitalWrite(WR, 1);
  digitalWrite(A0, 0);
  digitalWrite(RD, 1);
  usb_delay_us(20);
  digitalWrite(RD, 0);
  usb_delay_us(20);
  uint8_t d = read();
  digitalWrite(RD, 1);
  printf("RAD: %02X\n", d);
  return d;
}

void init_ch375()
{
  pinMode(WR, OUTPUT);
  pinMode(RD, OUTPUT);
  pinMode(A0, OUTPUT);

  pinMode(INT, INPUT_PULLUP);

  digitalWrite(A0, 0);
  digitalWrite(WR, 1);
  digitalWrite(RD, 1);

  write(0x00);
}

void ch375_reset()
{
  // 复位
  write_cmd(CMD_RESET_ALL);
  usb_delay_us(40000);
}

void ch375_set_model(uint8_t mode)
{
  write_cmd(CMD_SET_USB_MODE);
  write_data(mode);
}

void ch375_host()
{
  // 设置HOST模式
  ch375_set_model(0x07);
  usb_delay_us(20);
  ch375_set_model(0x06);
  usb_delay_us(20);
}

inline uint8_t ch375_get_status()
{
  write_cmd(CMD_GET_STATUS);
  return read_data();
}

inline uint8_t ch375_test_connect()
{
  write_cmd(CMD_TEST_CONNECT);
  usb_delay_us(2);
  return read_data();
}

boolean ch375_interrupt()
{
  return !digitalRead(INT);
}

unsigned char wait_interrupt()
{ /* 主机端等待操作完成, 返回操作状态 */
  unsigned short i;
  for (i = 0; !ch375_interrupt(); i++)
  { /* 如果CH375的中断引脚输出高电平则等待,通过计数防止超时 */
    usb_delay_2us();
    if (i == 0xF000)
      write_cmd(CMD_ABORT_NAK); /* 如果超时达61mS以上则强行终止NAK重试,中断返回USB_INT_RET_NAK */
  }
  return ch375_get_status();
}

boolean ch375_exist()
{
  uint8_t d = 0xAA;
  write_cmd(CMD_CHECK_EXIST);
  write_data(~d);
  return d == read_data();
}

/* 数据同步 */
/* USB的数据同步通过切换DATA0和DATA1实现: 在设备端, USB打印机可以自动切换;
   在主机端, 必须由SET_ENDP6和SET_ENDP7命令控制CH375切换DATA0与DATA1.
   主机端的程序处理方法是为设备端的各个端点分别提供一个全局变量,
   初始值均为DATA0, 每执行一次成功事务后取反, 每执行一次失败事务后将其复位为DATA1 */

void toggle_recv(bool tog)
{ /* 主机接收同步控制:0=DATA0,1=DATA1 */
  write_cmd(CMD_SET_ENDP6);
  write_data(tog ? 0xC0 : 0x80);
  usb_delay_2us();
}

void toggle_send(bool tog)
{ /* 主机发送同步控制:0=DATA0,1=DATA1 */
  write_cmd(CMD_SET_ENDP7);
  write_data(tog ? 0xC0 : 0x80);
  usb_delay_2us();
}

unsigned char clr_stall(unsigned char endp_addr)
{ /* USB通讯失败后,复位设备端的指定端点到DATA0 */
  write_cmd(CMD_CLR_STALL);
  write_data(endp_addr);
  return wait_interrupt();
}

/* 数据读写, 单片机读写CH375芯片中的数据缓冲区 */

unsigned char rd_usb_data(unsigned char *buf)
{ /* 从CH37X读出数据块 */
  unsigned char i, len;
  write_cmd(CMD_RD_USB_DATA); /* 从CH375的端点缓冲区读取接收到的数据 */
  len = read_data();          /* 后续数据长度 */
  for (i = 0; i != len; i++)
    *buf++ = read_data();
  return len;
}

void wr_usb_data(unsigned char len, unsigned char *buf)
{                              /* 向CH37X写入数据块 */
  write_cmd(CMD_WR_USB_DATA7); /* 向CH375的端点缓冲区写入准备发送的数据 */
  write_data(len);             /* 后续数据长度, len不能大于64 */
  while (len--)
    write_data(*buf++);
}

/* 主机操作 */
unsigned char endp_out_addr; /* 打印机数据接收端点的端点地址 */
unsigned char endp_out_size; /* 打印机数据接收端点的端点尺寸 */
bool tog_send;               /* 打印机数据接收端点的同步标志 */
unsigned char endp_in_addr;  /* 双向打印机发送端点的端点地址,一般不用 */
bool tog_recv;               /* 双向打印机发送端点的同步标志,一般不用 */
unsigned char issue_token(unsigned char endp_and_pid)
{ /* 执行USB事务 */
  /* 执行完成后, 将产生中断通知单片机, 如果是USB_INT_SUCCESS就说明操作成功 */
  write_cmd(CMD_ISSUE_TOKEN);
  write_data(endp_and_pid); /* 高4位目的端点号, 低4位令牌PID */
  return wait_interrupt();  /* 等待CH375操作完成 */
}

unsigned char issue_token_X(unsigned char endp_and_pid, unsigned char tog)
{ /* 执行USB事务,适用于CH375A */
  /* 执行完成后, 将产生中断通知单片机, 如果是USB_INT_SUCCESS就说明操作成功 */
  write_cmd(CMD_ISSUE_TKN_X);
  write_data(tog);          /* 同步标志的位7为主机端点IN的同步触发位, 位6为主机端点OUT的同步触发位, 位5~位0必须为0 */
  write_data(endp_and_pid); /* 高4位目的端点号, 低4位令牌PID */
  return wait_interrupt();  /* 等待CH375操作完成 */
}

void soft_reset_print()
{                          /* 控制传输:软复位打印机 */
  tog_send = tog_recv = 0; /* 复位USB数据同步标志 */
  toggle_send(0);          /* SETUP阶段为DATA0 */
  buffer[0] = 0x21;
  buffer[1] = 2;
  buffer[2] = buffer[3] = buffer[4] = buffer[5] = buffer[6] = buffer[7] = 0; /* SETUP数据,SOFT_RESET */
  wr_usb_data(8, buffer);                                                    /* SETUP数据总是8字节 */
  if (issue_token((0 << 4) | DEF_USB_PID_SETUP) == USB_INT_SUCCESS)
  {                 /* SETUP阶段操作成功 */
    toggle_recv(1); /* STATUS阶段,准备接收DATA1 */
    if (issue_token((0 << 4) | DEF_USB_PID_IN) == USB_INT_SUCCESS)
      return; /* STATUS阶段操作成功,操作成功返回 */
  }
}

#define USB_INT_RET_NAK 0x2A /* 00101010B,返回NAK */
void send_data(unsigned short len, unsigned char *buf)
{ /* 主机发送数据块,一次最多64KB */
  unsigned char l, s;
  while (len)
  {                                                          /* 连续输出数据块给USB打印机 */
    toggle_send(tog_send);                                   /* 数据同步 */
    l = len > endp_out_size ? endp_out_size : len;           /* 单次发送不能超过端点尺寸 */
    wr_usb_data(l, buf);                                     /* 将数据先复制到CH375芯片中 */
    s = issue_token((endp_out_addr << 4) | DEF_USB_PID_OUT); /* 请求CH375输出数据 */
    if (s == USB_INT_SUCCESS)
    {                       /* CH375成功发出数据 */
      tog_send = ~tog_send; /* 切换DATA0和DATA1进行数据同步 */
      len -= l;             /* 计数 */
      buf += l;             /* 操作成功 */
    }
    else if (s == USB_INT_RET_NAK)
    { /* USB打印机正忙,如果未执行SET_RETRY命令则CH375自动重试,所以不会返回USB_INT_RET_NAK状态 */
      /* USB打印机正忙,正常情况下应该稍后重试 */
      /* s=get_port_status( );  如果有必要,可以检查是什么原因导致打印机忙 */
    }
    else
    {                           /* 操作失败,正常情况下不会失败 */
      clr_stall(endp_out_addr); /* 清除打印机的数据接收端点,或者 soft_reset_print() */
                                /*			soft_reset_print();  打印机出现意外错误,软复位 */
      tog_send = 0;             /* 操作失败 */
    }
    /* 如果数据量较大,可以定期调用get_port_status()检查打印机状态 */
  }
}

unsigned char get_port_status()
{                 /* 查询打印机端口状态,返回状态码,如果为0FFH则说明操作失败 */
                  /* 返回状态码中: 位5(Paper Empty)为1说明无纸, 位4(Select)为1说明打印机联机, 位3(Not Error)为0说明打印机出错 */
  toggle_send(0); /* 下面通过控制传输获取打印机的状态, SETUP阶段为DATA0 */
  buffer[0] = 0xA1;
  buffer[1] = 1;
  buffer[2] = buffer[3] = buffer[4] = buffer[5] = 0;
  buffer[6] = 1;
  buffer[7] = 0;          /* SETUP数据,GET_PORT_STATUS */
  wr_usb_data(8, buffer); /* SETUP数据总是8字节 */
  if (issue_token((0 << 4) | DEF_USB_PID_SETUP) == USB_INT_SUCCESS)
  {                 /* SETUP阶段操作成功 */
    toggle_recv(1); /* DATA阶段,准备接收DATA1 */
    if (issue_token((0 << 4) | DEF_USB_PID_IN) == USB_INT_SUCCESS)
    {                         /* DATA阶段操作成功 */
      rd_usb_data(buffer);    /* 读出接收到的数据,通常只有1个字节 */
      toggle_send(1);         /* STATUS阶段为DATA1 */
      wr_usb_data(0, buffer); /* 发送0长度的数据说明控制传输成功 */
      if (issue_token((0 << 4) | DEF_USB_PID_OUT) == USB_INT_SUCCESS)
        return (buffer[0]); /* 返回状态码 */
    }
  }
  return 0xFF; /* 返回操作失败 */
}

unsigned char get_port_status_X()
{ /* 查询打印机端口状态,返回状态码,如果为0FFH则说明操作失败,适用于CH375A */
  /* 返回状态码中: 位5(Paper Empty)为1说明无纸, 位4(Select)为1说明打印机联机, 位3(Not Error)为0说明打印机出错 */
  buffer[0] = 0xA1;
  buffer[1] = 1;
  buffer[2] = buffer[3] = buffer[4] = buffer[5] = 0;
  buffer[6] = 1;
  buffer[7] = 0;          /* 控制传输获取打印机状态,SETUP数据 */
  wr_usb_data(8, buffer); /* SETUP数据总是8字节 */
  if (issue_token_X((0 << 4) | DEF_USB_PID_SETUP, 0x00) == USB_INT_SUCCESS)
  { /* SETUP阶段DATA0操作成功 */
    if (issue_token_X((0 << 4) | DEF_USB_PID_IN, 0x80) == USB_INT_SUCCESS)
    {                         /* DATA阶段DATA1接收操作成功 */
      rd_usb_data(buffer);    /* 读出接收到的数据,通常只有1个字节 */
      wr_usb_data(0, buffer); /* 发送0长度的数据DATA1说明控制传输成功 */
      if (issue_token_X((0 << 4) | DEF_USB_PID_OUT, 0x40) == USB_INT_SUCCESS)
        return (buffer[0]); /* STATUS阶段操作成功,返回状态码 */
    }
  }
  return 0xFF; /* 返回操作失败 */
}

unsigned char get_descr(unsigned char type)
{ /* 从设备端获取描述符 */
  write_cmd(CMD_GET_DESCR);
  write_data(type);        /* 描述符类型, 只支持1(设备)或者2(配置) */
  return wait_interrupt(); /* 等待CH375操作完成 */
}

unsigned char set_addr(unsigned char addr)
{ /* 设置设备端的USB地址 */
  unsigned char status;
  write_cmd(CMD_SET_ADDRESS); /* 设置USB设备端的USB地址 */
  write_data(addr);           /* 地址, 从1到127之间的任意值, 常用2到20 */
  status = wait_interrupt();  /* 等待CH375操作完成 */
  if (status == USB_INT_SUCCESS)
  {                              /* 操作成功 */
    write_cmd(CMD_SET_USB_ADDR); /* 设置USB主机端的USB地址 */
    write_data(addr);            /* 当目标USB设备的地址成功修改后,应该同步修改主机端的USB地址 */
  }
  delayMicroseconds(5);
  return status;
}

unsigned char set_config(unsigned char cfg)
{                            /* 设置设备端的USB配置 */
  tog_send = tog_recv = 0;   /* 复位USB数据同步标志 */
  write_cmd(CMD_SET_CONFIG); /* 设置USB设备端的配置值 */
  write_data(cfg);           /* 此值取自USB设备的配置描述符中 */
  return wait_interrupt();   /* 等待CH375操作完成 */
}

#define PRINT_PUSB_DEV_DESCR(p) printf("bLength:%02X bDescriptorType:%02X bcdUSB:%04X bDeviceClass:%02X bDeviceSubClass:%02X bDeviceProtocol:%02X bMaxPacketSize0:%02X idVendor:%04X idProduct:%04X bcdDevice:%04X iManufacturer:%02X iProduct:%02X iSerialNumber:%02X bNumConfigurations:%02X\n", (p).bLength, (p).bDescriptorType, (p).bcdUSB, (p).bDeviceClass, (p).bDeviceSubClass, (p).bDeviceProtocol, (p).bMaxPacketSize0, (p).idVendor, (p).idProduct, (p).bcdDevice, (p).iManufacturer, (p).iProduct, (p).iSerialNumber, (p).bNumConfigurations)
#define PRINT_PUSB_CFG_DESCR(p) printf("bLength:%02X bDescriptorType:%02X wTotalLength:%04X bNumInterfaces:%02X bConfigurationValue:%02X iConfiguration:%02X bmAttributes:%02X MaxPower:%02X\n", (p).bLength, (p).bDescriptorType, (p).wTotalLength, (p).bNumInterfaces, (p).bConfigurationValue, (p).iConfiguration, (p).bmAttributes, (p).MaxPower)
#define PRINT_PUSB_ITF_DESCR(p) printf("bLength:%02X bDescriptorType:%02X bInterfaceNumber:%02X bAlternateSetting:%02X bNumEndpoints:%02X bInterfaceClass:%02X bInterfaceSubClass:%02X bInterfaceProtocol:%02X iInterface:%02X\n", (p).bLength, (p).bDescriptorType, (p).bInterfaceNumber, (p).bAlternateSetting, (p).bNumEndpoints, (p).bInterfaceClass, (p).bInterfaceSubClass, (p).bInterfaceProtocol, (p).iInterface)
#define PRINT_PUSB_ENDP_DESCR(p) printf("bLength:%02X bDescriptorType:%02X bEndpointAddress:%02X bmAttributes:%02X wMaxPacketSize:%02X wMaxPacketSize1:%02X bInterval:%02X\n", (p).bLength, (p).bDescriptorType, (p).bEndpointAddress, (p).bmAttributes, (p).wMaxPacketSize, (p).wMaxPacketSize1, (p).bInterval)

#define UNKNOWN_USB_DEVICE 0xF1
#define UNKNOWN_USB_PRINT 0xF2

unsigned char init_usb_device()
{ /* 初始化USB打印机,完成打印机枚举 */
#define p_dev_descr ((PUSB_DEV_DESCR)buffer)
#define p_cfg_descr ((PUSB_CFG_DESCR_LONG)buffer)
  unsigned char status, len, c;
  status = get_descr(1); /* 获取设备描述符 */
  puts("get_descr 1");
  if (status == USB_INT_SUCCESS)
  {
    puts("get_descr 1 ok");
    len = rd_usb_data(buffer); /* 将获取的描述符数据从CH375中读出到单片机的RAM缓冲区中,返回描述符长度 */
    if (len < 18 || p_dev_descr->bDescriptorType != 1)
      return (UNKNOWN_USB_DEVICE); /* 意外错误:描述符长度错误或者类型错误 */
    printf("---p_dev_descr----\n");
    PRINT_PUSB_DEV_DESCR(*p_dev_descr);
    if (p_dev_descr->bDeviceClass != 0)
      return (UNKNOWN_USB_DEVICE); /* 连接的USB设备不是USB打印机,或者不符合USB规范 */
    status = set_addr(3);          /* 设置打印机的USB地址 */
    if (status == USB_INT_SUCCESS)
    {
      puts("get_descr 2");
      status = get_descr(2); /* 获取配置描述符 */
      if (status == USB_INT_SUCCESS)
      { /* 操作成功则读出描述符并分析 */
        puts("get_descr 2 ok");
        len = rd_usb_data(buffer); /* 将获取的描述符数据从CH375中读出到单片机的RAM缓冲区中,返回描述符长度 */
        printf("---cfg_descr----\n");
        PRINT_PUSB_CFG_DESCR(p_cfg_descr->cfg_descr);
        printf("----itf_descr---\n");
        PRINT_PUSB_ITF_DESCR(p_cfg_descr->itf_descr);

        for (int i = 0; i < p_cfg_descr->itf_descr.bNumEndpoints && i < 10; ++i)
        {
          if (!p_cfg_descr->endp_descr[i].bLength)
            break;
          printf("----Endpoint %d---\n", i);
          PRINT_PUSB_ENDP_DESCR(p_cfg_descr->endp_descr[i]);
        }

        if (p_cfg_descr->itf_descr.bInterfaceClass != 7 || p_cfg_descr->itf_descr.bInterfaceSubClass != 1)
          return (UNKNOWN_USB_PRINT); /* 不是USB打印机或者不符合USB规范 */
        endp_out_addr = endp_in_addr = 0;
        c = p_cfg_descr->endp_descr[0].bEndpointAddress; /* 第一个端点的地址 */
        if (c & 0x80)
          endp_in_addr = c & 0x0f; /* IN端点的地址 */
        else
        { /* OUT端点 */
          endp_out_addr = c & 0x0f;
          endp_out_size = p_cfg_descr->endp_descr[0].wMaxPacketSize; /* 数据接收端点的最大包长度 */
        }
        if (p_cfg_descr->itf_descr.bNumEndpoints >= 2)
        { /* 接口有两个以上的端点 */
          if (p_cfg_descr->endp_descr[1].bDescriptorType == 5)
          {                                                  /* 端点描述符 */
            c = p_cfg_descr->endp_descr[1].bEndpointAddress; /* 第二个端点的地址 */
            if (c & 0x80)
              endp_in_addr = c & 0x0f; /* IN端点 */
            else
            { /* OUT端点 */
              endp_out_addr = c & 0x0f;
              endp_out_size = p_cfg_descr->endp_descr[1].wMaxPacketSize;
            }
          }
        }
        if (p_cfg_descr->itf_descr.bInterfaceProtocol <= 1)
          endp_in_addr = 0; /* 单向接口不需要IN端点 */
        if (endp_out_addr == 0)
          return (UNKNOWN_USB_PRINT);                                    /* 不是USB打印机或者不符合USB规范 */
        status = set_config(p_cfg_descr->cfg_descr.bConfigurationValue); /* 加载USB配置值 */
        if (status == USB_INT_SUCCESS)
        {
          write_cmd(CMD_SET_RETRY); /* 设置USB事务操作的重试次数 */
          write_data(0x25);
          write_data(0x89); /* 位7为1则收到NAK时无限重试, 位3~位0为超时后的重试次数 */
                            /* 如果单片机在打印机忙时并无事可做,建议设置位7为1,使CH375在收到NAK时自动重试直到操作成功或者失败 */
                            /* 如果希望单片机在打印机忙时能够做其它事,那么应该设置位7为0,使CH375在收到NAK时不重试,
   所以在下面的USB通讯过程中,如果USB打印机正忙,issue_token等子程序将得到状态码USB_INT_RET_NAK */
        }
      }
    }
  }
  return (status);
}

void reset_usb()
{
  ch375_reset();
  ch375_host();
  if (!ch375_exist())
  {
    printf("CH375 Not Exist!\n");
  }
  write_cmd(CMD_GET_IC_VER);
  // printf("CH375 Version: %02X\n", read_data());
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("init...");

  init_ch375();
  reset_usb();

  Serial.println("init ok!!!");
}

uint8_t v = 0;
void loop()
{
  // delay(500);
  char c = Serial.read();
  switch (c)
  {
  case 'a':
  case 'A':
    reset_usb();
    break;
  case 'b':
  case 'B':
    ch375_test_connect();
    break;
  case 'c':
  case 'C':
  {
    uint8_t s = init_usb_device();
    printf("init_usb_device=%02X\n", s);
    break;
  }
  default:
    break;
  }
  // printf("----------%02X\n", v);
  // write_cmd(0x06);
  // write_data(v);
  // // write(v);
  // delay(2);
  // read_data();
  // printf("R:%d\n", digitalRead(INT));
  // write_cmd(0x05);
  // printf("R:%d\n", digitalRead(INT));
  if (ch375_interrupt())
  {
    uint8_t status = ch375_get_status();
    printf("STATUS: %02X\n", status);
    usb_delay_us(2);
    if (status == USB_INT_CONNECT)
    {
      // USB设备连接
      // write_cmd(0x51);
      delay(1000);
      init_usb_device();
    }
    if (status == USB_INT_SUCCESS)
    {
      // 获取磁盘空间
      write_cmd(0x53);
    }
    if (status == USB_INT_DISCONNECT)
    {
      // 断开
      ch375_set_model(0x07);
      usb_delay_us(20);
    }
    if (status == 0x20)
    {
      reset_usb();
    }
  }
  else if (ch375_test_connect() == USB_INT_DISCONNECT)
  {
    // USB断开
    delay(1000);
    reset_usb();
  }
  ++v;
}
