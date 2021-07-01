/***************************************************** 
 - LED Alram
   · 전원 On               => Red LED On
   · IMU 초기화 종료         => Red LED Off
   · While루프 동작 중       => Yellow LED On
   · Gain 값 변경           => Red LED 깜빡임
****************************************************/

#define F_CPU 16000000UL  // 16MHz
#include <mega128.h>
#include <stdio.h>
#include <math.h>

#define LED0 PORTA.0
#define LED1 PORTA.1
#define SPI_SS_Low()   PORTB.0=0
#define SPI_SS_High()  PORTB.0=1

#define sbus_head     ~(0xF0)     // S-BUS Header
#define user_head     0xFDFEFF    // User Protocol Header
#define sbus_size     25          // S-BUS = 25Byte
#define imu_size      59          // IMU = 59Byte
#define pwm_tx_size   31          // To PWM Generator (31Byte)
#define spi_size      78          // From/To AVR1_SPI (78Byte)

#define u0_size     (sbus_size*3)   // S-BUS Buffer Size
#define u1_size     (imu_size*3)    // IMU Buffer Size

#define DEF_HEADINIT(u)         stUART[(u)].Packet_head = stUART[(u)].hIDX = 0x00  // Head, Index 초기화
#define DEF_Rshift(a, u)        ( (a)>>8*u )
#define DEF_Lshift(a, u)        ( (a)<<8*u )
#define DEF_pCONV_int(a)        ( (*(int*)(a)) )            // 2Byte
#define DEF_pCONV_float(a)      ( (*(float*)(a)) )          // 4Byte - 실수
#define DEF_pCONV32bit(a)       ( (*(unsigned long*)(a)) )
#define DEF_pow(a)              ( a*a )
#define DEF_Deg2Rad(a)          ( a*PI/180 )
#define DEF_Rad2Deg(a)          ( a*180/PI )

#define PACKET_ID_CNT   0xFFFF      // DataPacket ID 최대갯수
#define PACKET_ID_SPI   0x0057      // SPI ID 체크용 (From AVR1)
#define MIN_DataPacket  25          // 수신되는 DataPacket 중 최소 Byte수

// Controller Define
#define SBUS_max        1696
#define SBUS_mid        1024
#define SBUS_min        352
#define maxStickAng     30         // Max Angle[Degree]
#define Motor_N         1000        // 1000~2000 [us] 
#define length          0.25        // 1-3 모터간 거리 [m]

// Usable Variable 
enum uIDX {u0=0, u1, u2};   // Uart Number
struct structUART
{
    unsigned char buffer_size;
    unsigned char rx_head;
    unsigned char rx_tail;
    
    unsigned char Packet_head;
    unsigned char Packet_size;
    unsigned char hIDX;
};  
struct structUART stUART[3];
unsigned char rx0_buffer[u0_size];
unsigned char rx1_buffer[u1_size];
unsigned char rx0_Packet[u0_size];
unsigned char rx1_Packet[u1_size];
unsigned char rx_spi[spi_size]={0};  // from AVR1

// Communication Variable
char pwm_Tx[31]={0};                 // to PWM Generator  
char tx_spi[spi_size]={0};           // to AVR1
volatile unsigned char tx_cnt=0, tx_flag=0, cnt=0;

// Decode Variable 
unsigned int sbus[8]={0};   // Data Channel 1-8
float q[4] = {0};
float Quat[3]={0};          // IMU - Quat1, Quat2, Quat3
float Gyro[3]={0};          // IMU - GyroX, GyroY, GyroZ 

// Control Variable
float roll=0, pitch=0, yaw=0;       // Euler [Radian]
float Pctrl[3]={0}, Ictrl[3]={0}, Dctrl[3]={0};
float Control_input[3]={0};            // PID control value
float Motor[4]={0}, pwm_ch5=0, pwm_ch6=0, pwm_ch7=0;


// Control Gain
float SBUS_gain=0.74405;        // SBUS PWM[0-1344] -> Motor PWM[0-1000]
float Cont_gain=1;              // Stick Control Gain
float xPIDgain[3]={0.53, 0, 0.28};    // X axis P-I-D
float yPIDgain[3]={0.56, 0, 0.32};        // Y axis P-I-D
float zPIDgain[3]={0, 0, 0.5};       // Z axis P-I-D

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void MCU_init(void);
void UART_StructSet(void);
void Uart_EnQueue(unsigned char Ux, unsigned int data);
void increase_point_value(unsigned char Ux, unsigned char *data_p);
char Uart_DeQueue(unsigned char Ux);
_Bool Uart_Is_Empty(unsigned char Ux);
void SBUS_PacketDecode(void);
void SBUS_EnQueue(unsigned char Ux, char data);
void Packet_EnQueue(unsigned char Ux, char data);
_Bool PacketHeaderCheck(unsigned char* pBuff);
_Bool PacketCheckSum(unsigned char* data, unsigned char Buff_size);
void PacketDecode(unsigned char* pPacket, unsigned char Buff_size);
void SPI_Decode(unsigned char* pPacket);
_Bool PacketEnderCheck(unsigned char* pBuff);
void SPI_EnQueue(unsigned char Ux, char data);

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void Control(void);
void Quaternion2Euler();        // Quaternion to Euler [rad]
float PID_Control(int N, float Command, float state, float derivative, float* Gain);
float Limit_PWM(float pwm);

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void Servo_Tx(char pSize);
void SPI_Packet(unsigned char pSize);
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void UART0_putch(char ch) { while(!(UCSR0A&0x20)); UDR0 = ch; }               // Atmega -> PWM_Generator (1Byte 송신) 
void SPI_Tx(char data) { SPI_SS_Low(); SPDR = data; while(!(SPSR & 0x80)); }   // AVR2 -> AVR1 1Byte 송신
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void MCU_init(void)
{
    // LED Port
    DDRA  = 0xFF;
    PORTA = 0x0;


    // USART0 : RX_SBUS & TX_PWM Generator
    UCSR0A = 0X00;
    UCSR0B = 0X98;
    UCSR0C = 0X2E;
    UBRR0H = 0;
    UBRR0L = 9;

    // USART1 : RX_IMU
    UCSR1A = 0x00;
    UCSR1B = 0x98;
    UCSR1C = 0x06;
    UBRR1H = 0;
    UBRR1L = 8;

    // SPI Port _ Master    
    SPI_SS_High();
    DDRB = 0x07;    // MOSI, SCK, /SS
    SPCR = 0xD1;     // SPIE, SPE, MSTR, fck/16, spdx1

    // Timer1 A매치 - 16000000 / 1(1+15999) = 1000Hz = 1ms
    // SPI time between 1byte
    TCCR1B = 0x09;
    OCR1A  = 15999;
    TIMSK  = 0x10;
    TCNT1  = 0x00;

    // Timer3 A매치 - 16000000 / 256(1+6249) = 10Hz = 100ms
    // SPI Update rate
    TCCR3B = 0x0C;
    OCR3AH = 6249>>8;
    OCR3AL = 6249&0xFF;
    ETIMSK = 0x10;
}

void UART_StructSet(void)
{
    int i = 0;
    for(i=0;i<3;i++)
    {
        stUART[i].rx_head = 0; 
        stUART[i].rx_tail = 0;
        stUART[i].Packet_head = 0;
        stUART[i].hIDX = 0;   
    }
    stUART[0].Packet_size = sbus_size;
    stUART[1].Packet_size = imu_size;     
    stUART[2].Packet_size = spi_size; 
    
    stUART[0].buffer_size = u0_size;
    stUART[1].buffer_size = u1_size; 
}

void Uart_EnQueue(unsigned char Ux, unsigned int data)
{
    switch(Ux)
    {
        case 0: // Uart0
                rx0_buffer[stUART[Ux].rx_head] = (unsigned char)(data&0xFF);
                break;
            
        case 1: // Uart1
                rx1_buffer[stUART[Ux].rx_head] = (unsigned char)(data&0xFF);
                break; 
    }
    increase_point_value(Ux, &(stUART[Ux].rx_head)); // head 증가    
}

void increase_point_value(unsigned char Ux, unsigned char *data_p)
{
    (*data_p)++;
    
    if( (*data_p) >= stUART[Ux].buffer_size )
        (*data_p) = 0;
}

char Uart_DeQueue(unsigned char Ux)
{
    char val0=0, val1=0;
    
    switch(Ux)
    {
        case 0: // Uart0
        {
            val0 = rx0_buffer[stUART[Ux].rx_tail];
            increase_point_value(Ux, &(stUART[Ux].rx_tail)); // tail 증가
            return val0;
        }
        case 1: // Uart1
        {   
            val1 = rx1_buffer[stUART[Ux].rx_tail];
            increase_point_value(Ux, &(stUART[Ux].rx_tail)); // tail 증가
            return val1;
        }
    }
}

_Bool Uart_Is_Empty(unsigned char Ux)
{
    unsigned char head, tail;
    head = stUART[Ux].rx_head;
    tail = stUART[Ux].rx_tail;
    
    if(head!=tail)
        return 1;
    
    return 0;
}  

void SBUS_EnQueue(unsigned char Ux, char data)
{
    rx0_Packet[stUART[Ux].Packet_head++] = data;
    
    switch(stUART[Ux].hIDX)
    {
        case 0:     // Header
            if(rx0_Packet[stUART[Ux].hIDX]==sbus_head)   stUART[Ux].hIDX++;
            else    DEF_HEADINIT(Ux);
            break; 
        
        case 23:    // Flags
            if((rx0_Packet[stUART[Ux].hIDX]==0x00)||(rx0_Packet[stUART[Ux].hIDX]==0x0C)||(rx0_Packet[stUART[Ux].hIDX]==0x03))   stUART[Ux].hIDX++;
            else    DEF_HEADINIT(Ux);
            break;        
        
        case 24:    // EndByte
            if(rx0_Packet[stUART[Ux].hIDX]==0x00)   stUART[Ux].hIDX++;
            else    DEF_HEADINIT(Ux);
            break;
        
        default:    // Case 1 ~ Case 22
            stUART[Ux].hIDX++;
            break; 
    }
    
    if(stUART[Ux].Packet_head >= stUART[Ux].Packet_size)
    {   
        SBUS_PacketDecode();
        DEF_HEADINIT(Ux);        
    }
}

void SBUS_PacketDecode(void)
{        
    sbus[0] = ( ((int)rx0_Packet[2]<< 8) + ((int)rx0_Packet[1]) ) & 0x07FF;                                        // Ch1 Aileron (11bit)    
    sbus[1] = ( ((int)rx0_Packet[3]<< 5) + ((int)rx0_Packet[2]>>3) ) & 0x07FF;                                    // Ch2 Elevator
    sbus[2] = ( ((int)rx0_Packet[5]<<10) + ((int)rx0_Packet[4]<<2) + ((int)rx0_Packet[3]>>6) ) & 0x07FF;        // Ch3 Throttle
    sbus[3] = ( ((int)rx0_Packet[6]<< 7) + ((int)rx0_Packet[5]>>1) ) & 0x07FF;                                     // Ch4 Rudder
    sbus[4] = ( ((int)rx0_Packet[7]<< 4) + ((int)rx0_Packet[6]>>4) ) & 0x07FF;
    sbus[5] = ( ((int)rx0_Packet[9]<< 9) + ((int)rx0_Packet[8]<<1) + ((int)rx0_Packet[7]>>7) ) & 0x07FF;
    sbus[6] = ( ((int)rx0_Packet[10]<<6) + ((int)rx0_Packet[9]>>2) ) & 0x07FF;
    sbus[7] = ( ((int)rx0_Packet[11]<<3) + ((int)rx0_Packet[10]>>5) ) & 0x07FF;                                    // Ch8 
}

void Packet_EnQueue(unsigned char Ux, char data)
{
    int tmpID = 0x00;
    rx1_Packet[stUART[Ux].Packet_head++] = data;
    
    switch(stUART[Ux].hIDX)
    {
        case 0:     // Header #1
        case 1:     // Header #2
        case 2:     // Header #3      
            if(rx1_Packet[stUART[Ux].hIDX] == ((char)((DEF_Rshift(user_head, stUART[Ux].hIDX))&0xFF))) 
                stUART[Ux].hIDX++;      
            else    DEF_HEADINIT(Ux);
            break;
        
        case 3:     // Packet Length
            if(rx1_Packet[stUART[Ux].hIDX] > MIN_DataPacket)
            {
                stUART[Ux].hIDX++;
                stUART[Ux].Packet_size = data;
            }
            else    DEF_HEADINIT(Ux);
            break;
            
        case 4:     // Packet ID #1(L)
            stUART[Ux].hIDX++;
            tmpID = (int)data;            
            break;
        
        case 5:     // Packet ID #2(R)
            tmpID = tmpID + ((int)(data<<8)); 
            if(tmpID < PACKET_ID_CNT)    stUART[Ux].hIDX++;
            else    DEF_HEADINIT(Ux);
            break;
        default :
            stUART[Ux].hIDX++;
            break;
    }    
   
    if(stUART[Ux].Packet_head >= stUART[Ux].Packet_size)
    {   
        PacketDecode(rx1_Packet, stUART[Ux].Packet_size);         
        DEF_HEADINIT(Ux);       
    }
}

_Bool PacketHeaderCheck(unsigned char* pBuff)
{
    unsigned long tmpHead = 0x00;

    tmpHead = DEF_pCONV32bit(pBuff); 
    if( (tmpHead&0x00FFFFFF) == 0xFDFEFF )  return 1;
    
    return 0;
} 

_Bool PacketCheckSum(unsigned char* data, unsigned char Buff_size)
{
    int i =0;
    unsigned char bShiftCheck = 0x00;
    unsigned char CheckSumData = *(data + (Buff_size-1));    //CheckSum Data
    
    for(i=0;i<(Buff_size-1);i++)    bShiftCheck ^= *(data + i);
    if(CheckSumData == bShiftCheck) return 1;
    
    return 0;
}

void PacketDecode(unsigned char* pPacket, unsigned char Buff_size)
{
    int i = 0;
    if(PacketHeaderCheck(pPacket))  // Header 확인
    {
        if(PacketCheckSum(pPacket, Buff_size))  // CheckSum 확인
        {   
            for(i=0;i<Buff_size-7;i++)
                tx_spi[23+i] = rx1_Packet[6+i];  // Byte 수신
            
            for(i=0;i<3;i++)
            {
                Quat[i] = DEF_pCONV_float(pPacket+(4*i+6));
                Gyro[i] = DEF_pCONV_int(pPacket+(2*i+20));  // Degree
            }                                                                           
            if( (Quat[0]!=0)&&(Quat[1]!=0)&&(Quat[2]!=0) ) LED0 = 0;   // Red off after moment
        }
    }
}

void SPI_EnQueue(unsigned char Ux, char data)
{
    int tmpID=0;
    rx_spi[stUART[Ux].Packet_head++] = data&0xFF;
    
    switch(stUART[Ux].hIDX)
    {
        case 0:     // Header #1
        case 1:     // Header #2
        case 2:     // Header #3      
            if(rx_spi[stUART[Ux].hIDX] == ((char)((DEF_Rshift(user_head, stUART[Ux].hIDX))&0xFF))) 
                stUART[Ux].hIDX++;      
            else    DEF_HEADINIT(Ux);
            break;
        case 3:     // Packet Length
            if(rx_spi[stUART[Ux].hIDX] > MIN_DataPacket)
            {
                stUART[Ux].hIDX++;
                stUART[Ux].Packet_size = data;
            }
            else    DEF_HEADINIT(Ux);
            break;     
        case 4:     // Packet ID #1(L)
            stUART[Ux].hIDX++;
            tmpID = (int)data;            
            break;
        case 5:     // Packet ID #2(R)
            tmpID = (tmpID<<8) + (int)data&0xFF; 
            if(tmpID == PACKET_ID_SPI)    stUART[Ux].hIDX++; // SPI ID 체크
            else    DEF_HEADINIT(Ux);
            break;
        default :
            stUART[Ux].hIDX++;
            break;
    } 
    if(stUART[Ux].Packet_head >= stUART[Ux].Packet_size)
    {
        SPI_Decode(rx_spi);       
        DEF_HEADINIT(Ux);            
    }
}

_Bool PacketEnderCheck(unsigned char* pBuff)
{
    unsigned long tmpHead = 0x00;

    tmpHead = DEF_pCONV32bit(pBuff+75); 
    if( (tmpHead&0x00FFFFFF) == 0xFAFBFC )  return 1;
    
    return 0;
} 

void SPI_Decode(unsigned char* pPacket)
{
    int i=0;
    int xgain[3]={0}, ygain[3]={0}, zgain[3]={0};
    if(PacketHeaderCheck(pPacket))      // Header 확인
    {                        
        if(PacketEnderCheck(pPacket))   // Ender 확인
        {  
            if(rx_spi[6]==0xAA)     // Gain 변경 Flag
            {            
                LED0 = 1;           // Red On       
                for(i=0;i<3;i++)
                {
                     xgain[i] = DEF_pCONV_int(pPacket+2*i+7);
                     ygain[i] = DEF_pCONV_int(pPacket+2*i+13);
                     zgain[i] = DEF_pCONV_int(pPacket+2*i+19);
                     xPIDgain[i] = (float)xgain[i];
                     yPIDgain[i] = (float)ygain[i];
                     zPIDgain[i] = (float)zgain[i];  
                }
            }
        }
    }   
}

void Control(void)
{
    int  i = 0;
    float tmp[4]     = {0};
    float REF_Thrust;
    float Thrust[4] = {0};
    float Command[3]= {0};
    float Roll_ctrl, Pitch_ctrl, Yaw_ctrl;
    
    Quaternion2Euler();
    
    // SBUS 값 튜닝
    tmp[0] = (+1*((float)sbus[0]-SBUS_mid)) + (SBUS_mid-SBUS_min); // 0~1344 Aileron(Roll)
    tmp[1] = (+1*((float)sbus[1]-SBUS_mid)) + (SBUS_mid-SBUS_min); // 0~1344 Elevator(Pitch)
    tmp[2] = (-1*((float)sbus[2]-SBUS_mid)) + (SBUS_mid-SBUS_min); // 0~1344 Throttle
    tmp[3] = (+1*((float)sbus[3]-SBUS_mid)) + (SBUS_mid-SBUS_min); // 0~1344 Rudder(Yaw)
    
   
         
    
    // 조종기 ch5 up
    if(sbus[4]<1000)
    {
        if(tmp[2]<100)
        {
            for(i=0;i<4;i++) Motor[i] = 1000;
        }        
        
        else{
        // Command 값 계산
        REF_Thrust = tmp[2]*SBUS_gain + Motor_N;
        Command[0] = (float)maxStickAng/672*(tmp[0]-672);        // Roll Command
        Command[1] = (float)maxStickAng/672*(tmp[1]-672);        // Pitch Command
        Command[2] = (float)maxStickAng/672*(tmp[3]-672);        // Yaw Command        
        //Command[0] = 30; 
    
        // Attitude Control
        Roll_ctrl  = PID_Control(0, Command[0], roll,  Gyro[0], xPIDgain);
        Pitch_ctrl = PID_Control(1, Command[1], pitch, Gyro[1], yPIDgain);
        Yaw_ctrl   = PID_Control(2, Command[2], yaw,   Gyro[2], zPIDgain);  
    
    
                                           
        Thrust[0] = REF_Thrust + Roll_ctrl + Pitch_ctrl - Yaw_ctrl;
        Thrust[1] = REF_Thrust - Roll_ctrl + Pitch_ctrl + Yaw_ctrl;
        Thrust[2] = REF_Thrust - Roll_ctrl - Pitch_ctrl - Yaw_ctrl;
        Thrust[3] = REF_Thrust + Roll_ctrl - Pitch_ctrl + Yaw_ctrl;
    
        for(i=0;i<4;i++) Thrust[i] = Limit_PWM(Thrust[i]);
        for(i=0;i<4;i++) Motor[i] = Thrust[i];    
        }
        
    }
    
    // 조종기 ch5 down
    else
    {    for(i=0;i<4;i++) Motor[i] = 1000;}
   
   
   //printf("Motor[0] = %d\n\r",Motor[0]);
   
    // Aileron PWM
    pwm_ch5 = (tmp[0]*SBUS_gain) + Motor_N;        // 1000~2000us
    // Elevator PWM
    pwm_ch6 = (tmp[1]*SBUS_gain) + Motor_N;
    // Rudder PWM
    pwm_ch7 = (tmp[3]*SBUS_gain) + Motor_N;


    Servo_Tx(pwm_tx_size);  // 50Hz(20ms) PWM Transmit
    SPI_Packet(spi_size);   // SPI Transmit Packet 생성
}

void Quaternion2Euler()        // Quaternion to Euler [rad]
{
    int i = 0;
    float a,b,c,d,e;   
    
   //Quaternion to Euler Conversion [rad]
    
    for (i=0; i<3; i++){
    q[i+1]=Quat[i];
    }
    
    q[0] = sqrt(1 - q[1]*q[1] - q[2]*q[2] - q[3]*q[3]); //q0^2 + q1^2 + q2^2 +q3^2 = 1
    
    a = 2*(q[0]*q[1] + q[2]*q[3]);
    b = 1 - 2*(q[1]*q[1] + q[2]*q[2]);
    c = 2*(q[0]*q[2] - q[3]*q[1]);
    d = 2*(q[0]*q[3] + q[1]*q[2]);
    e = 1 - 2*(q[2]*q[2] + q[3]*q[3]);  
     
    roll = atan2(a,b);
    pitch = asin(c);
    yaw = atan2(d,e);
   
   roll = DEF_Rad2Deg(roll);
   pitch = DEF_Rad2Deg(pitch);
   yaw = DEF_Rad2Deg(yaw);
}

float PID_Control(int N, float Command, float state, float derivative, float* Gain)
{
    float err=0;
    float dt=0.020;
    
    err = Command - state;  
    
    Pctrl[N] = Gain[0]*err;
    Ictrl[N] = Ictrl[N] + Gain[1]*err*dt;
    Dctrl[N] = Gain[2]*(Command-derivative/(72.8));
    
    Control_input[N] = Pctrl[N]+Ictrl[N]+Dctrl[N];
    return Control_input[N];
}

float Limit_PWM(float pwm)
{
    if(pwm<1000) {pwm = 1000;}
    if(pwm>1800) {pwm = 1800;}
   return pwm;
}

void Servo_Tx(char pSize)
{
    int i = 0;
    int tmp = 1000;
    char chk = 0x00;
        
    pwm_Tx[0] = 0xFF;              // Header #1
    pwm_Tx[1] = 0xFE;              // Header #2
    pwm_Tx[2] = 0xFD;              // Header #3
    pwm_Tx[3] = pSize;             // Packet Size
    pwm_Tx[4] = 0x00;              // ID #1
    pwm_Tx[5] = 0x53;              // ID #2 

    pwm_Tx[6] = tmp&0xFF;    
    pwm_Tx[7] = DEF_Rshift(tmp, 1)&0xFF;    // PWM J1 - Null
    pwm_Tx[8] = (int)Motor[0]&0xFF;    
    pwm_Tx[9] = ((int)Motor[0]>>8)&0xFF;    // PWM J2 - Motor 1  
    pwm_Tx[10] = (int)Motor[1]&0xFF;         
    pwm_Tx[11] = ((int)Motor[1]>>8)&0xFF;   // PWM J3 - Motor 2 
    pwm_Tx[12] = (int)Motor[2]&0xFF;         
    pwm_Tx[13] = ((int)Motor[2]>>8)&0xFF;   // PWM J4 - Motor 3 
    pwm_Tx[14] = (int)Motor[3]&0xFF;
    pwm_Tx[15] = ((int)Motor[3]>>8)&0xFF;   // PWM J5 - Motor 4     
    pwm_Tx[16] = (int)pwm_ch5&0xFF;
    pwm_Tx[17] = ((int)pwm_ch5>>8)&0xFF;    // PWM J6 - Ch5
    pwm_Tx[18] = (int)pwm_ch6&0xFF;    
    pwm_Tx[19] = ((int)pwm_ch6>>8)&0xFF;    // PWM J7 - Ch6
    pwm_Tx[20] = (int)pwm_ch7&0xFF; 
    pwm_Tx[21] = ((int)pwm_ch7>>8)&0xFF;    // PWM J8 - Ch7

    pwm_Tx[22] = tmp&0xFF;    
    pwm_Tx[23] = DEF_Rshift(tmp, 1)&0xFF;    // PWM J9 - Null
    pwm_Tx[24] = tmp&0xFF;    
    pwm_Tx[25] = DEF_Rshift(tmp, 1)&0xFF;    // PWM J10 - Null
    pwm_Tx[26] = tmp&0xFF;    
    pwm_Tx[27] = DEF_Rshift(tmp, 1)&0xFF;    // PWM J11 - Null
    pwm_Tx[28] = tmp&0xFF;    
    pwm_Tx[29] = DEF_Rshift(tmp, 1)&0xFF;    // PWM J12 - Null


    for(i=0;i<pSize-1;i++)         // CheckSum 계산(XOR)
        chk ^= pwm_Tx[i];
    pwm_Tx[30] = chk;     
    
    for(i=0;i<pSize;i++)           // UART0 Transmit (to.PWM Generator)
        UART0_putch(pwm_Tx[i]);    // 50hz(20ms) Transmit
}

void SPI_Packet(unsigned char pSize)
{
    int i = 0; 
    tx_spi[0] = 0xFF;                // Header #1
    tx_spi[1] = 0xFE;                // Header #2
    tx_spi[2] = 0xFD;                // Header #3
    tx_spi[3] = pSize;               // Packet Size
    tx_spi[4] = 0x00;                // ID #1
    tx_spi[5] = 0x55;                // ID #2                 
            
    for(i=0;i<4;i++)
    {
        tx_spi[2*i+6] = (int)Motor[i]&0xFF;    
        tx_spi[2*i+7] = ((int)Motor[i]>>8)&0xFF;   // Motor 1 ~ 4
    }
    tx_spi[14] = (int)pwm_ch5&0xFF;
    tx_spi[15] = ((int)pwm_ch5>>8)&0xFF;   // Ch5
    tx_spi[16] = (int)pwm_ch6&0xFF;
    tx_spi[17] = ((int)pwm_ch6>>8)&0xFF;   // Ch6            
    tx_spi[18] = (int)pwm_ch7&0xFF;
    tx_spi[19] = ((int)pwm_ch7>>8)&0xFF;   // Ch7
    tx_spi[20] = sbus[7]&0xFF;              
    tx_spi[21] = (sbus[7]>>8)&0xFF;        // Ch8
    tx_spi[22] = rx0_Packet[23];           // S-Bus Flag

   tx_spi[30] = (int)roll&0xFF;
   tx_spi[31] = ((int)roll>>8)&0xFF;
   tx_spi[32] = (int)pitch&0xFF;
   tx_spi[33] = ((int)pitch>>8)&0xFF;
   tx_spi[34] = (int)yaw&0xFF;
   tx_spi[35] = ((int)yaw>>8)&0xFF;

    
    tx_spi[75] = 0xFC;  // Ender #1
    tx_spi[76] = 0xFB;  // Ender #2
    tx_spi[77] = 0xFA;  // Ender #3  
}

void main(void)
{
    MCU_init();
    UART_StructSet();
    
    LED0 = 1;       // Red On   
    SREG = 0x80;    // Interrupt enable
        
    while (1)
    {   
        LED1 = 1;   // Yellow On
        while(Uart_Is_Empty(u0))    SBUS_EnQueue(u0, Uart_DeQueue(u0));     // SBUS Decode
        while(Uart_Is_Empty(u1))    Packet_EnQueue(u1, Uart_DeQueue(u1));   // IMU Decode
        Control();
    }
}

interrupt [USART0_RXC] void usart0_rx_isr(void){ char sbus_raw=UDR0; Uart_EnQueue(u0, sbus_raw); }  // SBUS Rx Interrupt
interrupt [USART1_RXC] void usart1_rx_isr(void){ char imu_raw=UDR1; Uart_EnQueue(u1, imu_raw); }    // IMU Rx Interrupt
interrupt [TIM3_COMPA] void timer3_init(void)     // 5Hz Transmit (SPI start)
{
    cnt+=1;
    if(cnt==2)
    {    
        tx_flag=1;
        cnt=0;
    }    
}


interrupt [TIM1_COMPA] void timer1_initv(void)                   // 1ms Transmit (SPI 1Byte씩 송신)
{   
    if(tx_flag==1)
    {               
        SPI_Tx(tx_spi[tx_cnt++]);   // Data Transmit_SPI Communication        
        if(tx_cnt>=spi_size)        // tx_cnt >= 78
        {
            tx_cnt=0;
            tx_flag=0;
        }
    }
}

interrupt [SPI_STC] void spi_isr(void)
{
    char tmp = SPDR;
    SPI_SS_High();          // SPI 통신 종료
    SPI_EnQueue(u2, tmp);
}