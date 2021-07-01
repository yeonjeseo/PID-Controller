/*****************************************************
 - LED Alram
   · 전원 On               => Red LED On
   · While루프 동작 중     => Yellow LED On
   · 전원 11.1V 이하       => Red LED Off
****************************************************/ 

#define F_CPU 16000000UL  // 16MHz
#include <mega128.h>
#include <stdio.h>

#define LED0 PORTA.0
#define LED1 PORTA.1

#define user_head    0xFDFEFF        // User Protocol Header
#define uart0_size   25              // GCS
#define uart1_size   23              // ADC (8Port)
#define spi_size     78              // From/To AVR2

#define u0_size      (uart0_size*3)  // UART0 Buffer Size
#define u1_size      (uart1_size*3)  // UART1 Buffer Size
#define u2_size      (spi_size*3)    // SPI Buffer Size

#define imu_size     59              // IMU
#define gcs_tx_size  44              // To GCS

#define DEF_HEADINIT(u)         stUART[(u)].Packet_head = stUART[(u)].hIDX = 0x00  // Head, Index 초기화
#define DEF_Rshift(a, u)        ( (a)>>8*u )
#define DEF_Lshift(a, u)        ( (a)<<8*u )
#define DEF_CONV8bit(a)         ( (char)((a)&0xFF)) )
#define DEF_pCONV16bit(a)       ( (*(unsigned int*)(a)) )   // 2Byte
#define DEF_pCONV32bit(a)       ( (*(unsigned long*)(a)) )  // 4Byte - 정수
#define DEF_pCONV_int(a)        ( (*(int*)(a)) )            // 2Byte
#define DEF_pCONV_float(a)      ( (*(float*)(a)) )          // 4Byte - 실수

#define PACKET_ID_CNT   0xFFFF      // DataPacket ID 최대갯수
#define PACKET_ID_SPI   0x0055      // SPI ID 체크용 (From AVR2)
#define MIN_DataPacket  23          // 수신되는 DataPacket 중 최소 Byte수

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
unsigned char rx2_buffer[u2_size];

unsigned char rx0_Packet[uart0_size];   // GCS Receive data
unsigned char rx1_Packet[uart1_size];   // ADC Receive data
unsigned char rx2_Packet[spi_size];  // From AVR2

// Communication Variable
char tx_spi[spi_size]={0};                            // To AVR2
char imu_tx[imu_size]={0}, gcs_tx[gcs_tx_size]={0};   // To GCS
volatile unsigned char spi_tx_cnt=0;

// Analog Digital Conversion
float Battery=0;

// user variable

float Att[3]={0};



// User Function
void MCU_init(void);
void UART_StructSet(void);
void Uart_EnQueue(unsigned char Ux, unsigned int data);
void increase_point_value(unsigned char Ux, unsigned char *data_p);
char Uart_DeQueue(unsigned char Ux);
_Bool Uart_Is_Empty(unsigned char Ux);
void Packet_EnQueue(unsigned char Ux, char data, unsigned char pSize);
_Bool PacketCheckSum(unsigned char* data, unsigned char Buff_size);
_Bool PacketHeaderCheck(unsigned char* pBuff);
void SPI_Decode(unsigned char* pPacket);
void PacketDecode(unsigned char Ux, unsigned char* pPacket, unsigned char Buff_size);
void spi_Tx_Packet(char pSize);
void GCS_Tx_Packet(char pSize1, char pSize2);
void Batt_CHK(void);

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
void UART0_putch(char ch) { while(!(UCSR0A&0x20)); UDR0 = ch; }                 // Atmega -> GCS (1Byte 송신)

void MCU_init(void)
{
    // LED Port
    DDRA = 0xFF;
    PORTA = 0x00;

    // SPI Port _ Slave    
    DDRB = 0x08;
    SPCR = 0xC1;        
    
    // UART0 Reg _ GCS (RxTx)    
    UCSR0A = 0X00;
    UCSR0B = 0X18;
    UCSR0C = 0X06;  // 8 Data, 1 Stop, No Parity
    UBRR0H = 0;
    UBRR0L = 8;     // baudrate 115200bps

    // UART1 Reg _ ADC (Rx)
    UCSR1A = 0X00;
    UCSR1B = 0X98;
    UCSR1C = 0X06;  // 8 data, 1 stop, no parity
    UBRR1H = 0;
    UBRR1L = 8;     // baudrate 115200bps

    // ADC Reg
   ADMUX = 0x00;
   ADCSRA = 0xe7; // 0b11000111; free - running
    
    
     
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
    stUART[0].buffer_size = u0_size;
    stUART[1].buffer_size = u1_size;
    stUART[2].buffer_size = u2_size;
     
    stUART[0].Packet_size = uart0_size;
    stUART[1].Packet_size = uart1_size;
    stUART[2].Packet_size = spi_size;     
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
        
        case 2: // Uart2
                rx2_buffer[stUART[Ux].rx_head] = (unsigned char)(data&0xFF); 
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
    char val0=0, val1=0, val2=0;
    
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
        case 2: // Uart2
        {   
            val2 = rx2_buffer[stUART[Ux].rx_tail];
            increase_point_value(Ux, &(stUART[Ux].rx_tail)); // tail 증가
            return val2;
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

void Packet_EnQueue(unsigned char Ux, char data, unsigned char pSize)
{
    int i=0;
    int tmpID=0;
    if(Ux==0)
    {      
        rx0_Packet[stUART[Ux].Packet_head++] = data&0xFF;
        switch(stUART[Ux].hIDX)
        {
            case 0:     // Header #1
            case 1:     // Header #2
            case 2:     // Header #3      
                if(rx0_Packet[stUART[Ux].hIDX] == ((char)((DEF_Rshift(user_head, stUART[Ux].hIDX))&0xFF))) 
                    stUART[Ux].hIDX++;      
                else    DEF_HEADINIT(Ux);
                break;
            case 3:     // Packet Length
                if(rx0_Packet[stUART[Ux].hIDX] >= MIN_DataPacket)
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
        if(stUART[Ux].Packet_head >= pSize)
        {
            PacketDecode(Ux, rx0_Packet, pSize);          
            DEF_HEADINIT(Ux);            
        }
    }
    else if(Ux==1)    
    {
        rx1_Packet[stUART[Ux].Packet_head++] = data&0xFF;
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
                if(rx1_Packet[stUART[Ux].hIDX] >= MIN_DataPacket)
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
            //for(i=0;i<pSize;i++) UART0_putch(rx1_Packet[i]);    // ADC 수신 패킷확인용 
            PacketDecode(Ux, rx1_Packet, pSize);       
            DEF_HEADINIT(Ux);            
        }
    }
    else if (Ux==2)
    {
        rx2_Packet[stUART[Ux].Packet_head++] = data&0xFF; 
        //UART0_putch(0xAA);
        //UART0_putch(rx2_Packet[stUART[Ux].Packet_head-1]);
        switch(stUART[Ux].hIDX)
        {
            case 0:     // Header #1
            case 1:     // Header #2
            case 2:     // Header #3      
                if(rx2_Packet[stUART[Ux].hIDX] == ((char)((DEF_Rshift(user_head, stUART[Ux].hIDX))&0xFF))) 
                    stUART[Ux].hIDX++;      
                else    DEF_HEADINIT(Ux);
                break;
            case 3:     // Packet Length
                if(rx2_Packet[stUART[Ux].hIDX] == pSize)
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
            //for(i=0;i<pSize;i++) UART0_putch(rx2_Packet[i]);    // SPI 수신 패킷확인용  
            SPI_Decode(rx2_Packet);       
            DEF_HEADINIT(Ux);            
        }
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
    int i = 0;
    unsigned char bShiftCheck = 0x00;
    unsigned char CheckSumData = *(data + (Buff_size-1));    //CheckSum Data
    
    for(i=0;i<(Buff_size-1);i++)    bShiftCheck ^= *(data + i);
    if(CheckSumData == bShiftCheck) return 1;
    
    return 0;
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
    if(PacketHeaderCheck(pPacket))      // Header 확인
    {       
        if(PacketEnderCheck(pPacket))   // Ender 확인
        {   
            for(i=0;i<imu_size-7;i++)
                imu_tx[i+6]=rx2_Packet[i+23];
			
			for(i=30;i<33;i++)
				


                               
            GCS_Tx_Packet(imu_size, gcs_tx_size);     // Transmit to GCS (5Hz)      
        }
    }   
}

void PacketDecode(unsigned char Ux, unsigned char* pPacket, unsigned char Buff_size)
{
    int i=0;
    if(PacketHeaderCheck(pPacket))  // Header 확인
    {    
        if(PacketCheckSum(pPacket, Buff_size))  // CheckSum 확인
        {   
            switch(Ux)
            {
                case 0:
                {
                    tx_spi[6]=0xAA; // Gain 변경 Flag
                    
                    for(i=0;i<Buff_size-7;i++)
                        tx_spi[i+7] = rx0_Packet[i+6];                        
                    break;
                }
                
                case 1:
                {   



                    break; 
                }
            }    
        }
    }
}

void spi_Tx_Packet(char pSize)  // To. AVR2
{
    int i = 0;
    tx_spi[0] = 0xFF;                // Header #1
    tx_spi[1] = 0xFE;                // Header #2
    tx_spi[2] = 0xFD;                // Header #3
    tx_spi[3] = pSize;               // Packet Size
    tx_spi[4] = 0x00;                // ID #1
    tx_spi[5] = 0x57;                // ID #2             
    // tx_spi[6] = Gain Flag
    // tx_spi[7] ~ tx_spi[24] : P-I-D Gain
    for(i=0;i<50;i++)
        tx_spi[i+25] = 0x00;          // Null Data (50Byte)

    tx_spi[75] = 0xFC;                // Ender #1
    tx_spi[76] = 0xFB;                // Ender #2
    tx_spi[77] = 0xFA;                // Ender #3
}

void GCS_Tx_Packet(char pSize1, char pSize2)
{
    int i = 0;
    char chk1 = 0x00,  chk2 = 0x00;;


    imu_tx[0] = 0xFF;       // Header #1
    imu_tx[1] = 0xFE;       // Header #2
    imu_tx[2] = 0xFD;       // Header #3
    imu_tx[3] = pSize1;     // Packet Size
    imu_tx[4] = 0x13;       // ID #1
    imu_tx[5] = 0x00;       // ID #2          
    // imu_tx[6]  ~ imu_tx[48] - IMU 수신패킷
    for(i=0;i<pSize1-1;i++)       // CheckSum 계산(XOR)
        chk1 ^= imu_tx[i];
    imu_tx[pSize1-1] = chk1; 

    for(i=0;i<pSize1;i++)       // UART0 Transmit (to.GCS)
    UART0_putch(imu_tx[i]);    
  
    gcs_tx[0] = 0xFF;       // Header #1
    gcs_tx[1] = 0xFE;       // Header #2
    gcs_tx[2] = 0xFD;       // Header #3
    gcs_tx[3] = pSize2;     // Packet Size
    gcs_tx[4] = 0x12;       // ID #1
    gcs_tx[5] = 0x00;       // ID #2   
    
    //Motor PWM
    gcs_tx[6] = rx2_Packet[6];
    gcs_tx[7] = rx2_Packet[7];
    gcs_tx[8] = rx2_Packet[8];
    gcs_tx[9] = rx2_Packet[9];  
     
 
    Batt_CHK();
    for(i=0;i<29;i++)
   gcs_tx[i+10]=0;
    for(i=0;i<pSize2-1;i++)       // CheckSum 계산(XOR)
        chk2 ^= gcs_tx[i];
    gcs_tx[43] = chk2; 
    for(i=0;i<pSize2;i++)       // UART0 Transmit (to.GCS)
      UART0_putch(gcs_tx[i]);      
    //printf("%f\r\n",  Battery);
   
    // gcs_tx[39] ~ gcs_tx[42] - Battery Voltage
}

void Batt_CHK(void) // Battery Voltage Check
{
    int i=0;
    float ADC_I, ADC_F, Batt_Sum=0, ADC_batt=0, Batt[200]={0};
        
    ADCSRA|=0x40;
    while((ADCSRA&0x10)==0);
        ADCSRA|=0x10;
    
    ADC_I = ADCW;
    ADC_F = (float)ADC_I * 5.0/1023.0;    
    ADC_batt = (-0.1185*ADC_F*ADC_F*ADC_F*ADC_F*ADC_F)+(1.8583*ADC_F*ADC_F*ADC_F*ADC_F)-(10.92*ADC_F*ADC_F*ADC_F)+(28.61*ADC_F*ADC_F)-(25.873*ADC_F)+1.5789;   
    for(i=0;i<200;i++)  // 평균치 사용
    {
        Batt[i]=ADC_batt;
        Batt_Sum += Batt[i];
    }   
    Battery = Batt_Sum/200;
    
    gcs_tx[39] = DEF_pCONV32bit(&Battery);    
    gcs_tx[40] = DEF_Rshift(DEF_pCONV32bit(&Battery), 1);
    gcs_tx[41] = DEF_Rshift(DEF_pCONV32bit(&Battery), 2);
    gcs_tx[42] = DEF_Rshift(DEF_pCONV32bit(&Battery), 3);    
    
    if(Battery <= 11.1)
        LED0 = 0;       // Red Off        
}


void main(void)
{   
    MCU_init();
    UART_StructSet();
    
    LED0 = 1;       // Red On    
    SREG = 0x80;
    while (1)
    {
        LED1 = 1;   // Yellow On
        spi_Tx_Packet(spi_size);
        while(Uart_Is_Empty(u0))    Packet_EnQueue(u0, Uart_DeQueue(u0), uart0_size);   // GCS Decode
        while(Uart_Is_Empty(u1))    Packet_EnQueue(u1, Uart_DeQueue(u1), uart1_size);   // ADC Decode     
        while(Uart_Is_Empty(u2))    Packet_EnQueue(u2, Uart_DeQueue(u2), spi_size);     // SPI Decode
    }
}

interrupt [USART0_RXC] void usart0_rx_isr(void){ char tmp0 = UDR0; Uart_EnQueue(u0, tmp0); }    // UART0 Rx Interrupt - 이벤트성
interrupt [USART1_RXC] void usart1_rx_isr(void){ char tmp1 = UDR1; Uart_EnQueue(u1, tmp1); }    // UART1 Rx Interrupt - 10Hz
interrupt [SPI_STC] void spi_isr(void)  // Master쪽 주기로 수신
{
    char tmp = SPDR;                // SPI 데이터 수신  
    //UART0_putch(tmp);
    Uart_EnQueue(u2, tmp);  
    

    SPDR = tx_spi[spi_tx_cnt++];    // SPI 데이터 송신
    if(spi_tx_cnt>=spi_size)
    {
        spi_tx_cnt=0;
        if(tx_spi[6]==0xAA)
            tx_spi[6]=0x00;        
    }    
}