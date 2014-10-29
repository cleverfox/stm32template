#include "stm32f10x_conf.h"
#include "usart.h"

#define STACK_TOP 0x20000800
#define NVIC_CCR ((volatile unsigned long *)(0xE000ED14))
//Declarations
int main(void);
void myDelay(unsigned long delay);
void dispa (int d0, int d4, int d3, int d2, int d1);
void rtc_init (void);
void clk_init (void);
void disp(int digit, int val);
uint8_t handled=0;

// VARIABLES
static GPIO_InitTypeDef GPIO_InitStructure;
static EXTI_InitTypeDef EXTI_InitStructure;
//NVIC_InitTypeDef NVIC_InitStructure; 
int Led=0;
int t=0;
int bright=350;
int speed=0;
int timezone=4;
int32_t offset=0;
int menu=0;
int speeds[]={9600,38400,57600,115200,0};

static uint16_t li=0;
static uint8_t rxb;
static uint8_t prxp[100];
unsigned long measuretime;

void saveOffset(void){
    PWR_BackupAccessCmd(ENABLE);
    BKP_WriteBackupRegister(BKP_DR3,offset>>16);
    BKP_WriteBackupRegister(BKP_DR4,offset&0xff);
    PWR_BackupAccessCmd(DISABLE);
}

void startmeasuretime(void){
    measuretime=0;
    TIM_SetCounter(TIM3,0);
    TIM_Cmd(TIM3, ENABLE);
}

unsigned long stopmeasuretime(void){
    TIM_Cmd(TIM3, DISABLE);
    measuretime+=TIM_GetCounter(TIM3);
    return measuretime;
}

void TIM3_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM3, 0xff);
    measuretime+=65535;
    //TIM_Cmd(TIM3, DISABLE);
    Uart2WriteStr("t3\r\n");
}

void incout(unsigned int cn){
    if(cn>999999)
        Uart2SendChar((cn/1000000)%10+'0');
    if(cn>99999)
        Uart2SendChar((cn/100000)%10+'0');
    if(cn>9999)
        Uart2SendChar((cn/10000)%10+'0');
    if(cn>999)
        Uart2SendChar((cn/1000)%10+'0');
    if(cn>99)
        Uart2SendChar((cn/100)%10+'0');
    Uart2SendChar((cn/10)%10+'0');
    Uart2SendChar(cn%10+'0');
}

void ilncout(unsigned int cn){
    Uart2SendChar('\r');
    Uart2SendChar('\n');
    incout(cn);
}

void icout(unsigned int cn){
    incout(cn);
    Uart2SendChar('\r');
    Uart2SendChar('\n');
}


static char set[]="01234567890ABCDEF";
void xcout(unsigned char c){
   Uart2SendChar(set[(c>>4)&0x0f]);
   Uart2SendChar(set[c&0x0f]);
}
uint8_t ii2i(unsigned char *c){
    int r=0;
    if(c[0]>='0' && c[0]<='9') r=(c[0]-'0')*10; 
    if(c[1]>='0' && c[1]<='9') r+=c[1]-'0';
    return r;
}

uint8_t xx2i(unsigned char *c){
    int r=0;
    if(c[0]>='0' && c[0]<='9'){
        r+=c[0]-'0';
    }else if(c[0]>='a' && c[0]<='f'){
        r+=c[0]-'a'+10;
    }else if(c[0]>='A' && c[0]<='F'){
        r+=c[0]-'A'+10;
    }
    r<<=4;

    if(c[1]>='0' && c[1]<='9'){
        r+=c[1]-'0';
    }else if(c[1]>='a' && c[1]<='f'){
        r+=c[1]-'a'+10;
    }else if(c[1]>='A' && c[1]<='F'){
        r+=c[1]-'A'+10;
    }
    return r;
}

void ResetTimeout(){
    TIM_Cmd(TIM3, DISABLE);
}
void SetTimeout(uint16_t val){
    TIM_SetCounter(TIM3, val);
    li=val;
    TIM_Cmd(TIM3, ENABLE);
}

void RTC_IRQHandler(void) {
    static exitmenu=0;
    RTC_ClearITPendingBit(RTC_IT_SEC|RTC_IT_OW|RTC_IT_ALR);

    uint8_t left=!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13);
    uint8_t right=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_14);
    static uint8_t showsec=0;

    if(menu){
        if(left && right){
            exitmenu++;
            if(exitmenu>3)
                menu=0;
            else
                return;
        }else{
            exitmenu=0;
            return;
        }
    }

    if(left && right){
        if(!exitmenu)
            menu=1;
        handled=1;
    }else{
        if(left){
            Uart2WriteStr("<");
            bright-=50;
            showsec=5;
            TIM_SetCompare4(TIM4, bright);
        }else if(right){
            Uart2WriteStr(">");
            bright+=50;
            showsec=5;
            TIM_SetCompare4(TIM4, bright);
        }else
            exitmenu=0;
    }

    int time=RTC_GetCounter();
    time+=(3600*timezone);
    time+=offset;
    int hour=time%86400/3600;
    incout(hour);
    Uart2WriteStr(":");
    int min=time%3600/60;
    incout(min);
    Uart2WriteStr(":");
    int sec=time%60;
    icout(sec);
    if(!showsec)
        dispa(time%60,hour>=10?hour/10:16,hour%10,min/10,min%10);
    else{
        dispa(time%60,min/10,min%10,sec/10,sec%10);
        showsec--;
    }

}

uint32_t st=0;
void USART2_IRQHandler(void){
    USART_ClearITPendingBit(USART2, 0xff);
    //$GPGGA,163403.00,5143.92069,N,03607.39416,E,1,03,6.58,202.6,M,16.5,M,,*5F
    uint8_t c=Uart2ReadChar();
    if(c=='\n' || c=='\r'){
        prxp[rxb]=0;
        /*
        int i=0;
        if(prxp[0]=='S'){
            i=1;
            st=xx2i(prxp+1)<<8|xx2i(prxp+3);
            icout(st);
        }else
            for(;i<rxb;i++){
                icout(prxp[i]);
            }
            */
        if(prxp[0]=='$' && prxp[1]=='G' && prxp[2]=='P' && prxp[3]=='G' &&
                prxp[4]=='G' && prxp[5]=='A' && prxp[6]==','){
            //Uart2WriteStr(prxp);
            prxp[16]=0;
            Uart2WriteStr("GPS UTC: ");
            Uart2WriteStr(prxp+7);
            Uart2WriteStr("\r\n");
            int tod=ii2i(prxp+7)*3600+ii2i(prxp+9)*60+ii2i(prxp+11);
            int time=RTC_GetCounter()%86400;
            int xoffset=tod-time+1;
            if(xoffset-offset){
                offset=xoffset;
                saveOffset();
            }
        }
        rxb=0;
    }else{
        prxp[rxb]=c;
        if(rxb<99)
            rxb++;
    }

}

void init_ind(){
    //ind1
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //ind 2
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 
        | GPIO_Pin_13 | GPIO_Pin_14 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

//ind 3
    BKP_TamperPinCmd(DISABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 
        | GPIO_Pin_15 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // digit 4

//    AFIO->MAPR|=AFIO_MAPR_PD01_REMAP;
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 
        | GPIO_Pin_8 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);



    //dots
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOD,GPIO_Pin_0 ,0);
    GPIO_WriteBit(GPIOD,GPIO_Pin_1 ,0);
    GPIO_PinRemapConfig(GPIO_Remap_PD01,ENABLE);
}

uint8_t chars[]={
    0b0111111, //0
    0b0000110, //1
    0b1011011, //2
    0b1001111, //3
    0b1100110, //4
    0b1101101, //5
    0b1111101, //6
    0b0000111, //7
    0b1111111, //8
    0b1101111, //9
    0b1110111, //A
    0b1111100, //b
    0b0111001, //C
    0b1011110, //d
    0b1111001, //E
    0b1110001, //F
    0b0000000, //16 space
    0b1110100, //17 h
    0b1010000, //18 r
    0b1111000, //19 t
    0b1000000, //20 -
    0b1010100, //21 m1
    0b1000100, //22 m2
    0b1011100, //23 o
};

int rotate=1;
void dispa (int d0, int d4, int d3, int d2, int d1){
    if(d0>-1) disp(0, d0);
    if(d1>-1) disp(1, d1);
    if(d2>-1) disp(2, d2);
    if(d3>-1) disp(3, d3);
    if(d4>-1) disp(4, d4);
}
void disp(int digit, int val){
    if(digit==0){
        uint16_t up=rotate?GPIO_Pin_1:GPIO_Pin_0;
        uint16_t dn=rotate?GPIO_Pin_0:GPIO_Pin_1;

        if(val<15){
            GPIO_WriteBit(GPIOD,dn ,val%2); 
            GPIO_WriteBit(GPIOD,up ,0); 
        }else if(val<30){
            GPIO_WriteBit(GPIOD,dn ,val%2); 
            GPIO_WriteBit(GPIOD,up ,1); 
        }else if(val<45){
            GPIO_WriteBit(GPIOD,up ,val%2); 
            GPIO_WriteBit(GPIOD,dn ,0); 
        }else{
            GPIO_WriteBit(GPIOD,up ,val%2); 
            GPIO_WriteBit(GPIOD,dn ,1); 
        }
    }

    uint8_t p=chars[val];
    if(rotate){
       uint8_t p1=0;
       p1|=(p&1)?8:0; 
       p1|=(p&2)?16:0; 
       p1|=(p&4)?32:0; 
       p1|=(p&8)?1:0; 
       p1|=(p&16)?2:0; 
       p1|=(p&32)?4:0; 
       p1|=(p&64)?64:0; 
       p=p1;
       switch (digit){
           case 1: digit=4; break;
           case 2: digit=3; break;
           case 3: digit=2; break;
           case 4: digit=1; break;
       }
    }

    if(digit==4){
        GPIO_WriteBit(GPIOA,GPIO_Pin_0 ,(p&64)!=0); //G
        GPIO_WriteBit(GPIOA,GPIO_Pin_1 ,(p&4)!=0); //C
        GPIO_WriteBit(GPIOA,GPIO_Pin_4 ,(p&8)!=0); //D
        GPIO_WriteBit(GPIOA,GPIO_Pin_5 ,(p&2)!=0); //B
        GPIO_WriteBit(GPIOA,GPIO_Pin_6 ,(p&1)!=0); //A
        GPIO_WriteBit(GPIOA,GPIO_Pin_7 ,(p&32)!=0); //F
        GPIO_WriteBit(GPIOB,GPIO_Pin_0 ,(p&16)!=0); //E
    }

    if(digit==3){
        GPIO_WriteBit(GPIOB,GPIO_Pin_1 ,(p&16)!=0); //E
        GPIO_WriteBit(GPIOB,GPIO_Pin_2 ,(p&8)!=0); //D
        GPIO_WriteBit(GPIOB,GPIO_Pin_10 ,(p&32)!=0); //F
        GPIO_WriteBit(GPIOB,GPIO_Pin_11 ,(p&1)!=0); //A
        GPIO_WriteBit(GPIOB,GPIO_Pin_12 ,(p&2)!=0); //B
        GPIO_WriteBit(GPIOB,GPIO_Pin_13 ,(p&64)!=0); //G
        GPIO_WriteBit(GPIOB,GPIO_Pin_14 ,(p&4)!=0); //C
    }

    if(digit==2){
        GPIO_WriteBit(GPIOB,GPIO_Pin_15 ,(p&1)!=0); //A
        GPIO_WriteBit(GPIOA,GPIO_Pin_8 ,(p&2)!=0); //B
        GPIO_WriteBit(GPIOA,GPIO_Pin_9 ,(p&64)!=0); //G
        GPIO_WriteBit(GPIOA,GPIO_Pin_10 ,(p&4)!=0); //C
        GPIO_WriteBit(GPIOA,GPIO_Pin_11 ,(p&32)!=0); //F
        GPIO_WriteBit(GPIOA,GPIO_Pin_12 ,(p&16)!=0); //E
        GPIO_WriteBit(GPIOC,GPIO_Pin_13 ,(p&8)!=0); //D
    }

    if(digit==1){
        GPIO_WriteBit(GPIOA,GPIO_Pin_15 ,(p&32)!=0); //F
        GPIO_WriteBit(GPIOB,GPIO_Pin_3 ,(p&16)!=0); //E
        GPIO_WriteBit(GPIOB,GPIO_Pin_4 ,(p&8)!=0); //D
        GPIO_WriteBit(GPIOB,GPIO_Pin_5 ,(p&4)!=0); //C
        GPIO_WriteBit(GPIOB,GPIO_Pin_6 ,(p&64)!=0); //G
        GPIO_WriteBit(GPIOB,GPIO_Pin_7 ,(p&2)!=0); //B
        GPIO_WriteBit(GPIOB,GPIO_Pin_8 ,(p&1)!=0); //A
    }

        /*
         * A 1
         * B 2
         * C 4
         * D 8
         * E 16 
         * F 32
         * G 64
         */

}

/*************************************************************************
 * Function Name: main
 * Parameters: none
 * Return: Int32U
 *
 * Description: The main subroutine
 *
 *************************************************************************/
int main(void) {

    *NVIC_CCR = *NVIC_CCR | 0x200; /* Set STKALIGN in NVIC */
    // Init clock system
    clk_init();
    rtc_init();

    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);


    InitUSART2(speeds[speed]);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    Uart2WriteStr("Hello\r\n");

    Uart2WriteStr("tz   "); icout(BKP_ReadBackupRegister(BKP_DR2)); //timezone
    Uart2WriteStr("off1 "); icout(BKP_ReadBackupRegister(BKP_DR3)); //offset1 MSB + SIGN
    Uart2WriteStr("off2 "); icout(BKP_ReadBackupRegister(BKP_DR4)); //offset2 LSB
    Uart2WriteStr("br   "); icout(BKP_ReadBackupRegister(BKP_DR5)); //bright
    Uart2WriteStr("rot  "); icout(BKP_ReadBackupRegister(BKP_DR6)); //rot
    Uart2WriteStr("uart "); icout(BKP_ReadBackupRegister(BKP_DR7)); //uart

    // buttons @ SWD port
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInitTypeDef timer_base;
    TIM_TimeBaseStructInit(&timer_base);
    timer_base.TIM_Prescaler = 512*16;
    timer_base.TIM_CounterMode = TIM_CounterMode_Up;
    timer_base.TIM_Period=65535;
    TIM_TimeBaseInit(TIM3, &timer_base);
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);






    //pwm timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    timer_base.TIM_Prescaler = 1;
    timer_base.TIM_CounterMode = TIM_CounterMode_Up;
    timer_base.TIM_Period=1000;
    TIM_TimeBaseInit(TIM4, &timer_base);
    TIM_OCInitTypeDef outputChannelInit;
    TIM_OCStructInit(&outputChannelInit);
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM2;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_Pulse = 400;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init(TIM4, &outputChannelInit);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    TIM_Cmd(TIM4, ENABLE);

    TIM_SetCompare4(TIM4, 350);

    //rtc interrupt
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    RTC_ITConfig(RTC_IT_SEC,ENABLE);
    RTC_ExitConfigMode();


    init_ind();

    myDelay(4000000);
    myDelay(4000000);

    Uart2WriteStr("Main loop with sleep\r\n");
    while(1){    
        if(st){
            uint16_t lov=RTC->CRH;
            Uart2WriteStr("Set time\r\n"); icout(st);
            RTC_WaitForLastTask();
            RTC_EnterConfigMode();
            RTC->CNTH = 0;
            RTC->CNTL = st;
            RTC_ClearITPendingBit(0xff);
            RTC_ExitConfigMode();
            st=0;
        }
        
        if(menu){
            static int8_t off;
            static uint32_t menutimeout=0;
            int time;
            if(menu>=4 && menu<=6){
                time=RTC_GetCounter();
                time+=(3600*timezone);
                time+=offset;
                icout(time%86400);
            }
            if(menu==1){ //brightness
                dispa(0,0x0b,18,bright/100,(bright/10)%10);
            }else if(menu==2){ //tz
                dispa(0,19,timezone<0?20:16,abs(timezone)/10,abs(timezone)%10);
            }else if(menu==3){ 
                dispa(0,18,23,19,rotate);
            }else if(menu==4){ //hour
                int d2=time%86400/3600;
                icout(d2);
                d2+=off;
                icout(d2);
                dispa(0,17,16,d2/10,d2%10);
            }else if(menu==5){ //min
                int d2=(time%3600)/60;
                icout(d2);
                d2+=off;
                icout(d2);
                dispa(0,21,22,d2/10,d2%10);
            }else if(menu==6){ //sec
                int d2=time%60;
                icout(d2);
                d2+=off;
                icout(d2);
                dispa(0,5,16,d2/10,d2%10);
            }else if(menu==7){
                int spd=speeds[speed]/1000;
                dispa(0,0x0a,spd/100,(spd/10)%10,spd%10);
            }else if(menu==8){
                dispa(0,20,20,20,20);
            }else if(menu==9){
                menu=0;
            }
            uint8_t left=!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13);
            uint8_t right=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_14);
            if(!(left&&right) && (left||right)){
                //wait 10 msec more for second button
                myDelay(10000);
                left=!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13);
                right=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_14);
            }
            if(left&&right){
                menutimeout=0;
                if(!handled){
                    if(menu>=4 && menu<=6){
                        int8_t goff[3];
                        goff[0]=offset/3600;
                        goff[1]=(offset/60)%60;
                        goff[2]=offset%60;
                        if(menu==4) goff[0]+=off;
                        if(menu==5) goff[1]+=off;
                        if(menu==6) goff[2]+=off;
                        off=0;
                        offset=goff[0]*3600+goff[1]*60+goff[2]+1;
                        saveOffset();
                    }else if(menu==1){
                        PWR_BackupAccessCmd(ENABLE);
                        BKP_WriteBackupRegister(BKP_DR5,bright);
                        PWR_BackupAccessCmd(DISABLE);
                    }else if(menu==2){
                        PWR_BackupAccessCmd(ENABLE);
                        BKP_WriteBackupRegister(BKP_DR2,timezone);
                        PWR_BackupAccessCmd(DISABLE);
                    }else if(menu==3){
                        PWR_BackupAccessCmd(ENABLE);
                        BKP_WriteBackupRegister(BKP_DR6,rotate);
                        PWR_BackupAccessCmd(DISABLE);
                    }else if(menu==7){
                        PWR_BackupAccessCmd(ENABLE);
                        BKP_WriteBackupRegister(BKP_DR7,speed);
                        PWR_BackupAccessCmd(DISABLE);
                    }
                    menu++;
                    Uart2WriteStr("*");
                    handled=1;
                }
            }else if(left){
                menutimeout=0;
                if(!handled){
                    if(menu==1){ if(bright>20){bright-=20; TIM_SetCompare4(TIM4, bright);} }
                    else if(menu==2){ timezone--; }
                    else if(menu==3){ rotate=!rotate; }
                    else if(menu>=4 && menu<=6){ off--; }
                    else if(menu==7){ speed--; }
                    Uart2WriteStr("<");
                    handled=1;
                }
            }else if(right){
                menutimeout=0;
                if(!handled){
                    if(menu==1){ if(bright<980){ bright+=20; TIM_SetCompare4(TIM4, bright);} }
                    else if(menu==2){ timezone++; }
                    else if(menu==3){ rotate=!rotate; }
                    else if(menu>=4 && menu<=6){ off++; }
                    else if(menu==7){ speed++; }
                    Uart2WriteStr(">");
                    handled=1;
                }
            }else{
                menutimeout++;
                if(menutimeout>300){
                    menu=0;
                }
                handled=0;
                Uart2WriteStr(".");
            }

        }else{
            Uart2WriteStr(".");
        }
        if(!menu)
            asm("wfi");
        else
            myDelay(100000);
    }
}
void nmi_handler(void)
{
    return;
}

void hardfault_handler(void)
{
    return;
}
//Functions definitions
void myDelay(unsigned long delay )
{ 
    while(delay) delay--;
}

/*************************************************************************
 * Function Name: Clk_Init
 * Parameters: Int32U Frequency
 * Return: Int32U
 *
 * Description: Init clock system
 *
 *************************************************************************/
void rtc_init (void){
    if (BKP_ReadBackupRegister(BKP_DR1) != 0xA0) 
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
        PWR_BackupAccessCmd(ENABLE);
        BKP_DeInit();
        RCC_LSEConfig(RCC_LSE_ON);
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {}
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        RCC_RTCCLKCmd(ENABLE);
        RTC_WaitForSynchro();
        RTC_WaitForLastTask();
        RTC_SetPrescaler(32767); 
        RTC_WaitForLastTask();
        RTC_SetCounter(1);
        PWR_BackupAccessCmd(ENABLE);
        BKP_WriteBackupRegister(BKP_DR1,0xA0);
        PWR_BackupAccessCmd(DISABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
        RTC_WaitForSynchro();
        RTC_WaitForLastTask();
        timezone=BKP_ReadBackupRegister(BKP_DR2);
        bright=BKP_ReadBackupRegister(BKP_DR5);
        int32_t off;
        off=BKP_ReadBackupRegister(BKP_DR3);
        off<<=16;
        off|=BKP_ReadBackupRegister(BKP_DR4);
        offset=off;
        rotate=BKP_ReadBackupRegister(BKP_DR6);
        speed=BKP_ReadBackupRegister(BKP_DR7);
    } 
}

void clk_init (void) {
    // 1. Cloking the controller from internal HSI RC (8 MHz)
    RCC_HSICmd(ENABLE);
    // wait until the HSI is ready
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    /*
    // 2. Enable ext. high frequency OSC
    RCC_HSEConfig(RCC_HSE_ON);
    // wait until the HSE is ready
    while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
    */
    // 3. Init PLL
    //RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9); // 72MHz
    //RCC_PLLConfig(RCC_PLLSource_HSE_Div2,RCC_PLLMul_9); // 36MHz
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_4); // 16MHz
    RCC_PLLCmd(ENABLE);
    // wait until the PLL is ready
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    // 4. Set system clock divders
    //RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    // Flash 1 wait state 
    *(vu32 *)0x40022000 = 0x12;
    // 5. Clock system from PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

