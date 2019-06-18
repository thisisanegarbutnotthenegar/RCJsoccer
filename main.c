/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 8/27/2018
Author  : 
Company : 
Comments: 


Chip type               : ATmega64A
Program type            : Application
AVR Core Clock frequency: 18.432000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 1024
*******************************************************/

#include <mega64a.h>
#include <delay.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
// I2C Bus functions
#include <i2c.h>
// Alphanumeric LCD functions
#include <alcd.h>
#include <myi2c.h>
#include <vl53.h>
#include <misc.h>

/////////////////////

enum outStatment{FF, FB, FL, FR, FFR, FFL, FBR, FBL, unKnown, clear};
enum outStatment outFlow[5], current , lastCurrent, firstOut;
 
enum kafupStatment {inField , inOut};
enum kafupStatment outMode;
int forward, backward, left, right, FRcorner, FLcorner, BLcorner, BRcorner;

int outSum;
int fastForward = 0, fastBackward = 0;
/////////////////////////////////Global Vars\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
///Localization
int globalX,globalY;
int num_local = 20 ;
int lastValidF[num_local]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000}, lastValidB[num_local]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000} , lastValidR[num_local]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000}, lastValidL[num_local]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000};
int lastVelF[num_local-1]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000},lastVelB[num_local-1]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000},lastVelR[num_local-1]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000},lastVelL[num_local-1]={-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000};
int maxDiffF = -1, maxDiffB = -1, maxDiffR = -1, maxDiffL = -1;
int fieldHight = 600, fieldWidth = 400;
int diffF, diffB, diffR, diffL;       
////////
int movementOffset = 0;

/////Motor 
int mg[4] = {0};
int cmpEff = 1;
///Battery
int batteryVoltage = 0;
		
int laser;		
int stuckCounter = 0;
///Sens
int biggestTimer = 0;
int sens[16] = {0},sensS[16];
int biggestValue = 0 , biggestInd = 0;
///Kaf
int sensKafL[3] = {0} , sensKafR[3] = {0} , sensKafF[2] = {0} , sensKafB[3] = {0} , sensKafRF = 0 , sensKafLF = 0;
int sensKafLB = 0 , sensKafRB = 0;

int sensKafLS[3] = {0} , sensKafRS[3] = {0} , sensKafFS[2] = {0} , sensKafBS[3] = {0} , sensKafRFS = 0 , sensKafLFS = 0;
int sensKafLBS = 0 , sensKafRBS = 0;


eeprom int sensKafLSE[3] = {0} , sensKafRSE[3] = {0} , sensKafFSE[2] = {0} , sensKafBSE[3] = {0} , sensKafRFSE = 0 , sensKafLFSE = 0;
eeprom int sensKafLBSE = 0 , sensKafRBSE = 0;

///////CMP
int maxX=0,minX=0,maxY=0,minY=0;
eeprom int maxXE,minXE,maxYE,minYE;
int cmpRaw;
int cmp;
int cmpS;
eeprom int cmpSE;

///IMU
int yawS, aimedYaw;
eeprom int yawSE;
unsigned char gyroData[23] = {0};
bit firstFlag = false, secondFlag = false;
int gyroCounter;
int gyroZ,magX,magY,magZ;
unsigned int yaw;
int yawInt;
int finalYaw;
////////////////// MPU
int offset = 0;
float gyroVal = 0;
float gyroYaw = 0;
float fusionYaw = 0;

///SRF
int srfR,srfL,srfB,srfF;
/////////////BT
int isMaster;
char btPacket[20] = {0};
char recPacket [20] = {0};
int slaveMode = 0;
int role = 0;
////////////
//////////////////////////////////////////////////////////////////////////////////////
#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

//////////////////////////////////Defines\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
/////////////IMU
#define cmpSLV
#define PWR_MGMT_1 0x6B
/////////////Mux
#define ADD0 PORTB.3
#define ADD1 PORTB.4
#define ADD2 PORTB.1
#define ADD3 PORTB.2
////////////Motors

#define PWM1 OCR3AL
#define PWM2 OCR3BL
#define PWM3 OCR1A
#define PWM4 OCR1B
/////
#define M1I1 PORTE.2
#define M1I2 PORTA.0
#define M2I1 PORTE.6
#define M2I2 PORTE.5
#define M3I1 PORTD.7
#define M3I2 PORTD.6
#define M4I1 PORTC.0
#define M4I2 PORTC.1
////////////LED
#define LED0 PORTG.0
#define LED1 PORTD.4
#define LED2 PORTD.5
///////////Key
#define KEY_C PING & (1<<4)
#define KEY_F PING & (1<<2)
#define KEY_B PINB.7
#define KEY_R PING & (1<<1)
#define KEY_L PING & (1<<3)
///////////DIP
#define DIP0 PINA.2 
#define DIP1 PINA.3
#define DIP2 PINA.4
#define DIP3 PINA.5
///////////ADC
#define ADC_SENS read_adc(0)
#define ADC_KAF read_adc(1)
#define ADC_SHOOT read_adc(2)
#define ADC_CURRENT read_adc(3)
#define ADC_BATTERY read_adc(7)
////////////////////////////////////////////////////////////////////////////////////
// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 8
char rx_buffer0[RX_BUFFER_SIZE0];

#if RX_BUFFER_SIZE0 <= 256
unsigned char rx_wr_index0=0,rx_rd_index0=0;
#else
unsigned int rx_wr_index0=0,rx_rd_index0=0;
#endif

#if RX_BUFFER_SIZE0 < 256
unsigned char rx_counter0=0;
#else
unsigned int rx_counter0=0;
#endif

// This flag is set on USART0 Receiver buffer overflow
bit rx_buffer_overflow0;
int btPackCounter = 0;
int packNum = 0;
int test = 0;
unsigned long int btTimer = 0;
unsigned long int gTimer = 0;
// USART0 Receiver interrupt service routine
interrupt [USART0_RXC] void usart0_rx_isr(void)
{
char status,data;
status=UCSR0A;
data=UDR0;
btTimer = gTimer;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
      if(data == 0x99) {
        btPackCounter = 0; 
        packNum++;
      } else {           
        if(btPackCounter < 6) {
            recPacket[btPackCounter] = data;
            btPackCounter ++;      
        }
        if(btPackCounter == 6) { 
            putchar(btPacket[0]);
            putchar(btPacket[1]);
            putchar(btPacket[2]);
            putchar(btPacket[3]);
            putchar(btPacket[4]);  
            putchar(btPacket[5]);  
            putchar(btPacket[6]);  
          
          btPackCounter++;
        }
      }                   
      
   }
}

#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART0 Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void)
{
char data;
while (rx_counter0==0);
data=rx_buffer0[rx_rd_index0++];
#if RX_BUFFER_SIZE0 != 256
if (rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
#endif
#asm("cli")
--rx_counter0;
#asm("sei")
return data;
}
#pragma used-
#endif

// USART1 Receiver buffer
#define RX_BUFFER_SIZE1 8
char rx_buffer1[RX_BUFFER_SIZE1];

#if RX_BUFFER_SIZE1 <= 256
unsigned char rx_wr_index1=0,rx_rd_index1=0;
#else
unsigned int rx_wr_index1=0,rx_rd_index1=0;
#endif

#if RX_BUFFER_SIZE1 < 256
unsigned char rx_counter1=0;
#else
unsigned int rx_counter1=0;
#endif

// This flag is set on USART1 Receiver buffer overflow
bit rx_buffer_overflow1;

// USART1 Receiver interrupt service routine
interrupt [USART1_RXC] void usart1_rx_isr(void)
{
char status,data;
#asm("cli")
status=UCSR1A;
data=UDR1;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
        if(data == 0x5A) {
            if(firstFlag == false) {
                firstFlag = true;
            } else if(secondFlag == false) {
                secondFlag = true;     
                gyroCounter = 0;    
                return;
            } 
        } else {
            if(firstFlag ==true && secondFlag == false) {
                firstFlag = false;
            }
        }
        
        if(secondFlag == true) {
           gyroData[gyroCounter] = data;  
           gyroCounter ++;
        }       
        if(gyroCounter >= 21) {
            firstFlag = false;
            secondFlag = false; 
            gyroCounter = 0;
        }
   }
   #asm("sei")
}

// Get a character from the USART1 Receiver buffer
#pragma used+
char getchar1(void)
{
char data;
while (rx_counter1==0);
data=rx_buffer1[rx_rd_index1++];
#if RX_BUFFER_SIZE1 != 256
if (rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1=0;
#endif
#asm("cli")
--rx_counter1;
#asm("sei")
return data;
}
#pragma used-
// Write a character to the USART1 Transmitter
#pragma used+
void putchar1(char c)
{
while ((UCSR1A & DATA_REGISTER_EMPTY)==0);
UDR1=c;
}
#pragma used-

// Standard Input/Output functions
#include <stdio.h>
///////////////////////////////interrupt based funcs
/////////////////////////////////PID
float pidOut;
int cmpLast,gyroLast;
int _static = 1;
void pidStatic(float kpCmp,float kdCmp,float kpGyro,float kdGyro)
{
    int errorCmp = finalYaw;            
    float pCmp = (kpCmp * errorCmp);    
    int dCmp = (int)(gyroVal*kdCmp);
    int desiredVel = (dCmp + pCmp);  
    int pGyro = kpGyro * (desiredVel - gyroVal);
    int dGyro = kdGyro * (gyroVal - gyroLast);
    
        pidOut =  desiredVel + pGyro + dGyro;

} 


////////////////////////////////////
//////////////////CMP
/////////////////MPU9250

void mpu_init()
{
    myi2c_write(0XD0,PWR_MGMT_1,0x00);
  /////////////gyro
    /////////01--->2760
    /////////10 --------->1400
    /////////11
    //////////00------------->40000
    myi2c_write(0XD0,0x1B,0X18);
  /////////////////////pwr
    myi2c_write(0XD0,0x1A,0x00);
  /////////////////////////////acc
    myi2c_write(0XD0,0x1C,0x10);
    //////////////////////////////////int
}
void gyro()
{
    int data_l,data_h,data;
    data_l=myi2c_read(0XD0,68);
    data_h=myi2c_read(0XD0,67);
    data=(data_h*256)+data_l;
    gyroVal = data - offset;
    gyroVal = ((float)gyroVal)/ 16.4;
}

void mag() {
  int data_l,data_h,data;
  myi2c_write(0xD0,0x6A, 0x00); //Disable Master Aux I2C mode
  #asm("wdr")
  myi2c_write(0xD0,0x37, 0x22); //Enable I2C Aux Bypass mode
  #asm("wdr")
  ////////////mag add = 0x18
  myi2c_write(0x18,0x0A,0x01);
  myi2c_write(0x18,0x11,150);
  myi2c_write(0x18,0x12,150);  
}

void readIMU() {
  int data_l,data_h,data;
  data_l=myi2c_read(0X18,0x03);
  data_h=myi2c_read(0X18,0x04);
  magX=(data_h*256)+data_l;
  #asm("wdr")    
  
  data_l=myi2c_read(0X18,0x07);
  data_h=myi2c_read(0X18,0x08);
  magY=(data_h*256)+data_l;
  #asm("wdr")
  gyro();    
  #asm("wdr")
  mag();
}

int gyroForOffset()
{
    int data_l,data_h,data;
    #asm("wdr")
    data_l=myi2c_read(0XD0,68);
    data_h=myi2c_read(0XD0,67);
    data=(data_h*256)+data_l;
    return data;
}
void findOffset()
{
    float x = 0.0;
    int i = 0;
  for (i = 0 ; i < 100 ; i ++ ) {
    #asm("wdr")
        x += gyroForOffset();
    }  
  x = x / 100;
    offset = (int)x;
}


void readCMP()
{
    float mY,mX;
    mX = ((((float)magX - minX)/(maxX - minX)) * 2)-1;
    mY = ((((float)magY - minY)/(maxY - minY)) * 2)-1;
    cmpRaw = (atan2(mY,mX)/3.1415)*180 + 180;
}

void fusion() {
  gyroYaw = gyroYaw * 0.98 + cmpRaw *0.02;
}

//////////////////GY955

void yawDetection (void){
    int a;
    if (outSum != 0){
        yawS = aimedYaw;
    }
    a = gyroYaw - yawS;                
    if (a >= 180)
        finalYaw =  a - 360;
   
    else if (a < -180)
        finalYaw =  a + 360;

    else
        finalYaw =  a;
}
void yawSet ()
{
    if(KEY_B == 0) {
        delay_ms(200);
        yawS = gyroYaw;
        aimedYaw = yawS;
        yawSE = yawS;
    }
}
void gy955DataExtraction()
{
    if(abs((gyroData[2] << 8) + gyroData[3]) < 500)
        magX = magX * 0.7 + ((gyroData[2] << 8) + gyroData[3]) * 0.3;
    if(abs((gyroData[4] << 8) + gyroData[5]) < 500)
        magY = magY * 0.7 + ((gyroData[4] << 8) + gyroData[5]) * 0.3;
    gyroZ = (gyroData[12] << 8) + gyroData[13];
    yaw = (gyroData[14] << 8) + gyroData[15];
    yawInt = (int)(yaw / 10);
}
void gy955init()
{
    putchar1(0xAA);  
    delay_ms(1);
    putchar1(0xCE);
    delay_ms(1);
    putchar1(0x78);
    delay_ms(1);
}

////////////////////Motor
void motor(int motor1,int motor2,int motor3,int motor4,char comp){
    int omega = 0,biggest;
    
    if(comp == 1){
                         
    if(motor1 == 0 && motor2 == 0 && motor3 == 0 &&motor4 == 0)
        _static = 1;
    else
        _static = 0;
                
        omega = pidOut;
        motor1 += omega;
        motor2 += omega;
        motor3 += omega;
        motor4 += omega;
    }
    
    if (motor1 > 255)
        motor1 = 255;
        
    else if (motor1 < -255)
        motor1 = -255;
            
    
    if (motor2 > 255)
        motor2 = 255;
        
    else if (motor2 < -255)
        motor2 = -255;
                
    
    if (motor3 > 255)
        motor3 = 255;
        
    else if (motor3 < -255)
        motor3 = -255;
                
    
    if (motor4 > 255)   
        motor4 = 255;
            
    else if (motor4 < -255)
        motor4 = -255;
       
    if(motor1 < 0){
        M1I1 = 0;
        M1I2 = 1;        
        
        PWM1 = motor1*-1;
        
    }
        
    else{
        M1I1 = 1;
        M1I2 = 0;
        PWM1 = motor1;
    }
        
    if(motor2 < 0){
        M2I1 = 1;
        M2I2 = 0;
        PWM2 = motor2*-1;
    }
        
    else{
        M2I1 = 0;
        M2I2 = 1;
        PWM2 = motor2;
    }
        
    if(motor3 < 0){
        M3I1 = 0;
        M3I2 = 1;
        PWM3 = motor3*-1;
    }

        
    else{
        M3I1 = 1;
        M3I2 = 0;
        PWM3 = motor3;
    }
        
    if(motor4 < 0){
        M4I1 = 0;
        M4I2 = 1;
        PWM4 = motor4*-1;
    }
        
    else{
        M4I1 = 1;
        M4I2 = 0;
        PWM4 = motor4;
    }
}
/////////////////////////////read srf vars\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
int srfNum = 0;
///////////////////////////////////////////
int BTcounter = 0;

unsigned long int supporterTimer = 0;
int mF[4] = {0};
// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
      int i;
      readCMP();   
      gyroYaw += (gyroVal* 0.0142222222);
      while (gyroYaw > 360 )
        {
            gyroYaw -= 360;                                                         
        }        
        while(gyroYaw < 0)
        {
            gyroYaw +=360;
        }         
      fusion();      
      yawDetection(); 

////////////////PID                                                            
    if(_static) {
        pidStatic(4.2,0.4,0.05,0);
    } else {
        pidStatic(3.8,0.4,0,0);
    }
    cmpLast = -1* finalYaw;   
    gyroLast = gyroZ/16;
///////////////   
    BTcounter ++;
    if(BTcounter > 7) {
        BTcounter = 0;
        if(isMaster == 0) {
            putchar(0x99);
            putchar(btPacket[1]);
            putchar(btPacket[2]);
            putchar(btPacket[3]);
            putchar(btPacket[4]);
            putchar(btPacket[5]);   
            putchar(btPacket[6]);   
             
        }
    }            
    if(firstOut == clear && forward) {
      fastForward = 1;
      LED1 = 1;
    }
    if(firstOut == clear && backward) {
      fastBackward = 1;
    }              
    gTimer ++;
      
    for (i = 0; i < 4; i ++){
        mF[i] = 0.7 * mF[i] + 0.3 * mg[i];
    }            
      motor (mF[0],mF[1],mF[2],mF[3],cmpEff);                            
}

// Timer2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
// Reinitialize Timer2 value
TCNT2=0xB8;
// Place your code here

}

// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCW;
}


///////////////////////////////////////Functions\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\/
/////////////////SRF
void readSrf()
{

//    switch(srfNum) {
//        case 0:
//            mmyi2c_write(0xE2,0,0x51);
//            srfR = mmyi2c_read16(0xE4,2);
//            srfNum = 1;
//            delay_ms(0);
//        break;         
//        
//        case 1:
//            mmyi2c_write(0xE4,0,0x51);
//            srfB = mmyi2c_read16(0xE8,2);
//            srfNum = 2;     
//            delay_ms(0);
//        break;
//        
//        
//        case 2:
//            mmyi2c_write(0xE8,0,0x51);
//            srfL = mmyi2c_read16(0xE6,2);
//            srfNum = 3; 
//            delay_ms(0);
//        break;         
//        
//        
//        case 3:
//            mmyi2c_write(0xE6,0,0x51);
//            srfF = mmyi2c_read16(0xE2,2);
//            srfNum = 0;  
//            delay_ms(0);
//        break;
//    }
    switch(srfNum) {
        case 0:
            mmyi2c_write(0xE2,0,0x51); 
            mmyi2c_write(0xE8,0,0x51);
            srfR = mmyi2c_read16(0xE4,2);
            srfL = mmyi2c_read16(0xE6,2);
            srfNum = 1;
            delay_ms(0);
        break;         
        
        case 1:
            mmyi2c_write(0xE4,0,0x51);
            mmyi2c_write(0xE6,0,0x51);
            srfB = mmyi2c_read16(0xE8,2);
            srfF = mmyi2c_read16(0xE2,2);
            srfNum = 0;     
            delay_ms(0);
        break;
        
    }                
                
}
///////////////// Mux Read
int muxRead (int muxNum , char inputz){
    if(muxNum == 0)
    inputz = 15 - inputz;
    
    ADD0 = (inputz % 2);
    ADD1 = (inputz / 2) % 2;
    ADD2 = (inputz / 4) % 2;
    ADD3 = (inputz / 8) % 2;
    delay_us(1);
    if(muxNum == 0) {
        return ADC_SENS;
    } 
    else {
        return ADC_KAF;
    }
}
//////////////// Sens Process
void sensup (void){
    char j;    
    for(j = 0;j < 16;j++){
        sens[j] = sens[j]*0.5 +(1023 - muxRead(0,j))*0.5 ;
        delay_us(100);         
    }     
}

void sensor_max (void){ 
    int j;   
    int firstBiggest = 0 , secondBiggest = 0;
    int firstValue = 0, secondValue = 0;
    biggestValue = 0;
    biggestInd = 0;
    
    for(j = 0;j < 16;j++){                                             
        if(sens[j] > firstValue){
            firstValue = sens[j];
            firstBiggest = j;
        }
    }    
    
    for(j = 0;j < 16;j++){                                             
        if(sens[j] > secondValue  && j != firstBiggest){
            secondValue = sens[j];
            secondBiggest = j;
        }
    } 
    
    biggestInd = firstBiggest *2;   
    biggestValue = firstValue;
          
    if(biggestValue > 140) {
        biggestTimer ++;   
    } else {
        biggestTimer --;
    }
    
    if(biggestTimer > 50) {
        biggestTimer = 50;
    }                   
    if(biggestTimer < 0) {
        biggestTimer = 0;
    }  
    
    laser = muxRead(1,4);    
}
/////////////////////////Kaf process

void kafup (void){
    char j;
    sensKafL[0] = muxRead(1,1) - sensKafLS[0];   
    sensKafL[1] = muxRead(1,2) - sensKafLS[1];
    sensKafL[2] = muxRead(1,3) - sensKafLS[2];
    
    sensKafR[0] = muxRead(1,10) - sensKafRS[0];
    sensKafR[1] = muxRead(1,9) - sensKafRS[1];
    sensKafR[2] = muxRead(1,8) - sensKafRS[2];
    
    
    sensKafB[0] = muxRead(1,14) - sensKafBS[0];
    sensKafB[1] = muxRead(1,13) - sensKafBS[1];
    sensKafB[2] = muxRead(1,12) - sensKafBS[2];   
    
    
    sensKafF[0] = muxRead(1,6) - sensKafFS[0];
    sensKafF[1] = muxRead(1,7) - sensKafFS[1];        
    
    
    sensKafLF = muxRead(1,0) - sensKafLFS;
    sensKafRF = muxRead(1,5) - sensKafRFS;
    sensKafLB = muxRead(1,15) - sensKafLBS;
    sensKafRB = muxRead(1,11) - sensKafRBS;  
}
void kafSet()
{

    if(KEY_B == 0) {
    sensKafLS[0] = muxRead(1,1);
    sensKafLS[1] = muxRead(1,2);
    sensKafLS[2] = muxRead(1,3);
    
    sensKafRS[0] = muxRead(1,10);
    sensKafRS[1] = muxRead(1,9);
    sensKafRS[2] = muxRead(1,8);
    
    
    sensKafBS[0] = muxRead(1,14);
    sensKafBS[1] = muxRead(1,13);
    sensKafBS[2] = muxRead(1,12);   
    
    
    sensKafFS[0] = muxRead(1,6);
    sensKafFS[1] = muxRead(1,7);        
    
    
    sensKafLFS = muxRead(1,0);
    sensKafRFS = muxRead(1,5);
    sensKafLBS = muxRead(1,15);
    sensKafRBS = muxRead(1,11);
    
    sensKafLSE[0] = muxRead(1,1);
    sensKafLSE[1] = muxRead(1,2);
    sensKafLSE[2] = muxRead(1,3);
    
    sensKafRSE[0] = muxRead(1,10);
    sensKafRSE[1] = muxRead(1,9);
    sensKafRSE[2] = muxRead(1,8);
    
    
    sensKafBSE[0] = muxRead(1,14);
    sensKafBSE[1] = muxRead(1,13);
    sensKafBSE[2] = muxRead(1,12);   
    
    
    sensKafFSE[0] = muxRead(1,6);
    sensKafFSE[1] = muxRead(1,7);        
    
    
    sensKafLFSE = muxRead(1,0);
    sensKafRFSE = muxRead(1,5);
    sensKafLBSE = muxRead(1,15);
    sensKafRBSE = muxRead(1,11);
    
    
    }

}
///////////////////////////////init\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
void init() 
{
delay_ms(100);
///gyro init
//gy955init();
mpu_init();
///laser init   
//VL53L0X_InitDevices();
//VL53L0X_SetVcselPulsePeriod(&vl53l0xDev[0], 0, 20);
//VL53L0X_SetVcselPulsePeriod(&vl53l0xDev[0], 1, 14);                                
//VL53L0X_ContinuousReading(&vl53l0xDev[0], 0);

/////////////////////////////////EEPROM RESTORE\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//////////////Kaf
    sensKafRS[0] = sensKafRSE[0];
    sensKafRS[1] = sensKafRSE[1];
    sensKafRS[2] = sensKafRSE[2];
    
    sensKafLS[0] = sensKafLSE[0];
    sensKafLS[1] = sensKafLSE[1];
    sensKafLS[2] = sensKafLSE[2];
    
    
    sensKafBS[0] = sensKafBSE[0];
    sensKafBS[1] = sensKafBSE[1];
    sensKafBS[2] = sensKafBSE[2];   
    
    
    sensKafFS[0] = sensKafFSE[0];
    sensKafFS[1] = sensKafFSE[1];        
    
    
    sensKafLFS = sensKafLFSE;
    sensKafRFS = sensKafRFSE;
    sensKafLBS = sensKafLBSE;
    sensKafRBS = sensKafLBSE;

////////////////yaw
    maxX = maxXE;
    maxY = maxYE;
    minX = minXE;
    minY = minYE;       
    yawS = yawSE;
    aimedYaw = yawS;
/////////////////
    readIMU();
    delay_ms(10);
    readIMU();
    readCMP();
    gyroYaw = cmpRaw;
}
//////////////////////////////set 
void setAll()
{
    int j;
    kafSet();
    yawSet();
    
   /* if(KEY_B == 0)
    {   
        for(j = 0;j < 16;j++){
        sensS[j] = 1023 - muxRead(0,j);
        delay_us(100);         
        
        }
        
         
    }*/
}
/////////////////////////////void Calib
void calibCmp() {
    while(1) {  
    LCD(0,0,minY);
    LCD(0,1,maxY);  
    LCD(10,0,maxX);
    LCD(10,1,minX);     
    LCD(5,0,cmpRaw);
        
    readIMU();
    delay_ms(10);
    if(magX > maxX)
        maxX = magX;
    if(magX < minX)
        minX = magX;
    if(magY > maxY)
        maxY = magY;
    if(magY < minY)
        minY = magY;
        
    minXE = minX;
    maxXE = maxX;
    minYE = minY;
    maxYE = maxY;
    mg[0] = mg[1] = mg[2] = mg[3] = 100;
    #asm("wdr")
    if(DIP0 == 1 || DIP1 == 1 || DIP2 == 1 || DIP3 == 1) {
        break;
    }
    
    }                     
    
}
//////////////////////////////Disp 
void mode()
{
    lcd_gotoxy(15,0);
    lcd_putchar('0'+isMaster); 
    
    batteryVoltage = ((read_adc(7) * (0.12))-111)/1.3;    
    if(batteryVoltage > 9)
        batteryVoltage = 9;
    if(batteryVoltage < 0)
        batteryVoltage = 0;
    lcd_gotoxy(15,1);
    lcd_putchar('0'+batteryVoltage); 
    
    if(DIP0 == 0 && DIP1 == 0 && DIP2 == 0 && DIP3 == 0) {
        minX =  0;
        maxXE = 0;
        minYE = 0;
        maxYE = 0;
        if(KEY_B) {
            calibCmp();
        }
    }
    if(DIP0 == 0 && DIP1 == 1 && DIP2 == 1 && DIP3 == 1) {
        LCD(0,0,srfF);
        LCD(5,0,srfL);
        LCD(10,0,srfR);
        LCD(0,1,srfB);
        LCD(5,1,globalX);
        LCD(10,1,globalY);    
    }
    if(DIP0 == 1 && DIP1 == 0 && DIP2 == 1 ) {
        LCD(0,0,cmpRaw);
        LCD(0,1,gyroYaw);
        LCD(7,0,finalYaw);        
        
    }                             
    
    if(DIP0 == 1 && DIP1 == 1 && DIP2 == 0 && DIP3 == 1) {
        LCD(0,0,current);
        LCD(0,1,firstOut);
        LCD(1,0,outSum);
        
        LCD(2,1,outFlow[0]);
        LCD(4,1,outFlow[1]);
        LCD(6,1,outFlow[2]);
        LCD(8,1,outFlow[3]);
        LCD(10,1,outFlow[4]);       
    }
    if(DIP0 == 1 && DIP1 == 1 && DIP2 == 1) {
        LCD(7,0,biggestInd);
        LCD(7,1,biggestValue);
        //LCD(5,1,sens[15]);
        //LCD(0,1,sens[0]);
        //1LCD(5,0,sens[1]);
        
    }     
    
    if(DIP3 == 1) {
        isMaster = 1;
    } else {
        isMaster = 0;
    }
}



///////////////////////////////Jacob
void calcJacob(float speed,float ang) {
    
    int i;
    float m[4] = {0};
    float max = 0;
    float v1,v2,v3,v4,c,s,f;
    float si,co,m1,m2,m3,m4;
           

    /*if(role != 0) {
      if(ang >= 90 && ang <= 95) {
         ang = 100; 
      } else if(ang >= 85 && ang <= 90) {
          ang = 80;
      }
      if(ang <= -90 && ang >= -95) {
         ang = -100; 
      } else if(ang <= -85 && ang >= -90) {
          ang = -80;
      }              
    } */
    
   ang = (ang - 90) + movementOffset; 
  
    co=cos(ang * 3.1415 / 180);
    si=sin(ang * 3.1415 / 180);
    m[0]=((0.5*co)+(0.8657598395*si));
    m[1]=((-0.5*co)+(0.8657598395*si));
    m[2]=((-0.5*co)-(0.8657598395*si));
    m[3]=((0.5*co)-(0.8657598395*si));
                                   
    for(i=0;i<4;i++)
    {  
        if(m[i] > max)
            max = m[i];
    }                  
    for(i = 0 ; i < 4 ; i++) {
        m[i] = -255*(m[i] / max)*speed;
        mg[i] = m[i];
    }
                         
    
}
int fieldH(int Y){
 int x;
 if(Y < 40 && Y > -40){
  x = 560; 
 }
 else
  x = 600; 
 return(x);  
}

void velArrayFuncF(int lastValid[]){
    int n;                       
        for(n=num_local-2;n>0;n--){
        lastVelF[n-1] =lastVelF[n];                                                                  
        }
        if (lastValid[num_local-1]==-1000 || lastValid[num_local-2]==-1000){
        lastVelF[num_local-2]=-1000;
        }
        else {
        lastVelF[num_local-2]=lastValid[num_local-1]-lastValid[num_local-2]; 
        }        
}
void velArrayFuncB(int lastValid[]){
    int n;
        for(n=num_local-2;n>0;n--){
        lastVelB[n-1] =lastVelB[n];   
        }
        if (lastValid[num_local-1]==-1000 || lastValid[num_local-2]==-1000){
        lastVelB[num_local-2]=-1000;
        }
        else{
        lastVelB[num_local-2]=lastValid[num_local-1]-lastValid[num_local-2]; 
        }        
}
void velArrayFuncR(int lastValid[]){
    int n;
        for(n=num_local-2;n>0;n--){
        lastVelR[n-1] =lastVelR[n];   
        }
        if (lastValid[num_local-1]==-1000 || lastValid[num_local-2]==-1000){
        lastVelR[num_local-2]=-1000;                              
        }
        else{
        lastVelR[num_local-2]=lastValid[num_local-1]-lastValid[num_local-2]; 
        };         
}
void velArrayFuncL(int lastValid[]){
    int n;
        for(n=num_local-2;n>0;n--){
        lastVelL[n-1] =lastVelL[n];   
        }
        if (lastValid[num_local-1]==-1000 || lastValid[num_local-2]== -1000){
        lastVelL[num_local-2]=-1000;
        }
        else{
        lastVelL[num_local-2]=lastValid[num_local-1]-lastValid[num_local-2]; 
        }         
}
int lastValidIndex(int x[], bool is_vel){
    int counterx=-20;
    int i=num_local;
    int j;
    int rem = 1;
    if (is_vel){
    rem = 2;
    }  
          
    for(j=i-rem;j>=0;j--){ 
        if (x[j]>-1000){
            counterx=j;
            break;
        }
        }  
    return counterx ;
}

int firstValidIndex(int x[],int i){
    int counterx=-20; 
    int j;      
    for(j=i-1;j>=0;j--){ 
        if (x[j]<=-1000){
            counterx=j+1;
            break;
        }
        }  
    return counterx ;
}



void localization() 
{
        int firstindexR;
        int firstindexL;
        int firstindexF;
        int firstindexB;  
        int lastindexR;
        int lastindexL;
        int lastindexF;
        int lastindexB;
        int prevelR;
        int prevelL;
        int prevelF;
        int prevelB;
        int lastPosValidF, lastPosValidB, predsrfB, predsrfF;
        int lastPosValidR, lastPosValidL, predsrfL, predsrfR;
        int THR = 20;
       //////Y  
       int n=4;
       if(abs(fieldWidth -(srfL+srfR))<THR){
       globalY = (srfL - srfR)/2;
        if(lastValidL[num_local-1] >= 0){
        diffL = abs(lastValidL[num_local-1] - srfL);
        if(maxDiffL<diffL)
          maxDiffL = diffL;                       
        }
        if(lastValidR[num_local-1] >= 0){
        diffR = abs(lastValidR[num_local-1] - srfR);
        if(maxDiffR<diffR)
          maxDiffR = diffR;
        }
        for(n=num_local-1;n>0;n--){
        lastValidL[n-1] =lastValidL[n];  
        lastValidR[n-1] =lastValidR[n]; 
        }
        lastValidL[num_local-1] = srfL;
        lastValidR[num_local-1] = srfR;
       velArrayFuncR(lastValidR);  
       velArrayFuncL(lastValidL);  
       }
       else if(lastValidL[num_local-1] >=0 || lastValidR[num_local-1] >= 0){
         if(lastValidL[num_local-1] >= 0){
          diffL = abs(lastValidL[num_local-1] - srfL);
         }
         else
          diffL = 1000; 
       if(lastValidR[num_local-1] >= 0){
          diffR = abs(lastValidR[num_local-1] - srfR);
       }
         else
          diffR = 1000; 
          
       if(diffR < diffL && diffR <= maxDiffR){
        globalY = (fieldWidth / 2) - srfR;
        
//        diffR = abs(lastValidR[num_local-1] - srfR);
//        if(maxDiffR<diffR)
//          maxDiffR = diffR;

       for(n=num_local-1;n>0;n--){
        lastValidL[n-1] =lastValidL[n];
        lastValidR[n-1]=lastValidR[n];   
        }
       lastValidR[num_local-1] = srfR;
       lastValidL[num_local-1] = -1000;
       velArrayFuncR(lastValidR);  
       velArrayFuncL(lastValidL);
       }
       else if(diffL < diffR && diffL <= maxDiffL){
         globalY = srfL - (fieldWidth / 2);
//         diffL = abs(lastValidL[4] - srfL);
//        if(maxDiffL<diffL)
//          maxDiffL = diffL;   
          
       for(n=num_local-1;n>0;n--){
        lastValidL[n-1] =lastValidL[n];
        lastValidR[n-1]=lastValidR[n];   
        }
        lastValidR[num_local-1] = -1000;
        lastValidL[num_local-1] = srfL; 
        velArrayFuncL(lastValidL);  
        velArrayFuncR(lastValidR);
       }
       else 
       {
       for(n=num_local-1;n>0;n--){
        lastValidL[n-1] =lastValidL[n];
        lastValidR[n-1]=lastValidR[n];   
        }
        lastValidR[num_local-1] = -1000;
        lastValidL[num_local-1] = -1000;
        velArrayFuncR(lastValidR);
        velArrayFuncL(lastValidL);    
        globalY = -1000; 
        
       }  
        }
      
       else
       {        
       for(n=num_local-1;n>0;n--){
        lastValidL[n-1] =lastValidL[n];
        lastValidR[n-1]=lastValidR[n];   
        }
        globalY = -1000;
        lastValidR[num_local-1] = -1000;
        lastValidL[num_local-1] = -1000; 
        velArrayFuncR(lastValidR);    
        velArrayFuncL(lastValidL); 

        lastindexR=lastValidIndex(lastVelR,1);  
        lastindexL=lastValidIndex(lastVelL,1); 
        
        if (lastindexR>=0){
         firstindexR=firstValidIndex(lastVelR,lastindexR); 
         if (firstindexR>=0){ 
         //prevelR=lastVelR[lastindexR]+lastVelR[firstindexR];
         prevelR=lastVelR[lastindexR];
         } 
         else{
         prevelR=lastVelR[lastindexR];  
         }
         }
         else{
         prevelR=-1000;
         }
         
         if (lastindexL>=0){
         firstindexL=firstValidIndex(lastVelL,lastindexL); 
         if (firstindexL>=0){ 
         //prevelL=(lastVelL[lastindexL]+lastVelL[firstindexL])/2;
         prevelL=lastVelL[lastindexL];
         } 
         else{
         prevelL=lastVelL[lastindexL];  
         }
         }
         
         
         else{
         prevelL=-1000;
         }
         ///// new added /////// 
         lastPosValidR = lastValidIndex(lastValidR,0);
         lastPosValidL = lastValidIndex(lastValidL,0);
         if (lastPosValidR > lastPosValidL && prevelR != -1000){
         predsrfR = lastValidR[lastPosValidR] + ((num_local - lastPosValidR - 1) * prevelR); 
         globalY = (fieldWidth / 2) - predsrfR;
         /////globalY=prevelR;
         }
         else if (prevelL != -1000){
         predsrfL = lastValidL[lastPosValidL] + ((num_local - lastPosValidL - 1) * prevelL); 
         globalY = predsrfL - (fieldWidth / 2);
         ////globalY=prevelL;
         
         }
        //globalY = validarray(lastValidL);
        //else{
        //globalY = -1000;
        //}
         
        }
        
        fieldHight = fieldH(globalY);        
       //////X coordinate
       if(abs(fieldHight -(srfB+srfF))<THR){
       globalX = (srfB - srfF)/2;
        if(lastValidF[num_local-1] >= 0){
        diffF = abs(lastValidF[num_local-1] - srfF);
        if(maxDiffF<diffF)
          maxDiffF = diffF;
        }
         if(lastValidB[num_local-1] >= 0){
        diffB = abs(lastValidB[num_local-1] - srfB);
        if(maxDiffB<diffB)
          maxDiffB = diffB;
        }
        for(n=num_local-1;n>0;n--){
        lastValidF[n-1] =lastValidF[n];
        lastValidB[n-1]=lastValidB[n];   
        }
        lastValidF[num_local-1] = srfF;
        lastValidB[num_local-1] = srfB;
        velArrayFuncF(lastValidF);   
        velArrayFuncB(lastValidB);
       }
       else if(lastValidF[num_local-1] >=0 || lastValidB[num_local-1] >= 0){
          if(lastValidF[num_local-1] >= 0){
          diffF = abs(lastValidF[num_local-1] - srfF);
         }
         else
          diffF = 1000; 
       if(lastValidB[num_local-1] >= 0){
          diffB = abs(lastValidB[num_local-1] - srfB);
       }
         else
          diffB = 1000; 
       
       if(diffF < diffB && diffF<maxDiffF){
        globalX = (fieldHight / 2) - srfF;
//        diffF = abs(lastValidF[num_local-1] - srfF);
//        if(maxDiffF<diffF)
//          maxDiffF = diffF;

       for(n=num_local-1;n>0;n--){
        lastValidF[n-1] =lastValidF[n];
        lastValidB[n-1]=lastValidB[n];   
        } 
        lastValidF[num_local-1] = srfF;
        lastValidB[num_local-1] = -1000;  
        velArrayFuncF(lastValidF);
        velArrayFuncB(lastValidB);
       }
       else if(diffB < diffF && diffB < maxDiffB){
         globalX = srfB - (fieldHight / 2);
//         diffB = abs(lastValidB[num_local-1] - srfB);
//        if(maxDiffB<diffB)
//          maxDiffB = diffB;

        for(n=num_local-1;n>0;n--){
        lastValidF[n-1] =lastValidF[n];
        lastValidB[n-1]=lastValidB[n];   
        }
        lastValidF[num_local-1] = -1000;
        lastValidB[num_local-1] = srfB; 
         velArrayFuncF(lastValidF); 
         velArrayFuncB(lastValidB);
       }
       else 
       {
        
        for(n=num_local-1;n>0;n--){
        lastValidF[n-1] =lastValidF[n];
        lastValidB[n-1]=lastValidB[n];   
        } 
        lastValidF[num_local-1] = -1000;
        lastValidB[num_local-1] = -1000;
         velArrayFuncF(lastValidF);  
         velArrayFuncB(lastValidB);  
         
        
        //globalX = validarray(lastValidF);
        globalX = -1000;
        }
        }
       
       else
       {
        
        for(n=num_local-1;n>0;n--){
        lastValidF[n-1] =lastValidF[n];
        lastValidB[n-1]=lastValidB[n];   
        }
        globalX = -1000;
        lastValidF[num_local-1] = -1000;
        lastValidB[num_local-1] = -1000;
        velArrayFuncF(lastValidF);
        velArrayFuncB(lastValidB);  
          
        lastindexF=lastValidIndex(lastVelF,1);  
        lastindexB=lastValidIndex(lastVelB,1);
        firstindexF=firstValidIndex(lastVelF,lastindexF);  
        firstindexB=firstValidIndex(lastVelB,lastindexB); 
         
        if (lastindexF>=0){
         firstindexF=firstValidIndex(lastVelF,lastindexF); 
         if (firstindexF>=0){ 
         //prevelF=(lastVelF[lastindexF]+lastVelF[firstindexF])/2;
         prevelF=lastVelF[lastindexF];
         } 
         else{
         prevelF=lastVelF[lastindexF];  
         }
         }
         else{
         prevelF=-1000;
         }
         
         if (lastindexB>=0){
         firstindexB=firstValidIndex(lastVelB,lastindexB); 
         if (firstindexB>=0){ 
         //prevelB=(lastVelB[lastindexB]+lastVelB[firstindexB])/2;
         prevelB=lastVelB[lastindexB];
         } 
         else{
         prevelB=lastVelB[lastindexB];  
         }
         }
         else{
         prevelB=-1000;
         }
         ///// new added /////// 
         lastPosValidF = lastValidIndex(lastValidF,0);
         lastPosValidB = lastValidIndex(lastValidB,0);
         if (lastPosValidB > lastPosValidF && prevelB != -1000){
         predsrfB = lastValidB[lastPosValidB] + ((num_local - lastPosValidB - 1) * prevelB); 
         globalX = predsrfB - (fieldHight / 2);
         ///globalX = prevelB;
         }
         else if (prevelF != -1000){
         predsrfF = lastValidF[lastPosValidF] + ((num_local - lastPosValidF - 1) * prevelF); 
         globalX = (fieldHight / 2) - predsrfF;
         //globalX = prevelF;
         
         }
        //globalX = validarray(lastValidF);
        //else{
        //globalX = -1000;
        //}
        }
       
}

float calcSpeed(int dist){
 if(dist < 5)
    return 0;
 if(dist<10)
    return 0.4;
 else if(dist > 100)
    return 1;
 else
    return 0.00778 * dist +  0.3;
    
}

void gotoPoint(int x , int y)
{   

    int deltaY = y - globalY;
    int deltaX = x - globalX;
    float desiredAng;       
    float dist = 0;
    float speed =0.5;
    //LED1 = 0;
    LED2 = 0;
           
    if(globalX != -1000 && globalY != -1000){    
        dist = sqrt((deltaX*deltaX) + (deltaY*deltaY));
        speed = calcSpeed(dist);
        if((abs(globalY) > 25) && (srfB < 110))
        {    
            if(deltaY <0)
                calcJacob(speed * 1.5,90);
            else
                calcJacob(speed * 1.5,-90);    
            //LED1 = 1;
         
        }
        else {
            desiredAng = atan2(deltaX,deltaY)*180/3.1415 - 90;
            calcJacob(speed,desiredAng);
        }
    }
    
    else if(globalX == -1000 && globalY != -1000){
        dist = abs(deltaY);                                                              
        speed =0;
        
        speed = calcSpeed(dist);
        if(deltaY <0)
            calcJacob(speed,90);
        else
            calcJacob(speed,-90);    
    }
    
    else if(globalX != -1000 && globalY == -1000){
        dist = abs(deltaX) * 2;
        speed =0;                                                                
        speed = calcSpeed(dist);
        if(deltaX > 0)
            calcJacob(speed,0);
        else
            calcJacob(speed,180); 
    }
                                
    else{
        dist = 0;
        speed =0;
        calcJacob(0,0);
        LED2 = 1;
    }
    
}
///////////////////////////////Catch BAll                                       
bool isStuck()
{
    if(!(biggestInd < 3 || biggestInd > 28)) {
        stuckCounter = 0;
        return 0 ;   
    
    } else {
        if(ADC_CURRENT > 530 && laser < 250) {
            stuckCounter++;      
            if(stuckCounter > 10)
                stuckCounter = 10;
        }                  
        else {          
            stuckCounter-=10;
            if(stuckCounter < 0)
                stuckCounter = 0;
        }
    }                      
        
    LCD(12,0,stuckCounter);
    LCD(12,1,ADC_CURRENT);
    if(stuckCounter>=10)
        return 1;
    else
        return 0;
}
void dribble() {
    int i = 0;
    /*if(isStuck()) {
        LED1 = 1;        
        yawS = yawSE - 90;
        for(i = 0 ; i < 7000 ; i++) {
            #asm("wdr");
            //calcJacob(0.7,-70);
            mg[0] = 255;
            mg[1] = 255;
            mg[2] = 255;
            mg[3] = 255;
        }
    }  
    else */
    static int dribbleCounter = 0;
    if((biggestInd <2 || biggestInd >30) && (laser < 250) && (isStuck()) && (globalY != 1000)) {
        //LCD(5,0,1);
        //LED1 = 1;         
        if (globalX > 0){
          if(globalY > 0) {    
            yawS = aimedYaw + 100;
            dribbleCounter = 15;    
          }
          else if(globalY < 0){           
            yawS = aimedYaw - 100;
            dribbleCounter = 15;    
          }
          else{
              dribbleCounter --;
              if(dribbleCounter < 0) {
                dribbleCounter = 0;
              }
          }  
        }
        if (globalX < 0){
          if(globalY > 0) {    
            yawS = aimedYaw + 90;
            dribbleCounter = 15;    
          }
          else if(globalY < 0){           
            yawS = aimedYaw - 90;
            dribbleCounter = 15;    
          }
          else{
              dribbleCounter --;
              if(dribbleCounter < 0) {
                dribbleCounter = 0;
              }
          }  
        }
    }
       
    else {   
      dribbleCounter --;
      if(dribbleCounter < 0) {
        dribbleCounter = 0;
      }
    
    }  
    if(dribbleCounter == 0) {
          yawS = aimedYaw;
          //LED1 = 0;  
    }  
}

void aimedShoot(int i)
{
 if (outSum != 0){
        yawS = aimedYaw;
    
    }
 
else if (i && isStuck()){
    dribble();
}
else{

   
      if ((biggestInd == 0 || biggestInd == 1 || biggestInd == 2|| biggestInd == 31 || biggestInd == 30) && /*((sens[0] + sens[1]/2 + sens[15]/2) > 500) &&*/ (laser < 250) ){
        
        if (globalY != -1000 && globalX != -1000){
                movementOffset = globalY;
                yawS = (aimedYaw + (sign(globalY)*(abs(globalY) + (globalX/3))) * 1.5   ) ;//* ((globalX + abs(globalX)) / 2 * abs(globalX) + 1)) ;
                         
        }
        else if (globalY != -1000){
          movementOffset = globalY ;
          yawS = (aimedYaw  + sign(globalY)*abs(globalY/1.5) ) * 1.5;
        } 
        
      }  
      else{
          movementOffset = 0;
          yawS = aimedYaw;
      }
      #asm("wdr")
    }
}


//void pass(){
//    if (isStuck()){ 
//        if (isMaster){
//            otherRole = 2; 
//        }  
//        else {
//            
//        }
//    }
//
//}

void catchBall(int i)
{
    int sensAng = (32 - biggestInd) * 11.25; 
    int ang;           
    float havij; 
      
                           
    if(biggestTimer >= 2) {     
    
        havij = biggestValue;
        if(biggestValue < 150) {
            havij = 150;
        }    
        if(biggestValue > 400) {
            havij = 400;
        } 
        havij -= 150;
        havij *= 0.2;

        if((biggestInd == 0 || biggestInd == 1 || biggestInd == 2|| biggestInd == 31 || biggestInd == 30) && /*((sens[0] + sens[1]/2 + sens[15]/2) > 500) &&*/ laser < 250) {
             calcJacob(1,0);
        }  
        else if (((biggestInd == 0 || biggestInd == 1 || biggestInd == 2|| biggestInd == 31 || biggestInd == 30) && ((sens[0] + sens[1]/2 + sens[15]/2) > 500)) || biggestInd == 0 ){
              calcJacob(1,0);
        } 
        else if (biggestInd == 2){
              calcJacob(1,sensAng - havij  - 10);
        }
        else if (biggestInd == 30){         
              calcJacob(1,sensAng + havij + 10);
        }  
        else if (biggestInd == 4 || biggestInd == 6  || biggestInd == 8 ||  biggestInd == 10){
              calcJacob(1,sensAng - havij - 10);
        }
        else if (biggestInd == 28 || biggestInd == 26 ||  biggestInd == 24 ||  biggestInd == 22){
                calcJacob(1,sensAng + havij + 10);                   
        }                      
        else if(biggestInd < 16) {
            calcJacob(1,sensAng - 25 - havij);  
        }                            
        else {
                calcJacob(1,sensAng + 25 + havij);    
                
        }
    }  else { 
          if (gTimer - btTimer < 100){
              gotoPoint(-10,-10);
          }
          else {
                gotoPoint(-65,0);
          }          
        //calcJacob(0,0);
    }                             
   
}

/////////////////////////// goal keeper

//void goalKeeper()
//{
//    int sensAng = (32 - biggestInd) * 11.25; 
//    int ang;           
//    float havij;                        
//    if(biggestTimer >= 20) {     
//        if ((biggestInd < 5 )||(biggestInd > 28))
//        { 
//            catchBall(0);
//        }
//        else if (biggestInd > 12 && biggestInd < 22)
//        { 
//            catchBall(0);
//        } 
//        else 
//        {  
//            calcJacob(1,sensAng);  
//            
//        }                        
//    }
//    else{
//        gotoPoint(-110,0);
//        
//    }
//}
// void goalKeeper()
// {
//     if(biggestInd <= 32 && biggestTimer >= 20)
//     { 
//     catchBall(0);
//     } 
//     else
//     {
//     gotoPoint(-60,0);
//     } 
//     
// }
////////////newww///
void goalKeeper(){
    
    int sensAng = (32 - biggestInd) * 11.25; 
    int ang;           
    float havij;
    
    int GXerror;
    GXerror = (-60 - globalX) * 0.5;                                
    //LCD(12,1,GXerror);                        
    if(biggestTimer >= 4) {     
        if(biggestInd == 0 || (sens[0] + (sens[1] + sens[15]) / 2) > 500)
        {   
               
          if (globalX != -1000){
              if (GXerror > 5){                 
                  calcJacob(1,0);
              }
              else if (GXerror < -5){
                  calcJacob(0.8,180);
              }
              else {
                  calcJacob(0,0);
              }
          }  
          else {
              calcJacob(0,0);
          }
            
        }
        else if ((biggestInd < 7 ))
        {   
            
            if(globalY == -1000 || globalY <20) {
                if (globalX != -1000 && 0){ 
                  calcJacob(1,-85 + GXerror);     
                } 
                else {
                  calcJacob(1,-85);
                }            
                    
            }   
            else {
                gotoPoint(-65,20);
            }
        }                   
        else if(biggestInd > 25)
        {        
            
            if(globalY == -1000 || globalY > -20) {  
                if (globalX != -1000 && 0){
                  calcJacob(1,85 - GXerror);      
                } 
                else {
                    calcJacob(1,85);
                }
            }   
            else {
                gotoPoint(-65,-20);
            } 
        }
        else if (biggestInd > 14 && biggestInd <= 16)
        { 
            //catchBall(0);         
            calcJacob(1,sensAng - 75);
        }  
        else if (biggestInd > 16 && biggestInd < 20){
            calcJacob(1,sensAng + 75);
        }
        
        else if (biggestInd >= 20 ) 
        {   
            if(globalY == -1000 || globalY > -20) { 
                calcJacob(1,sensAng);
            }
               
            else {
              if (globalX != -1000){
                if (GXerror > 5){                 
                  calcJacob(1,0);
                }
                else if (GXerror < -5){
                  calcJacob(1,180);
                }
                else {
                  calcJacob(0,0);
                }
              }  
            else {
              calcJacob(0,0);
            }
            
            }
            
              
            
        } 
        else {
           if(globalY == -1000 || globalY < 20) { 
                calcJacob(1,sensAng);
            }
               
            else {
              if (globalX != -1000){
                if (GXerror > 5){                 
                  calcJacob(0.8,0);
                }
                else if (GXerror < -5){
                  calcJacob(0.8,180);
                }
                else {
                  calcJacob(0,0);
                }
              }  
            else {
              calcJacob(0,0);
            }
            
            }
            
            
        
        }                        
    }
    else{
        gotoPoint(-55,0);
        
    }
}

//////////////////////////


void unKnownDetection(){
    if (outFlow[4] == FF){
      current = FF;
    }
    else if (outFlow[4] == FB){
      current = FB;
    }
    else if (outFlow[4] == FL){
      current = FL;
    }
    else if (outFlow[4] == FR){
      current = FR;
    }
    else if (outFlow[4] == FFR){
      current = FFR;
    }
    else if (outFlow[4] == FFL){
      current = FFL;
    }
    else if (outFlow[4] == FBR){
  
    current = FBR;
    }
    else if(outFlow[4] == FBL){
      current = FBL;
    }
}


void currentDetection(){
    int clearCounter = 0;
    static int outSensor = 0;
    
    if (outSum == 0){
      current = clear;
      fastForward = 0;
      fastBackward = 0;
      //LED1 = 0;  
    }       

    else if (outSum == 1){
        if(forward){   
          current = FF;
          }
        else if(backward){  
          current = FB;
          }
        else if(left == 1){ 
          current = FL;
          }
        else if(right){ 
          current = FR;
          }
    }  
    

    /*else if(sum == 2){
        current = unKnown;
        // F and B || L and R
    } */
    else if (outSum == 3) {
        if(forward && right && FRcorner){   
            current = FFR;
            }
        else if(forward && left && FLcorner){
            current= FFL;
            }
        else if(backward && right && BRcorner){
            current = FBR;
            }
        else if(backward && left && BLcorner){
            current = FBL;
            }
    } 
                             
    
    
    if(current != outFlow[4]) {
      outFlow[0] = outFlow[1];
      outFlow[1] = outFlow[2];
      outFlow[2] = outFlow[3];
      outFlow[3] = outFlow[4];
      
      outFlow[4] = current;  
    }
              
    
    /*else{
      current = unKnown;
      return ;
    }
    
    if (current == unKnown){
      unKnownDetection();
    } 
    */
      
    
    
    if (outFlow[3] == clear && outFlow[4] != clear){
      firstOut = outFlow[4];
      outSensor = biggestInd;
    } 
    if(outFlow[4] == clear) {
      if(outSensor == 0 || outSensor == 1 || outSensor == 31 || outSensor == 30){
            if(!(abs(outSensor - biggestInd) < 3 || biggestInd > 28 || biggestInd < 2)){
                firstOut = clear;
            }   
      }  
      else{
        if (abs(outSensor - biggestInd) > 3){
             firstOut = clear;   
          }
      }   
      
      if (firstOut == FB){
          firstOut = clear;      
      }
      
    }
      
    if(outFlow[3] == FF && outFlow[4] == FFR){
      firstOut = FFR;
    }
    if(outFlow[3] == FR && outFlow[4] == FFR){
      firstOut = FFR;
    }
    if(outFlow[3] == FF  && outFlow[4] == FFL){
        firstOut = FFL ;
    }
    if(outFlow[3] == FL && outFlow[4] == FFL){
        firstOut = FFL ;
    }
    if(outFlow[3] ==  FB && outFlow[4] == FBR ){
        firstOut = FBR ;
    }
    if(outFlow[3] ==  FR && outFlow[4] == FBR){
        firstOut = FBR ;
    }
    if(outFlow[3] ==  FB && outFlow[4] == FBL){
        firstOut = FBL ;
    }
    if(outFlow[3] ==  FL && outFlow[4] == FBL){
        firstOut = FBL;
    }
       
}




void newOut(){
    int fastCounter;

    forward = (sensKafF[0] > 50 || sensKafF[1] > 50) || (sensKafRF > 50 && sensKafLF > 50); 
    
    if (role == 0){
        backward = (/*sensKafB[0] > 50 ||*/ sensKafB[1] > 50 || sensKafB[2] > 50) || (sensKafRB  > 50&& sensKafLB > 50);
    }                     
    else{
        backward = (sensKafB[0] > 50 ||sensKafB[1] > 50 || sensKafB[2] > 50) || (sensKafRB  > 50&& sensKafLB > 50);
    }
    left = (sensKafL[0] > 50 || sensKafL[1] > 50 || sensKafL[2] > 50) || (sensKafLF > 50 && sensKafLB > 50);
    right = (sensKafR[0] > 50 || sensKafR[1] > 50 || sensKafR[2] > 50) || (sensKafRF > 50 && sensKafRB > 50);
    FRcorner = (forward && right); //|| sensKafRF > 50;
    FLcorner = (forward && left); //|| sensKafLF > 50;
    BRcorner = (backward && right); //|| sensKafRB > 50;
    BLcorner = (backward && left); //|| sensKafLB > 50;
    
      
    /*LCD(0,0,forward );
    LCD(1,0,right );
    LCD(2,0,left );
    LCD(3,0,backward);
    LCD(4,0,FRcorner );
    LCD(5,0,FLcorner);
    LCD(6,0,BRcorner);
    LCD(7,0,BLcorner );
     */
    outSum = forward + backward + right + left + FRcorner + FLcorner + BRcorner + BLcorner;
    if (outSum != 0){
     outMode = inOut;
    }
    else{
     outMode = inField;    
    }    
      
    
    if (fastBackward){
      for(fastCounter = 0; fastCounter < 20; fastCounter++) {
              #asm("wdr")
              readIMU();
              if (globalY != -1000 ) {
                  if ( globalY < -20){
                      calcJacob(1,45);
                      delay_ms(10);                                
                  } 
                  else if (globalY > 20){
                      calcJacob(1,-45);
                      delay_ms(10);
                  }
                  else{
                      calcJacob(1,0);
                      delay_ms(10);
                  }          
              } 
              else {
                  calcJacob(1,0);
                  delay_ms(10);
              } 
      }
      
      fastBackward = 0; 
    } 
    currentDetection();
}


void outAvoiding(){
int OAcounter = 0;
    if(outMode == inOut){
        if(firstOut == FF){
          for(OAcounter = 0; OAcounter < 20; OAcounter++) {
              #asm("wdr")
              readIMU();
              if (globalY != -1000 ) {
                  if ( globalY < -20 || (right && !(left))){
                      calcJacob(1,-140);
                      delay_ms(10);                                
                  } 
                  else if (globalY > 20 || (left && !(right)) ){
                      calcJacob(1,140);
                      delay_ms(10);
                  }
                  else{
                      calcJacob(1,180);
                      delay_ms(10);
                  }          
              } 
              else {
                  calcJacob(1,180);
                  delay_ms(10);
              } 
          }
         }
             
        else if(firstOut == FB){
           for(OAcounter = 0; OAcounter < 20; OAcounter++) {
              #asm("wdr")
              readIMU();
              if (globalY != -1000 ) {
                  if ( globalY < -20){
                      calcJacob(1,-45);
                      delay_ms(10);                                
                  } 
                  else if (globalY > 20){
                      calcJacob(1,45);
                      delay_ms(10);
                  }
                  else{
                      calcJacob(1,0);
                      delay_ms(10);
                  }          
              } 
              else {
                  calcJacob(1,0);
                  delay_ms(10);
              } 
          }
      
        } 
    
        
        
        else if(firstOut == FL){
          for(OAcounter; OAcounter < 10; OAcounter++){
          #asm("wdr")
          readIMU();
          calcJacob(1 , -90);
          newOut();    
          delay_ms(10);
           }
          } 
        
        
    
        else if(firstOut == FR){
           for(OAcounter; OAcounter < 10; OAcounter++){
           #asm("wdr")
           readIMU();
           calcJacob(1,90);
           newOut();
           delay_ms(10);
            }
          }
        else if(firstOut == FFR){
          for(OAcounter; OAcounter < 30; OAcounter++){
          #asm("wdr")
          readIMU();
          calcJacob(1,135);
          newOut();              
          delay_ms(10);
               }
          }
        else if(firstOut == FFL){
          for(OAcounter; OAcounter < 30; OAcounter++){
          #asm("wdr")
          readIMU();
          calcJacob(1,-135);
          newOut();   
          delay_ms(10); 
          }
        }
        else if(firstOut == FBR){
          for(OAcounter; OAcounter < 10; OAcounter++){
          #asm("wdr")
          readIMU();
          calcJacob(1,45);
          newOut();   
          delay_ms(10); 
          }
        }
        else if(firstOut == FBL){
          for(OAcounter; OAcounter < 10; OAcounter++){
          #asm("wdr")
          readIMU();
          calcJacob(1,-45);
          newOut();   
          delay_ms(10);
          }
        }

    }  
     
    
     
    else{
     if(firstOut == clear){
      if(role == 0 )
      {
        goalKeeper();   
      } 
      else {
        catchBall(1);   
      
      }
      
      newOut();
      
     } else {
      calcJacob(0,0);
     }

    } 

           
      
    
}




/////////////////OUT
void out()
{
  static int firstL = 0 ,firstR = 0,firstB = 0,firstF = 0;
  int noOut = (sensKafF[0] <= 50 && sensKafF[1] <= 50 && sensKafR[0] <= 50 && sensKafR[1] <= 50 && sensKafR[2] <= 50 && sensKafB[0] <= 50 && 
    sensKafB[1] <= 50 && sensKafB[2] <= 50 && sensKafL[0] <= 50 && sensKafL[1] <= 50 && sensKafL[1] <= 50);
  int i = 0;
  if(firstL == 0 && firstR == 0 && firstB == 0 && firstF == 0) {
    if(sensKafF[0] > 50 || sensKafF[1] > 50) {
        firstF = 1;    
        for(i = 0 ; i <400 ;i ++) {
            kafup();
            noOut = (sensKafF[0] <= 50 && sensKafF[1] <= 50 && sensKafR[0] <= 50 && sensKafR[1] <= 50 && sensKafR[2] <= 50 && sensKafB[0] <= 50 && 
            sensKafB[1] <= 50 && sensKafB[2] <= 50 && sensKafL[0] <= 50 && sensKafL[1] <= 50 && sensKafL[1] <= 50);
            if(noOut  == 1) {
                break;
            }         
            calcJacob(0.5,180);
            #asm("wdr")
        }
    }
    if(sensKafL[0] > 50 || sensKafL[1] > 50|| sensKafL[2] > 50) {
        for(i = 0 ; i <100 ;i ++) {
            firstL = 1;
            calcJacob(0.6,-90);                
        }
    }
    if(sensKafR[0] > 50 || sensKafR[1] > 50|| sensKafR[2] > 50) {
        for(i = 0 ; i <100 ;i ++) {
            firstR = 1;
            calcJacob(0.6,90);                
        }
    }
    if(sensKafB[0] > 50 || sensKafB[1] > 50|| sensKafB[1] > 50) {
        firstB = 1;
    }
    
  }      
  
  if(sensKafF[0] <= 50 && sensKafF[1] <= 50 && sensKafR[0] <= 50 && sensKafR[1] <= 50 && sensKafR[2] <= 50 && sensKafB[0] <= 50 && 
    sensKafB[1] <= 50 && sensKafB[2] <= 50 && sensKafL[0] <= 50 && sensKafL[1] <= 50 && sensKafL[1] <= 50) {
      firstL = 0;
      firstR = 0;  
      firstB = 0;
      firstF = 0;  
    } 
    
  if(firstL == 1)  {
    if(sensKafR[0] > 50 || sensKafR[1] > 50|| sensKafR[2] > 50) {
        calcJacob(0.7,-90);
    }                 
    else if(biggestInd >1 && biggestInd < 14) {
        catchBall(1);
    }
    else {
        calcJacob(0,0);
    }
  } 
  else if(firstR == 1)  {
    if(sensKafL[0] > 50 || sensKafL[1] > 50|| sensKafL[2] > 50) {
        calcJacob(0.7,90);
    }  
    else if(biggestInd >18 && biggestInd < 30) {
        catchBall(1);
    }
    else {
        calcJacob(0,0);
    }               
    
  }
  else if(firstF == 1)  {
    if(sensKafB[0] > 50 || sensKafB[1] > 50|| sensKafB[2] > 50) {
        calcJacob(0.7,180);
    }     
    else if(biggestInd > 9  && biggestInd < 23) {
        catchBall(1);
    }
    else {
        calcJacob(0,0);
    }
  }
  else if(firstB == 1)  {
    if(sensKafF[0] > 50 || sensKafF[1] > 50) {
        calcJacob(0.7,0);
    } 
    else if(biggestInd < 5  && biggestInd > 26) {
        catchBall(1);
    }
    else {
        calcJacob(0,0);
    }
  }    
  else {
    catchBall(1);
  }
}


////////////////////////////////////////////////////////BT related\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

int otherGX,otherGY;


void supporter(){  
      gotoPoint(otherGX - 20,otherGY - 30*sign(otherGY));
}
void slave() {
    btPacket[0] = 0x99;     
    if(globalX == -1000){
        btPacket[1] = 127;
    }
    else {
        btPacket[1] = abs(globalX) /2;
    }                             
    
    if(globalY == -1000){
        btPacket[2] = 127;
    }
    else {
        btPacket[2] = abs(globalY) /2;
    }
    
    btPacket[3] = biggestInd;
    btPacket[4] = biggestValue / 8;
    btPacket[5] = 0;
    if(globalX < 0) {
        btPacket[5] +=1;
    }                   
    if(globalY < 0) {
        btPacket[5] +=2;
    }  
    btPacket[6] = isStuck();   
          
    
///////////////receive    
    otherGX = recPacket[0]*2;
    if((recPacket[4] & 0x01)) {
        otherGX *= -1;
    }
    otherGY = recPacket[1]*2;
    if((recPacket[4] & 0x02)) {
        otherGY *= -1;
    }                    
/////////////////role
    role = recPacket[2];
}
int passFlag = 0;

void master() {
    int i = 0;
    int slaveSensNum = recPacket[2];   
    int slaveSensValue = recPacket[3]*8;
    static unsigned long int lTimer = 0;
    static int lastRole = 0;
    int slaveIsStuck = 0;
    
    btPacket[0] = 0x99;
    if(globalX == -1000){
        btPacket[1] = 127;
    }
    else {
        btPacket[1] = abs(globalX) /2;
    }                             
    
    if(globalY == -1000){
        btPacket[2] = 127;
    }
    else {
        btPacket[2] = abs(globalY) /2;
    }
    
//    if(isStuck()) {
//       slaveMode = 2;  
//       
// }   
//    if( abs (otherGX - 20) < 10 && abs(otherGY - 30*sign(otherGY) ) < 10) {
//        yawS = yawSE - 90;
//        for(i = 0 ; i < 1000 ; i++ )
//        {                 
//            cmpEff = 0;
//            delay_us(100); 
//            mg[0] = mg [1] = mg [2] = mg [3] = 255;
//        }              
//        cmpEff = 1;
//        slaveMode = 1;         
//        yawS = yawSE;
//        role = 0;
//    }              
   
///////////////////////////////                       
    
    btPacket[4] = 0;      
    btPacket[5] = 0;

    
    if(globalX < 0) {
        btPacket[5] +=1;
    }                   
    if(globalY < 0) {
        btPacket[5] +=2;
    }    
    
///////////////receive    
    
    otherGX = recPacket[0]*2;
    if((recPacket[4] & 0x01)) {
        otherGX *= -1;
    }
    otherGY = recPacket[1]*2;
    if((recPacket[4] & 0x02)) {
        otherGY *= -1;
    }
    if (recPacket[0] == 127){
        otherGX = -1000;
    }
    if (recPacket[1] == 127){
        otherGY = -1000;
    }                     
     
    slaveIsStuck = recPacket[5] & 0x01;
    
/////////////////////////manage role
    if(((gTimer - lTimer) > 200) && (biggestTimer >= 4 || slaveSensValue >250) ) {             
      if((abs(biggestInd - 16) + (biggestValue - 250)/15) >= (abs(slaveSensNum - 16)+(slaveSensValue - 250)/15)) {
          role = 1;
          slaveMode = 0;
      } else if(slaveSensValue > 200){
          role = 0;
          slaveMode = 1;
      } else {
          role = 1;
          slaveMode = 0;  
      } 
      
      if(slaveSensValue > 200 && biggestTimer < 4) {
          role = 0;
          slaveMode = 1;
      }
    }
    
  /*  if (isStuck() && role == 1 && globalX > 10 && globalX != -1000 && globalY != -1000){
        role = 1;
        slaveMode = 2;           
        supporterTimer = gTimer;
    }
    
    
    if (slaveIsStuck && slaveMode == 1 && otherGX > 10 && otherGX != -1000 && otherGX != -1000){
        role = 2;
        slaveMode = 1;
        supporterTimer = gTimer;
    }
*/    
    if(role != lastRole)
    {
      lTimer = gTimer;
    }
    
    
    
    
    btPacket[3] = slaveMode;
    lastRole = role;              
}

///////////// COMM
void action() {
    if (gTimer - btTimer > 100){
        role = 1;
    }
    if(role == 0 /*&& 0*/) {
        newOut();
        outAvoiding();
    } 
    else if (role == 1){
        newOut();
        outAvoiding();
        if (globalX > 10 && globalX != -1000 && globalY != -1000  && otherGX != -1000 && otherGY != -1000){
            aimedShoot(1);       
        }    
        else {
            aimedShoot(1);
        }
    }
    else if(role == 2) {           
        supporter();
    } else if(role == 3) {
        catchBall(0);
    } else if(role == 4) {
        catchBall(1);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
void main(void)
{
// Declare your local variables here

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=Out 
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (1<<DDA0);
// State: Bit7=T Bit6=T Bit5=P Bit4=P Bit3=P Bit2=P Bit1=T Bit0=0 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (1<<PORTA5) | (1<<PORTA4) | (1<<PORTA3) | (1<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=In Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=In 
DDRB=(0<<DDB7) | (1<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (0<<DDB0);
// State: Bit7=P Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=T 
PORTB=(1<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=Out 
DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=0 Bit0=0 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=T Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Port E initialization
// Function: Bit7=In Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In 
DDRE=(0<<DDE7) | (1<<DDE6) | (1<<DDE5) | (1<<DDE4) | (1<<DDE3) | (1<<DDE2) | (0<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=T Bit0=T 
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit4=In Bit3=In Bit2=In Bit1=In Bit0=Out 
DDRG=(0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (1<<DDG0);
// State: Bit4=P Bit3=P Bit2=P Bit1=P Bit0=0 
PORTG=(1<<PORTG4) | (1<<PORTG3) | (1<<PORTG2) | (1<<PORTG1) | (0<<PORTG0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 18.000 kHz
// Mode: Normal top=0xFF
// OC0 output: Disconnected
// Timer Period: 14.222 ms
ASSR=0<<AS0;
TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (1<<CS02) | (1<<CS01) | (1<<CS00);
TCNT0=0x00;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 288.000 kHz
// Mode: Fast PWM top=0x00FF
// OC1A output: Non-Inverted PWM
// OC1B output: Non-Inverted PWM
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.88889 ms
// Output Pulse(s):
// OC1A Period: 0.88889 ms Width: 0 us
// OC1B Period: 0.88889 ms Width: 0 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (1<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 72.000 kHz
// Mode: Normal top=0xFF
// OC2 output: Disconnected
// Timer Period: 1 ms
TCCR2=(0<<WGM20) | (0<<COM21) | (0<<COM20) | (0<<WGM21) | (1<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0xB8;
OCR2=0x00;

// Timer/Counter 3 initialization
// Clock source: System Clock
// Clock value: 288.000 kHz
// Mode: Fast PWM top=0x00FF
// OC3A output: Non-Inverted PWM
// OC3B output: Non-Inverted PWM
// OC3C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.88889 ms
// Output Pulse(s):
// OC3A Period: 0.88889 ms Width: 0 us
// OC3B Period: 0.88889 ms Width: 0 us
// Timer3 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR3A=(1<<COM3A1) | (0<<COM3A0) | (1<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (1<<WGM30);
TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (1<<WGM32) | (0<<CS32) | (1<<CS31) | (1<<CS30);
TCNT3H=0x00;
TCNT3L=0x00;
ICR3H=0x00;
ICR3L=0x00;
OCR3AH=0x00;
OCR3AL=0x00;
OCR3BH=0x00;
OCR3BL=0x00;
OCR3CH=0x00;
OCR3CL=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (1<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (1<<TOIE0);
ETIMSK=(0<<TICIE3) | (0<<OCIE3A) | (0<<OCIE3B) | (0<<TOIE3) | (0<<OCIE3C) | (0<<OCIE1C);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
// INT3: Off
// INT4: Off
// INT5: Off
// INT6: Off
// INT7: Off
EICRA=(0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);

// USART0 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART0 Receiver: On
// USART0 Transmitter: On
// USART0 Mode: Asynchronous
// USART0 Baud Rate: 115200
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL0) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x09;

// USART1 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART1 Receiver: On
// USART1 Transmitter: On
// USART1 Mode: Asynchronous
// USART1 Baud Rate: 9600
UCSR1A=(0<<RXC1) | (0<<TXC1) | (0<<UDRE1) | (0<<FE1) | (0<<DOR1) | (0<<UPE1) | (0<<U2X1) | (0<<MPCM1);
UCSR1B=(1<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (1<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);
UCSR1C=(0<<UMSEL1) | (0<<UPM11) | (0<<UPM10) | (0<<USBS1) | (1<<UCSZ11) | (1<<UCSZ10) | (0<<UCPOL1);
UBRR1H=0x00;
UBRR1L=0x77;

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);

// ADC initialization
// ADC Clock frequency: 576.000 kHz
// ADC Voltage Reference: AVCC pin
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (1<<ADPS0);
SFIOR=(0<<ACME);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Bit-Banged I2C Bus initialization
// I2C Port: PORTD
// I2C SDA bit: 1
// I2C SCL bit: 0
// Bit Rate: 100 kHz
// Note: I2C settings are specified in the
// Project|Configure|C Compiler|Libraries|I2C menu.
i2c_init();

// Alphanumeric LCD initialization
// Connections are specified in the
// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
// RS - PORTA Bit 7
// RD - PORTC Bit 7
// EN - PORTC Bit 6
// D4 - PORTC Bit 5
// D5 - PORTC Bit 4
// D6 - PORTC Bit 3
// D7 - PORTC Bit 2
// Characters/line: 16
lcd_init(16);

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/512k
#pragma optsize-
WDTCR=(1<<WDCE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
WDTCR=(0<<WDCE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Global enable interrupts
// for delay sake
delay_ms(100);
init();
delay_ms(100);
findOffset();
#asm("sei")   
    
role = 1;
while (1)
      {      
      if (fastForward){
      int fastCounter = 0;
      for(fastCounter = 0; fastCounter < 30; fastCounter++) {
              #asm("wdr")
              readIMU();
              if (globalY != -1000 ) {
                  if ( globalY < -20){
                      calcJacob(1,-140);
                      delay_ms(10);                                
                  } 
                  else if (globalY > 20){
                      calcJacob(1,140);
                      delay_ms(10);
                  }
                  else{
                      calcJacob(1,180);
                      delay_ms(10);
                  }          
              } 
              else {
                  calcJacob(1,180);
                  delay_ms(10);
              } 
      }
      
      fastForward = 0; 
    }
      LED2 ^= 1;            
      #asm("wdr")
      readIMU();
      readSrf(); 
      mode();       
      sensup();        
      sensor_max();
      kafup();
      setAll(); 
      localization();
      //gotoPoint(0,0);                
      //catchBall(1);
      //goalKeeper();       
      action();
      //aimedShoot();
      //dribble();
      //LCD(12,1,laser);  
      if (laser < 250){
          PORTG |= 1;
      }              
      else {
          PORTG &= 0;
      }
      LCD(0,0,role);    
      if(isMaster)
      {
        master();
      }        
      else {
        slave();
      }   
}
}
