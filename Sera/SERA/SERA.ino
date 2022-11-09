/**
 * @example CompButton.ino
 * 
 * @par How to Use
 * This example shows that when the button component on the Nextion screen is released,
 * the text of this button will plus one every time.      
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/7/10
 * @updated 2016/12/25 bring HMI up to v0.32 to avoid too old issues
 * @convert by Patrick Martin, no other changes made
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#include <stdlib.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include "Nextion.h"


////////////////////////////////////////////////// DIGITAL INPUT ///////////////////////////////
const int Q0_NMOS_1 = 5;
const int Q3_NMOS_2 = 6;
const int Q2_Relay = 7;
const int Q1_Relay_HEATER = 8;
const int Q4_LIGHT = 9;
const int Q5_FAN = 10;
const int Q6_SULAMA = 11;
const int Q7 = 12; 
////////////////////////////////////////////////// ANALOG READ ///////////////////////////////
#define DI3 3 
#define AI0_pin A0 // 
#define AI1_pin A1 //
#define AI2_pin A2 //
#define AI3_pin A3 //
#define AI6_pin A6 //

int PWM = 0; 

bool BlinkTurn,BlinkFog,BlinkRev;
/////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////PWM ///////////////////////////////
bool PWM_Enable=false;
unsigned long PWM1_Frequency_Top,PWM1_Frequency_Max,PWM_TAIL1_Duty_Value,PWM_TAIL2_Duty_Value,PWM_FREQ;
int Timer2_ms,ADC_Current_Dead_Zone=10;//(ADC_Current_Dead_Zone *10ms) waits current protection activation .
float PWM1_Frequency_Value=200; // pwm fREQUENCY
int DutyCycle_LED=0; //  Tail 1 duty cycle setting here
int DutyCycle_CLIMATE_FAN=100;//   Tail 2 duty cycle setting here
int FreqMode=0;

bool TURN,DRL,POS;
int Timer_ADJ,Timer_TURN;
bool Turn_On;
bool ADJ_On;
float PWM_TURN_Freq =1.5;  // 1HZ - 1.5 HZ
float  PWM_TURN_Duty =50 ;
float PWM_ADJ_Freq  =200;  // 50HZ - 500 HZ
float   PWM_ADJ_Duty  =100 ; // %100 dc , 1-100 arası değiştirilebilir.
////////////////////////////////////////////calc///////////////////////////////////
int PWM_TURN_Freq_Value =100*1/PWM_TURN_Freq; //interrupt 100us için
int PWM_TURN_Duty_Value_H =PWM_TURN_Freq_Value*(PWM_TURN_Duty/100); //interrupt 100us için
int PWM_TURN_Duty_Value_L =PWM_TURN_Freq_Value*((100-PWM_TURN_Duty)/100); //interrupt 100us için

//////////////////////////////////////////////////EEPROM RW ///////////////////////////////
short int EEAdress_FreqMode=2,
          EEAdress_PWM1_Frequency_Value=4,
          EEAdress_DutyCycle=8;           
float EERead_PWM1_Frequency_Value; 
int EERead_DutyCycle,EERead_FreqMode;

int ACTCurrent;  
int  Temp,Temp_Protect_Value=61; // Kutu iç sıcaklığı max değer
float Temperature;
int Voltage_Protect_Calculation=0;
float  Voltage;
float Vmax=26; // voltaj değeri ayarı
float ADC_to_Voltage=Vmax/1024;
float Max_Supply_Voltage_Value= 18; // Power Supply Max AkÄ±m deÄŸeri

int Analog_Read_times=6; // Analog duty(+) iÃ§inde kaÃ§ kez okuma yapacak?
unsigned int Analog_Read_Current,Analog_Current_Result;
unsigned long Analog_Current_Total,Shortcut_Current_Read_Total;
unsigned int toggle = 0;  //used to keep the state of the LED
unsigned int count = 0;   //used to keep count of how many interrupts were fired
float Current;
float Imax=10.15*1.00;  //1024 e karÅŸÄ±lÄ±k gelen AkÄ±m deÄŸeri Opamp 2*Kv (CAlibre %1 )/*9.47*/
float Max_Supply_Current_Value= 6; // Power Supply Max AkÄ±m deÄŸeri
int Current_Limit_Value= (Max_Supply_Current_Value / Imax)*1.1*1024; // Max yÃ¼kÃ¼n %20 FazlasÄ± //ilk kalkmada %15 anma akÄ±mÄ± 100 ms boyunca Ã§eker
int Shortcut_Current_Result,Shortcut_count,Shortcut_Time_ms=2; //5// ksadevre koruma sÃ¼resi    ideal < 10ms
bool Shortcut_Active,Current_Read_Enable=false;
int Opennig=0;

int TempCounter=0;
bool tempAlarm=false;


/*
 * Declare a button object [page id:0,component id:1, component name: "b0"]. 
 */
NexButton b0 = NexButton(1, 1, "b0");
NexButton b1 = NexButton(1, 2, "b1");
NexButton b2 = NexButton(1, 3, "b2");
NexButton b3 = NexButton(1, 4, "b3");
NexButton b4 = NexButton(1, 5, "b4");
NexButton b5 = NexButton(1, 6, "b5");
NexButton b6 = NexButton(1, 7, "b6");
NexButton b7 = NexButton(1, 8, "b7");

NexDSButton P0_BTN_AUTO = NexDSButton(0, 5, "bt0");

// Declare a button object [page id:0,component id:1, component name: "t0"]. 
NexText P0_TEMP       = NexText(0, 8, "P0_TEMP"); // TEMP
NexText P0_NEM        = NexText(0, 9, "P0_NEM"); // NEM
NexText P0_FAN        = NexText(0, 10, "P0_FAN"); // FAN HIZI
NexText P0_LIGHT      = NexText(0, 13, "P0_LIGHT"); // LIGHT SENSOR

NexText P1_A3         = NexText(1, 17, "P1_A3"); // A3
NexText P1_A2         = NexText(1, 18, "P1_A2"); // A2
NexText P1_A1         = NexText(1, 19, "P1_A1"); // A1
NexText P1_A0         = NexText(1, 20, "P1_A0"); // A0
NexText P1_SUN        = NexText(1, 23, "P1_SUN"); // A0
NexText P1_DI3        = NexText(1, 21, "P1_DI3");// DI3 

NexText P2_SUNSET     = NexText(2, 4,  "P2_SUNSET"); // SUN SET
NexText P2_NEMSET     = NexText(2, 5,  "P2_NEMSET"); // SUN SET
NexText P2_HEATERSET  = NexText(2, 13, "P2_HEATERSET"); // SUN SET
NexText P2_COOLINGSET = NexText(2, 14, "P2_COOLINGSET"); // SUN SET



NexText P4_FANWAIT    = NexText(4, 8,  "P4_FANWAIT"); // FAN WAIT 
NexText P4_FANRUN     = NexText(4, 11, "P4_FANRUN"); // FAN RUN 
NexText P4_WATERWAIT  = NexText(4, 9,  "P4_WATERWAIT"); // WATER WAIT
NexText P4_WATERRUN   = NexText(4, 12, "P4_WATERRUN"); // WATER RUN
NexText P4_HEATERWAIT = NexText(4, 10, "P4_HEATERWAIT"); // WATER WAIT
NexText P4_HEATERRUN  = NexText(4, 13, "P4_HEATERRUN"); // HEATER RUN



//NexSlider h0 = NexSlider(0, 10, "h0");
//NexText t0 = NexText(0, 11, "t0"); // ALARM
//NexText t1 = NexText(0, 12, "t1"); // fREQUENCY
//NexText t4 = NexText(0, 15, "t4"); // DUTY%
char buffer[100] = {0};

/*
 * Register a button object to the touch event list.  
 */
NexTouch *nex_listen_list[] = 
{
    &b0, // STOP
    &b1, // TAIL
    &b2, // TURN
    &b3, // REVERSE
    &b4, // FOG
    &b5, // %90
    &b6, // %50
    &b7, // %10
    &P0_BTN_AUTO,
//    &h0, // %SLİDER
    NULL
};

//void h0PopCallback(void *ptr)
//{       
//    uint32_t number = 0;
//    char Duty[10] = {0};
//    
//    //dbSerialPrintln("h0PopCallback");
//
//    h0.getValue(&number);
//    utoa(number, Duty, 10);
//    t4.setText(Duty);
//    PWM=number;
//}

/*
 * Button bOn component pop callback function. 
 * When the ON button is released, the LED turns on and the state text changes. 
 */ 

bool  toogle_Q0_NMOS_1;
bool  toogle_Q3_NMOS_2;
bool  toogle_Q1_Relay_HEATER;
bool  toogle_Q2_Relay;
bool  toogle_Q4_LIGHT;
bool  toogle_Q5_FAN;
bool  toogle_Q6_SULAMA;
bool  toogle_Q7;

void b0PopCallback(void *ptr) {
//  t0.setText("ACTIVE!!");
    if(toogle_Q0_NMOS_1) toogle_Q0_NMOS_1=0; else toogle_Q0_NMOS_1=true;
    digitalWrite(Q0_NMOS_1, toogle_Q0_NMOS_1);    
}
void b1PopCallback(void *ptr) {
    if(toogle_Q1_Relay_HEATER) toogle_Q1_Relay_HEATER=0; else toogle_Q1_Relay_HEATER=true;
   //if(toogleTAIL) PWM_ON();     else PWM_OFF();
    digitalWrite(Q1_Relay_HEATER, toogle_Q1_Relay_HEATER);
}
void b2PopCallback(void *ptr) {
    if(toogle_Q2_Relay) toogle_Q2_Relay=0; else toogle_Q2_Relay=true;
    digitalWrite(Q2_Relay, toogle_Q2_Relay);
  //BlinkTurn=true;
}
void b3PopCallback(void *ptr) {
    if(toogle_Q3_NMOS_2) toogle_Q3_NMOS_2=0; else toogle_Q3_NMOS_2=true;
  digitalWrite(Q3_NMOS_2, toogle_Q3_NMOS_2);
}
void b4PopCallback(void *ptr) {
    if(toogle_Q4_LIGHT) toogle_Q4_LIGHT=0; else toogle_Q4_LIGHT=true;
  digitalWrite(Q4_LIGHT, toogle_Q4_LIGHT);
}
void b5PopCallback(void *ptr) {
    if(toogle_Q5_FAN){
      toogle_Q5_FAN=0; 
      b5.Set_background_color_bco(48631);
    }else{ 
      toogle_Q5_FAN=true;
      b5.Set_background_color_bco(63488);
    }
  digitalWrite(Q5_FAN, toogle_Q5_FAN);
}
void b6PopCallback(void *ptr) {
    if(toogle_Q6_SULAMA) toogle_Q6_SULAMA=0; else toogle_Q6_SULAMA=true;
  digitalWrite(Q6_SULAMA, toogle_Q6_SULAMA);
}
void b7PopCallback(void *ptr) {
    if(toogle_Q7) toogle_Q7=0; else toogle_Q7=true;
  digitalWrite(Q7, toogle_Q7);
}

void bt0PopCallback(void *ptr)
{}

bool D_in3;
int AI0_Value;
int AI1_Value;
int AI2_Value;
int AI3_Value;
int AI6SUN_Value;
  
void inputOku(){
    //#define DI3 3 
    //#define AI0_pin A0 // 
    //#define AI1_pin A1 //
    //#define AI2_pin A2 //
    //#define AI3_pin A3 //
    //#define AI6_pin A6 //
    char Sayi[5] = {0};
    //  itoa(rakam, onluksistem, 10); //    utoa(number, Duty, 10);//  char Duty[10] = {0};  t4.setText(Duty);//    itoa(i, Sayi, 10);
        
      // read the state of the pushbutton value:
    D_in3 = digitalRead(DI3);
    AI0_Value = analogRead(AI0_pin);
    AI1_Value = analogRead(AI1_pin);
    AI2_Value = analogRead(AI2_pin);
    AI3_Value = analogRead(AI3_pin);
    AI6SUN_Value = analogRead(AI6_pin);
    
    itoa(AI0_Value, Sayi, 10);    P1_A0.setText(Sayi);
    itoa(AI1_Value, Sayi, 10);    P1_A1.setText(Sayi);
    itoa(AI2_Value, Sayi, 10);    P1_A2.setText(Sayi);
    itoa(AI3_Value, Sayi, 10);    P1_A3.setText(Sayi);
    itoa(AI6SUN_Value, Sayi, 10); P1_SUN.setText(Sayi);
    
    if(D_in3) P1_DI3.setText("1"); else P1_DI3.setText("0");
  
}
void setup(void)
{    Serial.begin(9600);
    /* Set the baudrate which is for debug and communicate with Nextion screen. */
    nexInit();

    /* Register the pop event callback function of the current button component. */
    b0.attachPop(b0PopCallback, &b0);
    b1.attachPop(b1PopCallback, &b1);
    b2.attachPop(b2PopCallback, &b2);
    b3.attachPop(b3PopCallback, &b3);
    b4.attachPop(b4PopCallback, &b4);
    b5.attachPop(b5PopCallback, &b5);
    b6.attachPop(b6PopCallback, &b6);
    b7.attachPop(b7PopCallback, &b7);    
 // h0.attachPop(h0PopCallback);    

    //dbSerialPrintln("setup done");

    
    pinMode(DI3, INPUT);
    pinMode(AI0_pin, INPUT);
    pinMode(AI1_pin, INPUT); 
    
    pinMode(Q0_NMOS_1, OUTPUT);
    pinMode(Q1_Relay_HEATER, OUTPUT);
    pinMode(Q2_Relay, OUTPUT);
    pinMode(Q3_NMOS_2, OUTPUT);
    pinMode(Q4_LIGHT, OUTPUT);
    pinMode(Q5_FAN, OUTPUT);
    pinMode(Q6_SULAMA, OUTPUT);
    pinMode(Q7, OUTPUT);


///////////////////////////////////// TÄ°MER1 PWM MODU  /////////////////////////////////////////////
// WGM13  WGM12  WGM11  WGM10   MODE
//    0     0      0      0     NORMAL
//    0     0      0      1     PWM,PHASE CORRECT 8-BIT
//    0     0      1      0     PWM,PHASE CORRECT 9-BIT
//    0     0      1      1     PWM,PHASE CORRECT 10-BIT
//    0     1      0      0     CTC  OCR1A
//    0     1      0      1     FAST PWM, 8-BIT
//    0     1      1      0     FAST PWM, 9-BIT
//    0     1      1      1     FAST PWM, 10-BIT
//    1     0      0      0     PWM,PHASE AND FREQUENCY CORRECT ICR1
//    1     0      0      1     PWM,PHASE AND FREQUENCY CORRECT OCR1A
//    1     0      1      0     PWM,PHASE  CORRECT ICR1
//    1     0      1      1     PWM,PHASE  CORRECT OCR1A
//    1     1      0      0     CTC  ICR1
//    1     1      0      1     RESERVED
//    1     1      1      0     FAST PWM, ICR1
//    1     1      1      1     FAST PWM, OCR1A
///////////////////////////////////////////////////////////////////////////////////////////////
   //  DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
 
//    TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
//    // set none-inverting mode
//    TCCR1A &= ~(1<<WGM10);  // bOŞ OLUNCA SİSTEM 0LAMIYOR.
//    TCCR1A |= (1 << WGM11);
//    TCCR1B |= (1 << WGM12)|(1 << WGM13);
    // set Fast PWM mode using ICR1 as TOP
// CS12 CS11 CS10 ///////////////////////////////////////////////////////////
//  0     0   1    OSC/1    tam deÄŸer olmadÄ±ÄŸÄ± iÃ§in KullanmÄ±lamaz
//  0     1   0    OSC/8  ///  sÃ¼re olarak 0.5us   pmax=32ms
//  0     1   1    OSC/64 ///  sÃ¼re olarak 250 us  pmax= 262ms
//  1     0   0    OSC/256  tam deÄŸer olmadÄ±ÄŸÄ± iÃ§in   KullanmÄ±lamaz
//  1     0   1    OSC/1024 tam deÄŸer olmadÄ±ÄŸÄ± iÃ§in   KullanmÄ±lamaz
//  1     1   0    EXT T1 RISING EDGE
//  1     1   0    EXT T1 FALLING EDGE
////////////////////////////////////////////////////////////////////////////////////////////////////////    
// prescale 8 olarak ayarlanÄ±yor.
//   TCCR1B &= ~(1<<CS10);
//   TCCR1B |=  (1<<CS11);
//   TCCR1B &= ~(1<<CS12);
//prescale 64 olarak ayarlanacak
//     TCCR1B |=  (1<<CS10);
//    TCCR1B |=  (1<<CS11);
//    TCCR1B &= ~(1<<CS12);
//prescale 256 olarak ayarlanacak
//    TCCR1B &= ~(1<<CS10);
//    TCCR1B &= ~(1<<CS11);
//    TCCR1B |=  (1<<CS12);
//    ICR1 = 100-1;     /*10000-1 = 200HZ  20000-1=100HZ @ prescale 8*/  /*ICR1 = 9; 200khz*/ /*ICR1 = 99; 20khz*/
                      /*10000 = 200HZ  20000=100HZ @ prescale 8*/ 
                      /*prescale 256 iken 1hz iÃ§in ICR1 = 62500*//*prescale 256 iken 1.5hz iÃ§in ICR1 = 41667*/
    //OCR1A =1; //65535 pwm pin9 duty cycle
    //OCR1B =1; //65535 pwm pin10 duty cycle
    
//   TIMSK1 |= (1 << TOIE1);  // pwm lik baÅŸlangÄ±Ã§ yakalama
   Timer2_Setup();
    Set_Adc_Clock();
  //  analogReference(INTERNAL); //1v1 ref kullanÄ±lmazsa yanlÄ±ÅŸ deÄŸerler okur
    //PWM_OFF();
   PWM_Setting();
  // PWM_ON();
//  int i=analogRead(Current_pin); // Analog interrupt vektÃ¶r atamasÄ± Mutlaka burda yapÄ±lmalÄ±dÄ±r.
}
int i;
char Sayi[5] = {0};
void loop(void)
{   
  i++;
  delay(100);
    /*
     * When a pop or push event occured every time,
     * the corresponding component[right P id and component id] in touch event list will be asked.
     */
    nexLoop(nex_listen_list);
         
//  delay(10000);PWM_OFF();   delay(10000);PWM_ON();   
//  Temp_Protection();
 
//  itoa(rakam, onluksistem, 10); //    utoa(number, Duty, 10);
//  char Duty[10] = {0};  t4.setText(Duty);
    itoa(i, Sayi, 10);
//    P0_TEMP.setText(Sayi);
//    P0_NEM.setText(Sayi);
//    P0_FAN.setText(Sayi);
//    P0_LIGHT.setText(Sayi);
//    
//    P2_SUNSET.setText(Sayi);
//    P2_NEMSET.setText(Sayi);
//    P2_HEATERSET.setText(Sayi);
//    P2_COOLINGSET.setText(Sayi);
//    
//    P4_FANWAIT.setText(Sayi);
//    P4_FANRUN.setText(Sayi);
//    P4_WATERWAIT.setText(Sayi);
//    P4_WATERRUN.setText(Sayi);
//    P4_HEATERWAIT.setText(Sayi);
//    P4_HEATERRUN.setText(Sayi); 
inputOku();

}

void PWM_ON(){
    Opennig=0;
    PWM_Enable=true;        // ?tIMER REG E Ä°CR DEÄžERÄ°NÄ° DENE
//   pinMode(TAIL_PIN1, OUTPUT);  //digitalWrite(TAIL_PIN1, HIGH); //FreqMode_Selection();  //
//   pinMode(TAIL_PIN2, OUTPUT);
    TCNT1=0; // timer iÃ§inde deÄŸer varsa ilk cycle full Ã§Ä±kÄ±ÅŸ verebiliyor.
    TCCR1A |= (1<<COM1A1); // PWM Pin9 ON
    TCCR1A |= (1<<COM1B1); // PWM Pin10 ON
    Timer2_Setup();  
    //Get_EEprom_First_Values();

}

  void PWM_OFF(){
     PWM_Enable=false; 
     Current_Read_Enable=false;
     TCCR1A &= ~(1<<COM1A1); //PWM Pin9 Close
     TCCR1A &= ~(1<<COM1B1);  //PWM Pin10 Close
//    digitalWrite(TAIL_PIN1, LOW); //DDR ye yazar; //PORTB &= ~(1<<PB2); /* porta yazar pin10 PB2*/ 
//    digitalWrite(TAIL_PIN2, LOW);
}

void PWM_Setting(){ // PWM1_Frequency_Value,DutyCycle_LED

    Edit_Prescale();
    PWM1_Frequency_Top = PWM1_Frequency_Max/ PWM1_Frequency_Value;
    PWM_TAIL1_Duty_Value = (PWM1_Frequency_Top*DutyCycle_LED)/100;
    PWM_TAIL2_Duty_Value = (PWM1_Frequency_Top*DutyCycle_CLIMATE_FAN)/100;
    ICR1 = PWM1_Frequency_Top-1;     /*10000-1 = 200HZ  20000-1=100HZ @ prescale 8*/  /*ICR1 = 9; 200khz*/ /*ICR1 = 99; 20khz*//*10000 = 200HZ  20000=100HZ @ prescale 8*/ 
                                      /*prescale 256 iken 1hz iÃ§in ICR1 = 62500*//*prescale 256 iken 1.5hz iÃ§in ICR1 = 41667*/
    /*OCR1B*/ OCR1A =PWM_TAIL1_Duty_Value; //65535 pwm pin10 duty cycle //OCR1A =1; //65535 pwm pin9 duty cycle
              OCR1B =PWM_TAIL2_Duty_Value; //65535 pwm pin10 duty cycle //OCR1A =1; //65535 pwm pin9 duty cycle
    if(ICR1<OCR1A) OCR1A=ICR1; // Safety for pwm Bugg
}

void Edit_Prescale(){

   TCCR1B &= ~(1<<CS10);
   TCCR1B |=  (1<<CS11);
   TCCR1B &= ~(1<<CS12);
   PWM1_Frequency_Max=(16000000/8);  //0.5us per pulse ++ , 1 saniyede 2.000.000 puls
}

//void Get_EEprom(){
//    EEPROM.get(EEAdress_FreqMode, EERead_FreqMode);
//    EEPROM.get(EEAdress_PWM1_Frequency_Value, EERead_PWM1_Frequency_Value);
//    EEPROM.get(EEAdress_DutyCycle, EERead_DutyCycle);     
//}
//
//void Set_EEprom(){
//    for (int i = 0 ; i < 20 ; i++) { EEPROM.write(i, 0); } // Eeprom temizlenmeyince hatalÄ± bilgi veriyor
//    EEPROM.put(EEAdress_FreqMode, FreqMode);
//    EEPROM.put(EEAdress_PWM1_Frequency_Value, PWM1_Frequency_Value);
//    EEPROM.put(EEAdress_DutyCycle, DutyCycle_LED);    
//}
//
//void Get_EEprom_First_Values(){
//    Get_EEprom();
//    FreqMode = EERead_FreqMode;
//    PWM1_Frequency_Value = EERead_PWM1_Frequency_Value;
//    DutyCycle_LED = EERead_DutyCycle;
//    PWM_Setting();
//    // Serial.print(FreqMode);Serial.print("   ");Serial.print(PWM1_Frequency_Value);;Serial.print("   ");Serial.println(DutyCycle_LED);
//}

int resultNumber;
bool toogleTEST;

void Timer2_Setup(){
  //Setup Timer2 to fire every 1ms
    Timer2_ms=0;  
    TCCR2B = 0x00;        //Disbale Timer2 while we set it up
    TCCR2A |=(1<<WGM21); // CTC mode seÃ§imi yapÄ±lÄ±yor.
    TCCR2B |=(1<<CS20); // prescale 1024 olarak ayarlanÄ±yor.  // 1024 Ã¼n altÄ±nda yaparsan SÄ°STEM SÃœREKLÄ° RESET ALIR.
    TCCR2B |=(1<<CS21);
    TCCR2B |=(1<<CS22);
    TCNT2=0;
    TIMSK2 |= (1<<TOIE2); // Timer2 Overflow Interrupt Enable
}

//Timer2 Overflow Interrupt Vector, called every 1ms Timer2_Setup
ISR(TIMER2_OVF_vect) { 
  ////////////////////////////////////iNTRRUPT PWM AYARI ///////////////////////////////////////////////////
   TCNT2 = 99;           //Reset Timer to 99 out of 255  10ms
//if(toogleTURN){
//  digitalWrite(TURN_PIN, Turn_On);
// // digitalWrite(PWM_OC0A_TURN,Turn_On);
//  //  digitalWrite(PWM_OC0A_TURN,toogleTEST);
//
//    //    if(toogleTEST) toogleTEST=0; else toogleTEST=true;
//     
//     if(Turn_On){ if(++Timer_TURN>PWM_TURN_Duty_Value_H) {Turn_On=false; Timer_TURN=1;}   } 
//     if(!Turn_On){ if(++Timer_TURN>PWM_TURN_Duty_Value_L) {Turn_On=true;Timer_TURN=1;}    }
////     if(ADJ_On){ if(++Timer_ADJ>PWM_ADJ_Duty_Value_H) {ADJ_On=false; Timer_ADJ=1;}   } 
////     if(!ADJ_On){ if(++Timer_ADJ>PWM_ADJ_Duty_Value_L) {ADJ_On=true;Timer_ADJ=1;}    }
//
//    // if(!TURN) Turn_On=false;
//     if(!DRL && !POS) ADJ_On=false;
//}     /////////////////////////////////////////////////////////////////////////////////////////////////////
//    int Protect_Current_Total=0;
//    if(Current_Read_Enable){
//        //Protect_Current_Total +=  analogRead(Current_pin);
//        Protect_Current_Total +=  analogRead(Current_pin);
//        Protect_Current_Total +=  analogRead(Current_pin);
//        Protect_Current_Total=Protect_Current_Total/2;
//        if((Protect_Current_Total>Current_Limit_Value)) { ShortcutProtection();} 
//        Protect_Current_Total=0;
//    }
//    else if(PWM_Enable){
//        if(Timer2_ms++>ADC_Current_Dead_Zone){ 
//        Timer2_ms=0; 
//        //  TIMSK2 &= ~(1<<TOIE2); //Tmr2 interrupt disable
//        //  toogleTEST=0;
//        Current_Read_Enable=true;
//        }
//   }    
  TCNT2 = 99;           //Reset Timer to 99 out of 255  10ms
// TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
// TIMSK2 &= ~(1<<TOIE2); // Timer2 Overflow Interrupt Disable
  
  if(toogle_Q7){
      DutyCycle_LED= analogRead(A6);
      if( DutyCycle_LED>200){
        DutyCycle_LED=1500; //65535 için
        PWM_Setting();
      }
      else{
        DutyCycle_LED=0; //65535 için
        PWM_Setting();
      }
      PWM_ON();
  }
  else{
        DutyCycle_LED=0; //65535 için
        PWM_Setting();
  }
  
  if(toogle_Q6_SULAMA){
        DutyCycle_CLIMATE_FAN=1500; //65535 için
        PWM_Setting();
        PWM_ON();
  }
  else{
        DutyCycle_CLIMATE_FAN=0; //65535 için
        PWM_Setting();
  }  

}

void Set_Adc_Clock(){
  // ADPS2 ADPS1 ADPS0 ///////ADC conversion clock frequency divider////////////
//    0      0     1    OSC/2   //7,3 us  
//    0      1     0    OSC/4   //9 us 
//    0      1     1    OSC/8   //12,6 us
//    1      0     0    OSC/16  //19 us
//    1      0     1    OSC/32  //32 us
//    1      1     0    OSC/64  //60 us
//    1      1     1    OSC/128 //120 us 
 ADCSRA|= (1<<ADPS2); //ADC conversion clock frequency divider to 64. ADC Clock 16MHz/64 = 250kHz
 ADCSRA&= ~(1<<ADPS1);
 ADCSRA&= ~(1<<ADPS0);
}



void CurrentCalc(){  
   Current= (Analog_Current_Result*Imax)/1024*1.06/*1.06 hata katsayÄ±sÄ±*/;  
}

void ShortcutProtection(){
   if(++Opennig>0) {Opennig=0;
    // PWM_OFF();

//             if(toogleTEST) toogleTEST=0; else toogleTEST=true;
//             digitalWrite(PWM_TEST,toogleTEST);
   }
}

void Temp_Protection(){
   //Temp = analogRead(Temp_pin);    
   readTemp();
   //Serial.println(Temperature,1);
   if(Temperature>Temp_Protect_Value){ TempCounter++;
   }else TempCounter=0;

  if(TempCounter>3){ 
    TempCounter=0;
    tempAlarm=true;
    //PWM_OFF();

  }
}

void Voltage_Protection(){
  bool VoltageAlarm;
  Voltage=Voltage*ADC_to_Voltage;
  if(Voltage>18 || Voltage<10 ){
    if(Voltage_Protect_Calculation++>3){
      VoltageAlarm=true;
      //PWM_OFF();
  }
  }else Voltage_Protect_Calculation=0;
  
}

void readTemp(){
    int Vo; // Integer value of voltage reading
    float R = 22000.0; // Fixed resistance in the voltage divider
    /*
    * Steinhart-Hart Equation coefficients.
    * Calculated from datasheet reference temps/resistances.
    */
    float c1 = 1.3084634E-03;
    float c2 = 2.344772E-04;
    float c3 = 1.04177209450756E-07;
    float logRt,Rt,T;
    int k=analogRead(A0);
    Vo = 1023 - k/4.5;  // 1v1 ref için Analog değer 4.5 e bölünür.
    Rt = R*( 1023.0 / (float)Vo - 1.0 );    // Calculate thermister resistance
    logRt = log(Rt);
 
    // Apply Steinhart-Hart equation.
    T = ( 1.0 / (c1 + c2*logRt + c3*logRt*logRt*logRt ) );
   Temperature= T - 273.15;
    
    Serial.print(" ");    Serial.print(k);    Serial.print(" ");    Serial.print(Rt);   
    Serial.print(" Temperature : ");
    Serial.println(Temperature,1);

//    return Temperature;
}
