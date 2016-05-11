/* Arduino Wireless RC Trainning cable
* Copyright (C) 2016 by Pedro Albuquerque
*
*
* This file is part of the Arduino PATinySPI Library. This library is for
* Winbond NOR flash memory modules. In its current form it enables reading
* and writing individual data variables, structs and arrays from and to various locations;
* reading and writing pages; continuous read functions; sector, block and chip erase;
* suspending and resuming programming/erase and powering down for low power operation.
*
* This Library is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This Library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License v3.0
* along with the Arduino PATinySPI Library.  If not, see
* <http://www.gnu.org/licenses/>.
*/
/*

Code tobe used in a wireless trainner cable built with Moteino board

Wtrainer_in will receive a hardware PPM signal on pin 3 (PPMIN_PIN 3)
decode 8 channels (PPM_numChn 8)
and send the duration (in miliseconds) of each channel
by radio (moteino) within an array of integers
*/


/*
 * Version 0.1 -concept proof
 * by Pedro Albuquerque
 * 2016.03.01
 */

 /* version 1.0 - first complete version
 * by Pedro Albuquerque
 * 2016.04.25
 */



#include <SPI.h>
#include <RFM69.h>

// compile option
#define SEND_RADIO
#define DEBUG


//Defining some Radio stuff
#define PPMSTUDENTID      2    //unique for each node on same network
#define NETWORKID   400  //the same on all nodes that talk to each other
#define PPMTRAINERID   1
#define FREQUENCY   RF69_433MHZ//Match frequency to the hardware version of the radio on your Moteino
#define ENCRYPTKEY  "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW  //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME    5 // max # of ms to wait for an ack
#define LED           9 // Moteinos hsave LEDs on D9


#define SERIAL_BAUD 115200

#define PPMIN_PIN 3
#define MULTIPLIER (F_CPU/8000000)
#define PPM_numChn 8 // [0 - 7]
#define default_servo_value 1500

RFM69 radio; //Initialize the radio
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network



 // value in microseconds [1000-2000] of each channel after decoding

struct WIRELESSPPM
{
  int PWM[PPM_numChn]; // value in microseconds [1000-2000] of each channel after decoding
  //int DIF[PPM_numChn]; // diference since last change in milisenconds [1000-2000]
};

WIRELESSPPM ppm, lastPPM; // value in microseconds [1000-2000] of each channel after decoding

#define LED 9


// loop() control variables

  unsigned int radioRefresh = 15;   // time (ms) between radio packet send
  int LED_duration = 200; // on for 200 ms and then off to show its alive

  static long RADIO_timer;
  static long LED_timer;
  static boolean LED_on = false;

  char str_USBcmd = '0';
  static unsigned int  analogPWM = 0;

// ISR routine vars
volatile  unsigned int PWM_len;
volatile  unsigned int counter;
volatile  byte channel;        // for 8 channel system




// ********  DEBUG vars *****
#define DB_TXpin 4 // pin to debug transmiter time
bool DB_TXstate = LOW;

#define DB_ISRpin 5 // pin to give feedback about ISR status routine
bool DB_ISRstatus = LOW;

#define DB_CHRDpin 6 // pin to debug if channels are being read
bool DB_CHRDstatus = LOW;

byte DB_ch = 0;  // channel to debug to be read from Serial port during execution





void setup()
{

 
  #ifdef SEND_RADIO
    //Initialize the radio
    radio.initialize(FREQUENCY, PPMSTUDENTID, NETWORKID); //Initialize the radio
    #ifdef IS_RFM69HW
      radio.setHighPower(); //only for RFM69HW!
    #endif
    //radio.encrypt(ENCRYPTKEY);//Turn the radio Encryption ON
    radio.promiscuous(promiscuousMode);//Turn the radio Promiscuous mode according to what in the promiscuousMode variable
  #endif


  Serial.begin(SERIAL_BAUD);
  Serial.println("Wireless Trainner Cable IN");


  //initiallize default ppm values
  for (int i = 0; i<PPM_numChn; i++)
  {
    ppm.PWM[i] = default_servo_value;
   Serial.print(i);Serial.print(" ");Serial.println(ppm.PWM[i]);
  }
  lastPPM = ppm;


  /// Set timer 1 register to increment on every 0.5us but NO INTERRUPT generated
  /// TCNT1 will hold the counting as soon as timer1 is enabled via TCCR1B set to 8 

  TIMSK1 = 0; // disable timer 1 interrupt

  // set timer1 (16bits) to measure PPM channel values
  TCCR1A = 0;  //Stop timer1
  TCCR1B = 0;
  TCNT1 = 0;      // reset timer1 counter to 0

  // enable timer 1
  TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us (clock /8)

   
   // enable Timer1 overflow interrupt:
   //TIMSK1 |= (1 << TOIE1); //Atmega8 has no TIMSK1 but a TIMSK register


  // prepare input pin for PPM as input and attach interrupt routine
  pinMode(PPMIN_PIN, INPUT);            // define pin input for ppm
  attachInterrupt(digitalPinToInterrupt(PPMIN_PIN), readPPM, FALLING); // activate interrupt on that pin


  
  // prepare hardware feedback pin
  // LED will blink to show loop activity
  // DB_TXpin used to measure transmission 
  // DB_ISRpin used to debug. may vary along deveopment phase
  
  pinMode(LED, OUTPUT);


  // initialize DEBUG pins
  pinMode(DB_TXpin, OUTPUT); // pin to give feedback on RF transmite time 
  digitalWrite(DB_TXpin, DB_TXstate); // set it low to star with

  pinMode(DB_ISRpin,OUTPUT); // pin to give ISR status feedback
  digitalWrite(DB_ISRpin,LOW);

  pinMode(DB_CHRDpin,OUTPUT);
  digitalWrite(DB_CHRDpin,LOW);

  
  RADIO_timer= millis(); // initialize RADIO_timervar

}

void loop()
{

#ifdef DEBUG    
  if(Serial.available())
  {
    str_USBcmd = Serial.read();
    Serial.print("********************** ");Serial.println(str_USBcmd);

    if( str_USBcmd >= '0' and str_USBcmd <= '9')
    {
      DB_ch = str_USBcmd - 48;
      Serial.print("MON DB_ch***************:");Serial.println(DB_ch);

      Serial.print("PWM val:");Serial.println(lastPPM.PWM[DB_ch]);
      analogPWM= (unsigned)(lastPPM.PWM[DB_ch]-950)/4 ; // convert PWM [1000-2000] to [0-256]
      Serial.print("out val:");Serial.println(analogPWM);
    }
  }

  analogPWM= (unsigned)(lastPPM.PWM[DB_ch]-950)/4 ; // convert PWM [1000-2000] to [0-256]
  analogWrite(DB_ISRpin, analogPWM);
#endif
 
  // LED feedback code to confirm software is runing

  if(LED_timer > millis()) LED_timer = millis();
  if(millis()-LED_timer > LED_duration)
    {
      LED_timer = millis();
      switchState(&LED_on); 
      digitalWrite(LED,LED_on); 
      
    }
  // send to Radio code only send on limited interval (radioRefresh)to limit redundancy

  if (RADIO_timer> millis()) RADIO_timer= millis();
  
  if (millis() - RADIO_timer> radioRefresh ) // 33Hz as each frame should have arround 22.5ms so no point in sending sooner
  {
    #ifdef SEND_RADIO
    RADIO_timer= millis();

    #ifdef DEBUG
    // hardware feedback(FB) on pin FB_TXpin received
    digitalWrite(DB_TXpin, HIGH);
    #endif
    
    //radio.sendWithRetry(PPMTRAINERID, (const void*)(&lastPPM), sizeof(lastPPM), 1, ACK_TIME); // takes 20ms to transmit
    radio.send(PPMTRAINERID, (const void*)(&lastPPM), sizeof(lastPPM),false); // takes 5ms to transmit
    
    #ifdef DEBUG    
    digitalWrite(DB_TXpin, LOW);
    #endif
    
    #endif  // #SEND_RADIO

  }
}

bool switchState(bool* bvar )
{
  if(*bvar == false) *bvar = true; else *bvar = false;
}

void readPPM()
{
  #ifdef DEBUG
     // debug if passing this point echo on DB_CHRDpinn
    switchState(&DB_CHRDstatus); 
    digitalWrite(DB_CHRDpin,DB_CHRDstatus); 
  #endif
  
  counter = TCNT1;  // record how many cycle have passed since last timer reset
            // TCNT1 is the timer internal count register incremented
            // every CPU clock cycle divide by 8 (TCCR1B divider = 8)
            // so will increment every 8 cycles on a 16MHhz CPU = 0.5us
            
  PWM_len = (unsigned) counter/MULTIPLIER; // convert counter do us
  TCNT1 = 0;      // reset timer

  if (PWM_len > 2500 ) //sync pulses over 1910us
  {
    channel = 0;
  
  }
  
  //servo values between 0us and 2500us will end up here
  else   // only count pulse width on falling edge
  {
        
    // unexpected channel debug code 
    if (channel > PPM_numChn -1)
    {
      #ifdef DEBUG
      Serial.print("channel:");Serial.println(channel);
      Serial.print("PWM:");Serial.println(ppm.PWM[channel]);
      #endif
    }
    else  // for expected channel, save value to array 
    {
      ppm.PWM[channel] = PWM_len;

    }
       
    lastPPM = ppm;

    channel++;
    
  }
  return; // return from readPPM() (ISR routine)
}
