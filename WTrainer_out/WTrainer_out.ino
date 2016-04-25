#include <RFM69.h>




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

Code to be used in a wireless trainner cable built with Moteino board

Wtrainer_out will receive set of channel duration by radio (Moteino)
the set is in an array of int with each channel duration (in miliseconds)
a hardware PPM signal is generated on pin 3 (PPM_PIN 4 ) for the 10 channels (PPM_numChn 10)
PPM signalpolarity can be set  positive or negatve (onState 0 )
*/


/* Module:WTrainer_out
 * Version 0.1 -concept proof
 * by Pedro Albuquerque
 * 2016.03.01
 */


//#define DEBUG

#include <SPI.h>




#define PPM_numChn 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000�s)
#define PPM_PulseLen 300  //set the pulse length in us
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define multiplier (F_CPU/8000000)

#define PPM_PIN 4  //set PPM signal output pin on the arduino
#define BUZZER 5 // pin connected to a buzzer

#define Radio_failsafe 1000 // timeout failsafe duration (milisecond)

#define PPMTRAINERID 1
#define NETWORKID 400 //the same on all nodes that talk to each other
#define FREQUENCY RF69_433MHZ //match with the correct radio frequency
#define ENCRYPTKEY "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define SERIAL_BAUD 115200 //To comunicate with serial monitor for debug



/*this array holds the servo values for the ppm signal
change theese values in your code (usually servo values move between 1000 and 2000)*/
struct WIRELESSPPM
{
  int PWM[PPM_numChn];  // value in microseconds [1000-2000] of each channel after decoding
};

WIRELESSPPM ppm;

RFM69 radio;// Initialize the radio instance
  // promiscuousmode must be off for wireless programming
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network


void setup()
{
  // initialize USB Serial Port
  Serial.begin(SERIAL_BAUD); //Initialize the Serial port at the specified baud rate
  Serial.println("Wireless Trainner Cable OUT");

  //Radio Setup
  radio.initialize(FREQUENCY, PPMTRAINERID, NETWORKID); //Initialize the radio
  radio.setHighPower();// only for RFM69HW

  // radio.encrypt(ENCRYPTKEY);//Turn the radio Encryption ON
  radio.promiscuous(promiscuousMode);//Set Promiscuous mode according to what's in the promiscuousMode variable


  //initiallize default ppm values
  for (int i = 0; i<PPM_numChn; i++)
  {
    ppm.PWM[i] = default_servo_value;
  }


  // set PPM pin for output
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, !onState);  //set the PPM signal pin to the default state (off)

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  delay(500);
  digitalWrite(BUZZER, LOW);


  // set timerbased interrupt to work as CTC (compare time counter)
  cli();  // disable ALL interrupts for a moment

  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz

  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  OCR1A = 100;  // compare match register, change this

  sei();  // enable ALL interrupts that are active

  Serial.print("Start!");

}

void loop()
{
  static long timer, timer_buz;
  static boolean packetFail = true, BuzOn = false;
  int buz_duration = 100;
  static long lastTimer, lastMillis;

  char str[100];

  // if packet fail, activate Buzzer
  // packet fail is set when no packets received for Radio_failsafe ms
  if (packetFail)
    {
      packetFail = false;
      BuzOn = true;
      digitalWrite(BUZZER, HIGH);
      timer_buz = millis();
      Serial.println("Fail");
      Serial.print("timer:"); Serial.println(lastTimer);
      Serial.print("millis:"); Serial.println(lastMillis);
  }


  if (BuzOn) // if buzzer activated in some part of the code,keep it buzzing for buz_duration then switch off
  {
    if (millis() - timer_buz > buz_duration)
    {
      timer_buz = millis();
      digitalWrite(BUZZER, LOW);
      BuzOn = false;
    }

  }




  // if radio packet receive,decode it
  if(radio.receiveDone() )
  {

    if (radio.TARGETID == PPMTRAINERID && radio.DATALEN == sizeof(ppm))
    {
      // if ack is requested send it
      if (radio.ACK_REQUESTED)
      {
        radio.sendACK();
        //Serial.print(" - ACK sent");
      }
      ppm = *(WIRELESSPPM*)radio.DATA;
#ifdef DEBUG
      Serial.print("RSSI:");Serial.println(radio.RSSI);
      Serial.print("mil:"); Serial.println(millis());

      for (int i = 0; i < PPM_numChn; i++)
      {
        sprintf(str, "ch[%d] = %d\n", i, ppm.PWM[i]);
        Serial.print(str);
      }

#endif
      timer = millis();
      packetFail = false;
    }
  }
  else
  {
    if (timer > millis()) timer = millis();
    if (millis() - timer > Radio_failsafe) // packet failed
    {
      packetFail = true;
      lastTimer = timer;
      lastMillis = millis();
      #ifdef DEBUG
      Serial.print("*********      Failsafe!  ********");
      Serial.print("mil:"); Serial.println(millis());
      Serial.print("timer:"); Serial.println(timer);
      Serial.print("Bad len:"); Serial.println(radio.PAYLOADLEN);
      Serial.print(sizeof(ppm)); Serial.println(" no radio received!");
      #endif

      timer = millis();
      for (int i = 0; i < PPM_numChn; i++)
      {
        ppm.PWM[i] = default_servo_value;
      }

    }

  }



}

ISR(TIMER1_COMPA_vect)
{
  static boolean startPulse = true; // start with a pulse

  TCNT1 = 0;  // start timer counter to 0

  if (startPulse) {  //start pulse
    digitalWrite(PPM_PIN, onState);
    OCR1A = PPM_PulseLen * multiplier - 1;  // load compare register with duration amount
    startPulse = false;
  }
  else {  //end pulse and calculate when to start the next pulse
    static byte channel;
    static unsigned int calc_rest;

    digitalWrite(PPM_PIN, !onState);
    startPulse = true;

    if (channel >= PPM_numChn) {
      channel = 0;
      calc_rest  +=  PPM_PulseLen;//
      OCR1A = (PPM_FrLen - calc_rest) * multiplier - 1;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm.PWM[channel] - PPM_PulseLen) * multiplier - 1;
      calc_rest += ppm.PWM[channel];
      channel++;
    }
  }
}
