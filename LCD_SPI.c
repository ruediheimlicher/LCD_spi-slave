//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include "adc.c"
#include "lcd.c"
#include "defines.h"

//***********************************
//LCD_SPI_PWM							*
//									*
//***********************************

#define TASTEPIN              1        // Startet Beleuchtung
#define ADCPIN                2        // Eingang Pot
#define PWMPIN                3        // Schaltet PWM
#define OSZIPIN               4


#define WAIT                  0

#define SPI_DELAY             5

#define CHECK                 1 // in ISR gesetzt, resetcount soll erhoeht werden






volatile uint8_t inindex=0;
volatile uint8_t spi_rxdata=0;
volatile uint8_t isrcontrol=0;


// ring buffer
#define RING_SIZE 80
typedef uint8_t ring_pos_t;
volatile ring_pos_t ring_head;
volatile ring_pos_t ring_tail;
volatile char ring_data[RING_SIZE];


volatile uint8_t cmd=0; // command
volatile uint8_t par=0; // parameter
volatile uint8_t col=0;
volatile uint8_t line=0;

volatile uint8_t data=0;
volatile uint8_t datastring[20]={};

volatile int16_t spidata=0;

volatile uint8_t spistatus=0;


void delay_ms(unsigned int ms);

volatile uint16_t	loopcount0=0;
volatile uint16_t	loopcount1=0;

volatile uint16_t	pwmcount=0; // Zaehlt wenn WAIT gesetzt ist: Delay fuer Relais

volatile uint16_t	timeoutcounterL=0; // Zeit_L, bis die Beleuchtung ausgeschaltet wird
volatile uint16_t	timeoutcounterH=0; // Zeit_H, bis die Beleuchtung ausgeschaltet wird

volatile uint8_t statusflag=0;

volatile uint8_t spi_rxbuffer[8] = {};
volatile uint8_t	spicount=0;
volatile uint8_t	ANZEIGE_LO_counter=0;

int add(char c)
{
   ring_pos_t next_head = (ring_head + 1) % RING_SIZE;
   if (next_head != ring_tail)
   {
      /* there is room */
      ring_data[ring_head] = c;
      ring_head = next_head;
      return 0;
   }
   else
   {
      /* no room left in the buffer */
      return -1;
   }
}

int remove(void)
{
   if (ring_head != ring_tail)
   {
      int c = ring_data[ring_tail];
      ring_tail = (ring_tail + 1) % RING_SIZE;
      return c;
   }
   else
   {
      return -1;
   }
}

void slaveinit(void)
{
   
   //CLKPR |= (1<<3);
   LOOPLEDDDR |= (1<<LOOPLEDPIN);
   
   OSZIDDR |= (1<<OSZI_PULS_A);        // Ausgang
   OSZIPORT |= (1<<OSZI_PULS_A);       // HI
   OSZIDDR |= (1<<OSZI_PULS_B);        // Ausgang
   OSZIPORT |= (1<<OSZI_PULS_B);       // HI
   
   ADCDDR &= ~(1<<PWM_ADC_PIN); // Eingang von PWM-Pot
   
   STARTDDR &= ~(1<<START_PIN); // Eingang fuer Start-Taste
   STARTPORT |= (1<<START_PIN); // HI
   
   PWMDDR |= (1<<PWM_OUT_PIN);	// Ausgang fuer PWM
   PWMPORT |= (1<<PWM_OUT_PIN);	// HI
   
   CMD_DDR |= (1<<LCD_ENABLE_PIN);
   CMD_PORT &= ~(1<<LCD_ENABLE_PIN); // LO
   
   CMD_DDR  |= (1<<LCD_RSDS_PIN);
   CMD_PORT &= ~(1<<LCD_RSDS_PIN); // LO
   
   LCD_DDR = 0xFF;
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms)
	{
		_delay_ms(0.96);
		ms--;
	}
}
/*
 void setSPI_Teensy(void)
 {
 uint8_t spidelay = 5;
 uint8_t spiwaitdelay = 4;
 
 uint8_t outindex=0;
 SPI_PORT &=  ~(1<<SPI_CS); // CS LO, Start, Slave soll erstes Byte laden
 _delay_us(spidelay);
 
 //SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start, Slave soll erstes Byte laden
 //_delay_us(1);
 
 //PORTB &= ~(1<<0);
 
 
 
 for (outindex=0;outindex < SPI_BUFFERSIZE;outindex++)
 //for (outindex=0;outindex < 4;outindex++)
 {
 ////OSZI_A_LO;
 // _delay_us(spidelay);
 SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start, Slave soll erstes Byte laden
 _delay_us(spidelay);
 
 SPDR0 = spi_txbuffer[outindex];
 
 while(!(SPSR0 & (1<<SPIF0)) && spiwaitcounter < WHILEMAX)
 {
 spiwaitcounter++;
 }
 spiwaitcounter=0;
 //_delay_us(spidelay);
 //uint8_t incoming = SPDR0;
 
 if (outindex == 0) // slave warten lassen, um code zu laden
 {
 uint8_t incoming = SPDR0;
 
 _delay_us(spiwaitdelay);
 }
 else if (outindex ==1) // code lesen, spistatus steuern
 {
 spi_rxbuffer[0] = SPDR0;
 
 if (spi_rxbuffer[0] & (1<<TEENSY_SEND))
 {
 spistatus |= (1<< TEENSY_SEND);
 _delay_us(spiwaitdelay);
 }
 else
 {
 spistatus &= ~(1<< TEENSY_SEND);
 spistatus &= ~(1<< TEENSY_RECV);
 }
 }
 else if (spistatus & (1<< TEENSY_SEND))
 {
 if (spi_rxbuffer[0] & 0x7F)
 {
 spi_rxbuffer[outindex-1] = SPDR0; // erster durchgang liest dummy
 _delay_us(spiwaitdelay);
 }
 //spi_rxbuffer[outindex] = incoming;
 }
 
 
 _delay_us(spidelay);
 SPI_PORT |= (1<<SPI_SS); // SS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
 ////OSZI_A_HI;
 
 
 }
 //PORTB |=(1<<0);
 arraypos++;
 arraypos &= 0x07;
 //spi_rxbuffer[outindex] = '\0';
 //outbuffer[outindex] = '\0';
 //char rest = SPDR;
 
 // wichtig
 _delay_us(10);
 
 //SPI_PORT |= (1<<SPI_SS); // SS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
 //_delay_us(1);
 
 SPI_PORT |= (1<<SPI_CS); // CS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
 ////OSZI_A_HI;
 //_delay_us(10);
 
 }
 */
/* Initializes the hardware timer to generate 1 millisecond ticks */

void timer2_init(void)
{
   // Timer 0 konfigurieren
   //TCCR2 = (1<<CS20);
   TCCR2 |= (1<<CS21);
   TCCR2 |= (1<<CS22);
   TCCR2 |= (1<<WGM20); // PWM, Phase correct
   TCCR2 |= (1<<WGM21);
   TIMSK |= (1<<TOIE2);// Overflow Interrupt erlauben

   TIMSK |= (1 << OCIE2); // Interrupt  enable
   
   OCR2 = 0x10;
}

ISR (TIMER2_OVF_vect)
{
   //OSZI_A_HI;
   PWMPORT &= ~(1<<PWM_OUT_PIN); // OFF
}

ISR(TIMER2_COMP_vect) //
{
   //OSZI_A_LO;
   //statusflag |= (1<<CHECK);
   PWMPORT |= (1<<PWM_OUT_PIN); // ON
}


void timer1_init(void)
{
   // Set timer
  // TCCR0 = (1 << WGM00)|(1 << WGM01);
   // Set prescaler /
   //TCCR1A|=(1<<WGM12)|(0<<WGM10);
   
   //TCCR1B=(1<<CS10)|(1<<CS11);//|(1<<WGM12); //Prescale = 1024, CTC
   TCCR1B |= (1 << WGM11)|(1<<WGM10) ;// 10Bit
   TCCR1B |= (1<<CS10)|(1<<CS11); // clock/64
   
   //OCR1A=F_CPU/1024; //Jede Sekunde ein IRQ
   
   //TIMSK|=(1<<OCIE1A); //Compare A Match Interrupt aktivieren

   //Set output compare register for 1ms ticks /
   //OCR0A = (F_CPU / 8) / 1000;
   // Enable output compare A interrupts /
   //TCCR1 |= (1 << WGM21);
   OCR1A = 0x8000;                       // Dutycycle of OC1A
   OCR1B = 0x80F0;                      // Dutycycle of OC1B

   TIMSK |= (1 << OCIE1A);
   TIMSK |= (1 << OCIE1B);
   TIMSK |= (1<<TOIE1);
  // TIMSK |= (1<<OCIE1B); // TOV2 Overflow, compA
   //ICR1H   = (TIMER1_COMPARE_VALUE >> 8);                      // set compare value for interrupt
   //ICR1L   = (TIMER1_COMPARE_VALUE & 0xFF);                    // set compare value for interrupt
   
}

ISR(TIMER1_OVF_vect)
{
   //OSZI_A_HI;
   OSZI_B_HI;
   if (statusflag & (1<<WAIT))
   {
      PWMPORT |= (1<<PWM_OUT_PIN);
   }
   if (statusflag & (1<<WAIT))
   {
      ANZEIGE_LO_counter++;
      if (ANZEIGE_LO_counter>0x2FFF)
      {
         statusflag &= ~(1<<WAIT);
         //  PORTB |= (1<<OSZIPIN);
      }
   }
   //OSZI_A_HI;
}

ISR(TIMER1_COMPA_vect) //
{
   //OSZI_A_LO;
   //statusflag |= (1<<CHECK);
   PWMPORT &= ~(1<<PWM_OUT_PIN);
   //OSZI_A_HI;
}
ISR(TIMER1_COMPB_vect) //
{
   OSZI_B_LO;
   //statusflag |= (1<<CHECK);
   PWMPORT &= ~(1<<PWM_OUT_PIN);
   //OSZI_A_HI;
}




void spi_slave_init (void)
{
   SPI_DDR = ~((1<<SPI_SS_PIN) | (1<<SPI_MOSI_PIN) | (1<<SPI_SCK_PIN));		// setze SCK,MOSI,PB0 (SS) als Eingang
   
   SPI_DDR |= (1<<SPI_MISO_PIN);							// setze MISO als Ausgang
   //	SPI_PORT &= ~(1<<SPI_MISO);
   //  SPI_PORT |= (1<<SPI_SCK) | (1<<SPI_SS);
   
   
   SPCR = (1<<SPE) | (1<<SPIE);			//Aktivierung des SPI + Interrupt
   SPDR=0;
   
   /*
    // Atmega328
    EICRA |= (1<<ISC01); // falling edge von INT0
    EIMSK |= (1<<INT0);
    */
   
   /*
    // Atmega8
    MCUCR |= (1<<ISC01); // falling edge von INT0
    GICR |= (1<<INT0);
    */
   
   // atmega32u4
   
}


ISR (SPI_STC_vect) // Neue Zahl angekommen // 3 us
{
   //OSZI_A_LO;
   isrcontrol++;
   //spi_rxbuffer[inindex] = SPDR;
   add(SPDR);
   
   //spidata = SPDR;
   SPDR = 0x00; // nichts zu lesen
   
   spi_rxdata=1;
   //OSZI_A_HI;
}



void main (void) 
{
	
	wdt_disable();
	MCUSR &= ~(1<<WDRF);
	wdt_reset();
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = 0x00;
	slaveinit();
   
   MCUCR |= (1<<ISC00);
   //GIMSK |= (1<<INT0);
   timer2_init();
   
   spi_slave_init ();
   
   initADC(1);
   
   lcd_initialize_spi(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   lcd_puts("Guten Tag\0");
   delay_ms(1000);

   uint8_t pos=0;
   sei();
   lcd_gotoxy(0,1);
   
#pragma mark while
	while (1)
   {
      //wdt_reset();
      //Blinkanzeige
      loopcount0++;
     // if (TCNT1 & 0x2000)         // Check the MSB of the timer
     //    OSZI_A_TOGG;          // Set PC0
      cli();
      if (loopcount0>=LOOPSTEP)
      {
         
         //lcd_gotoxy(0, 0);
         //lcd_putint(loopcount1);
         
         loopcount0=0;
         loopcount1++;
         if (loopcount1 >0xFF)
         {
            OCR2 = (readKanal(PWM_POT_PIN))>>2;
            LOOPLEDPORT ^=(1<<LOOPLEDPIN);
            loopcount1=0;
            //lcd_gotoxy(0,1);
            //lcd_puthex(spidata);

         }
         
      }
      sei();
      spidata = remove();
      
      //lcd_puthex(spidata);
      //if (spi_rxdata)
      if (spidata >=0)
      {
         OSZI_A_LO;
         
         lcd_gotoxy(19,0);
         lcd_putc('*');
         spi_rxdata = 0;
         lcd_gotoxy(0,3);
         lcd_puthex(spidata);

         if (spidata == 0x00) // end
         {
            datastring[pos] = '\0';
            lcd_gotoxy(0,2);
            lcd_putint(col);
            lcd_putc(' ');
            lcd_putint(line);
            //line = 1;
            //col = 2;

            lcd_gotoxy(col,line);
            lcd_puts(datastring);
            
          
         }
         else if (spidata == 0x0D) // neues paket
         {
//            datastring[pos] = '\0';
            lcd_gotoxy(0,2);
            lcd_putint(col);
            lcd_putc(' ');
            lcd_putint(line);
//            lcd_gotoxy(col,line);
 //           lcd_puts(datastring);
            //lcd_putc('*');
            pos=0; // pos im Datenpaket
            col=0;
            line=0;
            spistatus=0;
            spistatus = 1<<NEW_TASK;
            //lcd_clr_line(1);
         }
          else if ((spidata < 0x21) ) // cmd
          {
             //if (spistatus & (1<<NEW_TASK))
             {
                cmd = spidata;
                
                switch (cmd)
                {
                   case CHAR_TASK: // put c
                   {
                      spistatus |= 1<<CHAR_TASK;
                      par=0; // keine Parameter
                   }break;
                      
                   case 0x02: // goto xy
                   {
                      spistatus |= 1<<GOTO_TASK;
                      par=1; // goto-Daten
                      
                   } break;
                   
                   case END_TASK:
                   {
                      spistatus |= 1<<END_TASK;
                      
                   }break;
                      
                   default:
                      break;
                }// switch
                //spistatus &= ~(1<<NEW_TASK);
             }
             //lcd_gotoxy(pos++,1);
             //lcd_putc(spidata);
          }
          else if ((spidata >0x21) && (spidata < 0x5A)) // char
          {
             if (spistatus & (1<<CHAR_TASK))
             {
                datastring[pos++] = (uint8_t)spidata;
                spistatus &= ~(1<<CHAR_TASK);
               
             }
             else  if (spistatus & (1<<GOTO_TASK))
             {
                
                lcd_gotoxy(0,3);
                lcd_putint(spidata);
                lcd_putc(' ');
                data = (uint8_t)spidata;
                lcd_putint(data);
                //data = 0x21;
                line = data & 0x07;   // 5 bit col, 3 bit line
                col = (data & 0xF8)>>3;
                
                lcd_putc(' ');
                lcd_putint(col);
                lcd_putc(' ');
                lcd_putint(line);

                //line = 1;
                //col = 6;
                //lcd_gotoxy(col,line);
                //lcd_gotoxy(10,1);
                spistatus &= ~(1<<GOTO_TASK);
                
             }

             

                          //lcd_puthex(spidata);
          }

         
         /*
         if (pos > 1) // cmd & data
         {
            
            switch (cmd)
            {
               case 0x01: // put c
               {
                  if (spistatus & (1<<CHAR_TASK))
                  {
                     if ((data > 0x21))
                     {
                        lcd_putc(data);
                     }
                     spistatus &= ~(1<<CHAR_TASK);
                  }
                  
               }break;
                  
               case 0x02: // goto xy
               {
                  if (spistatus & (1<<GOTO_TASK))
                  {
                  uint8_t line = data & 0x07;   // 5 bit col, 3 bit line
                  uint8_t col = (data & 0xF8)>>3;
                  lcd_gotoxy(col,line);
                  spistatus &= ~(1<<GOTO_TASK);
                  
                  //lcd_gotoxy(col,line+1);
                 // lcd_puts(datastring);
                     }
               } break;
                  
               default:
                  break;
            }// switch

            
            //pos=0;
         }
         */
         //lcd_putc(' ');
         //lcd_puthex(isrcontrol);
         OSZI_A_HI;
      }
      else
      {
         //lcd_gotoxy(0,1);
         //lcd_puthex(0x00);
         //lcd_gotoxy(19,0);
         //lcd_putc('-');
         
      }

      
      //OSZIPORT &= ~(1<<OSZI_PULS_A);
      //OSZI_B_LO;
      if (STARTPIN & (1<<START_PIN))
      {
         if (!(statusflag & (1<<WAIT)))
         {
         statusflag |= (1<<WAIT);
         }
      }
      //OSZIPORT |= (1<<OSZI_PULS_A);
      //OSZI_B_HI;
      
      
      //pwmcount
      
      
      
   }//while
    
    
    //return 0;
}
