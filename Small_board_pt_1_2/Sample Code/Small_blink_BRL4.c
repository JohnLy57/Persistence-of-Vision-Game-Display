/*
 * File:        blink test
 * Author:      Bruce Land
 * For use with Sean Carroll's Small Board
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"
////////////////////////////////////

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_blink ;

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
     // set up LED to blink
     mPORTASetBits(BIT_0 );	//Clear bits to ensure light is off.
     mPORTASetPinsDigitalOut(BIT_0 );    //Set port as output
     
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(500) ;
        // toggle the LED on the big board
        mPORTAToggleBits(BIT_0);
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);
} // timer thread

// === Blinker Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_blink(struct pt *pt))
{
    PT_BEGIN(pt);
     // set up LED to blink
     mPORTBSetBits(BIT_0 );	//Clear bits to ensure light is off.
     mPORTBSetPinsDigitalOut(BIT_0 );    //Set port as output
     
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(250) ;
        // toggle the LED on the big board
        mPORTBToggleBits(BIT_0);
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);
} // timer thread

// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_blink);
  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_blink(&pt_blink));
      }
  } // main

// === end  ======================================================

