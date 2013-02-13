/*********************************************************************************************************
* Name: main.c
* Desc: A test suite for the ImageProc 2.2 system. These tests should not be
* considered rigorous, exhaustive tests of the hardware, but rather
* "smoke tests" - ie. turn on the functionality and make sure the 
* hardware/software doesn't start "smoking."
*
* The architecture is based on a function pointer queue scheduling model. The
* meat of the testing logic resides in test.c. If the radio has received a 
* command packet during the previous timer interval for Timer2, the appropriate
* function pointer is added to a queue in the interrupt service routine for 
* Timer2 (interrupts.c). The main loop simply pops the function pointer off
* the top of the queue and executes it. 
*
* Date: 2011-04-13
* Author: AMH, Ryan Julian
*********************************************************************************************************/
#include "p33Fxxxx.h"
#include "init.h"
#include "init_default.h"
#include "timer.h"
#include "utils.h"
#include "queue.h"
#include "radio.h"
#include "settings.h"
#include "tests.h" 
#include "dfmem.h"
#include "interrupts.h"
#include "sclock.h"
#include "ams-enc.h"
#include "tih.h"
#include "blink.h"
#include <stdlib.h>
#include "cmd.h"
#include "pid-ip2.5.h"
#include "steering.h"
#include "consts.h"
#include "adc_pid.h"
#include "led.h"

Payload rx_payload;
MacPacket rx_packet;
Test* test;
unsigned int error_code;

int main() {
    fun_queue = queueInit(FUN_Q_LEN);
    test_function tf;
    error_code = ERR_NONE;

    /* Initialization */
    SetupClock();
    SwitchClocks();
    SetupPorts();

    SetupInterrupts();
    adcSetup();   // DMA A/D
//    SetupTimer1(); setup in pidSetup
    SetupTimer2();
    sclockSetup();
    mpuSetup(1);        //cs==2
    amsHallSetup();
    dfmemSetup(0);      //cs==1
    tiHSetup();   // set up H bridge drivers
	cmdSetup();  // setup command table
	pidSetup();  // setup PID control

    // Radio setup
    radioInit(RADIO_RXPQ_MAX_SIZE, RADIO_TXPQ_MAX_SIZE, 0);     //cs==1
    radioSetChannel(RADIO_CHANNEL);
    radioSetSrcAddr(RADIO_SRC_ADDR);
    radioSetSrcPanID(RADIO_SRC_PAN_ID);
    setupTimer6(RADIO_FCY); // Radio and buffer loop timer
/**** set up steering last - so dfmem can finish ****/
	//steeringSetup(); // steering and Timer5 Int

    //Motor Test

    // int interval[3], delta[3], vel[3];
    // pidSetGains(0,400,0,0,0,0);
    // pidSetGains(1,400,0,0,0,0);

    // //set vel profile
    // int i;
    // for (i= 0; i < 4; i++)
    // {
    //     delta[i] = 0x4000;
    //     interval[i] = 256;
    //     vel[i] = 64;
    // }


    // setPIDVelProfile(0, interval, delta, vel);
    // setPIDVelProfile(1, interval, delta, vel);
    // //Set thrust closed loop
  
    // pidSetInput(0 ,0, 1000);
    // pidOn(0);
    // pidSetInput(1 ,0, 1000);
    // pidOn(1);

    //Open loop moto test
    // DisableIntT1;   // since PID interrupt overwrites PWM values

    // tiHSetDC(1, 0x200);
    // tiHSetDC(2, 0x200); 
    // delay_ms(4000);
    // tiHSetDC(1,0);
    // tiHSetDC(2,0);
    // EnableIntT1;

    LED_2 = ON;

    //EnableIntT2;
    DisableIntT1;
    while(1){
        amsGetPos(0);amsGetPos(1);
        delay_ms(10);

    }
    return 0;
}
