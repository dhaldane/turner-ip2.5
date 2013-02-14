/***************************************************************************
* Name: cmd.c
* Desc: Receiving and transmitting queue handler
* Date: 2010-07-10
* Author: stanbaek
**************************************************************************/

#include "cmd.h"
#include "cmd_const.h"
#include "cmd-motor.h"
#include "dfmem.h"
#include "utils.h"
#include "ports.h"
#include "sclock.h"
#include "led.h"
#include "blink.h"
#include "payload.h"
#include "mac_packet.h"
#include "dfmem.h"
#include "pid-ip2.5.h"
#include "radio.h"
#include "move_queue.h"
#include "steering.h"
#include "dfmem.h"
//  #include "dfmem_extra.h" replace with telemetry.h
#include "tests.h"
#include "queue.h"
#include "version.h"
#include "../MyConsts/settings.h"
#include "tiH.h"
#include "timer.h"
#include "telemetry.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

unsigned char tx_frame_[127];

extern MoveQueue moveq;
volatile Queue fun_queue;
extern int offsz;
extern pidPos pidObjs[NUM_PIDS];
extern telemU telemPIDdata;
extern TelemConStruct TelemControl;
extern unsigned long t1_ticks;
extern int samplesToSave;
unsigned long packetNum = 0; // sequential packet number

extern moveCmdT currentMove, idleMove, manualMove;
// updated version string to identify robot
//extern static char version[];

// use an array of function pointer to avoid a number of case statements
// MAX_HURRRRRRR is defined in cmd_const.h
// arg to all commands are: type, status, length, *frame
void (*HURRRRRRR[MAX_HURRRRRRR])(unsigned char, unsigned char, unsigned char, unsigned char*);
void cmdError(void);
/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/

static void cmdSteer(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdStartImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdStopImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdTxSavedImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdEraseMemSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

void cmdEcho(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

//User commands
static void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetPIDTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdSetCtrldTurnRate(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuLoopZGyro(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetMoveQueue(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdSetSteeringGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSoftwareReset(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSpecialTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdEraseSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

// commands defined in cmd-motor.c:
//static void cmdSetThrust(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdSetThrustClosedLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
void cmdSetup(void) {

    unsigned int i;

    // initialize the array of func pointers with Nop()
    for(i = 0; i < MAX_HURRRRRRR; ++i) {
        HURRRRRRR[i] = &cmdNop;
    }

    HURRRRRRR[CMD_ECHO] = &cmdEcho;
    HURRRRRRR[CMD_SET_THRUST] = &cmdSetThrust;
    HURRRRRRR[CMD_SET_STEER] = &cmdSteer;    
    HURRRRRRR[CMD_GET_IMU_DATA] = &cmdGetImuData;
    HURRRRRRR[CMD_GET_IMU_LOOP] = &cmdGetImuLoop;
    HURRRRRRR[CMD_START_IMU_SAVE] = &cmdStartImuDataSave;
    HURRRRRRR[CMD_STOP_IMU_SAVE] = &cmdStopImuDataSave;
    HURRRRRRR[CMD_TX_SAVED_IMU_DATA] = &cmdTxSavedImuData;
    HURRRRRRR[CMD_ERASE_MEM_SECTOR] = &cmdEraseMemSector;
	//Use commands
	HURRRRRRR[CMD_SET_THRUST_OPENLOOP] = &cmdSetThrustOpenLoop;
	HURRRRRRR[CMD_SET_THRUST_CLOSEDLOOP] = &cmdSetThrustClosedLoop;
	HURRRRRRR[CMD_SET_PID_GAINS] = &cmdSetPIDGains;
	HURRRRRRR[CMD_GET_PID_TELEMETRY] = &cmdGetPIDTelemetry;
	//HURRRRRRR[CMD_SET_CTRLD_TURN_RATE] = &cmdSetCtrldTurnRate;
	HURRRRRRR[CMD_GET_IMU_LOOP_ZGYRO] = &cmdGetImuLoopZGyro;
	HURRRRRRR[CMD_SET_MOVE_QUEUE] = &cmdSetMoveQueue;
	//HURRRRRRR[CMD_SET_STEERING_GAINS] = &cmdSetSteeringGains;
	HURRRRRRR[CMD_SOFTWARE_RESET] = &cmdSoftwareReset;
	HURRRRRRR[CMD_SPECIAL_TELEMETRY] = &cmdSpecialTelemetry;
	HURRRRRRR[CMD_ERASE_SECTORS] = &cmdEraseSector;
	HURRRRRRR[CMD_FLASH_READBACK] = &cmdFlashReadback;
	HURRRRRRR[CMD_SET_VEL_PROFILE] = &cmdSetVelProfile;
	HURRRRRRR[CMD_WHO_AM_I] = &cmdWhoAmI;
	HURRRRRRR[CMD_START_TELEM] = &cmdStartTelemetry;
	HURRRRRRR[CMD_ZERO_POS] = &cmdZeroPos;
}

void cmdHandleRadioRxBuffer(void) {

    Payload pld;
    unsigned char command, status;

    if ((pld = radioDequeueRxPacket()) != NULL) {

        status = payGetStatus(pld);
        command = payGetType(pld);


        //Due to bugs, command may be a surprious value; check explicitly
        if (command <= MAX_HURRRRRRR) {
            HURRRRRRR[command](command, status, pld->data_length, payGetData(pld));
        }

        payDelete(pld);
    }

    return;
}

// Jan 2013- new command handler using function queue

void cmdPushFunc(MacPacket rx_packet)
{   Payload rx_payload;
    unsigned char command, status;  
	 rx_payload = macGetPayload(rx_packet);
	 
	 Test* test = (Test*) malloc(sizeof(Test));
        if(!test) return;
	  test->packet = rx_packet;

        command = payGetType(rx_payload);
	   if( command < MAX_HURRRRRRR)
	  {     test->tf=HURRRRRRR[command];
		   queuePush(fun_queue, test); 
	  }   
	  else 
	 {  cmdError();   // halt on error - could also just ignore....
	 }

}

/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
 *         User function
-----------------------------------------------------------------------------*/
/* keyboard_telem2.5.py uses , 
 * cmd.c: FLASH_READBACK, ERASE_SECTORS, START_TELEM, GET_PID_TELEMETRY
* cmd-motor.c: SET_PID_GAINS,  SET_THRUST, SET_VEL_PROFILE, ZERO_POS, SET_THRUST_CLOSED_LOOP
*  cmd-aux.c: WHO_AM_I, ECHO, SOFTWARE_RESET
*/


static void cmdEraseSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{
//	eraseDFMemSectors0a0b();
/// updated for IP2.5
// hard code for now 300 samples at 300 Hz
	CRITICAL_SECTION_START   //  can't have interrupt process grabbing SPI2
	dfmemEraseSectorsForSamples( (unsigned long) 300, sizeof(telemStruct_t));
	CRITICAL_SECTION_END
}


// telemetry is saved at 300 Hz inside steering servo
// alternative telemetry which runs at 1 kHz rate inside PID loop
static void cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int idx=0;
       unsigned long temp;
     TelemControl.count = frame[idx] + (frame[idx+1] << 8); idx+=2;
   // start time is relative to current t1_ticks
       CRITICAL_SECTION_START
	temp = t1_ticks; // need atomic read due to interrupts
	CRITICAL_SECTION_END
	TelemControl.start = 
		(unsigned long) (frame[idx] + (frame[idx+1] << 8) )
						+ temp;
	idx+=2;
      samplesToSave = TelemControl.count; // **** this runs sample capture in T5 interrupt
	TelemControl.skip = frame[idx]+(frame[idx+1]<<8); 
	//sclockReset();
	if(TelemControl.count > 0) 
	{ TelemControl.onoff = 1;   // use just steering servo sample capture
	 } // enable telemetry last 
}

//send single radio packet with current PID state
static void cmdGetPIDTelemetry(unsigned char type, unsigned char status, unsigned char length,
								 unsigned char *frame)
{ 	unsigned int sampLen = sizeof(telemStruct_t);
	telemGetPID(packetNum);  // get current state
	 radioConfirmationPacket(RADIO_DST_ADDR,
						     CMD_SPECIAL_TELEMETRY, 
						     status, sampLen, (unsigned char *) &telemPIDdata);  
	packetNum++;
    // delay_ms(25);	// slow down for XBee 57.6 K
	blink_leds(1,20); // wait 20 ms to give plenty of time to send packets
}


static void cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{	unsigned int count;
	count = frame[0] + (frame[1] << 8);
	//telemFlashReadback(count);	
/**********  will need to disable mpuUpdate to read flash... ******/
//	readDFMemBySample(count);
}



//#include "cmd-motor.c"  // ZeroPos, SetThrust, SetVelProfile, SetPIDGains

#include "cmd-pt2.c"

/*-----------------------------------------------------------------------------
 *          AUX functions
-----------------------------------------------------------------------------*/
void cmdEcho(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) 
{ // MacPacket packet; Payload pld;
	//Send confirmation packet
	radioConfirmationPacket(RADIO_DST_ADDR, CMD_ECHO, status, length, frame);  
    return; //success     
}

static void cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    Nop();
}


static void cmdSoftwareReset(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{
	asm volatile("reset");
}

// send robot info when queried
void cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) 
{   unsigned char i, string_length; unsigned char *version_string;
// maximum string length to avoid packet size limit
	version_string = (unsigned char *)versionGetString();
	i = 0;
	while((i < 127) && version_string[i] != '\0')
	{ i++;}
	string_length=i;     
	radioConfirmationPacket(RADIO_DST_ADDR, CMD_WHO_AM_I, status, string_length, version_string);  
      return; //success
}

// handle bad command packets
// we might be in the middle of a dangerous maneuver- better to stop and signal we need resent command
// wait for command to be resent
void cmdError()
{ int i;
 	EmergencyStop();
	for(i= 0; i < 10; i++)
	 {	LED_1 ^= 1;
			delay_ms(200);
			LED_2 ^= 1;
			delay_ms(200);
			LED_3 ^= 1;
			delay_ms(200);
          }
}
