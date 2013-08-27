#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>   /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <cstdlib>
#include <math.h>
#include <time.h>

#include "capser.h"
#include "utils.h"
#include "laser.h"

using namespace std;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//set to 1 if that motor is actively connected and used (or if it should be used), 0 otherwise.
#define X_MOTOR_ACTIVE 1
#define Y_MOTOR_ACTIVE 1
#define Z_MOTOR_ACTIVE 1

#define X_AXIS_ID 1
#define Y_AXIS_ID 2
#define Z_AXIS_ID 3
#define LASER_ID  4

#define KILL 0
#define FIND_HOME 1
#define GOTO_HOME 2
#define REL_MOTION 3
#define ABS_MOTION 4
#define SEQ_1 5		
#define SEQ_2 6
#define GIVE_POS 7
#define RESET_HOME 8
#define SET_LASER_WIDTH 20
#define SET_LASER_AMP 21
#define TRIG_LASER 22

#define SLIDE_LENGTH 50.0					//length of x,y stage 
#define SLIDE_LENGTH_UM 50.0*10000.0	//length of x,y stage in um
#define XYSTEPSIZE 0.1						//default step size for the float boxes

#define IO7bit		15
#define IO6bit		14
#define IO5bit		13
#define IO4bit		12
#define IO3bit		6
#define IO2bit		5	
#define IO1bit		4

#define NUMSTART 48
#define BAUDRATE B57600
#define _POSIX_SOURCE 1         //POSIX compliant source
#define FALSE 0
#define TRUE 1

#define ERR_POS -1.2345
#define ENC_CTS_PER_REV 8000
#define SHAFT_CM_PER_REV 0.5
#define CM_PER_ENC_COUNTS 
#define uM_PER_CNT 0.625
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	
//~~~~~~~~~~~~~~~~~~ variables ~~~~~~~~~~~~~~~~~~
//dist in mm, all times in s

const double ticksPerMM = 15748.0;
const double ticksPerSecond = 1 / 0.000120;		//number of clock ticks per second
const double velFactor = 1000.0;
const double accFactor = 100.0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
class serialClass{
	public:
			
		//~~~~~~~~~~~~~~~~~~ CONSTANTS ~~~~~~~~~~~~~~~~~~
		int fd; // File descriptor for the motor's serial port
		int Ld; // File descriptor for the laser's serial port
	
		char devicename[80];
		#define Baud_Rate 57600;         // default Baud Rate (110 through 38400)


		//~~~~~~~~~~~ variables ~~~~~~~~~~~
		int IOstates;

		//~~~~~~~~~ CMD functions ~~~~~~~~~
		int clearPoll(int ID, int whichBit);//, bool debugOn = false);	//CPL
		int poll(int ID, char* strToReturn);		//COR
   	int readIO(int ID, char* strToReturn );			//RIO
		int initDualLoop(int ID);	//DLC
		int initSingleLoop(int ID);
		int halt(int ID);	//HAL
		int stop(int ID);	//STP
		int enableMotor(int ID); //EMD
		int moveRelativeTime( int ID, double dist, double rampTime, double totalTime );
		int moveAbsoluteTime( int ID, double pos, double acc, double totalTime );
		int moveRelativeVel( int ID, double dist, double acc, double vel );

		int resetMotor(int ID);
		int setupEncoder(int ID);
				
		//~~~~~~~~~~~ serial IO ~~~~~~~~~~~
		int initMotSerPort(void); 		//initialize the serial port going to the motors.
		int initLaserSerPort(void); 	//initialize the serial port going to the pulser.
		int closeSerialPort(void);
		int closeLaserSerialPort(void);
		void flush(void);
		void flushL(void);
		int changeACKdelay(int ID, double delay);
		int changeAntiHunt(int ID);
		int goClosedLoop(int ID);
		int getReply(int whichPort, char *result, int &numChars);
		int writeChar( char charArray[], int len );
		int checkForACK( int ID, char* inputArray );
      virtual ~serialClass ( void );
		serialClass ( void );
	
		
		//~~~~~~~~~~ serial utilities ~~~~~~~~~~~~
		int displayPSWdescriptions(char* inputArray);
		int displayIOdescriptions(char* inputArray);
		int displayNACKerrors(char* inputArray);
		int clearAllPSWbits(int ID);
		int clearInternalStatus(int ID);
		int returnNumDigits(int number);
		void printPSWmessage(int index);
		void printIOmessage(int index);
		int getIObitFromReply(char* inputArray, int whichBit);
		int getCloseSwitch(int ID);
		int getFarSwitch(int ID);
		int getHomeSwitch(int ID);
		int zeroTarget(int ID);
		int isCommandDone(int ID);
		int initIO(int ID);
		int setIObit(int ID, int whichBit, int state);
		int killActiveMotors(void);		
		int returnPosition( int ID, double* pos );
		int pulseLaser(double duration, double intensity);
		
		//~~~~~~~~~~~~  motion sets  ~~~~~~~~~~~~
		int movePosRel(int ID, double distance);
		int movePosAbs(int ID, double distance);
		int gotoHomePoint(int ID);
		int gotoAndSetHomePoint( int ID );
		int resetAsHomePoint( int ID );
		int stepAndPulseSequence(int ID, double startPos, double stepSize, 
							int numOfIntervals, double	duration, double intensity);
		int stepAndPulseSequenceBothAxes(int ID1, int ID2, double startPos1, double startPos2, 
						double stepSize1, double stepSize2, 
											int numOfSteps, double	laserWidth, double laserAmp);

		//~~~~~~~~~~~ Laser Pulser ~~~~~~~~~~~~~
		int requestLaserID( void );
		int turnOffLaserEcho( void );
		int laserRemoteEnable( void );
		int laserLocalEnable( void );
		int disableLaserFlowControl(void);
		int setLaserWidth_ns( double width );
		int setLaserAmp_mV(double amp);
		int sendLaserTrigger(void);
		int laserEnableOutput( void );
		int laserDisableOutput( void );
				
};//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif
