#ifndef __LASER_H__
#define __LASER_H__

//#define
#define FALSE 0
#define TRUE 1

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
//#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <cstdlib>
#include <math.h>
//#include <time.h>

#include "utils.h"
#include "capser.h"
//#include "stage.h"

using namespace std;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Forward declarations
//class serialClass;

class laserClass{
	private:
		//~~~~~~~~~~~~~~~~~~ CONSTANTS ~~~~~~~~~~~~~~~~~~
		int Ld; // File descriptor for the laser's serial port
	
		//~~~~~~~~~~~~~~~~~~ VARIABLES ~~~~~~~~~~~~~~~~~~
		char output[254];

		//~~~~~~~~~~~ serial IO ~~~~~~~~~~~
		int initLaserSerPort(void); 	//initialize the serial port going to the pulser.
		void flushL(void);
		int writeLas(const char *output);
		int getLasReply(char *result);
		int writeLasAndGetReply(const char *output, char *result);
		int writeLasAndCheckACK(const char *output);

		//~~~~~~~~~~~ Laser Pulser ~~~~~~~~~~~~~
		int requestLaserID( void );
		int turnOnLaserEcho( void );
		int laserRemoteEnable( void );
		int disableLaserFlowControl(void);
		int laserEnableOutput( void );
		int laserDisableOutput( void );
	public:	
	
		//~~~~~~~~~~~ Laser Pulser ~~~~~~~~~~~~~
		int closeLaserSerialPort(void);
		int laserInitSequence(void);
		int setLaserWidth_ns( double width );
		int setLaserAmp_mV(double amp);
		int sendLaserTrigger(void);
		int pulseLaser(double duration, double intensity);
		int laserLocalEnable( void );
		bool isIntTriggerOn();
		int laserTriggerInternal( void );
		int laserTriggerHold( void );
};

#endif
