//modified serial header for the client

#ifndef __SERIAL_H__
#define __SERIAL_H__

//#define
#define BAUDRATE B57600
//#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1         //POSIX compliant source
#define FALSE 0
#define TRUE 1

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


using namespace std;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//command type:
#define CPL 0
#define POR 1
#define RIO 2
#define NUMSTART 48
/*#define 
#define 
#define 
#define 
#define 
#define 
#define 
#define 

#define */

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



#define X_AXIS_ID 1
#define Y_AXIS_ID 2
#define Z_AXIS_ID 3
#define LASER_ID  4

//set to 1 if that motor is actively connected and used, 0 otherwise.
#define X_MOTOR_ACTIVE 1
#define Y_MOTOR_ACTIVE 1
#define Z_MOTOR_ACTIVE 1

	
#define SLIDE_LENGTH 50.0
	

#endif
