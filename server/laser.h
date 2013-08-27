#ifndef __LASER_H__
#define __LASER_H__

//#define
#define FALSE 0
#define TRUE 1

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <cstdlib>
#include <math.h>
#include <time.h>

#include "utils.h"
#include "serial.h"
//#include "stage.h"

using namespace std;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Forward declarations
class serialClass;

class laser{
	
	public:	
	
			//~~~~~~~~~~~~~~~~~~ CONSTANTS ~~~~~~~~~~~~~~~~~~
	
			
			
			//~~~~~~~~~~~~~~~~~~ VARIABLES ~~~~~~~~~~~~~~~~~~

	
			
			//~~~~~~~~~~~~~~~~~~ FUNCTIONS ~~~~~~~~~~~~~~~~~~
			
			int pulseLaser(double duration, double intensity);
			int initLaser(void);
			int setLaserIntensity(void);
			int setLaserDuration(void);
			
};

#endif
