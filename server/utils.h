#ifndef __UTILS_H__
#define __UTILS_H__

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <cstring>
#include <sys/types.h>
#include <cstdlib>
#include <stdlib.h>
#include <math.h>
#include <sstream>
#include "xmlwriter.h"

using namespace std;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
//variables:
#define NUMSTART 48


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
//functions:

		
int turnNumberIntoCharArray(int ID, char* resultingCharArray, double num );

int turnNumberIntoCharArray(int ID, char* resultingCharArray, int num );

void sleep(double timeInSeconds);

/*after receiving a command, here's an example process using the above:
getHexArrayFromReply (gets hex reply values) --> convertHexArrayToDec (takes the hex array and returns a
decimal number) --> decNumberToBinaryArray (fills out an array containing binary values).*/






#endif
