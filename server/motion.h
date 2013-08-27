#ifndef __MOTION_H__
#define __MOTION_H__

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <cstring>
#include <sys/types.h>
#include <cstdlib>
#include <stdlib.h>
#include <math.h>

using namespace std;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
//variables:
#define NUMSTART 48


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
//functions:

int stepPosRel(int ID, double distance);
int stepPosAbs(int ID, double distance);
int gotoHomePoint(int ID);
int gotoAndSetHomePoint(int ID);




#endif
