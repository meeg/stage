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

		
int copySubArray(char* subArray, char* input, int startIndex, int endIndex);

int parseCharReplyIntoArray( char* input, double* parsedReply );

double getNumFromCharArray(char* input);
		
int findNumSpaces(char* input);

int turnNumberIntoCharArray(int ID, char* resultingCharArray, double num );

int turnNumberIntoCharArray(int ID, char* resultingCharArray, int num );
				
int checkForACK( int ID, char* inputArray );

int getHexArrayFromReply( char* inputArray, char* hexResult, int numWordsToGet = 1 );

int convertHexArrayToDec(char* hexArray, long* decResult);

void decNumberToBinaryArray( int decVal, int* binaryArray);

void strreverse(char* begin, char* end);
	
void itoa(int value, char* str, int base);

void sleep(double timeInSeconds);

/*after receiving a command, here's an example process using the above:
getHexArrayFromReply (gets hex reply values) --> convertHexArrayToDec (takes the hex array and returns a
decimal number) --> decNumberToBinaryArray (fills out an array containing binary values).*/






#endif
