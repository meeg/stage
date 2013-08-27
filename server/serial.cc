/* 
Paul Csonka August 2010
Serial libraries for the stepper motor controllers and laser pulser
*/

#include "serial.h"
#define MAX_REPLY_LEN 254

//packet format:
// " @, ID (EACH DIGIT in ASCII), command number (EACH DIGIT in ASCII), parameters, <CR> (ASCII 13), NULL "

using namespace std;

//=================================================================================
//=================================================================================


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::moveRelativeTime(int ID, double dist, double rampTime, 
												double totalTime){

			
	
	//MRT
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 177;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	int currIndex;
	
	//~~~~~~~~~~ ERROR CHECK ~~~~~
	if ( ( rampTime <= 0.0 ) | ( totalTime <= 0.0 ) ){
		printf("Err: serialClass::moveRelativeTime().  Time parameters either zero or negative\n");
		printf("rampTime=%f, totalTime=%f\n", rampTime, totalTime);
		return -1;
	}
	if ( 2 * rampTime > totalTime ){
		printf("Err: serialClass::moveRelativeTime().  Ramp Time Too Large Compared to Total Time\n");
		return -1;
	}
	if (  ( dist > SLIDE_LENGTH ) | ( dist < -1.0*SLIDE_LENGTH ) ){
		printf("Err: serialClass::moveRelativeTime(). Dist Larger Than Slide Length\n");
		return -1;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//format to proper units:
	dist *= ticksPerMM;
	rampTime *= ticksPerSecond;
	totalTime *= ticksPerSecond;
	
	unsigned int distToOutput = (unsigned int) dist;
	unsigned int rampTimeToOutput = (unsigned int) rampTime;
	unsigned int totalTimeToOutput = (unsigned int) totalTime;
	
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	
	// ----> ID , SPC, CMD, SPC
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 32; 		//SP
	currIndex = startindex + numDigs + 1;
	
	//printf("currIndex start=%d\n", currIndex);
	
		// ----> dist, SPC
	char distChar[20];
	int numDistChars = sprintf( distChar, "%d", distToOutput); //convert decimal inte input to character array.
	//printf("distChar=%s\n", distChar); //printf("length=%d\n", numDistChars);
	for (int i = 0; i < numDistChars; i++){
			output[ currIndex ] = distChar[ i ];
		 	currIndex++;
	}
	//printf( "output(%d)=%d\n", i, output[ i ] );
	output[currIndex] = 32; currIndex ++;


	
	
		// ----> rampTime, SPC
	char rampChar[20];
	int numRampChars = sprintf( rampChar, "%u", rampTimeToOutput); //convert decimal inte input to character array.
	//printf("rampChar=%s\n", rampChar); //printf("length=%d\n", numRampChars);
	for (int i = 0; i < numRampChars; i++){
		output[ currIndex ] = rampChar[ i ];	
		currIndex ++;
		}
	output[ currIndex ] = 32;
	currIndex ++;	

	// ----> totalTime, SPC
	char timeChar[20];
	int numTimeChars = sprintf( timeChar, "%u", totalTimeToOutput); //convert decimal inte input to character array.
	//printf("timeChar=%s\n", timeChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numTimeChars; i++){
		output[ currIndex ] = timeChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[ currIndex ] = 32;
	currIndex ++;
	
		
	// ----> stopEnable, SPC
	char stopEnChar[20];
	int stopEnable = 0;			//pg.89 user manual
	int numStopEnChars = sprintf( stopEnChar, "%u", stopEnable); //convert decimal inte input to character array.
	//printf("timeChar=%s\n", timeChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numStopEnChars; i++){
		output[ currIndex ] = stopEnChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[ currIndex ] = 32;
	currIndex ++;	

	// ----> stopState, SPC
	char stopStChar[20];
	
	int stopState = 0;
	int numStopStChars = sprintf( stopStChar, "%u", stopState); //convert decimal inte input to character array.
	//printf("timeChar=%s\n", timeChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numStopStChars; i++){
		output[ currIndex ] = stopStChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[currIndex] = 13;
	currIndex ++;	
	output[currIndex] = 0;	//null
	currIndex ++;
	
	/*for (int i = 0; i < currIndex; i++){
		printf( "output(%d)=%d\n", i, output[ i ] );
	}*/
		
		
	//printf("output:%s\n", output);
	//test case
		/*
   output[0] = 64;	//@
	output[1] = 49;	//ID
	output[2] = 32;
	
	output[3] = 49;	//cmd = 177
	output[4] = 55;
	output[5] = 55;
	output[6] = 32;
		
	output[7] = 52;	//4000
	output[8] = 48;
	output[9] = 48;
	output[10] = 48;
	output[11] = 32;
	
	output[12] = 49;	//1000
	output[13] = 48;
	output[14] = 48;
	output[15] = 48;
	output[16] = 32;
	
	output[17] = 57;	//9000
	output[18] = 48;
	output[19] = 48;
	output[20] = 48;
	output[21] = 32;
	
	output[22] = 48;	//0
	output[23] = 32;
	
	output[24] = 48;	//0
	output[25] = 32;
	
	output[26] = 13;	//CR

	output[27] = 0;	//null
	for (int i = 0; i < 30; i++){
		printf( "output(%d)=%d\n", i, output[ i ] );
	}
	*/

			
	if (!writeport(fd, output)) {
		printf("Err: MRT write failed\n");
		//close(fd);
		return -1;
	}
	else{
		printf("MRT written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
		
	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( reply == 0) {
			printf("ACK MRT\n");
			return 0;
	}else{
		printf("Err: bad MRT response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::moveAbsoluteTime(int ID, double pos, double acc, 
												double totalTime){



			
	
	//MRT
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 176;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	int currIndex;
	
	//~~~~~~~~~~ ERROR CHECK ~~~~~
	/*if ( 2 * rampTime > totalTime ){
		printf("Err: serialClass::moveAbsoluteTime().  Ramp Time Too Large Compared to Total Time\n");
		return -1;
	}*/
	if ( ( acc <= 0.0 ) | ( totalTime <= 0.0 ) ){
		printf("Err: serialClass::moveAbsoluteTime().  Acc or time parameters either zero or negative\n");
		printf("acc=%f, totalTime=%f\n", acc, totalTime);

		return -1;
	}
	if (  ( pos > SLIDE_LENGTH ) | ( pos < -1.0 * SLIDE_LENGTH ) ){
		printf("Err: serialClass::moveAbsoluteTime(). Pos Larger Than Slide Length\n");
		return -1;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//format to proper units:
	pos *= ticksPerMM;
	acc *= ticksPerSecond;
	totalTime *= ticksPerSecond;
	
	unsigned int posToOutput = (unsigned int) pos;
	unsigned int accToOutput = (unsigned int) acc;
	unsigned int totalTimeToOutput = (unsigned int) totalTime;
			
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	
	// ----> ID , SPC, CMD, SPC
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 32; 		//SP
	currIndex = startindex + numDigs + 1;
	
	//printf("currIndex start=%d\n", currIndex);
	
		// ----> dist, SPC
	char posChar[20];
	int numDistChars = sprintf( posChar, "%d", posToOutput); //convert decimal inte input to character array.
	//printf("distChar=%s\n", distChar); //printf("length=%d\n", numDistChars);
	for (int i = 0; i < numDistChars; i++){
			output[ currIndex ] = posChar[ i ];
		 	currIndex++;
	}
	//printf( "output(%d)=%d\n", i, output[ i ] );
	output[currIndex] = 32; currIndex ++;


	
	
		// ----> rampTime, SPC
	char accChar[20];
	int numRampChars = sprintf( accChar, "%u", accToOutput); //convert decimal inte input to character array.
	//printf("rampChar=%s\n", rampChar); //printf("length=%d\n", numRampChars);
	for (int i = 0; i < numRampChars; i++){
		output[ currIndex ] = accChar[ i ];	
		currIndex ++;
		}
	output[ currIndex ] = 32;
	currIndex ++;	

	// ----> totalTime, SPC
	char timeChar[20];
	int numTimeChars = sprintf( timeChar, "%u", totalTimeToOutput); //convert decimal inte input to character array.
	//printf("timeChar=%s\n", timeChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numTimeChars; i++){
		output[ currIndex ] = timeChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[ currIndex ] = 32;
	currIndex ++;
	
		
	// ----> stopEnable, SPC
	char stopEnChar[20];
	int stopEnable = 0;			//pg.89 user manual
	int numStopEnChars = sprintf( stopEnChar, "%u", stopEnable); //convert decimal inte input to character array.
	//printf("timeChar=%s\n", timeChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numStopEnChars; i++){
		output[ currIndex ] = stopEnChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[ currIndex ] = 32;
	currIndex ++;	

	// ----> stopState, SPC
	char stopStChar[20];
	
	int stopState = 0;
	int numStopStChars = sprintf( stopStChar, "%u", stopState); //convert decimal inte input to character array.
	//printf("timeChar=%s\n", timeChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numStopStChars; i++){
		output[ currIndex ] = stopStChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[currIndex] = 13;
	currIndex ++;	
	output[currIndex] = 0;	//null
	currIndex ++;
	
	/*for (int i = 0; i < currIndex; i++){
		printf( "output(%d)=%d\n", i, output[ i ] );
	}*/
		
		
	//printf("output:%s\n", output);
	//test case
		/*
   output[0] = 64;	//@
	output[1] = 49;	//ID
	output[2] = 32;
	
	output[3] = 49;	//cmd = 176
	output[4] = 55;
	output[5] = 54;
	output[6] = 32;
		
	output[7] = 45;	//-
	output[8] = 53;	//5000
	output[9] = 48;
	output[10] = 48;
	output[11] = 48;
	output[12] = 32;
	
	output[13] = 49;	//1000
	output[14] = 48;
	output[15] = 48;
	output[16] = 48;
	output[17] = 32;
	
	output[18] = 57;	//9000
	output[19] = 48;
	output[20] = 48;
	output[21] = 48;
	output[22] = 32;
	
	output[23] = 48;	//0
	output[24] = 32;
	
	output[25] = 48;	//0
	output[26] = 32;
	
	output[27] = 13;	//CR

	output[28] = 0;	//null
	
	for (int i = 0; i < 30; i++){
		printf( "output(%d)=%d\n", i, output[ i ] );
	}*/
			
	if (!writeport(fd, output)) {
		printf("Err: MAT write failed\n");
		//close(fd);
		return -1;
	}
	else{
		printf("MAT written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
		
	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( reply == 0) {
			printf("ACK MAT\n");
			return 0;
	}else{
		printf("Err: bad MAT response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   	
int serialClass::moveRelativeVel(int ID, double dist, double acc, 
												double vel){
	
	//MRT
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 135;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	int currIndex;
	
	//~~~~~~~~~~ ERROR CHECK ~~~~~
	/*if ( ){
		printf("Err: serialClass::moveRelativeVel().  Ramp Time Too Large Compared to Total Time\n");
		return -1;
	}*/
	if ( ( acc <= 0.0 ) | ( vel <= 0.0 ) ){
		printf("Err: serialClass::moveRelativeVel().  Acc or vel parameters either zero or negative\n");
		printf("acc=%f, vel=%f\n", acc, vel);

		return -1;
	}
	if ( ( dist > SLIDE_LENGTH ) | ( dist < -1.0 * SLIDE_LENGTH ) ){
		printf("Err: serialClass::moveRelativeVel(). Dist Larger Than Slide Length\n");
		return -1;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	dist *= ticksPerMM;
	acc *= accFactor;
	vel *= velFactor;
	
	unsigned int distToOutput = (unsigned int) dist;
	unsigned int accToOutput = (unsigned int) acc;
	unsigned int velToOutput = (unsigned int) vel;
	
	
	// ----> ID , SPC, CMD, SPC
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 32; 		//SP
	currIndex = startindex + numDigs + 1;
	
	//printf("currIndex start=%d\n", currIndex);
	
		// ----> dist, SPC
	char distChar[20];
	int numDistChars = sprintf( distChar, "%d", distToOutput); //convert decimal inte input to character array.
	//printf("distChar=%s\n", distChar); 
	//printf("length=%d\n", numDistChars);
	for (int i = 0; i < numDistChars; i++){
			output[ currIndex ] = distChar[ i ];
		 	currIndex++;
	}
	//printf( "output(%d)=%d\n", i, output[ i ] );
	output[currIndex] = 32; currIndex ++;


	
	
		// ----> rampTime, SPC
	char accChar[20];
	int numRampChars = sprintf( accChar, "%u", accToOutput); //convert decimal inte input to character array.
	//printf("accChar=%s\n", accChar); //printf("length=%d\n", numRampChars);
	for (int i = 0; i < numRampChars; i++){
		output[ currIndex ] = accChar[ i ];	
		currIndex ++;
		}
	output[ currIndex ] = 32;
	currIndex ++;	

	// ----> totalTime, SPC
	char velChar[20];
	int numTimeChars = sprintf( velChar, "%u", velToOutput); //convert decimal inte input to character array.
	//printf("velChar=%s\n", velChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numTimeChars; i++){
		output[ currIndex ] = velChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[ currIndex ] = 32;
	currIndex ++;
	
		
	// ----> stopEnable, SPC
	char stopEnChar[20];
	int stopEnable = 0;
	int numStopEnChars = sprintf( stopEnChar, "%u", stopEnable); //convert decimal inte input to character array.
	//printf("timeChar=%s\n", timeChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numStopEnChars; i++){
		output[ currIndex ] = stopEnChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[ currIndex ] = 32;
	currIndex ++;	

	// ----> stopState, SPC
	char stopStChar[20];
	int stopState = 0;
	int numStopStChars = sprintf( stopStChar, "%u", stopState); //convert decimal inte input to character array.
	//printf("timeChar=%s\n", timeChar); printf("length=%d\n", numTimeChars);
	for (int i = 0; i <  numStopStChars; i++){
		output[ currIndex ] = stopStChar[ i ];
		//printf( "output(%d)=%d\n", i, output[ currIndex ] );
		currIndex ++;

	}
	output[currIndex] = 13;
	currIndex ++;	
	output[currIndex] = 0;	//null
	currIndex ++;
	
			
			
	//test case
	/*
	output[0] = 64;	//@
	output[1] = 49;	//ID
	output[2] = 32;
	
	output[3] = 49;	//cmd = 135
	output[4] = 51;
	output[5] = 53;
	output[6] = 32;
		
	output[7] = 52;	//4000
	output[8] = 48;
	output[9] = 48;
	output[10] = 48;
	output[11] = 32;
	
	output[12] = 49;	//1000
	output[13] = 48;
	output[14] = 48;
	output[15] = 48;
	output[16] = 32;
	
	output[17] = 57;	//9000
	output[18] = 48;
	output[19] = 48;
	output[20] = 48;
	output[21] = 32;
	
	output[22] = 48;	//0
	output[23] = 32;
	
	output[24] = 48;	//0
	output[25] = 32;
	
	output[26] = 13;	//CR

	output[27] = 0;	//null		
		
	//printf("output:%s\n", output);

	for (int i = 0; i < 30; i++){
		printf( "output(%d)=%d\n", i, output[ i ] );
	}*/

			
	if (!writeport(fd, output)) {
		printf("Err: MRV write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("MRV written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
		
	}	
	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( reply == 0) {
			printf("ACK\n");
			return 0;
	}else{
		printf("Err: bad MRV response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::clearPoll( int ID, int whichBit){//, bool debugOn ){
	//bit 0..15
	//CPL
	int cmdID = 1;
	int numToSend;
	numToSend = (int) pow(2, whichBit);
	//cout << numToSend << endl;
	int numWordDigs = returnNumDigits(numToSend);
	//cout << numDigs <<endl;
	int dig[ 6 ];		//always <6 anyways
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~`
	if (  ( whichBit > 15 ) | ( whichBit < 0 ) ){
		printf("Err: serialClass::clearPoll().  Invalid Bit Request\n");
		return -1;
	}
	
	
	for(int i = numWordDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + numToSend % 10;
		numToSend /= 10;
	}
	
	char output[20];
	//output = (char*) malloc( (4 + numWordDigs + 1) * sizeof(char) );		
	//free(output);
	//output = NULL;
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	
	output[3] = NUMSTART + cmdID;
	output[4] = 32;		// space	  

	int startindex = 5;
	for (int i = startindex; i < startindex+ numWordDigs; i++){
		output[i] = dig[ i-startindex ];
		//printf("%d\n", i);
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numWordDigs] = 13; 		//CR
	output[startindex + numWordDigs + 1] = 0; 	//null
	output[startindex + numWordDigs + 1] = 0; 	//null
		
	//tcflush(fd, TCIFLUSH);	//flush port

	if (!writeport(fd, output)) {
		printf("Err: CLP write failed\n");
		//close(fd);
		return -1;
	}
	else{
		//if ( debugOn )
		//printf("CLP written:%s\n", output);
	
		//tcflush(fd, TCIFLUSH);	//flush port
		//return checkForACK( ID, output );		//if reply is device ID, then all worked.
		
	}
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( reply == 0) {
			//printf("ACK CPL\n");
			return 0;
	}else{
		printf("Err: bad CPL response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::poll(int ID, char* strToReturn){
	//POL
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 27;
	const int numDigs = 2;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 13; 		//CR

	//tcflush(fd, TCIFLUSH);	//flush port
	
	if (!writeport(fd, output)) {
		printf("Err: POL write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("POL written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
		
	}
		
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( replyData[0] == '#') {
			printf("DATA POL\n");
			strcpy(strToReturn, replyData);
			return 0;
	}else{
		printf("Err: bad CPL response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		strcpy(strToReturn, replyData);
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int serialClass::returnPosition( int ID, double* pos ){
	//RIO
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 12;
	const int numDigs = 2;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 32;		//SPC
	output[startindex + numDigs + 1] = 49;	//1
	output[startindex + numDigs + 2] = 13; //CR	
	output[startindex + numDigs + 3] = 0; 	//null

	
	if (!writeport(fd, output)) {
		printf("Err: RRG write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("RRG written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
		//flush();
	}	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	
	
	char hexResult[12];
	if ( replyData[0] == '#') {
			printf("ACK RRG\n");
			printf("%s\n", replyData);
			int status = getHexArrayFromReply( replyData, hexResult, 2);			//pull out last two words
			long decResult;
			//hexResult[5] = 49;
			//hexResult[0] = 49;
			//printf("hexResult[0]=%d\n", hexResult[0]);
			//printf("hexResult[1]=%d\n", hexResult[1]);
			//printf("hexResult=%s\n", hexResult);
			
			status = convertHexArrayToDec(hexResult, &decResult);
			//printf("in rrg got decresult=%ld\n", decResult);
			
			if ( decResult < 2*ticksPerMM*SLIDE_LENGTH*10 ){		
				
				*pos = (double) decResult / (double) ticksPerMM;
				
			}else{
				
				//this means it's a 2's complement value, and must be negated to display a negative number
				//what I'm after is ffff_ffff - result[1]_result[0]
				//but I only have result
				//( pow(2,32) - pow(2, 16) ) + decResult[1];		
				*pos = double ( (long) pow(2,32) - (double) decResult) / (double) ticksPerMM;
				
			}
			
			//*pos = ( (double) decResult[1] ) / (double) ticksPerMM;
			//printf("returned decResult=%d\n", decResult[1]);
			
			//double combinedDecResult = decResult[0]*32768 + decResult[1];
			//printf("decResult[0]=%d, decResult[1]=%d, combinedResult=%d\n", decResult[0], decResult[1],combinedDecResult);
			//strcpy(strToReturn, replyData);
			return 0;
	}else{
		printf("Err: bad RRG response\n");
		printf("%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		//strcpy(strToReturn, replyData);
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::readIO(int ID, char* strToReturn){
	//RIO
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 21;
	const int numDigs = 2;
	//~~~~~~~~~~~~~~~~~~~
	
	//printf("in readIO\n");

	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 13; 		//CR	

	
	if (!writeport(fd, output)) {
		printf("Err: RIO write failed\n");
		//close(fd);
		return 1;
	}
	else{
		//printf("RIO written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( replyData[0] == '#') {
			//printf("ACK RIO\n");
			strcpy(strToReturn, replyData);
			//printf("%s\n", strToReturn);
			//printf("returning 0\n");
			return 0;
	}else{
		printf("Err: bad RIO response\n");
		printf("%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		strcpy(strToReturn, replyData);
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
int serialClass::initDualLoop(int ID){
	//DLC
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 243;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 13; 		//CR	
	output[startindex + numDigs + 1] = 0;	//null		
	
	std::stringstream stream;
	stream <<"@"<<ID<<" 243\n\0";
	strcpy(output, stream.str().c_str());
	
	if (!writeport(fd, output)) {
		printf("Err: DLC write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("DLC written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int status;
	
	status = getReply(fd, replyData, numChars);	
	status = checkForACK( ID, replyData );
	if ( status == 0) {
			printf("ACK DLC\n");
			return 0;
	}else{
		printf("Err: bad DLC response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::initSingleLoop(int ID){
	//SLC
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 244;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 13; 		//CR	
	output[startindex + numDigs + 1] = 0;	//null		
	
	if (!writeport(fd, output)) {
		printf("Err: SLC write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("SLC written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int status;
	
	status = getReply(fd, replyData, numChars);	
	status = checkForACK( ID, replyData );
	if ( status == 0) {
			printf("ACK SLC\n");
			return 0;
	}else{
		printf("Err: bad SLC response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::setupEncoder(int ID){
	//SEE
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 192;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	
	output[6] = 32;		//SPC
		
	output[7] = 48; 		//index source: 0
	output[8] = 32;		//SPC
	
	output[9] = 49;			//index state...0-20,000
	output[10] = 48;
	output[11] = 48;
	output[12] = 48;
	output[13] = 48;
	output[14] = 32;		

	output[15] = 48;			//encoder style
	output[16] = 13;
	
	output[17] = 0;
	output[18] = 0;
	output[19] = 0;
																			
	std::stringstream stream;
	stream <<"@"<<ID<<" 192 0 0 2\n\0";

	strcpy(output, stream.str().c_str());
	if (!writeport(fd, output)) {
		printf("Err: SEE write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("SEE written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
		
	}
		
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int status;
	
	status = getReply(fd, replyData, numChars);	
	status = checkForACK( ID, replyData );
	if ( status == 0) {
			printf("ACK SEE\n");
			return 0;
	}else{
		printf("Err: bad SEE response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::halt(int ID){
	//HAL
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 2;
	const int numDigs = 1;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 13; 		//CR	
	output[startindex + numDigs + 1] = 0; 		//null
		
	
	if (!writeport(fd, output)) {
		printf("Err: HALT write failed\n");
		//close(fd);
		return 1;
	}
	else{
		//printf("HALT written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		

	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int status;
	
	status = getReply(fd, replyData, numChars);	
	status = checkForACK( ID, replyData );
	if ( status == 0) {
			printf("ACK HAL\n");
			return 0;
	}else{
		printf("Err: bad HAL response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::killActiveMotors(void){
	//disables all motor drivers.
	int status;
	if ( X_MOTOR_ACTIVE ) status = halt( X_AXIS_ID );
		if ( status == -1 ) return -1;
	if ( Y_MOTOR_ACTIVE ) status = halt( Y_AXIS_ID );	
		if ( status == -1 ) return -1;
	if ( Z_MOTOR_ACTIVE ) status = halt( Z_AXIS_ID );	
		if ( status == -1 ) return -1;
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::stop(int ID){
	//HAL
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 3;
	const int numDigs = 1;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	
	output[startindex + numDigs] = 32;
	output[startindex + numDigs + 1] = 45;		//-
	output[startindex + numDigs + 2] = 49;		//1
	output[startindex + numDigs + 3] = 13; 		//CR	
	output[startindex + numDigs + 4] = 0; 		//null
		
	
	if (!writeport(fd, output)) {
		printf("Err: STOP write failed\n");
		//close(fd);
		return 1;
	}
	else{
		//printf("STOP written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		

	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int status;
	
	status = getReply(fd, replyData, numChars);	
	status = checkForACK( ID, replyData );
	if ( status == 0) {
			printf("ACK STOP\n");
			return 0;
	}else{
		printf("Err: bad STOP response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::enableMotor(int ID){
	//RIO
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 227;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 13; 		//CR	
	output[startindex + numDigs+1] = 0; 		//null
	
	
	if (!writeport(fd, output)) {
		printf("Err: EMD write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("EMD written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int status;
	
	status = getReply(fd, replyData, numChars);	
	status = checkForACK( ID, replyData );
	if ( status == 0) {
			printf("ACK EMD\n");
			return 0;
	}else{
		printf("Err: bad EMD response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::resetMotor(int ID){
	//RIO
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 4;
	const int numDigs = 1;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 13; 		//CR	
	output[startindex + numDigs + 1] = 0; 		//null

			
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	output[3] = NUMSTART + 4;		// space	  
	output[4] = 13;		// space	  
	output[5] = 0;		// space	  
	
	
	if (!writeport(fd, output)) {
		printf("Err: RST write failed\n");
		//close(fd);
		return -1;
	}
	else{
		printf("RST written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	//no ACK returned
	usleep(3000000);
	flush();		//processor resets, so flush the port
	tcflush(fd, TCIFLUSH);

	return 0;		
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~










//=================================================================================
//=================================================================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void serialClass::flush(void){
		tcflush(fd, TCIFLUSH);	//flush motor's port		
}	
		
void serialClass::flushL(void){
		tcflush(Ld, TCIFLUSH);	//flush laser's port		
}	

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::getReply(int whichPort, char *result, int &numChars){
	usleep(10000);		//N ms
	char sResult[MAX_REPLY_LEN];
	fcntl(whichPort, F_SETFL, FNDELAY); // don't block serial read

	if (!readport(whichPort,sResult)) {
		printf("ERR: getReply() read failed\n");
		close(whichPort);
		return -1;
	}		
	
	//now find how many characters are in the reply string.  Assume null termination for unfilled characters.
	string resultString(sResult);
	numChars = resultString.length() + 1;
	
	/*now check through for corrupted data.  There should never be anything greater than 102 (=f), since all
	symbols are below the numbers*/
	
	for(int i = 0; i < numChars; i++){
		if ( sResult[ i ] > 102 ){
			printf("ERR: getReply() illegal character\n");
			printf("readport=%s\n", sResult);
			flush();			//flush the port of illegal characters
			return -1;
		}
	}
	
	//printf("numChars received=%d\n", numChars);	strcpy(result, sResult);
	
	strcpy(result, sResult);
	result[ numChars - 1] = 13;		//add the CR
	result[ numChars ] = 0;		//add null
	
	//result = "hello";
	//printf("readport=%s\n", result);
	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    
serialClass::serialClass(){ }
serialClass::~serialClass ( ){ }



int serialClass::checkForACK( int ID, char* inputArray ){
	if (  inputArray[ 0 ] != '*' ){
		//printf("ERR in ACK seeing * as first character\n");
		return -1;
	}
	
	//NOTE: this only works for 1-digit device ID's:
	if ( ID != inputArray[ 3 ] - NUMSTART ){
		printf("ERR in seeing ID number in ACK\n");
		return -1;
	}
		
	//printf("ACK received. ReplyString=%s\n", inputArray);
	return 0;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::initMotSerPort() {
	#define BAUD 57600;                      // derived baud rate from command line

		fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		perror("open_port: Unable to open /dev/ttyS0 - ");
		return 1;
	} else {
		fcntl(fd, F_SETFL, 0);
	}

	
	struct termios options;
	// Get the current options for the port...
	tcgetattr(fd, &options);
	// Set the baud rates to 19200...
	int reply = cfsetispeed(&options, B57600);
	//printf("error = %d\n", reply);
	reply = cfsetospeed(&options, B57600);
	//printf("error = %d\n", reply);
	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD);

	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// Set the new options for the port...
	tcsetattr(fd, TCSANOW, &options);
		
   //printf("baud=%d\n", getbaud(fd));
	
	flush();

	return 1;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::changeAntiHunt(int ID) {
	//AHC
	

	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 150;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	char output[40];
	
	std::stringstream stream;
	stream <<"@"<<ID<<" 150 4 6\n\0";

	strcpy(output, stream.str().c_str());
	if (!writeport(fd, output)) {
		printf("Err: AHC write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("AHC written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( reply == 0) {
			printf("ACK AHC\n");
			//printf("here1\n");
			//delete replyData;
			return 0;
	}else{
		printf("Err: bad AHC response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		//delete replyData;
		return -1;
	}
	
	return -1;		//should never be here*/
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::goClosedLoop(int ID) {
	//GCL
	

	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 142;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	char output[40];
	
	std::stringstream stream;
	stream <<"@"<<ID<<" 142\n\0";

	strcpy(output, stream.str().c_str());
	if (!writeport(fd, output)) {
		printf("Err: GCL write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("GCL written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( reply == 0) {
			printf("ACK GCL\n");
			//printf("here1\n");
			//delete replyData;
			return 0;
	}else{
		printf("Err: bad GCL response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		//delete replyData;
		return -1;
	}
	
	return -1;		//should never be here*/
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::changeACKdelay(int ID, double delay) {
	//ADL
	

	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 173;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 32; 		//SPC
	output[startindex + numDigs + 1]	= 49;	//1		//the delay is number is 120us * this number.
	output[startindex + numDigs + 2]	= 48;	//0
	output[startindex + numDigs + 3] = 13;	//CR
	output[startindex + numDigs + 4] = 0;	//null	
	
	std::stringstream stream;
	stream <<"@"<<ID<<" 173 10\n\0";

	strcpy(output, stream.str().c_str());
	if (!writeport(fd, output)) {
		printf("Err: ADL write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("ADL written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	sleep(0.1);
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int reply;
	
	reply = getReply(fd, replyData, numChars);	
	reply = checkForACK( ID, replyData );
	if ( reply == 0) {
			printf("ACK ADL\n");
			//printf("here1\n");
			//delete replyData;
			return 0;
	}else{
		printf("Err: bad ADL response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		//delete replyData;
		return -1;
	}
	
	return -1;		//should never be here*/
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::closeSerialPort(void){
   close(fd);
	return 0;
}

int serialClass::closeLaserSerialPort(void){
   close(Ld);
	return 0;
}


int serialClass::returnNumDigits(int number){
	if ( number < 10 ){ return 1; }
	else if ( number < 100 ){ return 2; }
	else if ( number < 1000 ) { return 3; }
	else if ( number < 10000 ) { return 4; }
	else if ( number < 100000 ) { return 5; }
	else if ( number < 1000000 ) { return 6; }
	else{ cout << "Err: returnNumDigits" << endl; return 0; }
}


int serialClass::displayNACKerrors(char* inputArray) {
	//takes in a reply from the controller, and displays the NACK errors	
	return 0;
}


int serialClass::getIObitFromReply(char* inputArray, int whichBit){
	//takes in a reply and using getHexArrayFromReply() takes out the last 4 hex digits.
	//these 4 hex characters ( ) represent the polling status word PSW, are are then converted to bits.
	//then it displays all error messages.

	
	//printf("in getIObitFromReply(), array=%s\n", inputArray);
	char hexResult[4];	
	int status = getHexArrayFromReply( inputArray, hexResult, 1);
	if ( status != 0 ){ 
		printf("Err: serialClass::displayPSWdescriptions, getHexArrayFromReply\n" ); 
		return -1;
	}			
	//printf( "hexResult = %c%c%c%c\n", hexResult[0], hexResult[1], hexResult[2], hexResult[3] );
			
	long decResult;
	status = convertHexArrayToDec(hexResult, &decResult);
	//printf("getIObitFromReply:decResult=%ld\n", decResult);
	if ( status != 0){ 
		printf("Err: serialClass::displayPSWdescriptions, convertHexArrayToDec\n" ); 
		return -1;
	}

	/*if ( decResult > 10000 ){		//if it's huge, consider that a negative number
		decResult = pow(2, 32) - decResult;
	}else{
				
	}*/
	//printf("getting IO bit with decResult=%ld\n", decResult);
	int binaryArray[16];
	decNumberToBinaryArray( (int) decResult, binaryArray );

	/*for( int j = 15; j >= 0; j-- ){
		printf( "%d:%d, ", j, binaryArray[j] );
	}*/
	//printf("\n");
	//sleep(0.01);	
	//printf("--->getIObitFromReply result=%d\n", binaryArray[ whichBit ]); 
	return binaryArray[ whichBit ];
	
}

int serialClass::displayIOdescriptions(char* inputArray){
	//takes in a reply and using getHexArrayFromReply() takes out the last 4 hex digits.
	//these 4 hex characters ( ) represent the polling status word PSW, are are then converted to bits.
	//then it displays all error messages.

	char hexResult[4];	
	int status = getHexArrayFromReply( inputArray, hexResult, 1);
	if ( status != 0 ){ 
		printf("Err: serialClass::displayPSWdescriptions, getHexArrayFromReply\n" ); 
		return -1;
	}			
	//printf( "hexResult = %c%c%c%c\n", hexResult[0], hexResult[1], hexResult[2], hexResult[3] );
			
	long decResult;
	status = convertHexArrayToDec(hexResult, &decResult);
	//printf("dec=%d\n", decResult);
	if ( status != 0){ 
		printf("Err: serialClass::displayPSWdescriptions, convertHexArrayToDec\n" ); 
		return -1;
	}

	int binaryArray[16];
	decNumberToBinaryArray( (int) decResult, binaryArray );

	for( int i = 15; i >= 0; i-- ){
		printf( "%d", binaryArray[i] );
	}
	printf("\n"); 

	for( int i = 15; i >= 0; i-- ){
		//printf( "%d", binaryArray[i] );
		if ( binaryArray[ i ] == 1 ){		//==1
			printPSWmessage( i );
			
		}
		
	}
	return 0;
} //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::displayPSWdescriptions(char* inputArray) {
	//takes in a reply and using getHexArrayFromReply() takes out the last 4 hex digits.
	//these 4 hex characters ( ) represent the polling status word PSW, are are then converted to bits.
	//then it displays all error messages.

	char hexResult[4];	
	int status = getHexArrayFromReply( inputArray, hexResult, 1);
	if ( status != 0 ){ 
		printf("Err: serialClass::displayPSWdescriptions, getHexArrayFromReply\n" ); 
		return -1;
	}			
	//printf( "hexResult = %c%c%c%c\n", hexResult[0], hexResult[1], hexResult[2], hexResult[3] );
			
	long decResult;
	status = convertHexArrayToDec(hexResult, &decResult);
	//printf("dec=%d\n", decResult);
	if ( status != 0){ 
		printf("Err: serialClass::displayPSWdescriptions, convertHexArrayToDec\n" ); 
		return -1;
	}

	int binaryArray[16];
	decNumberToBinaryArray( (int) decResult, binaryArray );

	for( int i = 15; i >= 0; i-- ){
		printf( "%d", binaryArray[i] );
	}
	printf("\n"); 

	for( int i = 15; i >= 0; i-- ){
		//printf( "%d", binaryArray[i] );
		if ( binaryArray[ i ] == 1 ){		//==1
			printPSWmessage( i );
			
		}
		
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::clearAllPSWbits(int ID){
	//clear all bits of the polling status word
	int status;
	for(int i = 0; i < 16 ; i++){
		status = clearPoll(ID, i);
		if ( status == 0 ){
		}else{
			printf("Err: clearing PSW bits\n");
			return -1;
		}
	}
	printf("Cleared all PSW bits\n");
	return 0;
}

void serialClass::printIOmessage(int index){
	switch (index){
			case 0:
				printf("  IO.0: Aborted Packet.  Data Error or Previous Packet Collision\n");
				break;
			case 1:						
				printf("  IO.1: Invalid Checksum (9-bit Protocol Only)\n");
				break;
			case 2:
				printf("  IO.2: Soft Limit Reached (SSL)\n");
				break;		
			case 3:				
				printf("  IO.3: Device Shutdown due to Kill (KMC / KMX) \n");
				break;
			case 4:				
				printf("  IO.4: Packet Framing Error; Missing Bits\n");
				break;
			case 5:					
				printf("  IO.5: Message Too Long (>31 bytes)\n");
				break;
			case 6:				
				printf("  IO.6: Condition Met While Executing CKS Command\n");
				break;
			case 7:				
				printf("  IO.7: Serial Rx Overflow\n");
				break;
			case 8:				
				printf("  IO.8: Moving Error (ERL) Exceeded in Moving State\n");
				break;
			case 9:				
				printf("  IO.9: Holding Limit Error (ERL) Exceeded in Holding State\n");
				break;			
			case 10:				
				printf("  IO.10: Low/Over Voltage\n");
				break;			
			case 11:				
				printf("  IO.11: Motion Ended due to Input\n");
				break;			
			case 12:				
				printf("  IO.12: Command Error: parameter values or firmware\n");
				break;			
			case 13:				
				printf("  IO.13: Buffer Commands Completed\n");
				break;			
			case 14:
				printf("  IO.14: Checksum Error\n");
				break;		
			case 15:							
				printf("  IO.15: Immediate Command Done\n");
				break;
			
			default:
				printf("Err:serialClass::printPSWmessage()\n");
		}
										
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void serialClass::printPSWmessage(int index){
	switch (index){
			case 0:
				printf("  PSW.0: Aborted Packet.  Data Error or Previous Packet Collision\n");
				break;
			case 1:						
				printf("  PSW.1: Invalid Checksum (9-bit Protocol Only)\n");
				break;
			case 2:
				printf("  PSW.2: Soft Limit Reached (SSL)\n");
				break;		
			case 3:				
				printf("  PSW.3: Device Shutdown due to Kill (KMC / KMX) \n");
				break;
			case 4:				
				printf("  PSW.4: Packet Framing Error; Missing Bits\n");
				break;
			case 5:					
				printf("  PSW.5: Message Too Long (>31 bytes)\n");
				break;
			case 6:				
				printf("  PSW.6: Condition Met While Executing CKS Command\n");
				break;
			case 7:				
				printf("  PSW.7: Serial Rx Overflow\n");
				break;
			case 8:				
				printf("  PSW.8: Moving Error (ERL) Exceeded in Moving State\n");
				break;
			case 9:				
				printf("  PSW.9: Holding Limit Error (ERL) Exceeded in Holding State\n");
				break;			
			case 10:				
				printf("  PSW.10: Low/Over Voltage\n");
				break;			
			case 11:				
				printf("  PSW.11: Motion Ended due to Input\n");
				break;			
			case 12:				
				printf("  PSW.12: Command Error: parameter values or firmware\n");
				break;			
			case 13:				
				printf("  PSW.13: Buffer Commands Completed\n");
				break;			
			case 14:
				printf("  PSW.14: Checksum Error\n");
				break;		
			case 15:							
				printf("  PSW.15: Immediate Command Done\n");
				break;
			
			default:
				printf("Err:serialClass::printPSWmessage()\n");
		}
										
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


/*
//PSW:
15 Yes I-Cmd Done     Immediate Command Done (i.e. Host Command).
                      There was a checksum error while reading data from
14 Yes  NV Mem Error  or to the non-volatile memory. (SilverDust Rev 06
                      adds write protection to certain regions.)
                      All commands active in the Program Buffer finished
13 Yes  P-Cmd Done
                      executing.
                      There was an error associated with the command
12 Yes Command Error  execution. Unreasonable parameter values or not
                      support in this firmware
                      The motion ended when the selected exit/stop
11 Yes   Input Found
                      condition was met.
10 Yes  Low/Over Volt A low or over voltage error occurred.
                      Holding error limit set by the Error Limits (ERL)
9  Yes  Holding Error command was exceeded during a holding control
                      state.
                      Moving error limit set with the ERL command was
8  Yes   Moving Error
                      exceeded with the device in a moving control state.
7  Yes   Rx Overflow  Device serial receive (UART) buffer overflowed.
                      A condition was met while executing a CKS
                      command . One of the conditions set with the Check
6  Yes CKS Cond Met
                      Internal Status (CKS) command was met.
                      The received message was too big for the Serial
5  Yes  Msg Too Long
                      Buffer. Device rx packet > 31 bytes
                      There was a packet framing error in a received byte.
4  Yes  Framing Error
                      Device rx byte with missing bit
                      The device was shut down due to one or more
3  Yes    Shut Down   conditions set with the Kill Motor Condition (KMC)
                      command (or KMX command for SilverDust Rev 06).
                      A soft stop limit was reached as set by the Soft Stop
2  Yes     Soft Limit
                      Limit (SSL) command.
                      Device rx packet with an invalid checksum. Valid for
1  Yes  Rx Checksum
                      9 Bit Binary and Modbus only.
                      There was a data error or a new packet was received
0  Yes   Aborted Pkt
                      before the last packet was complete.

*/


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::getCloseSwitch(int ID){
	//return state of switch closest to home
	
	char replyData[MAX_REPLY_LEN];
	int status;
	status = readIO(ID, replyData);		//X axis is unit ID of 1
	
	return 1 - getIObitFromReply(replyData, IO2bit);		//1 for on, 0 for off
}

int serialClass::getFarSwitch(int ID){
	//return state of switch farthest from home	

	char replyData[MAX_REPLY_LEN];
	int status;
	status = readIO(ID, replyData);		//X axis is unit ID of 1

	return 1 - getIObitFromReply(replyData, IO1bit);		//1 for on, 0 for off
}

int serialClass::getHomeSwitch(int ID){
	//return state of home point switch
	
	char replyData[MAX_REPLY_LEN];
	int status;
	status = readIO(ID, replyData);		//X axis is unit ID of 1
	//printf("got from readIO:%s\n", replyData);
	//printf("about to get IObit\n");
	int IObit = getIObitFromReply(replyData, IO3bit);
	//printf("gotIObit=%d\n", IObit);
	return 1 - IObit;		//for all stage axes, 1 for on, 0 for off
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::zeroTarget(int ID){
	//ZTP
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 145;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	output[startindex + numDigs] = 13; 		//CR	
	output[startindex + numDigs + 1] = 0;	//null		
	
	if (!writeport(fd, output)) {
		printf("Err: ZTP write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("ZTP written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	int numChars;
	int status;
	
	status = getReply(fd, replyData, numChars);	
	status = checkForACK( ID, replyData );
	if ( status == 0) {
			printf("ACK ZTP\n");
			return 0;
	}else{
		printf("Err: bad ZTP response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::isCommandDone(int ID){
	//returns 1 if immediate command is done (PSW bit 15 is high) and 0 if not done (bit 15 is low still)
	//motion commands should only execute if that bit's set, and they should clear it at the start of their motion.
	char replyData[254];
	int status, reply;
	status = poll(ID, replyData);
	if ( status == 0) {
			printf("ACK in CMD_DONE\n");
	}else{
		printf("Err: bad CMD_DONE response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	reply = getIObitFromReply(replyData, 15);
	if ( reply == -1 ){
		printf("ERR: isCommandDone()\n");
		return -1;
	}else if ( reply == 0 ){
		printf("NOT DONE\n");
		return 0;
	}else if ( reply == 1 ){
		printf("DONE!\n");
		return 1;
	}
	printf("ERR: isCommandDone().  Should never be here.\n");
	return -1;		//should never be here
}

int serialClass::initIO(int ID){
	//prep the I/O, for laser connection, etc.

	setIObit(ID, IO7bit, 0);
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::setIObit(int ID, int whichBit, int state){
	//CIO
	//set an IO bit.  state = -1 for high-Z, 1/0 for high/low.
	//~~~~~~~~~~~~~~~~~~~
	const int cmdID = 188;
	const int numDigs = 3;
	//~~~~~~~~~~~~~~~~~~~
	
	int dig[ numDigs ];
	int num = cmdID;
	for(int i = numDigs - 1; i >= 0 ; i--)
	{
		dig[ i ] = NUMSTART + num % 10;
		num /= 10;
	}
	
	char output[40];
	output[0] = 64;		//@
	output[1] = NUMSTART + ID;	//device ID
	output[2] = 32;		// space	  
	int startindex = 3;
	for (int i = startindex; i < startindex + numDigs; i++){
		output[i] = dig[ i-startindex ];
		//cout << i-3 << ","<<dig[i-3] << endl;
	}
	
	
	//NOTE: we only should ever have an actual bit that's <=9, 
	//so if double-digits are needed then this code must be rewritten to be general
	output[6] = 32;	//SPC
	output[7] = whichBit + NUMSTART;	// I/O line #
	output[8] = 32;	//SPC
	
	printf("output[7] (bit)=%d\n", output[7]);
	
	if ( state == -1 ){
		output[9] =	45;	//-
		output[10] = 49;	//1
		output[11] = 13;
		output[12] = 0;
	}else{
		output[9] =	state + NUMSTART;	//-
		output[10] = 13;
		output[11] = 0;
	}
		
			
	if (!writeport(fd, output)) {
		printf("Err: CIO write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("CIO written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(fd, replyData, numChars);	
	status = checkForACK( ID, replyData );
	if ( status == 0) {
			printf("ACK CIO\n");
			return 0;
	}else{
		printf("Err: bad CIO response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::movePosRel(int ID, double distance){

	double time = max( fabs(distance / 4.0), 0.5);		
	//if the step is zero, I need a small time parameter so that errors don't appear later.
	
	//printf("in movePosRel(), time=%f, dist=%f\n", time, distance);
	
	int status = moveRelativeTime(ID, distance, time / 3.0, time );		//about 1 cm/s
	if ( status == -1 ){
			printf("ERR: movePosRel()\n");
			stop(ID);
			return -1;
	}
	sleep(time + 0.5);
	return 0;
}



int serialClass::movePosAbs(int ID, double distance){
	double pos;
	int status = returnPosition(ID, &pos);
	if ( status == -1 ) return -1;
	
	double time = max( fabs(distance - pos) / 3.0, 0.5 );
	status = moveAbsoluteTime(ID, distance, time / 6.0, time);
	if ( status == -1 ){
			printf("ERR: movePosAbs()\n");
			stop(ID);
			return -1;
	}	
	sleep( time + 0.5 );
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::stepAndPulseSequenceBothAxes(int ID1, int ID2, double startPos1, double startPos2, 
						double stepSize1, double stepSize2, 
											int numOfSteps, double	laserWidth, double laserAmp){
	//starts at a known position, and pulses after each motion step
	//step sizes in um, start positions in um.
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (   ( stepSize1 <= 0.0 ) | ( ID1 < 0 ) | ( ID1 > 100 ) | ( numOfSteps < 1 ) | ( laserWidth <2 ) | (stepSize1 >
		SLIDE_LENGTH_UM) | ( startPos1 < -1.0 * SLIDE_LENGTH_UM )  | ( startPos1 > SLIDE_LENGTH_UM )   ){
		printf("ERR: stepAndPulseSequence().  Incorrect parameter; outside allowed range\n");
		printf("ID=%d, startPos=%f, stepSize=%f, numOfIntervals=%d, duration=%f, intensity=%f\n", 
						ID1, startPos1, stepSize1, numOfSteps, laserWidth, laserAmp);
		stop(ID1);
		return -1;			
	}//~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
	//double pos;
	int status;
	//status = returnPosition(ID, &pos);
	//if ( status == -1 ) return -1;
	
	
	status = movePosAbs(ID1, startPos1 / 10000.0);			//moveAbs needs cm, so divide by 10000
	if ( status == -1 ){
		return -1;
	}
	status = movePosAbs(ID2, startPos2 / 10000.0);
	if ( status == -1 ){
		return -1;
	}
	
	for(int i = 0; i < numOfSteps; i++){
		double newPos1 = (startPos1 + i*stepSize1) / 10000.0;
		double newPos2 = (startPos2 + i*stepSize2) / 10000.0;
		status = movePosAbs(ID1, newPos1);
		if ( status == -1 ){
			printf("ERR: stepAndPulseSequence() while stepping\n");
			stop(ID1);
			return -1;
		}
		flush();
		status = movePosAbs(ID2, newPos2);
		if ( status == -1 ){
			printf("ERR: stepAndPulseSequence() while stepping\n");
			stop(ID2);
			return -1;
		}
		flush();
				
		laserWidth = 3.0;
		status = pulseLaser(laserWidth, laserAmp);
		if ( status == -1 ){
			printf("ERR: stepAndPulseSequence() setting laser\n");
			stop(ID1);			
			stop(ID2);						
			return -1;
		}
		
		printf("Pulsing Laser\n");
		sleep (0.1);
		
	}
	
	status = gotoHomePoint(ID1);
	if ( status == -1 ){
		printf("ERR: stepAndPulseSequenceBothAxes() going home ID1\n");
		stop(ID1);			
		return -1;
	}	
	status = gotoHomePoint(ID2);	
	if ( status == -1 ){
		printf("ERR: stepAndPulseSequenceBothAxes() going home ID2\n");
		stop(ID2);			
		return -1;
	}		
	return 0;	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::stepAndPulseSequence(int ID, double startPos, double stepSize, 
							int numOfIntervals, double	duration, double intensity){
	//starts at a known position, and pulses after each motion step
	
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (   ( stepSize <= 0.0 ) | ( ID < 0 ) | ( ID > 100 ) | ( numOfIntervals < 1 ) | ( duration <0 ) | (stepSize >
		100.0) | ( startPos < -1.0 * SLIDE_LENGTH )  | ( startPos > SLIDE_LENGTH )   ){
		printf("ERR: stepAndPulseSequence().  Incorrect parameter; outside allowed range\n");
		printf("ID=%d, startPos=%f, stepSize=%f, numOfIntervals=%d, duration=%f, intensity=%f\n", 
						ID, startPos, stepSize, numOfIntervals, duration, intensity);
		stop(ID);
		return -1;			
	}//~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
	//double pos;
	int status;
	//status = returnPosition(ID, &pos);
	//if ( status == -1 ) return -1;
	
	
	status = movePosAbs(ID, startPos);
	if ( status == -1 ){
		return -1;
	}
	
	for(int i = 0; i < numOfIntervals; i++){
		double newPos = startPos + i*stepSize;
		double time = stepSize;
		time = 0.7;
		//printf("time=%f\n",time);
		status = movePosAbs(ID, newPos);	
		if ( status == -1 ){
			printf("ERR: stepAndPulseSequence() while stepping\n");
			stop(ID);
			return -1;
		}
		flush();
				
		duration = 1.0;
		status = pulseLaser(duration, intensity);


		if ( status == -1 ){
			printf("ERR: stepAndPulseSequence() setting laser\n");
			stop(ID);			
			return -1;
		}
		
		printf("Pulsing Laser\n");
		sleep (0.1);
		
	}
	
	status = gotoHomePoint(ID);
	
	return 0;	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//====================================================================
int serialClass::pulseLaser(double duration, double intensity){
	//duration in ns
	
	int status;
	printf("Pulsing Laser\n");
	status = setIObit(X_AXIS_ID, 7, 1);
	if ( status == -1 ){
		return status;
	}
	
	//status = setIObit(X_AXIS_ID, 7, 0);
	
	status = setLaserWidth_ns(duration);	
	status = setLaserAmp_mV(intensity);	
	status = sendLaserTrigger();

			/*			
	if ( status == -1 ){
		return status;
	}*/
			
	printf("Laser OFF\n");
	
	return 0;

}//===================================================================


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::gotoHomePoint(int ID ){
	double pos;
	int status = returnPosition(ID, &pos);		//needed, to set the relative speeds with which to move.
	if ( status == -1 ) return -1;
	
		status = movePosAbs( ID, 0.0 );
		if ( status == -1 ){
			printf("ERR: gotoHomePoint() sending moveAbsoluteTime command\n");
			//stop(ID);
		return -1;
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::resetAsHomePoint(int ID ){
	double pos;
	int status = zeroTarget( ID );
	if ( status == -1 ){
			printf("ERR: resetAsHomePoint() sending zeroTarget command\n");
			//stop(ID);
	return -1;
	}else{
		return 0;
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~		
		
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::clearInternalStatus(int ID) {
        //ADL


        //~~~~~~~~~~~~~~~~~~~
        const int cmdID = 163;
        const int numDigs = 3;
        //~~~~~~~~~~~~~~~~~~~
        char output[40];

        std::stringstream stream;
        stream <<"@"<<ID<<" 163\n\0";

        strcpy(output, stream.str().c_str());
        if (!writeport(fd, output)) {
                printf("Err: CIS write failed\n");
                //close(fd);
                return 1;
        }
        else{
                printf("CIS written:%s\n", output);
               //tcflush(fd, TCIFLUSH);        //flush port
        }
        sleep(0.5);
        //~~~~~~~~ reply ~~~~~~~~~
        char replyData[MAX_REPLY_LEN];
        int numChars;
        int reply;

        reply = getReply(fd, replyData, numChars);
        reply = checkForACK( ID, replyData );
        if ( reply == 0) {
                        printf("ACK CIS\n");
                        //printf("got:%s\n", replyData);
                        //printf("here1\n");
                        //delete replyData;
                        return 0;
        }else{
                printf("Err: bad CIS response\n");
                printf("got:%s\n", replyData);
                //printf( "\n" );
                //status = displayPSWdescriptions( replyData );
               //delete replyData;
                return -1;
        }

        return -1;              //should never be here*/
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::gotoAndSetHomePoint(int ID){
	
	if ( ID < 1 ){
			printf("ERR: gotoAndSetHomePoint() invalid ID\n");
			return -1;
	}
	int eventuallyReturn = 0, status;
	

      if ( ID == Z_AXIS_ID ){
                //special case for Z axis which doesn't have homing switches
                flush();
                sleep(1.0);
                status = zeroTarget(ID);
                sleep(0.5);
                printf("About to move back towards home point\n");
                double time = 2.5;
                status = moveRelativeTime(ID, -4.0, time*3.0, time*6.0 );               //about 1 cm/s
                if ( status == -1 ){
                                printf("ERR: movePosRel()\n");
                                stop(ID);
                                return -1;
                }
                sleep(time + 0.5);
                /*status = resetMotor(ID);              //by now the motor should have shut off due to thermal overload, so reset.

                if ( status == -1 ){
                        printf("ERR: serialClass::gotoAndSetHomePoint() sending moveRelativeTime command\n");
                        eventuallyReturn = -1;                  //make sure it sticks around
                        stop(ID);
                        return -1;
                }*/
                sleep (1.0);
                printf("Sending HAL\n");
                status = stop( ID );
                sleep (0.01);

                status = clearInternalStatus(ID);
                if ( status == -1 ){
                        printf("ERR: serialClass::gotoAndSetHomePoint() sending CIS command\n");
                        eventuallyReturn = -1;                  //make sure it sticks around
                        stop(ID);
                        return -1;
                }

                status = enableMotor(ID);
                if ( status == -1 ){
                        printf("ERR: serialClass::gotoAndSetHomePoint() sending HLT command\n");
                        eventuallyReturn = -1;                  //make sure it sticks around
                        stop(ID);
                        return -1;
                }


                status = zeroTarget(ID);
                sleep (0.05);
                return 0;

	}else{
		
		//first move out 10cm or so to make sure the home switch is cleared, then set 0 point, 
		//then return from there, then reset zero point.

		printf("About to move away from home point\n");
		status = movePosRel(ID, 10.0);
		if ( status == -1 ){
			printf("ERR: gotoAndSetHomePoint() sending movePosRel command\n");
			eventuallyReturn = -1;			//make sure it sticks around
			stop(ID);
			return -1;
		}

		flush();
		sleep(1.0);
		status = zeroTarget(ID);
		sleep(0.5);
		printf("About to move back towards home point\n");
		status = moveRelativeTime(ID, -50.0, 0.1, 20.0);
		if ( status == -1 ){
			printf("ERR: serialClass::gotoAndSetHomePoint() sending moveRelativeTime command\n");
			eventuallyReturn = -1;			//make sure it sticks around
			stop(ID);
			return -1;
		}
		sleep (0.01);	
		while ( getHomeSwitch(ID) == 0){
			//printf("checking switches\n");			
			sleep ( 0.005 );		//should be 0.05, but is segfaulting on the cleanroom computer
		}
		status = stop( ID );
		sleep (0.01);
		status = zeroTarget(ID);
		sleep (0.01);


		if ( eventuallyReturn == -1 ){
			return eventuallyReturn;
		}
	
	
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::setLaserWidth_ns(double width){
	//width in nanoseconds

	
	printf("width received:%f\n", width);
	if ( (width < 2.0) | (width > 50.0) ){
		printf("Err:Pulse Width Out of Range\n");
		return -1;
	}
			
	char doubleArr[10];
	//char intArr[3];
	
	sprintf(doubleArr, "%01.2f", width);
	//sprintf(intArr, "%d", width);
	int number = (int) width;
	
			
	int numIntDigs = 0;
	if ( number >= 100 ){
		numIntDigs = 3;
	} else if ( number >= 10 ){
		numIntDigs = 2;
	} else if (number >= 0 ){
		numIntDigs = 1;
	}
	
	//string doubleStr(doubleArr);
	//int doubleLen = doubleStr.length();
	//printf("doubleArr=%s\n", doubleArr);
	//printf("intArr=%s\n", intArr);	
	
	char output[20]={"puls:widt "};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	
	
	ostringstream sout;
	sout << number;

	char *buff = new char[sout.str().length() + 1];
	strcpy(buff, sout.str().c_str());
	
	int buffIndex = 0;
	for( int i = len; i < len + numIntDigs; i++){
		output[ i ] = buff[buffIndex];
		buffIndex++;
	}
	
	output[ len + numIntDigs ] = 'n';
	output[ len + numIntDigs + 1 ] = 's';
	output[ len + numIntDigs + 2 ] = 13;	
	output[ len + numIntDigs + 3 ] = 0;		
	
	printf("output=%s<cr>\n", output);
	
	//delete [] buff;

	
	
		
	if (!writeport(Ld, output)) {
		printf("Err: laser width write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("width written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	
	return 0;
	
	//do not expect a reply from the laser, since it doesn't send messages with a CR.
	
	/*
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from laser width=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad laser width reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	*/
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::setLaserAmp_mV(double amp){
	//width in nanoseconds
	//amp in mV
	
	
	if ( (amp < 0.0) | (amp > 10000) ){
		printf("Pulse Amp Out of Range\n");
		return -1;
	}
			
	char doubleArr[10];
	//char intArr[3];
	
	sprintf(doubleArr, "%01.2f", amp);
	//sprintf(intArr, "%d", amp);
	int number = (int) amp;
	
			
	int numIntDigs = 0;
	if ( number >= 10000 ){
		numIntDigs = 5;
	} else if ( number >= 1000 ){
		numIntDigs = 4;
	} else if ( number >= 100 ){
		numIntDigs = 3;
	} else if (number >= 10 ){
		numIntDigs = 2;
	} else if (number >= 1 ){
		numIntDigs = 1;
	}
	
	//string doubleStr(doubleArr);
	//int doubleLen = doubleStr.length();
	//printf("doubleArr=%s\n", doubleArr);
	//printf("intArr=%s\n", intArr);	
	
	char output[20]={"voltage "};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	
	
	ostringstream sout;
	sout << number;

	char *buff = new char[sout.str().length() + 1];
	strcpy(buff, sout.str().c_str());
	
	int buffIndex = 0;
	for( int i = len; i < len + numIntDigs; i++){
		output[ i ] = buff[buffIndex];
		buffIndex++;
	}
	
	output[ len + numIntDigs ] = 'm';
	output[ len + numIntDigs + 1] = 'V';	
	output[ len + numIntDigs + 2 ] = 13;	
	output[ len + numIntDigs + 3 ] = 0;		
	
	printf("output=%s<cr>\n", output);
	
	//delete [] buff;

	
	
		
	if (!writeport(Ld, output)) {
		printf("Err: laser Amp write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("Amp written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	return 0;

	
	//do not expect a reply from the laser, since it doesn't send messages with a CR.
		
	/*
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from laser Amp=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad laser Amp reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	*/
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::sendLaserTrigger(void){
	
	char output[30]={"trig:sour IMM"};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	output[len] = 13;
	output[len+1] = 10;
	output[len+2] = 0;	
		
	if (!writeport(Ld, output)) {
		printf("Err: laser trigger write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("laser trigger written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	
	
	return 0;
	/*
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from laser trigger =%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad laser trigger reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	*/
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::turnOffLaserEcho( void ){
		
	char output[30]={"syst:comm:serial:echo off"};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	output[len] = 13;
	output[len+1] = 10;
	output[len+2] = 0;	
	
	if (!writeport(Ld, output)) {
		printf("Err: laser echo write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("laser echo written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}
	
	sleep (1.0);
	return 0;
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from echo=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad echo reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here		
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::disableLaserFlowControl(void){
	
	char output[50]={"syst:comm:serial:control:rts on"};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	output[len] = 13;
	output[len+1] = 10;
	output[len+2] = 0;
	
	if (!writeport(Ld, output)) {
		printf("Err: laser flowControl write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("flowControl written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	return 0;
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from flowControl=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad flowControl reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::requestLaserID(void){
	
	char output[30]={"*IDN?"};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	output[len] = 13;
	output[len+1] = 10;
	output[len+2] = 0;	
	
	if (!writeport(Ld, output)) {
		printf("Err: laser *IDN write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("*IDN written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	return 0;
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from *IDN=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad *IDN reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::initLaserSerPort() {

	
	//close(Ld);
			
	Ld = open("/dev/ttyUSB2", O_RDWR | O_NOCTTY | O_NDELAY);
	//Ld = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (Ld == -1) {
		perror("open_port: Unable to open /dev/ttyUSB2 - ");
		return -1;
	} else {
		fcntl(Ld, F_SETFL, 0);
	}

	//CRTSCTS
	
	struct termios options;
	// Get the current options for the port...
	tcgetattr(Ld, &options);
	// Set the baud rates to 1200...
	int reply = cfsetispeed(&options, B1200);
	//printf("error = %d\n", reply);
	reply = cfsetospeed(&options, B1200);
	//printf("error = %d\n", reply);
	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD );

	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~-CSTOPB;		// !! ONE stop bit for pulser
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	
	
   //options.c_lflag &= ~ECHO;
  //options.c_lflag |= ~(ICANON | ECHO | ECHOE);
 
	//~~~~~~~~~~~~~~~~~
	// options.c_cflag |= CRTSCTS;		// !! hardware flow control for the pulser
	//~~~~~~~~~~~~~~~~~
	
	
	// Set the new options for the port...
	tcsetattr(Ld, TCSANOW, &options);
		
   //printf("baud=%d\n", getbaud(fd));
	
	flushL();

	return 0;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::laserRemoteEnable( void ){
	//width in nanoseconds
	
	char output[20]={"remote"};
	//string outputStr(output);
	//int len = outputStr.length();
	//printf("len=%d\n", len);
	output[6] = 13;
	output[7] = 0;
	
	//output[len+1] = 0;
	//output[len+2] = 0;	
	
	if (!writeport(Ld, output)) {
		printf("Err: laser remote write failed\n");
		//close(fd);
		return 1;
	}
	else{
		printf("remote written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	
	sleep (0.1);
	return 0;
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from remote=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad remote reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::laserEnableOutput( void ){
	
	char output[20]={"output on"};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	output[len] = 13;
	output[len+1] = 0;
		
	if (!writeport(Ld, output)) {
		printf("Err: laser enable output write failed\n");
		//close(fd);
		return 1;			//don't want the program to shut down: the laser's replies are never received.
	}
	else{
		printf("local written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	
	sleep (0.1);
	return 0;
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from laser enable output=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad laser enable output reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::laserDisableOutput( void ){
	
	char output[20]={"output off"};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	output[len] = 13;
	output[len+1] = 0;
		
	if (!writeport(Ld, output)) {
		printf("Err: laser enable disable write failed\n");
		//close(fd);
		return 1;			//don't want the program to shut down: the laser's replies are never received.
	}
	else{
		printf("local written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	
	sleep (0.1);
	return 0;
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from laser disable output=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad laser enable disable reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::laserLocalEnable( void ){
	//width in nanoseconds
	
	char output[20]={"local"};
	string outputStr(output);
	int len = outputStr.length();
	//printf("len=%d\n", len);
	output[len] = 13;
	output[len+1] = 0;
		
	if (!writeport(Ld, output)) {
		printf("Err: laser local write failed\n");
		//close(fd);
		return 1;			//don't want the program to shut down: the laser's replies are never received.
	}
	else{
		printf("local written:%s\n", output);
		//tcflush(fd, TCIFLUSH);	//flush port		
	}	

	
	sleep (0.1);
	return 0;
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int numChars;
	int status;
	
	status = getReply(Ld, replyData, numChars);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from local=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad local reply\n");
		printf("got:%s\n", replyData);
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
