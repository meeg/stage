#include "laser.h"


//=================================================================================
//=================================================================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::initLaserSerPort() {

	
	//close(Ld);
			
	Ld = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	//Ld = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	//Ld = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (Ld == -1) {
		perror("open_port: Unable to open /dev/ttyUSB0 - ");
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
	options.c_cflag &= ~CSTOPB;		// !! ONE stop bit for pulser
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	
	options.c_cc[VMIN]=0;
        options.c_cc[VTIME]=10;
	
   //options.c_lflag &= ~ECHO;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE);
 
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

void laserClass::flushL(void){
	tcflush(Ld, TCIFLUSH);	//flush laser's port		
}	

int laserClass::writeLas(const char *output){
	int status = writeport(Ld,output);
	if (status) {
		printf("wrote to laser: %s\n",output);
	}
	else {
		printf("failed writing to laser: %s\n",output);
	}
	//sleep(0.1);
	return status;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::getLasReply(char *result){
	//usleep(100000);		//N ms
	char sResult[254];
	//fcntl(Ld, F_SETFL, 0); // block on serial read

	if (!readport_blocking(Ld,sResult,"Ready for command > ")) {
		printf("ERR: getReply() read failed\n");
		close(Ld);
		return -1;
	}		
	
	int numChars = strlen(sResult);
	
	/*now check through for corrupted data.  There should never be anything greater than 102 (=f), since all
	symbols are below the numbers*/
	
	for(int i = 0; i < numChars; i++){
		if ( sResult[ i ] > 126 ){
			printf("ERR: getReply() illegal character %d at position %d\n",sResult[i],i);
			printf("readport=%s\n", sResult);
			flushL();			//flush the port of illegal characters
			return -1;
		}
	}
	
	//printf("numChars received=%d\n", numChars);	strcpy(result, sResult);
	
	strcpy(result, sResult);
	return 0;
}

int laserClass::writeLasAndCheckACK( const char* output ){
	char replyData[254];
	writeLas(output);
	getLasReply( replyData);	
	char * inputPtr = replyData;

	//for (int i=0;i<strlen(outputArray);i++) printf("%d\t%d\n",inputPtr[i],outputArray[i]);
	if ( strncmp(inputPtr,output,strlen(output)) != 0 ){
		printf("No echo\n");
		//return -1;
	} else {
		inputPtr+=strlen(output);
	}

		
	char * ack="\r\n\r\nReady for command > ";
	//for (int i=0;i<strlen(inputPtr);i++) printf("%d\t%d\n",inputPtr[i],ack[i]);
	if ( strncmp(inputPtr,ack,strlen(ack)) != 0 ){
		printf("Not an ACK: ReplyString=%s\n",replyData);
		return -1;
	}
	printf("ACK received to command: %s\n", output);
	return 0;
}

int laserClass::writeLasAndGetReply( const char* output, char *result ){
	char replyData[254];
	writeLas(output);
	getLasReply( replyData);	

	char * inputPtr = replyData;
	if ( strncmp(inputPtr,output,strlen(output)) != 0 ){
		printf("No echo\n");
		return -1;
	} else {
		inputPtr+=strlen(output);
	}
	char * ack="\r\n\r\nReady for command > ";
	if ( strcmp(inputPtr+strlen(inputPtr)-strlen(ack),ack) != 0 ){
		printf("No echo\n");
		return -1;
	} else {
		inputPtr[strlen(inputPtr)-strlen(ack)] = '\0';
	}
	strcpy(result,inputPtr);
	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::requestLaserID(void){
	strcpy(output,"*IDN?\r");
	writeLas(output);
	printf("*IDN? written:%s\n", output);

	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int status;
	
	status = getLasReply(replyData);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from *IDN=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad *IDN reply\n");
		printf("got:%s\n", replyData);
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::turnOnLaserEcho( void ){
	strcpy(output,"syst:comm:serial:echo on\r");
	printf("laser echo written:%s\n", output);
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::laserRemoteEnable( void ){
	strcpy(output,"remote\r");
	writeLas(output);
	printf("remote written:%s\n", output);

	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int status;
	
	status = getLasReply(replyData);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from remote=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad remote reply\n");
		printf("got:%s\n", replyData);
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::disableLaserFlowControl(void){
	strcpy(output,"syst:comm:serial:control:rts on\r");
	printf("laser flowControl written:%s\n", output);
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::laserEnableOutput( void ){
	strcpy(output,"output on\r");
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::laserDisableOutput( void ){
	strcpy(output,"output off\r");
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::laserTriggerInternal( void ){
	strcpy(output,"trigger:source internal\r");
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::laserTriggerHold( void ){
	strcpy(output,"trigger:source hold\r");
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


int laserClass::closeLaserSerialPort(void){
   close(Ld);
	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::laserInitSequence( void ){
	int status;
	printf("Initializing Laser System\n");
	status = initLaserSerPort();
	
	status = laserRemoteEnable();
	status = disableLaserFlowControl();
	status = turnOnLaserEcho();
	status = laserTriggerHold();
	status = setLaserWidth_ns(2.0);
	status = setLaserAmp_mV(1000.0);
	status = laserEnableOutput();
		
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::setLaserWidth_ns(double width){
	//width in nanoseconds
	printf("width received:%f\n", width);
	if ( (width < 2.0) | (width > 50.0) ){
		printf("Err:Pulse Width Out of Range\n");
		return -1;
	}
			
	sprintf(output,"puls:widt %fns\r",width);
	printf("width written:%s\n", output);
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::setLaserAmp_mV(double amp){
	//amp in mV
	if ( (amp < 0.0) | (amp > 10000) ){
		printf("Pulse Amp Out of Range\n");
		return -1;
	}

	sprintf(output,"voltage %fmV\r",amp);
	printf("Amp written:%s\n", output);
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::sendLaserTrigger(void){
	strcpy(output,"trig:sour IMM\r");
	printf("laser trigger written:%s\n", output);
	return writeLasAndCheckACK(output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//====================================================================
int laserClass::pulseLaser(double duration, double intensity){
	//duration in ns
	
	int status;
	printf("Pulsing Laser\n");
	
	status = setLaserWidth_ns(duration);	
	status = setLaserAmp_mV(intensity);	
	status = sendLaserTrigger();

	printf("Laser OFF\n");
	
	return 0;

}//===================================================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int laserClass::laserLocalEnable( void ){
	//width in nanoseconds
	
	strcpy(output,"local\r");
	writeLas(output);
	printf("local written:%s\n", output);

	
	//sleep (0.1);
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int status;
	
	status = getLasReply(replyData);		//laser port Ld.
	if ( status >= 0 ){
		printf("replyData from local=%s\n", replyData);
		return 0;
	}else{
		printf("Err: bad local reply\n");
		printf("got:%s\n", replyData);
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool laserClass::isIntTriggerOn( void ){
	char state[254];
	writeLasAndGetReply("trigger:source?\r",state);
printf("%s\n",state);
	return strcmp(state,"INT");
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//=================================================================================
//=================================================================================
