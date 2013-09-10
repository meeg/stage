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
int serialClass::timeToTicks(double time){
	return (int) (time/120e-6); //120 us/tick
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double serialClass::screwPitch(int ID){
	switch (ID){
		case X_AXIS_ID:
		case Y_AXIS_ID:
			return 0.2*25.4;
		case Z_AXIS_ID:
			return 0.25*25.4;
		default:
			printf("Err: invalid ID");
			return 1.0; //dummy value
	}
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double serialClass::distPerCount(int ID){
	return screwPitch(ID)/8000.0; //8000 counts per revolution
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::distToCounts(int ID, double dist){
	return (int) (dist/distPerCount(ID));
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::velToSVU(int ID, double vel){
	double maxVel = screwPitch(ID)*4000.0/60.0; //4000 RPM
	int velInt = (int) (pow(2,31.0)*vel/maxVel);
	printf("vel = %f maxVel=%f velInt = %d\n",vel,maxVel,velInt);
	return velInt;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::accToSAU(int ID, double acc){
	double maxAcc = screwPitch(ID)*4000.0/60.0/2.0/120e-6; //maxVel/2/tick
	int accInt = (int) (pow(2,30.0)*acc/maxAcc);
	printf("acc = %f maxAcc=%f accInt = %d\n",acc,maxAcc,accInt);
	return accInt;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::moveRelativeTime(int ID, double dist, double rampTime, double totalTime){
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

	sprintf(output,"@%d %d %d %d %d 0 0\r",ID,MRT,distToCounts(ID,dist),timeToTicks(rampTime),timeToTicks(totalTime));
	return writeAndCheckACK(ID, output);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::moveAbsoluteTime(int ID, double pos, double accTime, double totalTime){
	//~~~~~~~~~~ ERROR CHECK ~~~~~
	/*if ( 2 * rampTime > totalTime ){
		printf("Err: serialClass::moveAbsoluteTime().  Ramp Time Too Large Compared to Total Time\n");
		return -1;
	}*/
	if ( ( accTime <= 0.0 ) | ( totalTime <= 0.0 ) ){
		printf("Err: serialClass::moveAbsoluteTime().  Acc or time parameters either zero or negative\n");
		printf("accTime=%f, totalTime=%f\n", accTime, totalTime);

		return -1;
	}
	if (  ( pos > SLIDE_LENGTH ) | ( pos < -1.0 * SLIDE_LENGTH ) ){
		printf("Err: serialClass::moveAbsoluteTime(). Pos Larger Than Slide Length\n");
		return -1;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	sprintf(output,"@%d %d %d %d %d 0 0\r",ID,MAT,distToCounts(ID,pos),timeToTicks(accTime),timeToTicks(totalTime));
	return writeAndCheckACK(ID, output);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   	
int serialClass::moveRelativeVel(int ID, double dist, double acc, double vel){
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
	
	sprintf(output,"@%d %d %d %d %d 0 0\r",ID,MRV,distToCounts(ID,dist),accToSAU(ID,acc),velToSVU(ID,vel));
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::clearPoll( int ID, int whichBit){//, bool debugOn ){
	//bit 0..15
	//~~~~~~~~~~~~~~~~~~~~~~~~~`
	if (  ( whichBit > 15 ) | ( whichBit < 0 ) ){
		printf("Err: serialClass::clearPoll().  Invalid Bit Request\n");
		return -1;
	}
	
	sprintf(output,"@%d %d %d\r",ID,CPL,1 << whichBit);
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::poll(int ID, char* strToReturn){
	sprintf(output,"@%d %d\r",ID,POR);
	writeMotSerPort(output);
		
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	
	if ( replyData[0] == '#') {
			printf("DATA POL\n");
			strcpy(strToReturn, replyData);
			return 0;
	}else{
		printf("Err: bad POR response\n");
		printf("got:%s\n", replyData);

		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		strcpy(strToReturn, replyData);
		return -1;
	}
	
	return -1;		//should never be here
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int serialClass::returnPosition( int ID, double* pos ){
	int posCount;
	int status = readRegister(ID,1,&posCount);
	
	*pos = (double) posCount / (double) ticksPerMM;
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int serialClass::printPSW(int ID){
	sprintf(output,"@%d %d\r",ID,POR);
	writeMotSerPort(output);
		
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	
	int psw[1];
	readData(ID,replyData,1,psw);
	for( int i = 15; i >= 0; i-- ){
		if (psw[0] & (1 << i)) {
			printPSWmessage( i );
		}
	}
}

int serialClass::printIO(int ID){
	sprintf(output,"@%d %d\r",ID,RIO);
	writeMotSerPort(output);
		
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	
	int iow[1];
	readData(ID,replyData,1,iow);
	for( int i = 15; i >= 0; i-- ){
		if (iow[0] & (1 << i)) {
			printIOmessage( i );
		}
	}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::readIO(int ID, char* strToReturn){
	sprintf(output,"@%d %d\r",ID,RIO);
	writeMotSerPort(output);

	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	
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
	sprintf(output,"@%d %d\r",ID,DLC);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::initSingleLoop(int ID){
	sprintf(output,"@%d %d\r",ID,SLC);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::setupEncoder(int ID){
	sprintf(output,"@%d %d %d %d %d\r",ID,SEE,0,0,2); //use external encoder on I/O #4-6, step+dir style
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::halt(int ID){
	sprintf(output,"@%d %d\r",ID,HLT);
	return writeAndCheckACK(ID,output);
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
int serialClass::killMotorConditions(int ID, int enableWord, int stateWord){
	sprintf(output,"@%d %d %d %d\r",ID,KMC,enableWord,stateWord);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::setKillMotorConditions(){
	//set KMC defaults for all axes
	int enableWord = 0;
	int stateWord = 0;

	//kill motor on I/O #1 or #2 high
	enableWord |= 1 << 4;
	stateWord |= 1 << 4;
	enableWord |= 1 << 5;
	stateWord |= 1 << 5;

	int status;
	if ( X_MOTOR_ACTIVE ) status = killMotorConditions(X_AXIS_ID, enableWord, stateWord);
		if ( status == -1 ) return -1;
	if ( Y_MOTOR_ACTIVE ) status = killMotorConditions(Y_AXIS_ID, enableWord, stateWord);
		if ( status == -1 ) return -1;
	if ( Z_MOTOR_ACTIVE ) status = killMotorConditions(Z_AXIS_ID, 0, 0);
		if ( status == -1 ) return -1;
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::writeInitProgram(int ID){
	sprintf(output,"@%d %d\r",ID,CLP);
	writeAndCheckACK(ID, output);

	sprintf(output,"@%d %d\r",ID,SDL);
	writeAndCheckACK(ID, output);

	//set ID
	sprintf(output,"@%d %d %d\r",ID,IDT,ID);
	writeAndCheckACK(ID, output);

	//set protocol to 8-bit, no parity, 2 stop bits
	sprintf(output,"@%d %d %d\r",ID,PRO,1);
	writeAndCheckACK(ID, output);

	//use RS-232
	sprintf(output,"@%d %d %d\r",ID,SIF,0);
	writeAndCheckACK(ID, output);

	//57600 baud
	sprintf(output,"@%d %d %d\r",ID,BRT,576);
	writeAndCheckACK(ID, output);

	//set ACK delay
	sprintf(output,"@%d %d %d\r",ID,ADL,10);
	writeAndCheckACK(ID, output);

	//low voltage trip at 10 V
	sprintf(output,"@%d %d %d\r",ID,LVT,10);
	writeAndCheckACK(ID, output);

	//high voltage trip at 52 V
	sprintf(output,"@%d %d %d\r",ID,OVT,52);
	writeAndCheckACK(ID, output);

	//read analog input cal
	sprintf(output,"@%d %d %d\r",ID,CAI,65524);
	writeAndCheckACK(ID, output);

	//torque limits to 0
	sprintf(output,"@%d %d %d %d %d %d\r",ID,TQL,0,0,0,0);
	writeAndCheckACK(ID, output);

	//go open-loop to set up motors
	sprintf(output,"@%d %d\r",ID,GOL);
	writeAndCheckACK(ID, output);

	//single loop control
	sprintf(output,"@%d %d\r",ID,SLC);
	writeAndCheckACK(ID, output);

	//set motor constants
//MCT for XY: 7057 20370 31445 475 379 739 14261 1902
//MCT for Z: 23341 19183 29296 144 68 210 4312 575
//PAC for XY: 10 270 70
//PAC for Z: 10 180 74
//FLC for XY: 29264 27139 24236 (150, 250, 400 Hz)
//FLC for Z: 24000 28000 30000 (413, 209, 117 Hz)
//CTC for XY: 100 0 6 6 20 20 500
//CTC for Z: 25 0 4 4 5 5 1000
	switch (ID){
		case X_AXIS_ID:
			sprintf(output,"@%d %d %d %d %d %d %d %d %d %d\r",ID,MCT,7005,20218,30973,479,348,739,14368,1916);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d\r",ID,PAC,10,270,70);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d\r",ID,FLC,29264,27139,24236);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d %d %d %d %d\r",ID,CTC,0,6,6,20,20,100,500);
			writeAndCheckACK(ID, output);
			break;
		case Y_AXIS_ID:
			sprintf(output,"@%d %d %d %d %d %d %d %d %d %d\r",ID,MCT,7057,20370,31445,475,379,739,14261,1902);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d\r",ID,PAC,10,270,70);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d\r",ID,FLC,29264,27139,24236);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d %d %d %d %d\r",ID,CTC,0,6,6,20,20,100,500);
			writeAndCheckACK(ID, output);
			break;
		case Z_AXIS_ID:
			sprintf(output,"@%d %d %d %d %d %d %d %d %d %d\r",ID,MCT,22747,18695,27789,148,35,210,4425,590);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d\r",ID,PAC,10,180,74);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d\r",ID,FLC,24000,28000,30000);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d %d %d %d %d\r",ID,CTC,0,4,4,5,5,25,1000);
			writeAndCheckACK(ID, output);
			break;
		default:
			printf("Err: invalid ID");
			return -1;
	}

	//no gravity offset
	sprintf(output,"@%d %d %d\r",ID,GOC,0);
	writeAndCheckACK(ID, output);

	//clockwise orientation
	sprintf(output,"@%d %d %d\r",ID,DIR,0);
	writeAndCheckACK(ID, output);

	//open loop phase
	sprintf(output,"@%d %d %d\r",ID,OLP,0);
	writeAndCheckACK(ID, output);

	//ramp up torque
	sprintf(output,"@%d %d %d %d\r",ID,TRU,20000,25);
	writeAndCheckACK(ID, output);

	//wiggle the motor
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRT,20,timeToTicks(0.03),timeToTicks(0.1),0,0);
	writeAndCheckACK(ID, output);
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRT,-40,timeToTicks(0.03),timeToTicks(0.1),0,0);
	writeAndCheckACK(ID, output);
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRT,20,timeToTicks(0.03),timeToTicks(0.1),0,0);
	writeAndCheckACK(ID, output);

	//wait
	sprintf(output,"@%d %d %d\r",ID,DLT,timeToTicks(0.2));
	writeAndCheckACK(ID, output);

	//go closed-loop
	sprintf(output,"@%d %d\r",ID,GCL);
	writeAndCheckACK(ID, output);

	//set default torque limits
	sprintf(output,"@%d %d %d %d %d %d\r",ID,TQL,15000,20000,6000,60000);
	writeAndCheckACK(ID, output);

	//set anti-hunt threshold
	sprintf(output,"@%d %d %d %d\r",ID,AHC,20,8);
	writeAndCheckACK(ID, output);

	//set anti-hunt delay
	sprintf(output,"@%d %d %d\r",ID,AHD,250);
	writeAndCheckACK(ID, output);

	//no S-curve
	sprintf(output,"@%d %d %d\r",ID,SCF,0);
	writeAndCheckACK(ID, output);

	//default error limits
	sprintf(output,"@%d %d %d %d %d\r",ID,ERL,20000,20000,timeToTicks(0.120));
	writeAndCheckACK(ID, output);

	//motor should stop on a kill condition
	sprintf(output,"@%d %d\r",ID,KDD);
	writeAndCheckACK(ID, output);

//KMR
//KMC temp, moving, holding, overvoltage
//PLR
	//filter digital inputs
	sprintf(output,"@%d %d %d %d\r",ID,DIF,0,timeToTicks(0.010));
	writeAndCheckACK(ID, output);

	//make sure done bit is off
	sprintf(output,"@%d %d\r",ID,DDB);
	writeAndCheckACK(ID, output);
//DDB
//MDC ?

/*
	//disable motor
	sprintf(output,"@%d %d\r",ID,DMD);
	writeAndCheckACK(ID, output);
*/

/*
	//stop move (set target to position)
	sprintf(output,"@%d %d\r",ID,146);
	writeAndCheckACK(ID, output);
	printf("TTP written:%s\n", output);

	//clear status
	sprintf(output,"@%d %d\r",ID,163);
	writeAndCheckACK(ID, output);
	printf("CIS written:%s\n", output);

	//enable motor
	sprintf(output,"@%d %d\r",ID,227);
	writeAndCheckACK(ID, output);
	printf("EMD written:%s\n", output);

	switch (ID){
		case X_AXIS_ID:
		case Y_AXIS_ID:
			status = s.writeInitProgramXY(ID);
			break;
		case Z_AXIS_ID:
			status = s.writeInitProgramZ(ID);
			break;
		default:
			printf("Err: invalid ID");
			return -1;
	}
*/

	//store program to NV memory as initialization
	sprintf(output,"@%d %d %d\r",ID,13,0);
	writeAndCheckACK(ID, output);
	printf("SPR written:%s\n", output);

	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::writeHomeProgramXY(int ID){
	sprintf(output,"@%d %d\r",ID,CLP);
	writeAndCheckACK(ID, output);

	sprintf(output,"@%d %d\r",ID,SDL);
	writeAndCheckACK(ID, output);

	//move 100 mm towards the middle
	int enableWord = 0;
	int stateWord = 0;
	enableWord |= 1 << 12; //if I/O #1 (positive limit) is true, reverse direction
	stateWord |= 1 << 12;
	enableWord |= 1 << 6; //record position when I/O #3 (home) is true - not actually used
	//stateWord |= 1 << 6;
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRT,distToCounts(ID,100.0),timeToTicks(0.1),timeToTicks(3.0),enableWord,stateWord);
	writeAndCheckACK(ID, output);

	//move -500 mm, stop when I/O #3 (home) is true
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRV,distToCounts(ID,-500.0),accToSAU(ID,100.0),velToSVU(ID,10.0),-3,0);
	writeAndCheckACK(ID, output);

	//move to point where I/O #3 triggered
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,RAV,4,accToSAU(ID,10.0),velToSVU(ID,1.0),0,0);
	writeAndCheckACK(ID, output);

	//this is home; zero target and position
	sprintf(output,"@%d %d\r",ID,ZTP);
	writeAndCheckACK(ID, output);

/*
	//rerun home program on a kill motor condition
	sprintf(output,"@%d %d 512\r",ID,KMR);
	writeAndCheckACK(ID, output);

	//set kill conditions - kill motor on I/O #1 or #2 high
	enableWord = 0;
	stateWord = 0;
	enableWord |= 1 << 4;
	stateWord |= 1 << 4;
	enableWord |= 1 << 5;
	stateWord |= 1 << 5;
	sprintf(output,"@%d %d %d %d\r",ID,KMC,enableWord,stateWord);
	writeAndCheckACK(ID, output);
*/

	//store program to NV memory, address 512
	sprintf(output,"@%d %d %d\r",ID,SPR,512);
	writeAndCheckACK(ID, output);
	printf("SPR written:%s\n", output);

	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::writeHomeProgramZ(int ID){
	sprintf(output,"@%d %d\r",ID,CLP);
	writeAndCheckACK(ID, output);
	printf("CLP written:%s\n", output);

	sprintf(output,"@%d %d\r",ID,SDL);
	writeAndCheckACK(ID, output);
	printf("SDL written:%s\n", output);

	//clear status
	sprintf(output,"@%d %d\r",ID,CIS);
	writeAndCheckACK(ID, output);

	//turn off kill motor conditions
	sprintf(output,"@%d %d %d %d\r",ID,KMC,0,0);
	writeAndCheckACK(ID, output);

	//set error limits low
	sprintf(output,"@%d %d %d %d %d\r",ID,ERL,50,50,500);
	writeAndCheckACK(ID, output);

	//set low torque limits
	sprintf(output,"@%d %d %d %d %d %d\r",ID,TQL,20000,10000,6000,10000);
	writeAndCheckACK(ID, output);

	//move slowly -100 mm, stop on moving error
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRV,distToCounts(ID,-100.0),accToSAU(ID,200.0),velToSVU(ID,2.0),-11,1);
	writeAndCheckACK(ID, output);

	//stop move (set target to position)
	sprintf(output,"@%d %d\r",ID,TTP);
	writeAndCheckACK(ID, output);

	//clear status
	sprintf(output,"@%d %d\r",ID,CIS);
	writeAndCheckACK(ID, output);

	//move down 10 mm
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRV,distToCounts(ID,1.0),accToSAU(ID,200.0),velToSVU(ID,10.0),0,0);
	writeAndCheckACK(ID, output);

	//this is home; zero target and position
	sprintf(output,"@%d %d\r",ID,ZTP);
	writeAndCheckACK(ID, output);

	//set normal torque limits
	sprintf(output,"@%d %d %d %d %d %d\r",ID,TQL,20000,20000,6000,30000);
	writeAndCheckACK(ID, output);

	//rerun home program on a kill motor condition
	sprintf(output,"@%d %d 512\r",ID,KMR);
	writeAndCheckACK(ID, output);

	//set normal kill conditions - kill motor on moving or holding error
	int enableWord = 0;
	int stateWord = 0;
	enableWord |= 1 << 8;
	stateWord |= 1 << 8;
	enableWord |= 1 << 9;
	stateWord |= 1 << 9;
	sprintf(output,"@%d %d %d %d\r",ID,KMC,enableWord,stateWord);
	writeAndCheckACK(ID, output);

	//store program to NV memory, address 512
	sprintf(output,"@%d %d %d\r",ID,SPR,512);
	writeAndCheckACK(ID, output);

	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::runHomeProgram(int ID){
	sprintf(output,"@%d %d %d\r",ID,LRP,512); //run program at NV address 0
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::runInitProgram(int ID){
	sprintf(output,"@%d %d 0\r",ID,LRP); //run program at NV address 0
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::stop(int ID){
	sprintf(output,"@%d %d -1\r",ID,STP);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::enableMotor(int ID){
	sprintf(output,"@%d %d\r",ID,EMD);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::resetMotor(int ID){
	sprintf(output,"@%d %d\r",ID,RST);
	
	writeMotSerPort(output);
	
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
int serialClass::getMotReply(char *result){
	//usleep(10000);		//N ms
	char sResult[MAX_REPLY_LEN];
	//fcntl(fd, F_SETFL, FNDELAY); // don't block serial read

	if (!readport_blocking(fd,sResult,"\r")) {
		printf("ERR: getReply() read failed\n");
		close(fd);
		return -1;
	}		
	
	int numChars = strlen(sResult);
	
	/*now check through for corrupted data.  There should never be anything greater than 102 (=f), since all
	symbols are below the numbers*/
	
	for(int i = 0; i < numChars; i++){
		if ( sResult[ i ] > 102 ){
			printf("ERR: getReply() illegal character %d at position %d\n",sResult[i],i);
			printf("readport=%s\n", sResult);
			flush();			//flush the port of illegal characters
			return -1;
		}
	}
	
	if (strchr(sResult,'\r') - sResult != numChars-1)
		printf("getMotReply: did not find \\r as last character of reply");

	//printf("numChars received=%d\n", numChars);	strcpy(result, sResult);
	
	strcpy(result, sResult);
	//result[ numChars - 1] = 13;		//add the CR
	//result[ numChars ] = 0;		//add null
	
	//result = "hello";
	//printf("readport=%s\n", result);
	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::getLasReply(char *result){
	//usleep(100000);		//N ms
	char sResult[MAX_REPLY_LEN];
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
			flush();			//flush the port of illegal characters
			return -1;
		}
	}
	
	//printf("numChars received=%d\n", numChars);	strcpy(result, sResult);
	
	strcpy(result, sResult);
	//result[ numChars - 1] = 13;		//add the CR
	//result[ numChars ] = 0;		//add null
	
	//result = "hello";
	//printf("readport=%s\n", result);
	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    
serialClass::serialClass(){ }
serialClass::~serialClass ( ){ }


const char * serialClass::cmdName(int cmdID) {
	switch (cmdID) {
		case 0: 	return "POL";
		case 1: 	return "CPL";
		case 163: 	return "CIS";
		case 27: 	return "POR";
		case 21: 	return "RIO";
		case 177: 	return "MRT";
		case 176: 	return "MAT";
		case 135: 	return "MRV";
		case 160: 	return "RAV";
		case 243: 	return "DLC";
		case 244: 	return "SLC";
		case 192: 	return "SEE";
		case 2: 	return "HLT";
		case 3: 	return "STP";
		case 4: 	return "RST";
		case 145: 	return "ZTP";
		case 167: 	return "KMC";
		case 8: 	return "CLP";
		case 9: 	return "SDL";
		case 13: 	return "SPR";
		case 156: 	return "LRP";
		case 155: 	return "IDT";
		case 173: 	return "ADL";
		case 185: 	return "PRO";
		case 186: 	return "SIF";
		case 174: 	return "BRT";
		case 212: 	return "LVT";
		case 213: 	return "OVT";
		case 149: 	return "TQL";
		case 143: 	return "GOL";
		case 211: 	return "CAI";
		case 172: 	return "PAC";
		case 168: 	return "MCT";
		case 169: 	return "FLC";
		case 148: 	return "CTC";
		case 237: 	return "GOC";
		case 184: 	return "DIR";
		case 152: 	return "OLP";
		case 222: 	return "TRU";
		case 140: 	return "DLT";
		case 142: 	return "GCL";
		case 150: 	return "AHC";
		case 230: 	return "AHD";
		case 195: 	return "SCF";
		case 151: 	return "ERL";
		case 183: 	return "KDD";
		case 252: 	return "DIF";
		case 171: 	return "DDB";
		case 227: 	return "EMD";
		case 228: 	return "DMD";
		default: 	return "unknown command";
	}
}

int serialClass::writeMotSerPort(char *output){
	int ID,cmdID;
	sscanf(output,"@%d %d",&ID,&cmdID);
	int status = writeport(fd,output);
	if (status) {
		printf("wrote %s to ID %d: %s\n",cmdName(cmdID),ID,output);
	}
	else {
		printf("failed writing %s to ID %d: %s\n",cmdName(cmdID),ID,output);
	}
	//sleep(0.1);
	return status;
}

int serialClass::writeLas(char *output){
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

int serialClass::writeAndCheckACK( int ID, char* output ){
	char replyData[MAX_REPLY_LEN];
	writeMotSerPort(output);
	getMotReply( replyData);	
	return checkForACK( ID, replyData );
}

int serialClass::checkForACK( int ID, char* inputArray ){
	if (  inputArray[ 0 ] != '*' ){
		printf("ERR in ACK seeing * as first character. ReplyString=%s\n",inputArray);
		return -1;
	}
	int readID;
	sscanf(inputArray,"* %x\r",&readID);
	
	if ( ID != readID ){
		printf("ERR in seeing ID number in ACK. ReplyString=%s\n",inputArray);
		return -1;
	}
		
	printf("ACK received. ReplyString=%s\n", inputArray);
	return 0;
}

int serialClass::checkLasACK( char* outputArray, char* inputArray ){
	char * inputPtr = inputArray;

	//for (int i=0;i<strlen(outputArray);i++) printf("%d\t%d\n",inputPtr[i],outputArray[i]);
	if ( strncmp(inputPtr,outputArray,strlen(outputArray)) != 0 ){
		printf("No echo\n");
		//return -1;
	} else {
		inputPtr+=strlen(outputArray);
	}

		
	char * ack="\r\n\r\nReady for command > ";
	//for (int i=0;i<strlen(inputPtr);i++) printf("%d\t%d\n",inputPtr[i],ack[i]);
	if ( strncmp(inputPtr,ack,strlen(ack)) != 0 ){
		printf("Not an ACK: ReplyString=%s\n",inputArray);
		return -1;
	}
	printf("ACK received to command: %s\n", outputArray);
	return 0;
}

int serialClass::readData( int ID, char* inputArray, int nData, int* dataArray ){
	char inputBuf[254];
	strcpy(inputBuf,inputArray);
	char * tok = strtok(inputBuf," ");
	if (strcmp(tok,"#") != 0){
		printf("ERR in DATA seeing # as first character. ReplyString=%s\n",inputArray);
		return -1;
	}
	tok = strtok(NULL," ");
	int readID;
	sscanf(tok,"%x",&readID);
	
	if ( ID != readID ){
		printf("ERR in seeing ID number in DATA. ReplyString=%s\n",inputArray);
		return -1;
	}

	tok = strtok(NULL," ");
	int readCmdID;
	sscanf(tok,"%x",&readCmdID);
	printf("response packet for command %s\n",cmdName(readCmdID));
		
	tok = strtok(NULL," ");
	for (int i=0;i<nData;i++) {
		sscanf(tok,"%x",&dataArray[i]);
		tok = strtok(NULL," ");
	}
	//printf("ACK received. ReplyString=%s\n", inputArray);
	return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::initMotSerPort() {
	#define BAUD 57600;                      // derived baud rate from command line

	fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
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
	options.c_cflag |= CSTOPB; //two stop bits
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	
	options.c_cc[VMIN]=0;
        options.c_cc[VTIME]=10;

	options.c_lflag &= ~(ICANON | ECHO | ECHOE);

	// Set the new options for the port...
	tcsetattr(fd, TCSANOW, &options);
		
   //printf("baud=%d\n", getbaud(fd));
	
	flush();

	return 1;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::changeAntiHunt(int ID) {
	sprintf(output,"@%d %d %d %d\r",ID,AHC,4,6);
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::goClosedLoop(int ID) {
	sprintf(output,"@%d %d\r",ID,GCL);
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::changeACKdelay(int ID, double delay) {
	sprintf(output,"@%d %d %d\r",ID,ADL,timeToTicks(delay/1000.0));
	return writeAndCheckACK(ID, output);
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
	//takes in a reply from RIO and takes out the last 4 hex digits.
	//these 4 hex characters ( ) represent the I/O status word IOS, are are then converted to bits.
	//then it displays all error messages.
	int cmdID;
	int responseID;
	int ios;
	sscanf(inputArray,"# %x %x %x\r",&responseID,&cmdID,&ios);
	printf("cmdID 0x%4x, IOS 0x%4x\n",cmdID,ios);
			
	for( int i = 15; i >= 0; i-- ){
		if (ios & (1 << i)) {
			printIOmessage( i );
		}
	}
	return 0;
} //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int serialClass::readRegister(int ID, int regID, int *data){
	sprintf(output,"@%d %d %d\r",ID,RRG,regID);
	writeMotSerPort(output);

	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	
	int nData;
	
//	readData(ID,replyData,
	if ( replyData[0] == '#') {
		printf("DATA RRG\n");
		int cmdID;
		int responseID;
		int data1, data2;
		sscanf(replyData,"# %x %x %x %x\r",&responseID,&cmdID,&data1,&data2);
		*data = ((data1 << 16) | data2);
		return 0;
	}else{
		printf("Err: bad RRG response\n");
		printf("got:%s\n", replyData);
		return -1;
	}
} //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::displayPSWdescriptions(char* inputArray) {
	//takes in a reply from POL/POR and takes out the last 4 hex digits.
	//these 4 hex characters ( ) represent the polling status word PSW, are are then converted to bits.
	//then it displays all error messages.
	int cmdID;
	int responseID;
	int psw;
	sscanf(inputArray,"# %x %x %x\r",&responseID,&cmdID,&psw);
	printf("cmdID 0x%4x, PSW 0x%4x\n",cmdID,psw);
			
	for( int i = 15; i >= 0; i-- ){
		if (psw & (1 << i)) {
			printPSWmessage( i );
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::clearAllPSWbits(int ID){
	sprintf(output,"@%d %d %d\r",ID,CPL,65535);
	return writeAndCheckACK(ID, output);
}

void serialClass::printIOmessage(int index){
	switch (index){
			case 0:
				printf("  IO.0: Index Found\n");
				break;
			case 1:						
				printf("  IO.1: Internal Index Found\n");
				break;
			case 2:
				printf("  IO.2: External Index Found\n");
				break;		
			case 3:				
				printf("  IO.3: Trajectory Generator Active\n");
				break;
			case 4:				
				printf("  IO.4: I/O #1\n");
				break;
			case 5:					
				printf("  IO.5: I/O #2\n");
				break;
			case 6:				
				printf("  IO.6: I/O #3\n");
				break;
			case 7:				
				printf("  IO.7: Temperature Ok\n");
				break;
			case 8:				
				printf("  IO.8: Moving Error\n");
				break;
			case 9:				
				printf("  IO.9: Holding Error\n");
				break;			
			case 10:				
				printf("  IO.10: Delay Counter\n");
				break;			
			case 11:				
				printf("  IO.11: Reserved\n");
				break;			
			case 12:				
				printf("  IO.12: I/O #4\n");
				break;			
			case 13:				
				printf("  IO.13: I/O #5\n");
				break;			
			case 14:
				printf("  IO.14: I/O #6\n");
				break;		
			case 15:							
				printf("  IO.15: I/O #7\n");
				break;
			
			default:
				printf("Err:serialClass::printIOmessage()\n");
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
	sprintf(output,"@%d %d\r",ID,ZTP);
	return writeAndCheckACK(ID, output);
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
	//~~~~~~~~~~~~~~~~~~~
	
	sprintf(output,"@%d %d %d %d\r",ID,cmdID,whichBit,state);
			
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
	getMotReply( replyData);	
	return checkForACK( ID, replyData );
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
	sprintf(output,"@%d %d\r",ID,CIS);
	writeMotSerPort(output);

        sleep(0.5);

        //~~~~~~~~ reply ~~~~~~~~~
        char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	
	return checkForACK( ID, replyData );
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
			
	//sprintf(intArr, "%d", width);
	int number = (int) width;
	
	sprintf(output,"puls:widt %dns\r",number);
	
	writeLas(output);
	printf("width written:%s\n", output);

	char replyData[254];
	int status;
	status = getLasReply(replyData);
	status = checkLasACK(output,replyData);
	
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::setLaserAmp_mV(double amp){
	//width in nanoseconds
	//amp in mV
	
	
	if ( (amp < 0.0) | (amp > 10000) ){
		printf("Pulse Amp Out of Range\n");
		return -1;
	}
			
	int number = (int) amp;
	
	sprintf(output,"voltage %dmV\r",number);
	writeLas(output);
	printf("Amp written:%s\n", output);
	
	char replyData[254];
	int status;
	status = getLasReply(replyData);
	status = checkLasACK(output,replyData);
	
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::sendLaserTrigger(void){
	
	strcpy(output,"trig:sour IMM\r");
	writeLas(output);
	printf("laser trigger written:%s\n", output);

	char replyData[254];
	int status;
	status = getLasReply(replyData);
	status = checkLasACK(output,replyData);
	
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::turnOnLaserEcho( void ){
		
	strcpy(output,"syst:comm:serial:echo on\r");
	
	writeLas(output);
	printf("laser echo written:%s\n", output);
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int status;
	
	status = getLasReply(replyData);		//laser port Ld.
	status = checkLasACK(output,replyData);
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::disableLaserFlowControl(void){
	
	strcpy(output,"syst:comm:serial:control:rts on\r");
	
	writeLas(output);
	printf("laser flowControl written:%s\n", output);
	
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[254];
	int status;
	
	status = getLasReply(replyData);		//laser port Ld.
	status = checkLasACK(output,replyData);
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::requestLaserID(void){
	
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
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::initLaserSerPort() {

	
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


	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::laserRemoteEnable( void ){
	//width in nanoseconds
	
	strcpy(output,"remote\r");
	
	writeLas(output);
	printf("remote written:%s\n", output);

	//sleep (0.1);
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
		//printf( "\n" );
		//status = displayPSWdescriptions( replyData );
		return -1;
	}
	
	return -1;		//should never be here
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::laserEnableOutput( void ){
	char replyData[254];
	int status;
//	status = getLasReply(replyData);
//	sleep (0.1);
	strcpy(output,"output on\r");
	writeLas(output);
	
	//sleep (0.1);

	//~~~~~~~~ reply ~~~~~~~~~
	
	status = getLasReply(replyData);
	status = checkLasACK(output,replyData);
	
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::laserDisableOutput( void ){
	char replyData[254];
	int status;
	strcpy(output,"output off\r");
		
	writeLas(output);
	printf("output off written:%s\n", output);
	
	//sleep (0.1);
	status = getLasReply(replyData);
	status = checkLasACK(output,replyData);
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int serialClass::laserLocalEnable( void ){
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
