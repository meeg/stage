/* 
Paul Csonka August 2010
Serial libraries for the stepper motor controllers and laser pulser
*/

#include "motion.h"
#define MAX_REPLY_LEN 254

//packet format:
// " @, ID (EACH DIGIT in ASCII), command number (EACH DIGIT in ASCII), parameters, <CR> (ASCII 13), NULL "

using namespace std;

//=================================================================================
//=================================================================================

bool motionClass::useExternalEnc(int ID) {
	switch (ID){
		case X_AXIS_ID:
			return X_USE_EXTERNAL_ENC;
		case Y_AXIS_ID:
			return Y_USE_EXTERNAL_ENC;
		case Z_AXIS_ID:
			return false;
		default:
			printf("Err: invalid ID");
			return false; //dummy value
	}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::timeToTicks(double time){
	return (int) (time/120e-6); //120 us/tick
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double motionClass::screwPitch(int ID){
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
double motionClass::internalEncoderResolution(int ID){
	return screwPitch(ID)/8000.0; //8000 counts per revolution = 0.000635 for XY
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double motionClass::distPerCount(int ID){
	if (useExternalEnc(ID)) return externalEncoderResolution;
	else return screwPitch(ID)/8000.0; //8000 counts per revolution = 0.000635 for XY
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::distToCounts(int ID, double dist){
	return (int) (dist/distPerCount(ID));
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::velToSVU(int ID, double vel){
	double maxVel = screwPitch(ID)*4000.0/60.0; //4000 RPM
	int velInt = (int) (pow(2,31.0)*vel/maxVel);
	//printf("vel = %f maxVel=%f velInt = %d\n",vel,maxVel,velInt);
	return velInt;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::accToSAU(int ID, double acc){
	double maxAcc = screwPitch(ID)*4000.0/60.0/2.0/120e-6; //maxVel/2/tick
	int accInt = (int) (pow(2,30.0)*acc/maxAcc);
	//printf("acc = %f maxAcc=%f accInt = %d\n",acc,maxAcc,accInt);
	return accInt;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::moveRelativeTime(int ID, double dist, double rampTime, double totalTime){
	//~~~~~~~~~~ ERROR CHECK ~~~~~
	if ( ( rampTime <= 0.0 ) | ( totalTime <= 0.0 ) ){
		printf("Err: motionClass::moveRelativeTime().  Time parameters either zero or negative\n");
		printf("rampTime=%f, totalTime=%f\n", rampTime, totalTime);
		return -1;
	}
	if ( 2 * rampTime > totalTime ){
		printf("Err: motionClass::moveRelativeTime().  Ramp Time Too Large Compared to Total Time\n");
		return -1;
	}
	if (  ( dist > SLIDE_LENGTH ) | ( dist < -1.0*SLIDE_LENGTH ) ){
		printf("Err: motionClass::moveRelativeTime(). Dist Larger Than Slide Length\n");
		return -1;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	sprintf(output,"@%d %d %d %d %d 0 0\r",ID,MRT,distToCounts(ID,dist),timeToTicks(rampTime),timeToTicks(totalTime));
	return writeAndCheckACK(ID, output);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::moveAbsoluteTime(int ID, double pos, double accTime, double totalTime){
	//~~~~~~~~~~ ERROR CHECK ~~~~~
	/*if ( 2 * rampTime > totalTime ){
		printf("Err: motionClass::moveAbsoluteTime().  Ramp Time Too Large Compared to Total Time\n");
		return -1;
	}*/
	if ( ( accTime <= 0.0 ) | ( totalTime <= 0.0 ) ){
		printf("Err: motionClass::moveAbsoluteTime().  Acc or time parameters either zero or negative\n");
		printf("accTime=%f, totalTime=%f\n", accTime, totalTime);

		return -1;
	}
	if (  ( pos > SLIDE_LENGTH ) | ( pos < -1.0 * SLIDE_LENGTH ) ){
		printf("Err: motionClass::moveAbsoluteTime(). Pos Larger Than Slide Length\n");
		return -1;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	sprintf(output,"@%d %d %d %d %d 0 0\r",ID,MAT,distToCounts(ID,pos),timeToTicks(accTime),timeToTicks(totalTime));
	return writeAndCheckACK(ID, output);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   	
int motionClass::moveRelativeVel(int ID, double dist, double acc, double vel){
	//~~~~~~~~~~ ERROR CHECK ~~~~~
	/*if ( ){
		printf("Err: motionClass::moveRelativeVel().  Ramp Time Too Large Compared to Total Time\n");
		return -1;
	}*/
	if ( ( acc <= 0.0 ) | ( vel <= 0.0 ) ){
		printf("Err: motionClass::moveRelativeVel().  Acc or vel parameters either zero or negative\n");
		printf("acc=%f, vel=%f\n", acc, vel);

		return -1;
	}
	if ( ( dist > SLIDE_LENGTH ) | ( dist < -1.0 * SLIDE_LENGTH ) ){
		printf("Err: motionClass::moveRelativeVel(). Dist Larger Than Slide Length\n");
		return -1;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	sprintf(output,"@%d %d %d %d %d 0 0\r",ID,MRV,distToCounts(ID,dist),accToSAU(ID,acc),velToSVU(ID,vel));
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   	
int motionClass::moveAbsoluteVel(int ID, double pos, double acc, double vel){
	//~~~~~~~~~~ ERROR CHECK ~~~~~
	/*if ( ){
		printf("Err: motionClass::moveRelativeVel().  Ramp Time Too Large Compared to Total Time\n");
		return -1;
	}*/
	if ( ( acc <= 0.0 ) | ( vel <= 0.0 ) ){
		printf("Err: motionClass::moveAbsoluteVel().  Acc or vel parameters either zero or negative\n");
		printf("acc=%f, vel=%f\n", acc, vel);

		return -1;
	}
	if ( ( pos > SLIDE_LENGTH ) | ( pos < -1.0 * SLIDE_LENGTH ) ){
		printf("Err: motionClass::moveAbsoluteVel(). Pos Larger Than Slide Length\n");
		return -1;
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	sprintf(output,"@%d %d %d %d %d 0 0\r",ID,MAV,distToCounts(ID,pos),accToSAU(ID,acc),velToSVU(ID,vel));
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::clearPoll( int ID, int whichBit){//, bool debugOn ){
	//bit 0..15
	//~~~~~~~~~~~~~~~~~~~~~~~~~`
	if (  ( whichBit > 15 ) | ( whichBit < 0 ) ){
		printf("Err: motionClass::clearPoll().  Invalid Bit Request\n");
		return -1;
	}
	
	sprintf(output,"@%d %d %d\r",ID,CPL,1 << whichBit);
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::poll(int ID, int* intToReturn){
	sprintf(output,"@%d %d\r",ID,POR);
	writeMotSerPort(output);

	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);
	return readData(ID,replyData,1,intToReturn);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::returnPosition( int ID, double* pos ){
	int status;
	if (useExternalEnc(ID)) status =  returnExternalPosition(ID,pos);
	else status = returnInternalPosition(ID,pos);
	*pos *= -1.0;
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::returnInternalPosition( int ID, double* pos ){
	int posCount;
	int status = readRegister(ID,1,&posCount);
	*pos = distPerCount(ID) * posCount;
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::returnExternalPosition( int ID, double* pos ){
	int posCount;
	int status = readRegister(ID,200,&posCount);
	
	*pos = (double) posCount*externalEncoderResolution;
	//*pos = distPerCount(ID) * posCount;
	return status;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int motionClass::printPSW(int ID){
	sprintf(output,"@%d %d\r",ID,POR);
	writeMotSerPort(output);
		
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	
	int psw[1];
	int status = readData(ID,replyData,1,psw);
	for( int i = 15; i >= 0; i-- ){
		if (psw[0] & (1 << i)) {
			printPSWmessage( i );
		}
	}
	return status;
}

int motionClass::printIO(int ID){
	sprintf(output,"@%d %d\r",ID,RIO);
	writeMotSerPort(output);
		
	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	
	int iow[1];
	int status = readData(ID,replyData,1,iow);
	for( int i = 15; i >= 0; i-- ){
		if (iow[0] & (1 << i)) {
			printIOmessage( i );
		}
	}
	return status;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::readIO(int ID, int* intToReturn){
	sprintf(output,"@%d %d\r",ID,RIO);
	writeMotSerPort(output);

	//~~~~~~~~ reply ~~~~~~~~~
	char replyData[MAX_REPLY_LEN];
	getMotReply(replyData);
	return readData(ID,replyData,1,intToReturn);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
int motionClass::initDualLoop(int ID){
	sprintf(output,"@%d %d\r",ID,DLC);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::initSingleLoop(int ID){
	sprintf(output,"@%d %d\r",ID,SLC);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::setupEncoder(int ID){
	sprintf(output,"@%d %d %d %d %d\r",ID,SEE,0,0,2); //use external encoder on I/O #4-6, step+dir style
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::halt(int ID){
	sprintf(output,"@%d %d\r",ID,HLT);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::killActiveMotors(void){
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
int motionClass::killMotorConditions(int ID, int enableWord, int stateWord){
	sprintf(output,"@%d %d %d %d\r",ID,KMC,enableWord,stateWord);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::setKillMotorConditions(){
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
int motionClass::writeHomeProgram(int ID){
	switch (ID){
		case X_AXIS_ID:
		case Y_AXIS_ID:
			return writeHomeProgramXY(ID);
		case Z_AXIS_ID:
			return writeHomeProgramZ(ID);
		default:
			printf("Err: invalid ID");
			return -1;
	}
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::writeInitProgram(int ID){
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
	//int kp = (int) (100.0*distPerCount(ID)/externalEncoderResolution);
	switch (ID){
		case X_AXIS_ID:
			sprintf(output,"@%d %d %d %d %d %d %d %d %d %d\r",ID,MCT,7005,20218,30973,479,348,739,14368,1916);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d\r",ID,PAC,10,270,70);
			writeAndCheckACK(ID, output);

			sprintf(output,"@%d %d %d %d %d\r",ID,FLC,29264,27139,24236);
			writeAndCheckACK(ID, output);

			if (useExternalEnc(ID))
				sprintf(output,"@%d %d %d %d %d %d %d %d %d\r",ID,CTC,0,6,6,20,20,20,500);
			else
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

			if (useExternalEnc(ID))
				sprintf(output,"@%d %d %d %d %d %d %d %d %d\r",ID,CTC,0,6,6,20,20,20,500);
			else
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

	//counterclockwise orientation
	sprintf(output,"@%d %d %d\r",ID,DIR,1);
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

	if (ID == X_AXIS_ID | ID == Y_AXIS_ID) {
		//use external encoder on I/O #4-6, step+dir style
		sprintf(output,"@%d %d %d %d %d\r",ID,SEE,0,0,0);
		writeAndCheckACK(ID,output);

		if (useExternalEnc(ID)) {
			//go dual-loop
			sprintf(output,"@%d %d\r",ID,DLC);
			writeAndCheckACK(ID,output);
		}
	}


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
int motionClass::writeHomeProgramXY(int ID){
	sprintf(output,"@%d %d\r",ID,CLP);
	writeAndCheckACK(ID, output);

	sprintf(output,"@%d %d\r",ID,SDL);
	writeAndCheckACK(ID, output);

	//move 50 mm towards the middle
	int enableWord = 0;
	int stateWord = 0;
	enableWord |= 1 << 12; //if I/O #1 (positive limit) is true, reverse direction
	stateWord |= 1 << 12;
	enableWord |= 1 << 6; //record position when I/O #3 (home) is true - not actually used
	//stateWord |= 1 << 6;
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRT,distToCounts(ID,-50.0),timeToTicks(0.1),timeToTicks(2.0),enableWord,stateWord);
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

	//move 500 mm, stop when I/O #3 (home) is true
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRV,distToCounts(ID,500.0),accToSAU(ID,100.0),velToSVU(ID,10.0),-3,0);
	writeAndCheckACK(ID, output);

	//move to point where I/O #3 triggered
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,RAV,4,accToSAU(ID,10.0),velToSVU(ID,1.0),0,0);
	writeAndCheckACK(ID, output);

	//wait
	sprintf(output,"@%d %d %d\r",ID,DLT,timeToTicks(0.2));
	writeAndCheckACK(ID, output);

	//this is home; zero target and position
	sprintf(output,"@%d %d\r",ID,ZTP);
	writeAndCheckACK(ID, output);

	//zero external encoder
	sprintf(output,"@%d %d %d %d\r",ID,WRP,200,0);
	writeAndCheckACK(ID, output);

	//kill motor on a kill motor condition
	sprintf(output,"@%d %d 0\r",ID,KMR);
	writeAndCheckACK(ID, output);

	//store program to NV memory, address 512
	sprintf(output,"@%d %d %d\r",ID,SPR,512);
	writeAndCheckACK(ID, output);
	printf("SPR written:%s\n", output);

	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::writeHomeProgramZ(int ID){
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

	//move up slowly 100 mm, stop on moving error
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRV,distToCounts(ID,100.0),accToSAU(ID,200.0),velToSVU(ID,2.0),-11,1);
	writeAndCheckACK(ID, output);

	//stop move (set target to position)
	sprintf(output,"@%d %d\r",ID,TTP);
	writeAndCheckACK(ID, output);

	//clear status
	sprintf(output,"@%d %d\r",ID,CIS);
	writeAndCheckACK(ID, output);

	//move down 1 mm
	sprintf(output,"@%d %d %d %d %d %d %d\r",ID,MRV,distToCounts(ID,-1.0),accToSAU(ID,200.0),velToSVU(ID,10.0),0,0);
	writeAndCheckACK(ID, output);

	//wait
	sprintf(output,"@%d %d %d\r",ID,DLT,timeToTicks(0.2));
	writeAndCheckACK(ID, output);

	//this is home; zero target and position
	sprintf(output,"@%d %d\r",ID,ZTP);
	writeAndCheckACK(ID, output);

	//set normal torque limits
	sprintf(output,"@%d %d %d %d %d %d\r",ID,TQL,20000,20000,6000,30000);
	writeAndCheckACK(ID, output);

	//kill motor on a kill motor condition
	sprintf(output,"@%d %d 0\r",ID,KMR);
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
int motionClass::runHomeProgram(int ID){
	sprintf(output,"@%d %d %d\r",ID,LRP,512); //run program at NV address 512
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::runInitProgram(int ID){
	sprintf(output,"@%d %d 0\r",ID,LRP); //run program at NV address 0
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::stop(int ID){
	sprintf(output,"@%d %d -1\r",ID,STP);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::enableMotor(int ID){
	sprintf(output,"@%d %d\r",ID,EMD);
	return writeAndCheckACK(ID,output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::resetMotor(int ID){
	sprintf(output,"@%d %d\r",ID,RST);

	writeMotSerPort(output);

	//no ACK returned
	usleep(3000000);
	flush();		//processor resets, so flush the port
	printPSW(ID);

	return 0;		
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~










//=================================================================================
//=================================================================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void motionClass::flush(void){
	tcflush(fd, TCIFLUSH);	//flush motor's port		
}	

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::getMotReply(char *result){
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
motionClass::motionClass(){ }
motionClass::~motionClass ( ){ }


const char * motionClass::cmdName(int cmdID) {
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

int motionClass::writeMotSerPort(const char *output){
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

int motionClass::writeAndCheckACK( int ID, const char* output ){
	char replyData[MAX_REPLY_LEN];
	writeMotSerPort(output);
	getMotReply( replyData);	
	return checkForACK( ID, replyData );
}

int motionClass::checkForACK( int ID, char* inputArray ){
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

int motionClass::readData( int ID, char* inputArray, int nData, int* dataArray ){
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
	printf("response packet for command %s: %s\n",cmdName(readCmdID),inputArray);

	tok = strtok(NULL," ");
	for (int i=0;i<nData;i++) {
		sscanf(tok,"%x",&dataArray[i]);
		tok = strtok(NULL," ");
	}
	//printf("ACK received. ReplyString=%s\n", inputArray);
	return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::initMotSerPort() {
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
int motionClass::goClosedLoop(int ID) {
	sprintf(output,"@%d %d\r",ID,GCL);
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::closeSerialPort(void){
	close(fd);
	return 0;
}


int motionClass::displayNACKerrors(char* inputArray) {
	//takes in a reply from the controller, and displays the NACK errors	
	return 0;
}


int motionClass::getIObitFromReply(int word, int whichBit){
	return (word >> whichBit) & 1;
}

int motionClass::readRegister(int ID, int regID, int *data){
	sprintf(output,"@%d %d %d\r",ID,RRG,regID);
	writeMotSerPort(output);

	char replyData[MAX_REPLY_LEN];
	getMotReply( replyData);	

	int words[2];
	int status = readData(ID,replyData,2,words);
	*data = ((words[0] << 16) | words[1]);

	return status;
} //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::clearAllPSWbits(int ID){
	sprintf(output,"@%d %d %d\r",ID,CPL,65535);
	return writeAndCheckACK(ID, output);
}

void motionClass::printIOmessage(int index){
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
			printf("Err:motionClass::printIOmessage()\n");
	}

}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void motionClass::printPSWmessage(int index){
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
			printf("Err:motionClass::printPSWmessage()\n");
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
int motionClass::getCloseSwitch(int ID){
	//return state of switch closest to home
	int iow;
	int status;
	status = readIO(ID, &iow);		//X axis is unit ID of 1

	return 1 - getIObitFromReply(iow,IO2bit);	//1 for on, 0 for off
}

int motionClass::getFarSwitch(int ID){
	//return state of switch farthest from home	
	int iow;
	int status;
	status = readIO(ID, &iow);		//X axis is unit ID of 1

	return 1 - getIObitFromReply(iow,IO1bit);	//1 for on, 0 for off
}

int motionClass::getHomeSwitch(int ID){
	//return state of home point switch
	int iow;
	int status;
	status = readIO(ID, &iow);		//X axis is unit ID of 1

	return 1 - getIObitFromReply(iow,IO3bit);	//1 for on, 0 for off
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::zeroTarget(int ID){
	sprintf(output,"@%d %d\r",ID,ZTP);
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::waitForPSW(int ID, int whichBit, double maxWait){
	int status;
	int psw;
	int count = 0;
	double delay = 0.1;

	while (true) {
		sleep(delay);
		status = poll(ID, &psw);

		if (getIObitFromReply(psw, whichBit)) {
			clearPoll(ID, whichBit);
			return 0;
		}
		count++;
		if (count*delay > maxWait) {
			return -1;
		}
	}
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::waitForMotionAndPSW(int ID, int whichBit, double maxWait){
	int status;
	int psw,iow;
	int count = 0;
	double delay = 0.1;

	while (true) {
		sleep(delay);
		status = poll(ID, &psw);

		if (getIObitFromReply(psw, whichBit)) {
			clearPoll(ID, whichBit);
			status = readIO(ID, &iow);
			if (!getIObitFromReply(iow,3))
				return 0;
		}
		count++;
		if (count*delay > maxWait) {
			return -1;
		}
	}
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


/*
   int motionClass::initIO(int ID){
//prep the I/O, for laser connection, etc.

setIObit(ID, IO7bit, 0);
return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::setIObit(int ID, int whichBit, int state){
	//set an IO bit.  state = -1 for high-Z, 1/0 for high/low.
	sprintf(output,"@%d %d %d %d\r",ID,CIO,whichBit,state);
	return writeAndCheckACK(ID, output);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::motInitSequence(int ID){
	int status;

	//printf("ID=%d\n",ID);
	status = resetMotor(ID);		//has built-in delay for resetting
	printPSW(ID);
	clearInternalStatus(ID);
	clearAllPSWbits(ID);

	//define the target as here, else it could be undefined and the absolute command may be undefined

	/*
	   status = s.setupEncoder(ID);
	   if ( status == -1 ) return -1;

	   status = s.initDualLoop(ID);		
	   if ( status == -1 ) return -1;
	 */

	printPSW(ID);
	printIO(ID);

	status = writeInitProgram(ID);
	status = waitForPSW(ID,15,1.0);
	printPSW(ID);
	printIO(ID);

	status = runInitProgram(ID);
	if ( status == -1 ) return -1;
	status = waitForMotionAndPSW(ID,13,5.0);
	printPSW(ID);
	printIO(ID);

	status = writeHomeProgram(ID);
	status = waitForPSW(ID,15,1.0);
	printPSW(ID);
	printIO(ID);

	status = runHomeProgram(ID);
	if ( status == -1 ) return -1;
	status = waitForMotionAndPSW(ID,13,100.0);
	printPSW(ID);
	printIO(ID);

	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::movePosRel(int ID, double distance){
	int status = moveRelativeVel(ID,-1.0*distance,100.0,10.0);
	if ( status == -1 ){
		printf("ERR: movePosRel()\n");
		stop(ID);
		return -1;
	}
	return waitForPSW(ID,13,1.0+distance/10.0);
}

int motionClass::movePosAbs(int ID, double newPos){
	double pos;
	int status = returnPosition(ID, &pos);
	if ( status == -1 ) return -1;

	double time = max( fabs(newPos - pos) / 10.0, 1.0 );
	status = moveAbsoluteVel(ID, -1.0*newPos, 100.0, 10.0);
	if ( status == -1 ){
		printf("ERR: movePosAbs()\n");
		stop(ID);
		return -1;
	}	
	return waitForPSW(ID,13,time);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::gotoHomePoint(int ID ){
	int status = movePosAbs( ID, 0.0 );
	if ( status == -1 ){
		printf("ERR: gotoHomePoint() sending moveAbsoluteTime command\n");
		return -1;
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int motionClass::resetAsHomePoint(int ID ){
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
int motionClass::clearInternalStatus(int ID) {
	sprintf(output,"@%d %d\r",ID,CIS);
	return writeAndCheckACK( ID, output );
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

