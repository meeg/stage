#ifndef __SERIAL_H__
#define __SERIAL_H__

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

#include "capser.h"
#include "utils.h"
//#include "laser.h"

using namespace std;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//set to 1 if that motor is actively connected and used (or if it should be used), 0 otherwise.
#define X_MOTOR_ACTIVE 1
#define Y_MOTOR_ACTIVE 1
#define Z_MOTOR_ACTIVE 1

#define X_AXIS_ID 1
#define Y_AXIS_ID 2
#define Z_AXIS_ID 3
#define LASER_ID  4

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

#define SLIDE_LENGTH 500.0					//length of x,y stage in mm
#define SLIDE_LENGTH_UM 50.0*10000.0	//length of x,y stage in um
#define XYSTEPSIZE 0.1						//default step size for the float boxes

#define IO7bit		15
#define IO6bit		14
#define IO5bit		13
#define IO4bit		12
#define IO3bit		6
#define IO2bit		5	
#define IO1bit		4

#define NUMSTART 48
#define BAUDRATE B57600
#define _POSIX_SOURCE 1         //POSIX compliant source
#define FALSE 0
#define TRUE 1

#define ERR_POS -1.2345
#define ENC_CTS_PER_REV 8000
#define SHAFT_CM_PER_REV 0.5
#define CM_PER_ENC_COUNTS 
#define uM_PER_CNT 0.625
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//silverlode command IDs
enum cmd_id {
	POL=0,
	CPL=1,
	HLT=2,
	STP=3,
	RST=4,
	RVN=5,
	RPB=6,
	CLP=8,
	SDL=9,
	RUN=10,
	WRI=11,
	RRG=12,
	SPR=13,
	LPR=14,
	VMI=15,
	RIS=20,
	RIO=21,
	IMW=25,
	POR=27,
	WRX=30,
	CII=31,
	RRW=32,
	PUP=33,
	ADX=64,
	CER=65,
	ETP=66,
	ETN=67,
	FL2=68,
	VLL=69,
	CT2=70,
	CBD=71,
	CDL=72,
	CID=73,
	CNL=74,
	T1F=75,
	T2S=76,
	T2K=77,
	PLS=78,
	PLT=79,
	CDR=80,
	CNR=81,
	SMD=86,
	JRB=89,
	PCB=89,
	SSI=92,
	EGM=93,
	PVC=93,
	END=128,
	PWO=129,
	SEF=130,
	LVP=131,
	MAV=134,
	MRV=135,
	JGE=137,
	JGR=137,
	JLE=137,
	JLT=137,
	JNE=137,
	JRE=137,
	WCL=138,
	WCW=139,
	DLT=140,
	DLY=140,
	WDL=141,
	GCL=142,
	GOL=143,
	ZTG=144,
	ZTP=145,
	TTP=146,
	CME=147,
	CTC=148,
	TQL=149,
	AHC=150,
	ERL=151,
	OLP=152,
	WRF=154,
	WRP=154,
	IDT=155,
	LRP=156,
	CLX=158,
	VMP=159,
	RAV=160,
	RRV=161,
	JMP=162,
	JOI=162,
	CIS=163,
	CKS=164,
	CLC=165,
	CLM=166,
	KMC=167,
	MCT=168,
	FLC=169,
	EEM=170,
	DDB=171,
	DEM=171,
	PAC=172,
	ADL=173,
	BRT=174,
	MAT=176,
	MRT=177,
	RAT=178,
	RRT=179,
	SSD=180,
	KMR=181,
	KED=182,
	KDD=183,
	DIR=184,
	PRO=185,
	SIF=186,
	EDL=187,
	CIO=188,
	MDS=189,
	MDC=190,
	MDT=191,
	EMN=192,
	SEE=192,
	ARI=193,
	WBS=194,
	SCF=195,
	RSM=196,
	RLM=197,
	RSN=198,
	RLN=199,
	CLD=200,
	PCI=201,
	PCL=201,
	PRI=202,
	PRT=202,
	WBE=204,
	SOB=205,
	COB=206,
	ACR=207,
	PLR=208,
	FOR=209,
	NXT=210,
	CAI=211,
	LVT=212,
	OVT=213,
	MTT=214,
	CTW=215,
	PIM=216,
	VIM=217,
	TIM=218,
	AHM=219,
	KMX=220,
	SSL=221,
	TRU=222,
	RSD=223,
	EMT=225,
	DMT=226,
	EMD=227,
	DMD=228,
	HSM=229,
	AHD=230,
	PCM=231,
	PCG=232,
	XRV=233,
	XAV=234,
	XRT=235,
	XAT=236,
	GOC=237,
	JNA=238,
	JOR=239,
	PMC=240,
	PMV=241,
	PMX=242,
	DLC=243,
	SLC=244,
	PCP=245,
	ATR=248,
	PMO=249,
	JAN=250,
	EDH=251,
	DIF=252,
	IMS=253,
	IMQ=254,
	RSP=255
};
	
//~~~~~~~~~~~~~~~~~~ variables ~~~~~~~~~~~~~~~~~~
//dist in mm, all times in s

const double ticksPerMM = 8000/(0.2*25.4);
const double ticksPerSecond = 1 / 0.000120;		//number of clock ticks per second
const double velFactor = 1000.0;
const double accFactor = 100.0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
class motionClass{
	private:
		//~~~~~~~~~~~~~~~~~~ CONSTANTS ~~~~~~~~~~~~~~~~~~
		int fd; // File descriptor for the motor's serial port
	
		char output[254];
		#define Baud_Rate 57600;         // default Baud Rate (110 through 38400)

		//~~~~~~~~~ unit conversion ~~~~~~~~~
		int timeToTicks(double time); //units of sec
		double screwPitch(int ID); //units of mm
		double distPerCount(int ID); //units of mm
		int distToCounts(int ID, double dist); //units of mm
		int velToSVU(int ID, double vel); //units of mm/s
		int accToSAU(int ID, double acc); //units of mm/s^2

		//~~~~~~~~~ CMD functions ~~~~~~~~~

		int poll(int ID, int* intToReturn);		//POR
		int clearPoll(int ID, int whichBit);//, bool debugOn = false);	//CPL
		int readIO(int ID, int* intToReturn );			//RIO
		int readRegister(int ID, int regID, int *data);
		int resetMotor(int ID);
		int clearInternalStatus(int ID);
		int halt(int ID);	//HAL
		int enableMotor(int ID); //EMD
		int initSingleLoop(int ID);
		int initDualLoop(int ID);	//DLC
		int setupEncoder(int ID);
		int zeroTarget(int ID);
		int goClosedLoop(int ID);
		int killMotorConditions(int ID, int enableWord, int stateWord);

		//int initIO(int ID);
		int setIObit(int ID, int whichBit, int state);
		int getCloseSwitch(int ID);
		int getFarSwitch(int ID);
		int getHomeSwitch(int ID);

		int moveRelativeTime( int ID, double dist, double rampTime, double totalTime );
		int moveAbsoluteTime( int ID, double pos, double accTime, double totalTime );
		int moveRelativeVel( int ID, double dist, double acc, double vel );
		int moveAbsoluteVel( int ID, double pos, double acc, double vel );

		int writeInitProgram(int ID);
		int writeHomeProgram(int ID);
		int runInitProgram(int ID);
		int writeHomeProgramXY(int ID);
		int writeHomeProgramZ(int ID);

		int waitForPSW(int ID, int whichBit, double maxWait);
		int waitForMotionAndPSW(int ID, int whichBit, double maxWait);

		//~~~~~~~~~~~ serial IO ~~~~~~~~~~~
		void flush(void);
		int writeMotSerPort(const char *output);
		int writeAndCheckACK( int ID, const char* output );
		int readData( int ID, char* inputArray, int nData, int* dataArray );
		int getMotReply(char *result);
		int checkForACK( int ID, char* inputArray );

		const char * cmdName(int cmdID);
		void printPSWmessage(int index);
		void printIOmessage(int index);
		int displayNACKerrors(char* inputArray);
		int getIObitFromReply(int word, int whichBit);

	public:
		virtual ~motionClass ( void );
		motionClass ( void );

		int motInitSequence(int ID);

		//~~~~~~~~~~~ serial IO ~~~~~~~~~~~
		int initMotSerPort(void); 		//initialize the serial port going to the motors.
		int closeSerialPort(void);

		//~~~~~~~~~~ serial utilities ~~~~~~~~~~~~
		int clearAllPSWbits(int ID);
		int printPSW(int ID);
		int printIO(int ID);
		int killActiveMotors(void);		
		int setKillMotorConditions(void);		
		int returnPosition( int ID, double* pos );
		int stop(int ID);	//STP
		
		//~~~~~~~~~~~~  motion sets  ~~~~~~~~~~~~
		int runHomeProgram(int ID);
		int movePosRel(int ID, double distance);
		int movePosAbs(int ID, double newPos);
		int gotoHomePoint(int ID);
		int resetAsHomePoint( int ID );

};//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif
