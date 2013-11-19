//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Paul Csonka
//current Aug 10, 2010
//controls the XYZ stages and laser pulser stage for KPIX calibration.
//connect to this program via TCP using the "client" program
//messages betwen client and server sent in XML
//optional QT interface panel

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <iostream>

//tcp related includes
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
//end tcp related includes

//std lib includes
#include <stdio.h>   /* Standard input/output definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <cstring>
#include <sys/signal.h>
#include <cstdlib>
#include <bitset>
#include <fstream>
//end std lib includes

//qt related includes
#include <qapplication.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qslider.h>
#include <qlcdnumber.h>
#include <qfont.h>
#include <qvbox.h>
#include <qgrid.h>
#include<qobject.h>
#include<qwidget.h>
#include<qdialog.h>
#include<qgroupbox.h>
#include<qhbox.h>
//end qt related includes

//custom local headers
#include "stage.h"
#include "irrXML.h"
#include "xmlwriter.h"
//#include "serial.h"
#include "utils.h"
#include "motion.h"
#include "gui.h"
//#include "test.h"
#include "laser.h"

//end local headers
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
using namespace std;
using namespace xmlw;
using namespace irr; // irrXML is located 
using namespace io;  // in the namespace irr::io

//#define MAX_XY_LENGTH 350	//max length of XY in mm
//#define MAX_Z_LENGTH 30		//max length of Z in mm

const int arrayLen = 254;

int sockfd, newsockfd, portno;
socklen_t clilen;
char buffer[arrayLen];
struct sockaddr_in serv_addr, cli_addr;
int n;
bool TCPinitDone = false;
int TCPportNumber = 55175;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~  VARIABLES ~~~~~~~~~~~~~~~~~~~~   
//motor serial port
motionClass s;
laserClass l;
int whichCommand;
int ID=5;
double pos;		//generic variable to hold the returned position from an axis, for example.  GUI accesses this.
		
//qt related
double desiredXrel = 0.0, desiredYrel = 0.0, desiredZrel = 0.0;
double desiredXabs, desiredYabs, desiredZabs;
double desiredLaserWidth = 10.0, desiredLaserAmp = 100.0;
//command parsing:
char receivedData[arrayLen];
//double parsedReply[arrayLen];
int numGroups;
double params[2];
char replyArr[arrayLen];
// strings for storing the data we want to get out of the file
std::string cmdStr;
std::string IDstr;
std::string param1Str;
std::string param2Str;
double laserWidth = 2.0;
double laserAmp = 50.0;

//xml:
const int XMLstrLen = arrayLen;
char XMLpacket[XMLstrLen];
//IrrXMLReader* xml = createIrrXMLReader("xml.xml");		//file input
IrrXMLReader* xml;
//string XMLstring(XMLpacket);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

//QT control or TCP control?  Comment out to use normal TCP connection
int useQTpanel;


	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
//~~~~~~~~~~~~~~~  MAIN LOOP ~~~~~~~~~~~~~~~~~~~~   

int main( int argc, char **argv )
{   	
	int status;
    
	 
	 //~~~~~~~~~~~~~~~~~~~~~~
	 if (argc < 2) {
       fprintf(stderr,"Err: usage: %s <port>\n ", argv[0]);
       exit(0);
    }
	 //~~~~~~~~~~~~~~~~~~~~~~
	 
	 	 
	//~~~~~~~~~~~~~~~~~~~~~~
	printf("\n");	
	printf("\n");	
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
   printf("     Stage/Laser  Controller     \n");;
	printf("    Paul Csonka, Aug 16  2010    \n");
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	printf("\n");
	printf("\n");		
	//~~~~~~~~~~~~~~~~~~~~~~
	
	
	//~~~~~~~~~~~~~~~~~~~~~~
	string qtStr("qt");
	string argvStr( argv[1] );

	if ( argvStr.compare(qtStr) == 0){
		printf("QT Interface Only (no TCP support)\n");
		useQTpanel = 1;
	}else{
		useQTpanel = 0;
		printf("TCP Interface Only (no QT support)\n");
		TCPportNumber = atoi( argv[1] );
	}//~~~~~~~~~~~~~~~~~~~~~~

	
	//~~~~~~~~~~~~~~~~~~~~~~ 
	//~~~~~~~~~~~~~~~~~~~~~~
	status = initEverything();
	endProgramOnError(status);	
	printf("Init Complete\n");	
	//~~~~~~~~~~~~~~~~~~~~~~
	//~~~~~~~~~~~~~~~~~~~~~~
	

	
	//~~~~~~~~~~~~~~~~~~~~~~
	memset(XMLpacket, 0, XMLstrLen); 
	//test XML output here
	//int numParams = 2;
	//just a test of the write capabilities of the server
	/*status = writeXMLfile(ID, GOTO_HOME, params, numParams, XMLpacket);
	status = addLFtoEndofXMLstring();
	ofstream File("xml.xml");
	File << XMLpacket;
	File.close();
	cout << XMLpacket << endl;*/
	//~~~~~~~~~~~~~~~~~~~~~~
	 

	
	//~~~~~~~  QT ~~~~~~~~~~
	if ( useQTpanel == 1){
	QApplication a( argc, argv );
	customWidget w;

	
   w.show();
   return a.exec();
	//how to shut down gracefully?
	endProgram();
	}//~~~~~~~~~~~~~~~~~~~~~~


	
	//~~~~~~~~~~~~~~~~~ MAIN LOOP ~~~~~~~~~~~~~~~~~~
  	printf("Waiting for TCP Command\n");
		
	//main command receiver loop
	while (1){
		
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~
		//wait for received command:
		printf("Waiting for Command\n");
		n = read(newsockfd,buffer,arrayLen-1);
		if (n < 0) {
			error("ERROR reading from socket");
			endProgramOnError(-1);
		}
		printf("\nHere is the message I received: %s\n",buffer);
		sleep (0.1);
		ofstream File("xml.xml", ios::trunc | ios::out);
		File << buffer;
		File.close();
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~		status = sendArrRepl
		

		//~~~~~~~~~~~~~~~~~~~~~~~~~~~
		xml = createIrrXMLReader( "xml.xml" );				//memory input
		//string XMLstring(XMLpacket);
		if ( xml == NULL ){
			printf("error opening file/packet\n");
			return -1;
		}
		sleep(0.05);		//necessary delay
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~		status = sendArrRepl
	  	
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~
		status = parseThroughXMLstring();		// parse through the xml string until end reached, and recover id,cmd, and params
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~		status = sendArrRepl

							
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~
		//normal command line parser (commented out, located at end of file) goes in here, if XML is not used:
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~		status = sendArrRepl
		

		//~~~~~~~~~~~~~~~~~~~~~~~~~~~
		status = checkForValidID();		//always returns 0, but can change the position to ERR_POS
		if (whichCommand == -1) {
			pos = ERR_POS;
			status = turnNumberIntoCharArray( ID, replyArr, pos );		//ERR_POS denotes command not completed.  
																					//Do this here because the cases below won't	execute
		}//~~~~~~~~~~~~~~~~~~~~~~~~~~
					
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~
		status = executeRequestedCommand(ID, whichCommand, params[0], numGroups);
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~
		
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~		
		status = writeXMLposToReturn(ID, pos, replyArr);
		cout << "about to send to client: " << replyArr << endl;
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~
		
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~		
		status = sendArrReply( replyArr );
		if ( status == -1 ) {
			printf("Err while sending\n");
			//the connection was probably closed, so restart the system to be ready again
			status = initEverything();
			endProgramOnError(status);		//if restarting worked, then just keep going.  But if it didn't, then exit	permanently
		}//~~~~~~~~~~~~~~~~~~~~~~~~~~~		status = sendArrRepl

		
	
	}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//if the program breaks and exits to here, then shut down gracefully, but first see what happened.
	s.clearAllPSWbits(ID);
	status = s.printPSW(ID);
	endProgramOnError(status);
					

	status = s.clearAllPSWbits(ID);
	
	sleep(2.0);	
	
	status = s.clearAllPSWbits(ID);
	status = s.printPSW(ID);
	endProgram();
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
   return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int initEverything(void){
	int status;
	
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~
	status = s.initMotSerPort();
	if ( status < 0 ) return -1;
	//~~~~~~~~~~~~~~~~~~~~~~~~~	
	
	//this is just a test:
	//status = s.changeACKdelay( 1, 32.0 );
	//printf("here2\n");
	//if ( status == -1 ) return -1;
	
	
	//exit(1);
			
	
	#ifdef useLaser
	status = l.laserInitSequence();
	//sleep (0.2);
	status = l.setLaserWidth_ns(desiredLaserWidth);
	//sleep (0.2);
	status = l.setLaserAmp_mV(desiredLaserAmp);
	#endif
			
		
	//~~~~~~~~~~~~~~~~~~~~~~~~~	
	if ( X_MOTOR_ACTIVE == 1 ){
		printf("\ninit X\n");
	   status = s.motInitSequence( X_AXIS_ID );
		if ( status < 0 ) return -1;
	}
	if ( Y_MOTOR_ACTIVE == 1 ){
		printf("\ninit Y\n");
	   status = s.motInitSequence( Y_AXIS_ID );
		if ( status < 0 ) return -1;
	}
	if ( Z_MOTOR_ACTIVE == 1 ){
		printf("\ninit Z\n");
	   status = s.motInitSequence( Z_AXIS_ID );
		if ( status < 0 ) return -1;
	}//~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
	//only init the tcp port if the qt panel isn't being used
	if ( useQTpanel == 0 ){
	//~~~~~~~~~~~~~~~~~~~~~~~~~
	if ( TCPinitDone == false ){
		int status = initTCP();
		if ( status < 0 ) return -1;
	}//~~~~~~~~~~~~~~~~~~~~~~~~~
	}
		
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   






//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int initTCP(void){

     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("Err: opening TCP socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = TCPportNumber;		//set above in main
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
		 error("Err: on TCP binding");
		 closeTCPport();
		 return -1;
		}
		
	//wait for client to connect.
	listen(sockfd,5);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr,&clilen);
	if (newsockfd < 0) {
   	 error("Err: on TCP accept");
 		 closeTCPport();
		 return -1;
	 }
	
	bzero(buffer,arrayLen);
	TCPinitDone = true;
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int endProgram(){
	int status;	
	
	printf("Killing Motors...\n");
	status = s.killActiveMotors();
	printf("closing All Serial connections...\n");
	status = s.closeSerialPort();
	status = l.laserLocalEnable();
	sleep (0.1);
	status = l.closeLaserSerialPort();
	printf("closing TCP connection...\n");
	closeTCPport();
	
	printf("Program Terminated\n");
	exit(1);
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int endProgramOnError(int status){
		
	if ( status == -1 ){
		endProgram();	
		while(1){};
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void closeTCPport(void){
	close(sockfd);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void error(char *msg){
    perror(msg);
    exit(1);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int isAxisID(int ID){
	//is the ID one of the axes?
	if ( (ID != X_AXIS_ID) & (ID != Y_AXIS_ID) & (ID != Z_AXIS_ID) ){
		return 0;
	}else{
		return 1;
	}
		
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   	
int writeXMLposToReturn(int ID, double pos, char* output){
	
	ostringstream f;
  	XmlStream xml(f);
	xml //<< prolog() // uncomment to add in the XML declaration
		<< tag("Packet ") 				// root tag
			
			<< tag("whichAxis")  			//child
				<< attr("ID") << ID		//value
			<< endtag("whichAxis")
			
			<< tag("position")
				<< attr("pos") << pos
			<< endtag("position")

			//<< chardata() << "Packet complete"
		<< endtag("Packet");
		
		string outstr( f.str() );
		strcpy( output, outstr.c_str() );
		//cout << output << endl;
		//int status = addLFtoEndofXMLstring();
		return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   			


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int writeXMLfile(int ID, int cmd, double* params, int numParams, char* output){


	printf("about to write xml\n");
			
	//ofstream f("z.xml");
	ostringstream f;
  	XmlStream xml(f);
  
	if ( ID == LASER_ID ){
		if ( cmd == SET_LASER_WIDTH ){
			xml 
			<< tag("Packet ") 
			<< tag("whichAxis")  			//child
				<< attr("ID") << ID		//value
			<< endtag("whichAxis")				

			<< tag("command")
					<< attr("cmd") << cmd		//value
			<< endtag("command")

			<< tag("laserWidth")
					<< attr("width") << params[0]		//value
			<< endtag("laserWidth")	
			<< endtag("Packet");
			string outstr( f.str() );
			strcpy( output, outstr.c_str() );
			return 0;

		}else if ( cmd == SET_LASER_AMP	){
			xml 
			<< tag("Packet ") 
			<< tag("whichAxis")  			//child
				<< attr("ID") << ID		//value
			<< endtag("whichAxis")				

			<< tag("command")
					<< attr("cmd") << cmd		//value
			<< endtag("command")

			<< tag("laserAmp")
					<< attr("amp") << params[0]		//value
			<< endtag("laserAmp")	
			<< endtag("Packet");

			string outstr( f.str() );
			strcpy( output, outstr.c_str() );
			return 0;
		}else if ( cmd == TRIG_LASER	){
			xml 
			<< tag("Packet ") 
			<< tag("whichAxis")  			//child
				<< attr("ID") << ID		//value
			<< endtag("whichAxis")				

			<< tag("command")
					<< attr("cmd") << cmd		//value
			<< endtag("command")

			<< endtag("Packet");

			string outstr( f.str() );
			strcpy( output, outstr.c_str() );
			return 0;
		}		
		return -1;		
	}else if ( ( ID == X_AXIS_ID ) | ( ID == Y_AXIS_ID ) | ( ID == Z_AXIS_ID ) ){
	

		//else normal stage axis IDs...
		if ( numParams == 0){
			xml //<< prolog() // uncomment to add in the XML declaration
			<< tag("Packet ") 				// root tag

				<< tag("whichAxis")  			//child
					<< attr("ID") << ID		//value
				<< endtag("whichAxis")

				<< tag("command")
					<< attr("cmd") << cmd
				<< endtag("command")

				//<< chardata() << "Packet complete"
			<< endtag("Packet");

			string outstr( f.str() );
			strcpy( output, outstr.c_str() );
			//cout << output << endl;
			return 0;
		}else	if ( numParams == 1){
			xml //<< prolog() // uncomment to add in the XML declaration
			<< tag("Packet ") 				// root tag

				<< tag("whichAxis")  			//child
					<< attr("ID") << ID		//value
				<< endtag("whichAxis")

				<< tag("command")
					<< attr("cmd") << cmd
				<< endtag("command")

				<< tag("parameter1")
					<< attr("param1") << params[0]
				<< endtag("parameter1")

				//<< chardata() << "Packet complete"
			<< endtag("Packet");

			string outstr( f.str() );
			strcpy( output, outstr.c_str() );
			//cout << output << endl;
			return 0;
		}else if ( numParams == 2 ){
			xml << prolog() // write XML file declaration
			<< tag("Packet ") 				// root tag

				<< tag("whichAxis")  			//child
					<< attr("ID") << ID		//value
				<< endtag("whichAxis")

				<< tag("command")
					<< attr("cmd") << cmd
				<< endtag("command")

				<< tag("parameter1")
					<< attr("param1") << params[0]
				<< endtag("parameter1")

				<< tag("parameter2")
					<< attr("param2") << params[1]
				<< endtag("parameter2")

				//<< chardata() << "Packet complete"
			<< endtag("Packet");
			string outstr( f.str() );
			strcpy( output, outstr.c_str() );
			return 0;			
		}	
	}
	
	printf("Err: invalid ID in xml creation, no string made\n");
	return -1;				

}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int checkForValidID( void ){
		printf ("ID to check:%d\n", ID);
		switch (ID){
			case X_AXIS_ID:
				if ( X_MOTOR_ACTIVE == 0 ){
					printf("Err: inactive motor\n");
					bzero(buffer,arrayLen);
					whichCommand = -1;		//change command over to -1 to denote faulty string
					return 0;		//ignore string, but keep the program running
				}
				break;
			case Y_AXIS_ID:
				if ( Y_MOTOR_ACTIVE == 0 ){
					printf("Err: inactive motor\n");
					bzero(buffer,arrayLen);
					whichCommand = -1;
					return 0;		//ignore string, but keep the program running
				}
				break;
			case Z_AXIS_ID:
				if ( Z_MOTOR_ACTIVE == 0 ){
					printf("Err: inactive motor\n");
					bzero(buffer,arrayLen);
					whichCommand = -1;
					return 0;		//ignore string, but keep the program running
				}
				break;
			case LASER_ID:
				printf("Laser command received\n");
				break;
			default:
				printf("Err: Invalid ID received\n");
				whichCommand = -1;	//make it so that no command is sent
				return 0;			//ignore string, but keep the program running
				break;
		}
		return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int executeRequestedCommand(int whichID, int commandToExecute, double theParameter, int numGroupsFound){
	int status;
	//to clear the array:
	memset(replyArr, 0, arrayLen); 

	//only do this if it's a valid motor ID	
	if ( commandToExecute != -1 )

	switch (commandToExecute){

		case KILL:
			printf("KILL\n");
			if ( (numGroupsFound != 1) | (isAxisID(whichID) == 0) ){
				printf("Err: Wrong parameters with KILL\n");
				bzero(buffer,arrayLen);
				//endProgramOnError(-1);
				return 0;		//just ignore the command
				break;					
			}
			status = s.killActiveMotors();
			if (status == -1 ) return status;		//could be (-1)
			status = s.returnPosition(whichID, &pos);
			if (status == -1 ) return status;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			if (status == -1 ) return status;
			break;

		case FIND_HOME:
			printf("FIND_HOME\n");
			if ( (numGroupsFound != 1) | (isAxisID(whichID) == 0) ) {
				printf("numGroupsFound=%d\n", numGroupsFound );
				printf("Err: Wrong parameters with FIND_HOME\n");
				bzero(buffer,arrayLen);
				return 0;
				break;					
			}
			status = s.runHomeProgram(whichID);
			if (status == -1 ) return status;
			status = s.returnPosition(whichID, &pos);
			if (status == -1 ) return status;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			if (status == -1 ) return status;
			break;

		case GOTO_HOME:
			printf("GOTO_HOME\n");
			if ( (numGroupsFound != 1) | (isAxisID(whichID) == 0) ) {
				printf("numGroupsFound=%d\n", numGroupsFound );
				printf("Err: Wrong parameters with GOTO_HOME\n");
				bzero(buffer,arrayLen);
				//endProgramOnError(-1);
				break;					
			}				

			status = s.gotoHomePoint(whichID);
			if (status == -1 ) return status;
			status = s.returnPosition(whichID, &pos);
			if (status == -1 ) return status;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			if (status == -1 ) return status;
			break;

		case REL_MOTION:
			printf("REL_MOTION\n");
			if ( (numGroupsFound != 2) | (isAxisID(whichID) == 0) ) {
				printf("Err: Wrong parameters with REL_MOTION.  Got: %d\n", numGroupsFound);
				bzero(buffer,arrayLen);
				//endProgramOnError(-1);
				break;					
			}		
			printf("ID=%d, theParameter=%f\n", whichID, theParameter);
			s.movePosRel(whichID, theParameter );
			//printf("status from movePosRel():%d\n", status);
			if (status == -1 ) return status;
			status = s.returnPosition(whichID, &pos);
			//printf("status from returnPosition():%d\n", status);
			if (status == -1 ) return status;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			//printf("status from turnNumberIntoCharArray():%d\n", status);
			if (status == -1 ) return status;
			break;


		case ABS_MOTION:
			printf("ABS_MOTION\n");
			if ( (numGroupsFound != 2) | (isAxisID(whichID) == 0) ) {
				printf("Err: Wrong number of parameters with ABS_MOTION\n");
				bzero(buffer,arrayLen);
				//endProgramOnError(-1);
				break;					
			}		

			s.movePosAbs(whichID, theParameter );	//param
			if (status == -1 ) return status;
			status = s.returnPosition(whichID, &pos);
			if (status == -1 ) return status;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			if (status == -1 ) return status;
			break;

		case SEQ_1:
			printf("SEQ_1\n");
			if ( (numGroupsFound != 1) | (isAxisID(whichID) == 0) ) {
				printf("Err: Wrong number of parameters with SEQ_1\n");
				bzero(buffer,arrayLen);
				//endProgramOnError(-1);
				return 0;
				break;
			}		

			//status = s.stepAndPulseSequence(whichID, 10.0, 1.0, 10, 0.0, 0.0);
			status = stepAndPulseSequenceBothAxes(X_AXIS_ID, Y_AXIS_ID, 1, 1, 4, 4, 3.0, 300.0);
			if (status == -1 ) return status;
			status = s.returnPosition(whichID, &pos);
			if (status == -1 ) return status;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			if (status == -1 ) return status;
			break;


		case GIVE_POS:
			printf("GIVE_POS\n");
			if ( (numGroupsFound != 1) | (isAxisID(whichID) == 0) ) {
				printf("Err: Wrong number of parameters with GIVE_POS\n");
				bzero(buffer,arrayLen);
				//endProgramOnError(-1);
				return 0;
			}		

			status = s.returnPosition(whichID, &pos);
			if (status == -1 ) return status;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			if (status == -1 ) return status;
			//cout << replyArr << endl;
			return status;					
		case SET_LASER_WIDTH:
			printf("LASER_WIDTH\n");
			//status = s.setLaserWidth_ns(laserWidth);
			status = l.setLaserWidth_ns(theParameter);
			//status = s.returnPosition(whichID, &pos);
			//endProgramOnError(status);
			pos = theParameter;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			return status;				
			break;

		case SET_LASER_AMP:
			printf("LASER_AMP\n");
			status = l.setLaserAmp_mV(theParameter);
			//status = s.setLaserAmp_mV(laserAmp);			
			//status = s.returnPosition(ID, &pos);
			//endProgramOnError(status);
			pos = theParameter;			
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			return status;				
			break;

		case TRIG_LASER:
			printf("LASER_TRIG\n");
			status = l.sendLaserTrigger();
			pos = 1234.5;				
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			return status;				
			break;
			
		case RESET_HOME:
			printf("RESET_HOME\n");
			if ( (numGroupsFound != 1) | (isAxisID(whichID) == 0) ) {
				printf("numGroups=%d\n", numGroups );
				printf("Err: Wrong parameters with RESET_HOME\n");
				bzero(buffer,arrayLen);
				//endProgramOnError(-1);
				break;					
			}				

			status = s.resetAsHomePoint(whichID);
			if (status == -1 ) return status;
			status = s.returnPosition(whichID, &pos);
			if (status == -1 ) return status;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			if (status == -1 ) return status;
			break;
			
		default:
			printf("Err: Invalid Command Received\n");
			pos = ERR_POS;
			status = turnNumberIntoCharArray( whichID, replyArr, pos );
			if (status == -1 ) return status;
			bzero(buffer,arrayLen);
			//endProgram();
			break;

	}//end switch
	return 0;
}	//end sub
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int parseThroughXMLstring( void ){
	printf("Parsing XML\n");
	numGroups = 0;
	
	//ofstream File("xml.xml");
	
	while(xml && xml->read())
	{
		//printf("in xml loop\n");
		switch(xml->getNodeType())
		{
		case EXN_TEXT:
			//printf("in text\n");
			// in this xml file, the only text which
			// occurs is the messageText
			cmdStr = xml->getNodeData();
			break;

		case EXN_ELEMENT:
			//printf("in element\n");
			if (!strcmp("whichAxis", xml->getNodeName() )){
			IDstr = xml->getAttributeValue("ID");
			//printf("IDstr=%s\n", IDstr.c_str());
			ID = atoi(IDstr.c_str());
			printf("\ngot ID=%d\n",ID);
		}else if (!strcmp("command", xml->getNodeName() )){
			cmdStr = xml->getAttributeValue("cmd");
			//printf("cmdStr=%s\n", cmdStr.c_str());
			whichCommand = atoi( cmdStr.c_str() );
			printf("got cmd=%d\n", whichCommand);
			numGroups++;
		}else if (!strcmp("parameter1", xml->getNodeName() )){
			param1Str = xml->getAttributeValue("param1");
			params[0] = atof( param1Str.c_str() );
			printf("got params[0]=%f\n", params[0]);
			numGroups++;				 
		}else if (!strcmp("parameter2", xml->getNodeName() )){
			 param1Str = xml->getAttributeValue("param2");
			 params[1] = atof( param1Str.c_str() );
			 printf("got params[1]=%f\n", params[1] );
			 numGroups++;				 
		}else if (!strcmp("laserWidth", xml->getNodeName() )){
			 param1Str = xml->getAttributeValue("width");
			 params[0] = atof( param1Str.c_str() );
			 printf("got laserWidth=%f\n", params[0] );

		}else if (!strcmp("laserAmp", xml->getNodeName() )){
			 param1Str = xml->getAttributeValue("amp");
			 params[0] = atof( param1Str.c_str() );
			 printf("got laserAmp=%f\n", params[0] );

		}
		break;
		}
	}
	  
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
		

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~				
int addLFtoEndofXMLstring( void ){
	 //now add on the LF (ascii 10) to the end, which is necessary for the string to send properly over tcp.
	 int replyLen=0;
	 for (int i = 0; i < arrayLen; i++){
		 //printf("%d:%d\n", i, XMLpacket[i]);
		 if ( XMLpacket[i] == 0 ){
			 replyLen = i;
			 if ( (XMLpacket[i] > 127) | (XMLpacket[i] < 32) ){
				 if ( XMLpacket[i] != 0 ){
					 printf("unexpected char at %d:%d\n", i, XMLpacket[i]);
				 }
			 }
			 //printf("Len=%d\n", replyLen);
			 break;
		 }
	 }
	 //replyLen = 60;

	 //for (int i = 0; i < replyLen + 1; i++){
		//buffer[i] = buffer[i] + 1;
		//printf("%d: %d\n", i, buffer[i]);
	// }
	 //XMLpacket[ replyLen ] = 13;			//LF
	 //XMLpacket[ replyLen + 1 ] = 10;		//terminate with null (probably not necessary)
	 //XMLpacket[ replyLen + 2 ] = 0;		 
	 for (int i = 0; i < replyLen+2; i++){
		 printf("%d:%d\n", i, XMLpacket[i]);
	 }
 }//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int sendArrReply( char* replyArr ){
		string checkLen(replyArr);
		int len = checkLen.length();
		if ( len >= arrayLen ){
			 error("Err: packet longer than allotted string\n");
	 		 memset(replyArr, 0, len); 
			 return 0;		//don't kill the program, just dont' send the command.
		 }
		n = write(newsockfd,replyArr,len);
		if (n < 0){
			 error("Err: Writing to TCP Socket");
			 //initTCP();
			 return -1;
		}
		
		memset(buffer, 0, arrayLen); 
		return  0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
double returnPosFromMain( void ){
	//printf("returned from main:%f\n", pos);
	return pos;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//normal command line parser goes here:
//numGroups = parseCharReplyIntoArray( buffer, parsedReply);
/*numGroups = parseCharReplyIntoArray( buffer, parsedReply);
//printf( "numGroups=%d\n", numGroups);
if ( numGroups == -1 ){
endProgram();
endProgramOnError(status);		//could be (-1)
exit(1);
}

whichCommand = buffer[0] - 48;

//NOTE: THE ORDER HERE IS IMPORTANT, THIS HAS TO GO BEFORE PARSEDREPLY[0] SO THAT COMMAND CAN BE OVERWRITTEN
whichCommand = (int) parsedReply[1];
double param1 = parsedReply[2];


ID = (int) parsedReply[0];
//XML string gives <ID> <command> <parameter>
*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int stepAndPulseSequenceBothAxes(int ID1, int ID2, double stepSize1, double stepSize2, int numOfSteps1, int numOfSteps2, double laserWidth, double laserAmp){
	//starts at a known position, and pulses after each motion step
	//step sizes in um, start positions in um.
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (   ( stepSize1 <= 0.0 ) | ( ID1 < 0 ) | ( ID1 > 100 ) | ( numOfSteps1 < 1 ) | ( laserWidth <2 ) | (stepSize1 >
		SLIDE_LENGTH_UM) ){
		printf("ERR: stepAndPulseSequence().  Incorrect parameter; outside allowed range\n");
		printf("ID=%d, stepSize=%f, numOfIntervals=%d, duration=%f, intensity=%f\n", 
						ID1, stepSize1, numOfSteps1, laserWidth, laserAmp);
		s.stop(ID1);
		return -1;			
	}//~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
	double pos1,pos2;
	int status;
	status = s.returnPosition(ID1, &pos1);
	status = s.returnPosition(ID2, &pos2);
	//if ( status == -1 ) return -1;
	
	for(int i = 0; i < numOfSteps1; i++){
		status = s.movePosRel(ID1,stepSize1);
		if ( status == -1 ){
			printf("ERR: stepAndPulseSequence() while stepping\n");
			s.stop(ID1);
			return -1;
		}
		for(int i = 0; i < numOfSteps1; i++){
			status = s.movePosRel(ID2, stepSize2);
			if ( status == -1 ){
				printf("ERR: stepAndPulseSequence() while stepping\n");
				s.stop(ID2);
				return -1;
			}
					
			laserWidth = 3.0;
			status = l.pulseLaser(laserWidth, laserAmp);
			if ( status == -1 ){
				printf("ERR: stepAndPulseSequence() setting laser\n");
				s.stop(ID1);			
				s.stop(ID2);						
				return -1;
			}
			
			printf("Pulsing Laser\n");
			sleep (0.1);
		}
	}
	
	status = s.movePosAbs(ID1,pos1);
	if ( status == -1 ){
		printf("ERR: stepAndPulseSequenceBothAxes() going home ID1\n");
		s.stop(ID1);			
		return -1;
	}	
	status = s.movePosAbs(ID2,pos2);
	if ( status == -1 ){
		printf("ERR: stepAndPulseSequenceBothAxes() going home ID2\n");
		s.stop(ID2);			
		return -1;
	}		
	return 0;	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int stepAndPulseSequence(int ID, double startPos, double stepSize, 
							int numOfIntervals, double	duration, double intensity){
	//starts at a known position, and pulses after each motion step
	
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (   ( stepSize <= 0.0 ) | ( ID < 0 ) | ( ID > 100 ) | ( numOfIntervals < 1 ) | ( duration <0 ) | (stepSize >
		100.0) | ( startPos < -1.0 * SLIDE_LENGTH )  | ( startPos > SLIDE_LENGTH )   ){
		printf("ERR: stepAndPulseSequence().  Incorrect parameter; outside allowed range\n");
		printf("ID=%d, startPos=%f, stepSize=%f, numOfIntervals=%d, duration=%f, intensity=%f\n", 
						ID, startPos, stepSize, numOfIntervals, duration, intensity);
		s.stop(ID);
		return -1;			
	}//~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	int status;
	
	status = s.movePosAbs(ID, startPos);
	if ( status == -1 ){
		return -1;
	}
	
	for(int i = 0; i < numOfIntervals; i++){
		double newPos = startPos + i*stepSize;
		status = s.movePosAbs(ID, newPos);	
		if ( status == -1 ){
			printf("ERR: stepAndPulseSequence() while stepping\n");
			s.stop(ID);
			return -1;
		}
				
		duration = 1.0;
		status = l.pulseLaser(duration, intensity);


		if ( status == -1 ){
			printf("ERR: stepAndPulseSequence() setting laser\n");
			s.stop(ID);			
			return -1;
		}
		
		printf("Pulsing Laser\n");
		sleep (0.1);
		
	}
	
	status = s.gotoHomePoint(ID);
	
	return 0;	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


