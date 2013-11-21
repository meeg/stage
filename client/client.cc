
//qt related includes
#include <qapplication.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qslider.h>
#include <qlcdnumber.h>
#include <qfont.h>
#include <q3vbox.h>
#include <q3grid.h>
#include<qobject.h>
#include<qwidget.h>
#include<qdialog.h>
#include<q3groupbox.h>
#include<q3hbox.h>
//end qt related includes


#include "client.h"
#include "guiClient.h"


const int arrayLen = 254;
char sResult[arrayLen];
double returnedPos;
const int XMLstrLen = arrayLen;
char XMLpacket[XMLstrLen];	 
int portno, n;
struct sockaddr_in serv_addr;
struct hostent *server;
char buffer[arrayLen];
int replyLen=0;
int endSession = false;
IrrXMLReader *xml;
//string resultString(sResult);
int sockfd;
double pos;
double desiredXrel, desiredYrel, desiredZrel;
double desiredXabs, desiredYabs, desiredZabs;
double desiredLaserWidth = 2.0, desiredLaserAmp = 50.0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void error(char *msg)
{
    perror(msg);
    exit(0);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void closeTCPport(void){
	close(sockfd);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

int main(int argc, char *argv[])
{
 	 printf ("Starting Client Program\n");

   //~~~~~~~~~~~~~~~~~~~~~~
	 int status, ID;
	 double pos;
	//~~~~~~~~~~~~~~~~~~~~~~


	 //~~~~~~~~~~~~~~~~~~~~~~
	 if (argc < 4) {
       fprintf(stderr,"Err: usage: %s <hostname> <port> <'cmd'|'qt'>\n ", argv[0]);
       exit(0);
    }//~~~~~~~~~~~~~~~~~~~~~~
    
	
	//~~~~~~~~~~~~~~~~~~~~~~
	string qtStr("qt");
	string cmdStr("cmd");	
	string argvStr( argv[3] );
	int useQTpanel;
	if ( argvStr.compare(qtStr) == 0){		//0 if true, -1 if false
		printf("QT Panel Interface\n");
		useQTpanel = 1;
	}else if (argvStr.compare(cmdStr) == 0){
		useQTpanel = 0;
		printf("CMD Line Interface\n");
	}else{
		printf("Err: Invalid Command Option (use 'qt' or 'cmd')\n");
		exit(1);
	}
	//~~~~~~~~~~~~~~~~~~~~~~
	
	//#define noTCP //use for debugging client when server isn't connected
	
	#ifndef noTCP
	 //~~~~~~~~~~~~~~~~~~~~~~
	 portno = atoi(argv[2]);
    	sockfd = socket(AF_INET, SOCK_STREAM, 0);
    	if (sockfd < 0) 
        	error("ERROR opening socket");
    	server = gethostbyname(argv[1]);
    	if (server == NULL) {
        	fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }//~~~~~~~~~~~~~~~~~~~~~~
    
	 
	 //~~~~~~~~~~~~~~~~~~~~~~
	 bzero((char *) &serv_addr, sizeof(serv_addr));
    	serv_addr.sin_family = AF_INET;
    	bcopy((char *)server->h_addr,
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    	serv_addr.sin_port = htons(portno);
  
	 if (connect(sockfd,(const sockaddr*)&serv_addr,sizeof(serv_addr)) < 0)
        error("Err connecting");
	 //~~~~~~~~~~~~~~~~~~~~~~
	#endif
	 double params[2]={0.0, 0.0};


	//~~~~~~~  QT ~~~~~~~~~~
	if ( useQTpanel == 1){
	QApplication a( argc, argv );
	customWidget w;
	//w.setFixedHeight(500);
   	w.show();
   	return a.exec();
	//how to shut down gracefully?
	exit(0);
	}//~~~~~~~~~~~~~~~~~~~~~~

	 //=======================================================
	 endSession = false;
	 while (1){

   	 //~~~~~~~~~~~~~~~~~~~~~~
		printf("Please enter the message: ");
   	 bzero(buffer,arrayLen);
   	 fgets(buffer,arrayLen,stdin);
	
		 //now get the parts of the entered message:
		 double parsedReply[5];
		 status = parseCharReplyIntoArray(buffer, parsedReply);
		 
		 double cmdArray[5];

		 
		 for (int i = 0; i < 3; i++){
			cmdArray[i] = parsedReply[ i+2 ];
		}
		/*printf("ID=%f\n", parsedReply[0] ); 
		printf("cmd=%f\n", parsedReply[1] );
		printf("param1:%f\n", cmdArray[ 0 ]);
		printf("param2:%f\n", cmdArray[ 1 ]);
		*/
		//~~~~~~~~~~~~~~~~~~~~~~ 
		 
		//~~~~~~~~~~~~~~~~~~~~~~
		 memset(XMLpacket, 0, XMLstrLen); 
		 int numParams = 0;
		 if (  (parsedReply[1] == GOTO_HOME) | (parsedReply[1] == FIND_HOME) | (parsedReply[1] == SEQ_1) |
				 						 	(parsedReply[1] == GIVE_POS) | (parsedReply[1] == KILL) ){		//takes no parameters
			 numParams = 0;
			 
		 }else if ( (parsedReply[1] == REL_MOTION) | (parsedReply[1] == ABS_MOTION) ){
			 numParams = 1;
		}		 			 
		 
		
		if ( (buffer[0] == 'e') & (buffer[1] == 'n') & (buffer[2] == 'd') ){
			//special case to kill the connection.
			//send a kill command first
			printf("Prepping to end session...\n");
			status = writeXMLfile(1, KILL, cmdArray, 0, XMLpacket);
			endSession = true;
		 }else{
			 //normal operation
	   	 status = writeXMLfile(parsedReply[0], parsedReply[1], cmdArray, numParams, XMLpacket);
		 }
		//~~~~~~~~~~~~~~~~~~~~~~
		 //cout << XMLpacket << endl;
		 
		 //~~~~~~~~~~~~~~~~~~~~~~~~~~~
		 //cout << "written:" << buffer << endl;
		 //for (int i = 0; i < replyLen + 3; i++) printf("%d: %d\n", i, buffer[i]);
		 //~~~~~~~~~~~~~~~~~~~~~~~~~~~
		 
		 status = sendXMLpacketToServer( XMLpacket );
		 
		 //~~~~~~~~~~~~~~~~~~~~~~~~~~~
	  	 
		status = getServerReply( &ID, &pos );	
  
	  cout << "Position data received from host <axis, pos>: " << ID << ", " << returnedPos << endl;
		//~~~~~~~~~~~~~~~~~~~~~~~~~
		 
		 
		 
		 
 	
		 
	 }//=======================================================
	 
	 
    return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int getServerReply( int *IDfromServer, double* valFromServer ){
	//return the reply found by the server
	 char localCopy[arrayLen];
		
	 bzero(localCopy,arrayLen);
   	 printf("Waiting for Server Reply...\n");
	 //printf("with length = %d\n", arrayLen-1);
	 n = read(sockfd,localCopy,arrayLen-1);
	 //~~~~~~~~~~~~~~~~~~~~~~~~~~~	 
   	 if (n < 0)
         error("ERROR reading from socket");
	 //~~~~~~~~~~~~~~~~~~~~~~~~~~~	 
		 
		 	 
	 //place xml reply string into this file:
	//cout << "reply from Host: " << buffer << endl;
	ofstream File("xml.xml", ios::trunc | ios::out);
	File << localCopy;
	File.close();
	 //delete &localCopy;
	
	//now open the file and create an xml object
	xml = createIrrXMLReader("xml.xml");		//file input
	//string replyString(localCopy);
	
	//IrrXMLReader* xml = createIrrXMLReader( replyFromServer );				//memory input
	
	if ( xml == NULL ){
		printf("Err: opening file/packet\n");
		return -1;
	}
				 
	  string cmdStr;
	  string IDstr;
	  string returnedPosStr;
	
		
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
				*IDfromServer = atoi(IDstr.c_str());
				//printf("got ID=%d\n",*IDfromServer);
			}else if (!strcmp("position", xml->getNodeName() )){
				returnedPosStr = xml->getAttributeValue("pos");
 				*valFromServer = atof( returnedPosStr.c_str() );
				//printf("got pos =%f\n", *valFromServer);
			}
			break;	 
   	 }
	  }

	  //strcpy(replyFromServer, localCopy);
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
double returnPosFromMain( void ){
	return returnedPos;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
			

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int writeXMLfile(int ID, int cmd, double* params, int numParams, char* output){


	//printf("about to write xml\n");
			
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
	}
	
	
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
		xml //<< prolog() // write XML file declaration
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
	return -1;				
	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~				
int addLFtoEndofXMLstring( char *packetToSend ){
	 //now add on the LF (ascii 10) to the end, which is necessary for the string to send properly over tcp.
	replyLen = 0;
	char localCopy[254];
	//printf("input string before CR=%s\n", packetToSend);
	strcpy(localCopy, packetToSend);
	for (int i = 0; i < arrayLen; i++){
		 //printf("%d:%d\n", i, XMLpacket[i]);
		 if ( localCopy[i] == 0 ){
			 replyLen = i;
			 if ( (localCopy[i] > 127) | (localCopy[i] < 32) ){
				 if ( localCopy[i] != 0 ){
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
	 localCopy[ replyLen ] = 13;			//CR
	 localCopy[ replyLen + 1 ] = 0;		 //null
	 /*for (int i = 0; i < replyLen+2; i++){
		 printf("%d:%d\n", i, localCopy[i]);
	 }*/
	strcpy(packetToSend, localCopy);	
	//printf("input string after CR=%s\n", packetToSend);
	 
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int sendXMLpacketToServer( char* packetToSend ){
	
	int status = addLFtoEndofXMLstring(packetToSend);
	
	//printf("about to send %d characters to server:%s\n", replyLen, packetToSend);
	n = write(sockfd, packetToSend, replyLen + 1);//replyLen + 1);
	if (n < 0)
   	error("Err: Writing to Socket");

	//check special case:
	if ( endSession == true ){
		close(sockfd);
		printf("Connection Closed\n");
		exit(1);
	}
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
