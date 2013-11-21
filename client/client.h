#ifndef _CLIENT_H_
#define _CLIENT_H_

#include <iostream>

//tcp related includes
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
//end tcp related includes

#include <stdio.h>   /* Standard input/output definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <cstring>
#include <sys/signal.h>
#include <cstdlib>
#include <bitset>
#include <unistd.h>
#include "irrXML.h"
//#include "xmlwriter.h"
#include "utils.h"
#include "serial.h"
#include "guiClient.h"
#include <fstream>


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

using namespace std;
using namespace xmlw;
using namespace irr; // irrXML is located 
using namespace io;  // in the namespace irr::io
	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

int writeXMLfile(int ID, int cmd, double* params, int numParams, char* output);
double returnPosFromMain( void );
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int writeXMLfile(int ID, int cmd, double* params, int numParams, char* output);
int addLFtoEndofXMLstring( char* packetToSend );
int sendXMLpacketToServer( char* packetToSend );
int getServerReply( int *IDfromServer, double* valFromServer );
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

#define useLaser

#define ERR_POS -1.2345

#endif
