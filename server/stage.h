//#ifdef __STAGE_H__
//#define __STAGE_H__
#include "irrXML.h"
#include "xmlwriter.h"
//~~~~~~~~~~~~~ function declarations ~~~~~~~~~~~
int initEverything(void);
int motInitSequence(int ID);
int initTCP(void);
void closeTCPport(void);
int endProgram(void);
int endProgramOnError(int status);
void error(char *msg);
int writeXMLposToReturn(int ID, double pos, char* output);
int writeXMLfile(int ID, int cmd, double* params, int numParams, char* output);
int isAxisID(int ID);
int checkForValidID( void );
int executeRequestedCommand(int whichID, int commandToExecute, double theParameter, int numGroupsFound);
int sendArrReply( char* replyArr );
int parseThroughXMLstring( void );
int addLFtoEndofXMLstring( void );
double returnPosFromMain( void );
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

//comment out if the laser isn't being used (ie. that serial port won't be opened, etc)
#define useMotors
#define useLaser


//#endif
