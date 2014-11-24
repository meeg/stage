//#ifdef __STAGE_H__
//#define __STAGE_H__
#include "irrXML.h"
#include "xmlwriter.h"
#include <libxml/tree.h>
#include <libxml/parser.h>

//~~~~~~~~~~~~~ function declarations ~~~~~~~~~~~
int initEverything(void);
int motInitSequence(int ID);
int initTCP(void);
void closeTCPport(void);
int endProgram(void);
int endProgramOnError(int status);
void error(char *msg);
//int writeXMLposToReturn(int ID, double pos, char* output);
//int writeXMLfile(int ID, int cmd, double* params, int numParams, char* output);
int isAxisID(int ID);
int checkForValidID( void );
int executeRequestedCommand(int whichID, int commandToExecute, double theParameter, int numGroupsFound);
int sendArrReply( char* replyArr );
int parseThroughXMLstring( xmlDocPtr commands );
//int addLFtoEndofXMLstring( void );
double returnPosFromMain( void );
int stepAndPulseSequence(int ID, double startPos, double stepSize, int numOfIntervals, double duration, double intensity);
int stepAndPulseSequenceBothAxes(int ID1, int ID2, double stepSize1, double stepSize2, int numOfSteps1, int numOfSteps2);
int executeXMLCommand( xmlNodePtr node, xmlNodePtr& responseNode );
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

//set to 1 if that motor is actively connected and used (or if it should be used), 0 otherwise.
#define X_MOTOR_ACTIVE 0
#define Y_MOTOR_ACTIVE 0
#define Z_MOTOR_ACTIVE 0

//comment out if the laser isn't being used (ie. that serial port won't be opened, etc)
#define useLaser


//#endif
