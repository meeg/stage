/*
Paul Csonka Aug 2010
Client-side GUI for the motor/laser stage system
*/

#include "guiClient.h"

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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
customWidget::customWidget( QWidget *parent, const char *name )
        : Q3VBox( parent, name )
{

	/*	
	printf("In GUI constructor\n");
	FloatSpinBox *testBox = new FloatSpinBox(0.0, 1.0, 3, 0.001F, this, "fspin");
	printf("back in gui.cc\n");
//	while(1){};
	testBox->setValue(100);
//	while(1){};
	*/
			
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//start value for the sliders, knobs, and displays
	int startValX = MAX_XY_LENGTH_um / 2 ;
	int startValY = MAX_XY_LENGTH_um / 2 ;	
	int startValZ = 0;
	desiredXabs = startValX;
	desiredYabs = startValY;
	desiredZabs = startValZ;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
					
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//define UI	
	QPushButton *gotoHome = new QPushButton( "Goto Home", this, "gotoHome" );
	QPushButton *findHome = new QPushButton( "Find Home", this, "findHome" );
	QPushButton *makeHomeButton = new QPushButton( "Make This Zero", this, "makeHome" );
	QPushButton *moveXabs = new QPushButton( "Move X Abs", this, "moveXabs" );
	QPushButton *moveYabs = new QPushButton( "Move Y Abs", this, "moveYabs" );
	QPushButton *moveZabs = new QPushButton( "Move Z Abs", this, "moveZabs" );
	
	//ABS spin boxes
	XabsBox = new QSpinBox( this, "X abs" );
	YabsBox = new QSpinBox( this, "Y abs" );
	ZabsBox = new QSpinBox( this, "Z abs" );
	XabsBox->setRange(0, MAX_XY_LENGTH_um);
	YabsBox->setRange(0, MAX_XY_LENGTH_um);
	ZabsBox->setRange(-MAX_Z_LENGTH_um, MAX_Z_LENGTH_um);	
	XabsBox->setSuffix("       X desired um Abs");
	YabsBox->setSuffix("       Y desired um Abs");	
	ZabsBox->setSuffix("       Z desired um Abs");	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
		
	
	//QGroupBox *horizontalGroupBox = new QGroupBox(tr("Horizontal layout"));
   //QHBox *layout = new QHBox;

	//layout->addWidget(posX);
	//layout->addWidget(posY);
	//setLayout(layout);

	//displays:
	//QLCDNumber *xposLCD  = new QLCDNumber( 4, this, "xlcd" );
	//QLCDNumber *yposLCD  = new QLCDNumber( 4, this, "ylcd" );
	//xposLCD->setSegmentStyle(QLCDNumber::Flat);
	//yposLCD->setSegmentStyle(QLCDNumber::Flat);	 	



	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//default values for sliders and absolute spin boxes
	xslider = new QSlider( Qt::Horizontal, this, "xslider" );
   	yslider = new QSlider( Qt::Vertical, this, "yslider" );
	xslider->setRange( 0, MAX_XY_LENGTH_um );
   	xslider->setValue( startValX ); 
	yslider->setRange( 0, MAX_XY_LENGTH_um );
   	yslider->setValue( startValY ); 
   	XabsBox->setValue( startValX ); 
	YabsBox->setValue( startValY ); 
	ZabsBox->setValue( startValZ ); 	
	XabsBox -> setLineStep(LINESTEP);
	YabsBox -> setLineStep(LINESTEP);	
	ZabsBox -> setLineStep(LINESTEP);
	xslider -> setLineStep(LINESTEP);
	yslider -> setLineStep(LINESTEP);
	
	//REL spin boxes
	//XrelBox = new FloatSpinBox ( this, "X rel" );
	//XrelBox = new FloatSpinBox ( -1*SLIDE_LENGTH, SLIDE_LENGTH, XYSTEPSIZE, this, "X rel" );
	XrelBox = new QSpinBox( this, "X rel" );
	YrelBox = new QSpinBox( this, "Y rel" );
	ZrelBox = new QSpinBox( this, "Z rel" );
   	XrelBox->setLineStep(LINESTEP);
	YrelBox->setLineStep(LINESTEP);
	ZrelBox->setLineStep(LINESTEP);

	XrelBox->setRange(-MAX_XY_LENGTH_um, MAX_XY_LENGTH_um);
	
	YrelBox->setRange(-MAX_XY_LENGTH_um, MAX_XY_LENGTH_um);
	ZrelBox->setRange(-MAX_XY_LENGTH_um, MAX_XY_LENGTH_um);	
	XrelBox->setSuffix("       X desired um Rel");
	YrelBox->setSuffix("       Y desired um Rel");	
	ZrelBox->setSuffix("       Z desired um Rel");
	
	QPushButton *moveXrel = new QPushButton( "Move X Rel", this, "moveXrel" );
	QPushButton *moveYrel = new QPushButton( "Move Y Rel", this, "moveYrel" );	
	QPushButton *moveZrel = new QPushButton( "Move Z Rel", this, "moveZrel" );	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
		
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	

	//now connect the spin boxes, sliders, and display together.
	//connect( XabsBox, SIGNAL(valueChanged(int)), xposLCD, SLOT(display(int)) );
	connect( XabsBox, SIGNAL(valueChanged(int)), xslider, SLOT(setValue(int)) );
	connect( xslider, SIGNAL(valueChanged(int)), XabsBox, SLOT(setValue(int)) );
	
	//connect( YabsBox, SIGNAL(valueChanged(int)), yposLCD, SLOT(display(int)) );
	connect( YabsBox, SIGNAL(valueChanged(int)), yslider, SLOT(setValue(int)) );
	connect( yslider, SIGNAL(valueChanged(int)), YabsBox, SLOT(setValue(int)) );

	
   //connect( xslider, SIGNAL(valueChanged(int)), xposLCD, SLOT(display(int)) );
   //connect( yslider, SIGNAL(valueChanged(int)), yposLCD, SLOT(display(int)) );	 	 
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//bottom of the panel:
	laserW = new QSpinBox( this, "laserwidth" );
	laserW->setSuffix("       Laser Width (ns)");
	laserW->setRange( 2, 50 );
   	laserW->setValue( desiredLaserWidth ); 
	laserA = new QSpinBox( this, "laseramp" );
	laserA->setSuffix("       Laser Amp (mV)");
	laserA->setRange( 0, 10000 );
   	laserA->setValue( desiredLaserAmp ); 
	QPushButton *loadLaserParams = new QPushButton( "Load Laser Params", this, "laserparams" );
	QPushButton *fireLaser = new QPushButton( "Trigger Laser", this, "firelaser" );
	
	QPushButton *getCurrPos = new QPushButton( "Get Positions", this, "getPositions" );
	currX  = new QLCDNumber( 8, this, "currX (cm)" );
	currY  = new QLCDNumber( 8, this, "currY (cm)" );
	currZ  = new QLCDNumber( 8, this, "currZ (cm)" );
	currX->setSegmentStyle(QLCDNumber::Flat);
	//currX->setSuffix("       Stage Position (mm)");
	currY->setSegmentStyle(QLCDNumber::Flat);
	currZ->setSegmentStyle(QLCDNumber::Flat);
	QPushButton *quit = new QPushButton( "Exit", this, "quit" );
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//fonts, etc.
	gotoHome->setFont( QFont( "Times", 18, QFont::Bold ) );	 
	findHome->setFont( QFont( "Times", 12, QFont::Bold ) );
	makeHomeButton->setFont( QFont( "Times", 10, QFont::Bold ) );
	XabsBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	YabsBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	ZabsBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	XrelBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	YrelBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	ZrelBox->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveXrel->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveYrel->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveZrel->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveXabs->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveYabs->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveZabs->setFont( QFont( "Times", 12, QFont::Bold ) );	
	loadLaserParams->setFont( QFont( "Helvetica", 10, QFont::Bold ) );	 
	fireLaser->setFont( QFont( "Helvetica", 10, QFont::Bold ) );	 
	getCurrPos->setFont( QFont( "Times", 15, QFont::Bold ) );
	quit->setFont( QFont( "Times", 10, QFont::Bold ) );
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//connect( XrelBox, SIGNAL(valueChanged(int)), XrelBox, SLOT(display(int)) );
		
	//connect buttons to slots
   	connect( quit, SIGNAL(clicked()), qApp, SLOT(quit()) );
   	connect( findHome, SIGNAL(clicked()), this, SLOT(locateHome()) );
	connect( makeHomeButton, SIGNAL(clicked()), this, SLOT(resetHome()) );	
   	connect( gotoHome, SIGNAL(clicked()), this, SLOT(goHome()) );
   	connect( moveXrel, SIGNAL(clicked()), this, SLOT(moveXrel()) );
   	connect( moveYrel, SIGNAL(clicked()), this, SLOT(moveYrel()) );
	connect( moveZrel, SIGNAL(clicked()), this, SLOT(moveZrel()) );		
   	connect( moveXabs, SIGNAL(clicked()), this, SLOT(moveXabs()) );
   	connect( moveYabs, SIGNAL(clicked()), this, SLOT(moveYabs()) );	
	connect( moveZabs, SIGNAL(clicked()), this, SLOT(moveZabs()) );	
  	// connect( getCurrPos, SIGNAL(clicked()), (GUIinterfaceClass *)GUIinterface, SLOT( getCurrentPositions() ) );	
	
	//connect desired position values to the hosting program
	connect( xslider, SIGNAL(valueChanged(int)), this, SLOT(updateDesiredXabs()) );
	connect( yslider, SIGNAL(valueChanged(int)), this, SLOT(updateDesiredYabs()) );
	connect( ZabsBox, SIGNAL(valueChanged(int)), this, SLOT(updateDesiredZabs()) );
	connect( XrelBox, SIGNAL(valueChanged(int)), this, SLOT(updateDesiredXrel()) );
	connect( YrelBox, SIGNAL(valueChanged(int)), this, SLOT(updateDesiredYrel()) );
	connect( ZrelBox, SIGNAL(valueChanged(int)), this, SLOT(updateDesiredZrel()) );
	connect( laserW, SIGNAL(valueChanged(int)), this, SLOT(updateLaserWidth()) );
	connect( laserA, SIGNAL(valueChanged(int)), this, SLOT(updateLaserAmp()) );
	connect( loadLaserParams, SIGNAL(clicked()), this, SLOT( sendLaserParamsToLaser() ) );
	connect( fireLaser, SIGNAL(clicked()), this, SLOT( pulseLaser() ) );
	connect( getCurrPos, SIGNAL(clicked()), this, SLOT( getPositionsFromMotors() ) );
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//connect the program interface to the GUI		    
	statusBar = new QStatusBar( this, "statusbar" );
	setFixedHeight(950);
	setFixedWidth(250);
	statusBar->message("System Ready");	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		

float customWidget::updateDesiredXrel( void ){
	desiredXrel = XrelBox->value();
	//printf("x=%f\n", desiredXrel );	
	return desiredXrel;
}

float customWidget::updateDesiredYrel( void ){
	desiredYrel = YrelBox->value();
	//printf("y=%f\n", desiredYrel );	
	return desiredYrel;
}


float customWidget::updateDesiredZrel( void ){
	desiredZrel = ZrelBox->value();
	return desiredZrel;
}

float customWidget::updateDesiredXabs( void ){
	//these update automatically as the sliders or spin boxes are changed
	desiredXabs = xslider->value();
	//printf("x=%f\n", desiredXabs );	
	return desiredXabs;
}

float customWidget::updateDesiredYabs( void ){
	//these update automatically as the sliders or spin boxes are changed
	desiredYabs = yslider->value();
	//printf("y=%f\n", desiredYabs );	
	return desiredYabs;
}

float customWidget::updateDesiredZabs( void ){
	//these update automatically as the sliders or spin boxes are changed
	desiredZabs = ZabsBox->value();
	return desiredZabs;
}


int customWidget::updateLaserWidth( void ){
	desiredLaserWidth = laserW->value();
	return 0;
}


int customWidget::updateLaserAmp( void ){
	desiredLaserAmp = laserA->value();
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::moveXrel( void ){
	//printf("Move X Rel\n");
	//moveXrelButton->setText("...");
	//show();
	int status;
	if ( X_MOTOR_ACTIVE == 1 ) {
		char replyArr[254];
		double param = desiredXrel / 10000.0;
		
		status = writeXMLfile(X_AXIS_ID, REL_MOTION, &param, 1, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X move Rel\n");
			moveXrelButton->setText("Move X Rel");			
			return -1;
		}else{
			//print out returned current position value for that axis
			//return 0;
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			statusBar->message("X Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::moveYrel( void ){
	//printf("Move Y Rel\n");	
	int status;
	if ( Y_MOTOR_ACTIVE == 1 ) {
		char replyArr[254];
		double param = desiredYrel / 10000.0;
		status = writeXMLfile(Y_AXIS_ID, REL_MOTION, &param, 1, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y move Rel\n");
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			statusBar->message("Y Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::moveZrel( void ){
	//printf("Move Z Rel\n");	
	int status;
	if ( Z_MOTOR_ACTIVE == 1 ) {
		char replyArr[254];
		double param = desiredZrel / 10000.0;
		status = writeXMLfile(Z_AXIS_ID, REL_MOTION, &param, 1, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z move Rel\n");
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			statusBar->message("Z Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::moveXabs( void ){
	//printf("Move X abs\n");	
	int status;
	if ( X_MOTOR_ACTIVE == 1 ) {
		char replyArr[254];
		double param = desiredXabs / 10000.0;
		status = writeXMLfile(X_AXIS_ID, ABS_MOTION, &param, 1, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X move Abs\n");
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			statusBar->message("X Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::moveYabs( void ){
	//printf("Move Y Abs\n");	
	int status;
	if ( Y_MOTOR_ACTIVE == 1 ){
		char replyArr[254];
		double param = desiredYabs / 10000.0;
		status = writeXMLfile(Y_AXIS_ID, ABS_MOTION, &param, 1, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y move Abs\n");
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			statusBar->message("Y Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::moveZabs( void ){
	//printf("Move Z Abs\n");	
	int status;
	if ( Z_MOTOR_ACTIVE == 1 ){
		char replyArr[254];
		double param = desiredZabs / 10000.0;
		status = writeXMLfile(Z_AXIS_ID, ABS_MOTION, &param, 1, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z move Abs\n");
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			statusBar->message("Z Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::locateHome( void ){
	int status;
	statusBar->message("Calibrated Home", statusDisplayTime);
	//printf("FindHome\n");	
	if ( X_MOTOR_ACTIVE == 1 ){
		//printf("Homing X Axis\n");
		//turn on BUSY light
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(X_AXIS_ID, FIND_HOME, &param, 0, replyArr);		//the stage takes in cm.
		//turn off BUSY light
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X locate home\n");
			return -1;			
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}	
	
	if ( Y_MOTOR_ACTIVE == 1 ){
		//printf("Homing Y Axis\n");
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(Y_AXIS_ID, FIND_HOME, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y locate home\n");
			return -1;			
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}	
	if ( Z_MOTOR_ACTIVE == 1 ){
		//printf("Homing Z Axis\n");
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(Z_AXIS_ID, FIND_HOME, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z locate home\n");
			return -1;			
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}	
		
	
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::goHome( void ){
	//printf("GoHome\n");	
	int status;
	if ( X_MOTOR_ACTIVE == 1 ){
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(X_AXIS_ID, GOTO_HOME, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X go home\n");
			return -1;			
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}
	if ( Y_MOTOR_ACTIVE == 1 ) {
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(Y_AXIS_ID, GOTO_HOME, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y go home\n");
			return -1;		
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}
	if ( Z_MOTOR_ACTIVE == 1 ){
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(Z_AXIS_ID, GOTO_HOME, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z go home\n");
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}
	statusBar->message("Moved Home", statusDisplayTime);
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::resetHome( void ){
	//printf("ResetHome\n");	
	int status;
	if ( X_MOTOR_ACTIVE == 1 ){
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(X_AXIS_ID, RESET_HOME, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X reset home\n");
			return -1;			
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}
	if ( Y_MOTOR_ACTIVE == 1 ) {
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(Y_AXIS_ID, RESET_HOME, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y reset home\n");
			return -1;		
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}
	if ( Z_MOTOR_ACTIVE == 1 ){
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(Z_AXIS_ID, RESET_HOME, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z reset home\n");
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}
	statusBar->message("New Home Set", statusDisplayTime);
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::getPositionsFromMotors( void ){
	int status;
	//printf("GetPositionsFromMotors\n");
	
	if ( X_MOTOR_ACTIVE == 1 ){
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(X_AXIS_ID, GIVE_POS, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X get pos\n");
			currX->display(ERR_POS);
			return -1;			
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
			
		}
	}
	if ( Y_MOTOR_ACTIVE == 1 ) {
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(Y_AXIS_ID, GIVE_POS, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y get pos\n");
			currY->display(ERR_POS);
			return -1;		
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
		}
	}
	if ( Z_MOTOR_ACTIVE == 1 ){
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(Z_AXIS_ID, GIVE_POS, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z get pos\n");
			currZ->display(ERR_POS);
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			updateCurrDisplay(ID, pos);
		}
	}
	statusBar->message("Updated Positions", statusDisplayTime);
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~				
void customWidget::updateCurrDisplay(int thisID, double thisPos){
	switch (thisID){
		case X_AXIS_ID:	
			currX->display( thisPos );
			break;
		case Y_AXIS_ID:	
			currY->display( thisPos );
			break;
		case Z_AXIS_ID:	
			currZ->display( thisPos );
			break;
		default:
			printf("Err: Incorrect ID in updateCurrDisplay(): got %d\n", thisID);
		break;
	}
}

double customWidget::getPosFromMain( void ){
	double position = returnPosFromMain();
	return position;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~		
int customWidget::pulseLaser( void ){
	int status;
	
	#ifdef useLaser
		//status = sp.sendLaserTrigger();
		char replyArr[254];
		double param = 0;	//dummy var
		status = writeXMLfile(LASER_ID, TRIG_LASER, &param, 0, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: trigger Laser\n");
			return -1;
		}else{
			//print out returned current position value for that axis
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
			//updateCurrDisplay(ID, pos);
		}
		statusBar->message("Laser Pulse Done", statusDisplayTime);
		return 0;		
	#else
		printf("Laser Not Active, Not Triggered\n");
		return -1;	
	#endif
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::sendLaserParamsToLaser( void ){
	int status;
	
	#ifdef useLaser
		char replyArr[254];
		status = writeXMLfile(LASER_ID, SET_LASER_WIDTH, &desiredLaserWidth, 1, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: sending Laser Width Command\n");
			return -1;
		}else{
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
		}

		status = writeXMLfile(LASER_ID, SET_LASER_AMP, &desiredLaserAmp, 1, replyArr);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: sending Laser Amp Command\n");
			return -1;
		}else{
			status = sendXMLpacketToServer(replyArr);
			int ID;
			double pos;
			status = getServerReply( &ID, &pos );	
		}					
		statusBar->message("Laser Parameters Loaded", statusDisplayTime);
		return 0;		//got through both successfully.		
	#else
		printf("Laser Not Active, Nothing Loaded\n");
		return -1;	
	#endif
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 
