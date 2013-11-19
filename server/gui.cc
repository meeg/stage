/*
Paul Csonka Aug 2010
Server-side GUI for the motor/laser stage system
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <qapplication.h>
#include <qpushbutton.h>
#include <qlineedit.h>
#include <qvalidator.h>
//Added by qt3to4:
#include <QLabel>
#include <gui.h>
#include "stage.h"
#include <math.h>
//#include "qfloatspinbox.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


customWidget::customWidget( QWidget *parent, const char *name )
        : Q3VBox( parent, name ){

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
	QPushButton *gotoHomeButton = new QPushButton( "Goto Home", this, "gotoHome" );
	QPushButton *findHomeButton = new QPushButton( "Find Home", this, "findHome" );
	QPushButton *makeHomeButton = new QPushButton( "Make This Zero", this, "makeHome" );
	QPushButton *sequenceButton = new QPushButton( "Execute Sequence", this, "sequence" );
	QPushButton *moveXabsButton = new QPushButton( "Move X Abs.", this, "moveXabs" );
	QPushButton *moveYabsButton = new QPushButton( "Move Y Abs.", this, "moveYabs" );
	QPushButton *moveZabsButton = new QPushButton( "Move Z Abs.", this, "moveZabs" );
	
	//ABS spin boxes
	XabsBox = new QSpinBox( this, "X abs" );
	YabsBox = new QSpinBox( this, "Y abs" );
	ZabsBox = new QSpinBox( this, "Z abs" );
	XabsBox->setRange(0, MAX_XY_LENGTH_um);
	YabsBox->setRange(0, MAX_XY_LENGTH_um);
	ZabsBox->setRange(0, MAX_Z_LENGTH_um);	
	XabsBox->setSuffix("       X desired um Abs.");
	YabsBox->setSuffix("       Y desired um Abs.");	
	ZabsBox->setSuffix("       Z desired um Abs.");	
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
	ZabsBox->setRange(-MAX_XY_LENGTH_um, MAX_XY_LENGTH_um);	
	XrelBox->setSuffix("       X desired um Rel.");
	YrelBox->setSuffix("       Y desired um Rel.");	
	ZrelBox->setSuffix("       Z desired um Rel.");
	
	QPushButton *moveXrelButton = new QPushButton( "Move X Rel.", this, "moveXrel" );
	QPushButton *moveYrelButton = new QPushButton( "Move Y Rel.", this, "moveYrel" );	
	QPushButton *moveZrelButton = new QPushButton( "Move Z Rel.", this, "moveZrel" );	
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
	QPushButton *loadLaserParamsButton = new QPushButton( "Load Laser Params", this, "laserparams" );
	QPushButton *fireLaserButton = new QPushButton( "Trigger Laser", this, "firelaser" );

	QPushButton *laserToggleButton = new QPushButton( "Laser Internal Trigger", this, "togglelaser" );
	
	QPushButton *getCurrPosButton = new QPushButton( "Get Positions", this, "getPositions" );
	currX  = new QLCDNumber( 8, this, "currX (cm)" );
	currY  = new QLCDNumber( 8, this, "currY (cm)" );
	currZ  = new QLCDNumber( 8, this, "currZ (cm)" );
	currX->setSegmentStyle(QLCDNumber::Flat);
	//currX->setSuffix("       Stage Position (mm)");
	currY->setSegmentStyle(QLCDNumber::Flat);
	currZ->setSegmentStyle(QLCDNumber::Flat);
	QPushButton *quitButton = new QPushButton( "Exit", this, "quit" );
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//fonts, etc.
	gotoHomeButton->setFont( QFont( "Times", 18, QFont::Bold ) );	 
	findHomeButton->setFont( QFont( "Times", 12, QFont::Bold ) );
	makeHomeButton->setFont( QFont( "Times", 10, QFont::Bold ) );
	sequenceButton->setFont( QFont( "Times", 10, QFont::Bold ) );
	XabsBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	YabsBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	ZabsBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	XrelBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	YrelBox->setFont( QFont( "Times", 12, QFont::Bold ) );
	ZrelBox->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveXrelButton->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveYrelButton->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveZrelButton->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveXabsButton->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveYabsButton->setFont( QFont( "Times", 12, QFont::Bold ) );	
	moveZabsButton->setFont( QFont( "Times", 12, QFont::Bold ) );	
	loadLaserParamsButton->setFont( QFont( "Helvetica", 10, QFont::Bold ) );	 
	fireLaserButton->setFont( QFont( "Helvetica", 10, QFont::Bold ) );	 
	laserToggleButton->setFont( QFont( "Helvetica", 10, QFont::Bold ) );	 
	getCurrPosButton->setFont( QFont( "Times", 15, QFont::Bold ) );
	quitButton->setFont( QFont( "Times", 10, QFont::Bold ) );
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//connect( XrelBox, SIGNAL(valueChanged(int)), XrelBox, SLOT(display(int)) );
		
	//connect buttons to slots
   	connect( quitButton, SIGNAL(clicked()), qApp, SLOT(quit()) );
  	connect( findHomeButton, SIGNAL(clicked()), this, SLOT(locateHome()) );
	connect( makeHomeButton, SIGNAL(clicked()), this, SLOT(resetHome()) );	
   	connect( gotoHomeButton, SIGNAL(clicked()), this, SLOT(goHome()) );
   	connect( sequenceButton, SIGNAL(clicked()), this, SLOT(sequence()) );
   	connect( moveXrelButton, SIGNAL(clicked()), this, SLOT(moveXrel()) );
   	connect( moveYrelButton, SIGNAL(clicked()), this, SLOT(moveYrel()) );	
   	connect( moveZrelButton, SIGNAL(clicked()), this, SLOT(moveZrel()) );	
   	connect( moveXabsButton, SIGNAL(clicked()), this, SLOT(moveXabs()) );
   	connect( moveYabsButton, SIGNAL(clicked()), this, SLOT(moveYabs()) );	
   	connect( moveZabsButton, SIGNAL(clicked()), this, SLOT(moveZabs()) );	
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
	connect( loadLaserParamsButton, SIGNAL(clicked()), this, SLOT( sendLaserParamsToLaser() ) );
	connect( fireLaserButton, SIGNAL(clicked()), this, SLOT( pulseLaser() ) );
	connect( laserToggleButton, SIGNAL(clicked()), this, SLOT( toggleLaser() ) );
	connect( getCurrPosButton, SIGNAL(clicked()), this, SLOT( getPositionsFromMotors() ) );
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//connect the program interface to the GUI		    
	statusBar = new QStatusBar( this, "statusbar" );
	laserTriggerLabel = new QLabel("text!",this);
	statusBar->addWidget(laserTriggerLabel,0,true);
	setFixedHeight(950);
	setFixedWidth(250);
	statusBar->message("System Ready");	
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
float customWidget::updateDesiredXrel( void ){
	
	//!!!!!!!!! desiredXrel = XrelBox->value()/10.0;
	desiredXrel = XrelBox->value();///10.0;
	
	//printf("x=%f\n", desiredXrel );
	//XrelBox->setValue(0.1);
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

		//	sp.setLaserWidth_ns(

		
int customWidget::updateLaserAmp( void ){
	desiredLaserAmp = laserA->value();
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::moveXrel( void ){
	printf("In move X Rel\n");	
	int status;
	if ( X_MOTOR_ACTIVE == 1 ) {
		status = executeRequestedCommand(X_AXIS_ID, REL_MOTION, desiredXrel / 10000.0, 2);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X move Rel\n");
			statusBar->message("Err in X Move", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
			statusBar->message("X Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}

int customWidget::moveYrel( void ){
	printf("In move Y Rel\n");	
	int status;
	if ( Y_MOTOR_ACTIVE == 1 ) {
		status = executeRequestedCommand(Y_AXIS_ID, REL_MOTION, desiredYrel / 10000.0, 2);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y move Rel\n");
			statusBar->message("Err in Y Move", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
			statusBar->message("Y Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}

int customWidget::moveZrel( void ){
	printf("In move Z Rel\n");	
	int status;
	if ( Z_MOTOR_ACTIVE == 1 ) {
		status = executeRequestedCommand(Z_AXIS_ID,REL_MOTION, desiredZrel / 10000.0, 2);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z move Rel\n");
			statusBar->message("Err in Z Move", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
			statusBar->message("Z Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::moveXabs( void ){
	printf("In move X abs\n");	
	int status;
	if ( X_MOTOR_ACTIVE == 1 ) {
		status = executeRequestedCommand(X_AXIS_ID, ABS_MOTION, desiredXabs / 1000.0, 2);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X move Abs\n");
			statusBar->message("Err in X Move", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
			statusBar->message("X Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}	

int customWidget::moveYabs( void ){
	printf("In move Y Abs\n");	
	int status;
	if ( Y_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(Y_AXIS_ID, ABS_MOTION, desiredYabs / 1000.0, 2);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y move Abs\n");
			statusBar->message("Err in Y Move", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
			statusBar->message("Y Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}	

int customWidget::moveZabs( void ){
	printf("In move Z Abs\n");	
	int status;
	if ( Z_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(Z_AXIS_ID, ABS_MOTION, desiredZabs / 1000.0, 2);		//the stage takes in cm.
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z move Abs\n");
			statusBar->message("Err in Z Move", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
			statusBar->message("Z Move Done", statusDisplayTime);
			return 0;
		}
	}
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::sequence( void ){
	printf("In Sequence\n");	
	int status;
	//execute a custom sequence.  It doesn't matter which ID is given, since it uses both X,Y axes.  
	//So just make	sure both are active and then call either one.
	if ( ( X_MOTOR_ACTIVE == 1 ) & ( Y_MOTOR_ACTIVE == 1 ) ){
		status = executeRequestedCommand(X_AXIS_ID, SEQ_1, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Sequence\n");
			statusBar->message("Err in Sequence", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
			statusBar->message("Sequence Completed", statusDisplayTime);
			printf("Sequence Completed\n");	
			return 0;
		}
	}
	statusBar->message("Err in Sequence: motors not active", statusDisplayTime);
	return 1;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::locateHome( void ){
	int status;
	printf("In findHome\n");	
	if ( X_MOTOR_ACTIVE == 1 ){
		//turn on BUSY light
		status = executeRequestedCommand(X_AXIS_ID, FIND_HOME, 0.0, 1);		
		//turn off BUSY light
printf("status=%d\n", status);
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X locate home\n");
			statusBar->message("Err in X Locate Home", statusDisplayTime);
			return -1;			
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
		}
	}	
	
	if ( Y_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(Y_AXIS_ID, FIND_HOME, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y locate home\n");
			statusBar->message("Err in Y Locate Home", statusDisplayTime);
			return -1;			
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
		}
	}	
	if ( Z_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(Z_AXIS_ID, FIND_HOME, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z locate home\n");
			statusBar->message("Err in Z Locate Home", statusDisplayTime);
			return -1;			
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
		}
	}	
	
	statusBar->message("Calibrated Home", statusDisplayTime);
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::goHome( void ){
	printf("In goHome\n");	
	int status;
	if ( X_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(X_AXIS_ID, GOTO_HOME, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X go home\n");
			statusBar->message("Err in X Go Home", statusDisplayTime);
			return -1;			
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
		}
	}
	if ( Y_MOTOR_ACTIVE == 1 ) {
		status = executeRequestedCommand(Y_AXIS_ID, GOTO_HOME, 0.0, 1);	
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y go home\n");
			statusBar->message("Err in Y Go Home", statusDisplayTime);
			return -1;		
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
		}
	}
	if ( Z_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(Z_AXIS_ID, GOTO_HOME, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z go home\n");
			statusBar->message("Err in Z Go Home", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
			getPositionsFromMotors();
		}
	}
	statusBar->message("Moved Home", statusDisplayTime);
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::resetHome( void ){
	int status;
	printf("In resetHome\n");
	
	if ( X_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(X_AXIS_ID, RESET_HOME, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X Reset Home\n");
			statusBar->message("Err in X Go Home", statusDisplayTime);
			currX->display(ERR_POS);
			return -1;			
		}else{
			//print out returned current position value for that axis
			currX->display( getPosFromMain() );
		}
	}
	if ( Y_MOTOR_ACTIVE == 1 ) {
		status = executeRequestedCommand(Y_AXIS_ID, RESET_HOME, 0.0, 1);	
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Y Reset Home\n");
			statusBar->message("Err in Y Go Home", statusDisplayTime);
			currX->display(ERR_POS);
			return -1;		
		}else{
			//print out returned current position value for that axis
			currY->display(getPosFromMain() );
		}
	}
	if ( Z_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(Z_AXIS_ID, RESET_HOME, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: Z Reset Home\n");
			statusBar->message("Err in Z Go Home", statusDisplayTime);
			currX->display(ERR_POS);
			return -1;
		}else{
			//print out returned current position value for that axis
			currZ->display(getPosFromMain() );
		}
	}
	statusBar->message("New Home Set", statusDisplayTime);
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int customWidget::getPositionsFromMotors( void ){
	int status;
	printf("In getPositionsFromMotors\n");
	
	if ( X_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(X_AXIS_ID, GIVE_POS, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: X get pos\n");
			statusBar->message("Err in X Get Pos", statusDisplayTime);
			currX->display(ERR_POS);
			return -1;			
		}else{
			//print out returned current position value for that axis
			currX->display( getPosFromMain() );
		}
	}
	if ( Y_MOTOR_ACTIVE == 1 ) {
		status = executeRequestedCommand(Y_AXIS_ID, GIVE_POS, 0.0, 1);	
		if ( status == -1 ) {
			//error indicator light		
			statusBar->message("Err in Y Get Pos", statusDisplayTime);
			currX->display(ERR_POS);
			return -1;		
		}else{
			//print out returned current position value for that axis
			currY->display(getPosFromMain() );
		}
	}
	if ( Z_MOTOR_ACTIVE == 1 ){
		status = executeRequestedCommand(Z_AXIS_ID, GIVE_POS, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			statusBar->message("Err in Z Get Pos", statusDisplayTime);
			currX->display(ERR_POS);
			return -1;
		}else{
			//print out returned current position value for that axis
			currZ->display(getPosFromMain() );
		}
	}
	statusBar->message("Updated Positions", statusDisplayTime);
	return 0;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~		
int customWidget::pulseLaser( void ){
	int status;
	
	#ifdef useLaser
		//status = sp.sendLaserTrigger();

		status = executeRequestedCommand(LASER_ID, TRIG_LASER, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: trigger Laser\n");
			statusBar->message("Err in Pulse Laser", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
		}
		statusBar->message("Laser Pulse Done", statusDisplayTime);
		return 0;		
	#else
		printf("Laser Not Active, Not Triggered\n");
		return -1;	
	#endif
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~		
int customWidget::toggleLaser( void ){
	int status;
	
	#ifdef useLaser
		//status = sp.sendLaserTrigger();
/*
		status = executeRequestedCommand(LASER_ID, TRIG_LASER, 0.0, 1);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: trigger Laser\n");
			statusBar->message("Err in Pulse Laser", statusDisplayTime);
			return -1;
		}else{
			//print out returned current position value for that axis
		}
*/
		laserTriggerLabel->setText("blah!");
		statusBar->message("Laser Internal Trigger", statusDisplayTime);
		return 0;		
	#else
		printf("Laser Not Active, Not Triggered\n");
		return -1;	
	#endif
}

int customWidget::sendLaserParamsToLaser( void ){
	int status;
	
	#ifdef useLaser
		status = executeRequestedCommand(LASER_ID, SET_LASER_WIDTH, desiredLaserWidth, 2);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: sending Laser Width Command\n");
			statusBar->message("Err in Set Laser Width", statusDisplayTime);
			return -1;
		}
		sleep (0.1);
		printf("Laser Amp:%f\n", desiredLaserAmp);
		status = executeRequestedCommand(LASER_ID, SET_LASER_AMP, desiredLaserAmp, 2);		
		if ( status == -1 ) {
			//error indicator light		
			printf("Err in QT: sending Laser Amp Command\n");
			statusBar->message("Err in Set Laser Amp", statusDisplayTime);
			return -1;
		}	
		statusBar->message("Laser Parameters Loaded", statusDisplayTime);
		return 0;		//got through both successfully.		
	#else
		printf("Laser Not Active, Nothing Loaded\n");
		return -1;	
	#endif
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double customWidget::getPosFromMain( void ){
	double position = returnPosFromMain();
	return position;
}//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



/*
FloatSpinBox::FloatSpinBox( QWidget* parent, const char* name )
  : QSpinBox( int(-2e6), int(2e6), 10, parent, name )
{
	//printf("in FloatSpinBox contructor\n");
  dVal = new QDoubleValidator( -2e8, 2e8, 2, this, "dVal" );
  setValidator( dVal );	
  setValue( 0.0 );
}

FloatSpinBox::FloatSpinBox( float minValue, float maxValue, int decimals, 
			    float step, QWidget* parent, const char* name )
  : QSpinBox( int( minValue * pow( 10, decimals ) ), int( maxValue * pow( 10, decimals ) ), 
	      int( rint( step * pow( 10, decimals ) ) ), parent, name ),
    dec( decimals )
{
  //printf("in floatspinbox constructor\n");
  dVal = new QDoubleValidator( minValue, maxValue, 2, this, "dVal" );
  setValidator( dVal );	
  setValue( 0.0 );
}


FloatSpinBox::~FloatSpinBox()
{
}  

QString FloatSpinBox::mapValueToText ( int value )
{
  QString s;
	//printf("mapValueToText\n");  
  s.setNum( float( value )/ pow( 10, dec ), 'f', dec );
  return s;
}

int FloatSpinBox::mapTextToValue ( bool * ok )
{
	printf("mapTextToValue\n");
  return int( cleanText().toFloat( ok ) * pow( 10, dec ) );
}

float FloatSpinBox::value() const
{
  return float( QRangeControl::value() ) / pow( 10, dec );
}

float FloatSpinBox::minValue() const
{
  return float( QRangeControl::minValue() ) / pow( 10, dec );
}

float FloatSpinBox::maxValue() const
{
  return float( QRangeControl::maxValue() ) / pow( 10, dec );
}

void FloatSpinBox::setValue( float value )
{
	//printf("set value\n");
  QRangeControl::setValue( int( value *  pow( 10, dec ) ) );
}

void FloatSpinBox::setRange( float minValue, float maxValue )
{
	//printf("set range\n");	
  QRangeControl::setRange( int( minValue *  pow( 10, dec ) ), 
			   int( maxValue *  pow( 10, dec ) ) );
  dVal->setRange( minValue, maxValue, 2 );
}

void FloatSpinBox::valueChange()
{
	//printf("value change\n");
  QSpinBox::valueChange();
  emit valueChanged( value() );
}

QSize FloatSpinBox::sizeHint() const
{
	
    QFontMetrics fm = fontMetrics();
    int h = fm.height();
    if ( h < 12 )       // ensure enough space for the button pixmaps
        h = 12;
    int w = 35;         // minimum width for the value
    int wx = fm.width( "  " );
    QString s;
    s.setNum( minValue(), 'f', dec );
    s.prepend( prefix() );
    s.append( suffix() );
    w = QMAX( w, fm.width( s ) + wx );
    s.setNum( maxValue(), 'f', dec );
    s.prepend( prefix() );
    s.append( suffix() );
    w = QMAX( w, fm.width( s ) + wx );
    s = specialValueText();
    w = QMAX( w, fm.width( s ) + wx );

   QSize r( h // buttons AND frame both sides
             + 6 // right/left margins
             + w, // widest value
             5 * 2 // top/bottom frame
             + 4 // top/bottom margins
             + h // font height
             );
				 
    return r;
	 
}
*/
		


