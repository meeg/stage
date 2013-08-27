#ifndef _GUI_H_
#define _GUI_H_

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
#include<qstatusbar.h>
//end qt related includes


#include "serial.h"
#include "client.h"

//class customWidget;
#define MAX_XY_LENGTH_mm 350		//max length of XY in mm
#define MAX_Z_LENGTH_mm 30			//max length of Z in mm
#define MAX_XY_LENGTH_um 350000	//max length of XY in um
#define MAX_Z_LENGTH_um 30000		//max length of Z in um
#define LINESTEP 10					//increment (in um) that the spin boxes move.
const int statusDisplayTime = 10000;		//time for the status messages to be displayed
extern double desiredXrel, desiredYrel, desiredZrel;
extern double desiredXabs, desiredYabs, desiredZabs;
extern double desiredLaserWidth, desiredLaserAmp;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

class customWidget : public QVBox{
	Q_OBJECT
	//friend class GUIinterfaceClass;		
	
	public:
		customWidget( QWidget *parent=0, const char *name=0 );
		//customWidget();
		double getPosFromMain( void );
		void updateCurrDisplay(int thisID, double thisPos);
					
	private:
		QSlider *xslider, *yslider;
		QSpinBox *XabsBox, *YabsBox, *ZabsBox;			
		QSpinBox *XrelBox, *YrelBox, *ZrelBox;	
		QSpinBox *laserW, *laserA;
		QStatusBar *statusBar;
		QLCDNumber *currX,*currY, *currZ;
		QPushButton *moveXrelButton, *moveYrelButton, *moveZrelButton;
		QPushButton *moveXabsButton, *moveYabsButton, *moveZabsButton;
		QPushButton *gotoHomeButton, *findHomeButton, *makeHomeButton;
		//string moveXrelText("Move X Rel");
		
		
	public slots:
		//variable updates
			
		float updateDesiredXrel( void );
		float updateDesiredYrel( void );
		float updateDesiredZrel( void );
		
		float updateDesiredXabs( void );
		float updateDesiredYabs( void );
		float updateDesiredZabs( void );	
		
		//motion commands
		int moveXrel( void );
		int moveYrel( void );		
		int moveZrel( void );		
		
		int moveXabs( void );		
		int moveYabs( void );		
		int moveZabs( void );
		
		int locateHome( void );
		int goHome( void );
		int resetHome( void );
		int getPositionsFromMotors( void );
		
		int updateLaserWidth( void );
		int updateLaserAmp( void );
		int sendLaserParamsToLaser( void );
		int pulseLaser( void );
		
};//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


#endif
