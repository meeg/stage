#ifndef _GUI_H_
#define _GUI_H_

#include <qapplication.h>
#include <qpushbutton.h>
#include <qslider.h>
#include <qlcdnumber.h>
#include <qfont.h>
#include <q3vbox.h>
#include <q3grid.h>
#include <qobject.h>
#include <qwidget.h>
#include <qspinbox.h>
#include <qpalette.h>
#include <qstatusbar.h>
#include <qspinbox.h>
#include <qlabel.h>
//#include </usr/include/kde/kled.h>
//#include "qfloatspinbox.h"

#include "motion.h"
//class customWidget;
#define MAX_XY_LENGTH_mm 350		//max length of XY in mm
#define MAX_Z_LENGTH_mm 30			//max length of Z in mm
#define MAX_XY_LENGTH_um 350000	//max length of XY in um
#define MAX_Z_LENGTH_um 30000		//max length of Z in um
#define LINESTEP 10					//step size of the spinboxes
const int statusDisplayTime = 10000;		//time for the status messages to be displayed
extern double desiredXrel, desiredYrel, desiredZrel;
extern double desiredXabs, desiredYabs, desiredZabs;
extern double desiredLaserWidth, desiredLaserAmp;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

class motionClass;
class FloatSpinBox;
class customWidget : public Q3VBox
{
	Q_OBJECT
	//friend class GUIinterfaceClass;		
	
	public:
		customWidget( QWidget *parent=0, const char *name=0 );
		//customWidget( serialClass *sp);
		double getPosFromMain( void );
				

	private:
		QSlider *xslider, *yslider;
		QSpinBox *XabsBox, *YabsBox, *ZabsBox;			
		QSpinBox *YrelBox, *ZrelBox;	
		//FloatSpinBox *XrelBox;
		QSpinBox *XrelBox;
		QSpinBox *laserW, *laserA;
		QStatusBar *statusBar;
		QLabel *laserTriggerLabel;
		QLCDNumber *currX,*currY, *currZ;
		//int printXvalOnChange( int val);
		//motionClass sp;		//passed in from stage.cc
		

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
		
		int sequence( void );

		int locateHome( void );
		int goHome( void );
		int getPositionsFromMotors( void );
		int resetHome( void );
		
		int updateLaserWidth( void );
		int updateLaserAmp( void );
		int sendLaserParamsToLaser( void );
		int pulseLaser( void );
		int toggleLaser( void );
		
};//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   


/*
#include <qvalidator.h>

class FloatSpinBox : public QSpinBox
{
  Q_OBJECT
 public:
  FloatSpinBox( QWidget* parent = 0, const char* name = 0 );
  FloatSpinBox( float minValue, float maxValue, int decimals = 2, float step = 1.0, 
		QWidget* parent = 0, const char* name = 0 );
  ~FloatSpinBox();

  virtual QSize sizeHint () const;

  float value() const;
  float minValue() const;
  float maxValue() const;

  void setRange( float minValue, float maxValue );

 public slots:
  void setValue( float value );

 signals:
  void valueChanged( float value );

 protected:
  virtual QString mapValueToText ( int value );
  virtual int mapTextToValue ( bool * ok = 0 );
  virtual void valueChange ();

 private:
  QDoubleValidator* dVal;

  int dec;
};

*/
		

#endif
