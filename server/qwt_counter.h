/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
00002  * Qwt Widget Library
00003  * Copyright (C) 1997   Josef Wilgen
00004  * Copyright (C) 2002   Uwe Rathmann
00005  * 
  * This library is free software; you can redistribute it and/or
  * modify it under the terms of the Qwt License, Version 1.0
  *****************************************************************************/
 
// vim: expandtab
ifndef QWT_COUNTER_H
#define QWT_COUNTER_H
#include <qwidget.h>
#include "qwt_global.h"
#include "qwt_double_range.h"
0class QWT_EXPORT QwtCounter : public QWidget, public QwtDoubleRange
{
Q_OBJECT
Q_PROPERTY( int numButtons READ numButtons WRITE setNumButtons )
Q_PROPERTY( double basicstep READ step WRITE setStep )
Q_PROPERTY( double minValue READ minVal WRITE setMinValue )
Q_PROPERTY( double maxValue READ maxVal WRITE setMaxValue )
Q_PROPERTY( int stepButton1 READ stepButton1 WRITE setStepButton1 )
Q_PROPERTY( int stepButton2 READ stepButton2 WRITE setStepButton2 )
Q_PROPERTY( int stepButton3 READ stepButton3 WRITE setStepButton3 )
Q_PROPERTY( double value READ value WRITE setValue )
Q_PROPERTY( bool editable READ editable WRITE setEditable )
public:
enum Button 
{   
Button1,    
Button2,    
Button3,    
ButtonCnt   
};
explicit QwtCounter(QWidget *parent = NULL);
#if QT_VERSION < 0x040000
explicit QwtCounter(QWidget *parent, const char *name);
#endif
virtual ~QwtCounter();
bool editable() const;
void setEditable(bool);
void setNumButtons(int n);
int numButtons() const;
void setIncSteps(QwtCounter::Button btn, int nSteps);
int incSteps(QwtCounter::Button btn) const;
virtual void setValue(double);
virtual QSize sizeHint() const;
virtual void polish();
//a set of dummies to help the designer
double step() const;
void setStep(double s);
double minVal() const;
void setMinValue(double m);
double maxVal() const;
void setMaxValue(double m);
void setStepButton1(int nSteps);
int stepButton1() const;
void setStepButton2(int nSteps);
int stepButton2() const;
void setStepButton3(int nSteps);
int stepButton3() const;
virtual double value() const;
signals:
void buttonReleased (double value);  
void valueChanged (double value);
protected:
virtual bool event(QEvent *);
virtual void wheelEvent(QWheelEvent *);
virtual void keyPressEvent(QKeyEvent *);
virtual void rangeChange();
private slots:
void btnReleased();
 void btnClicked();
 void textChanged();

private:
void initCounter();
void updateButtons();
void showNum(double);
virtual void valueChange();
   
class PrivateData;
PrivateData *d_data;
};

#endif
