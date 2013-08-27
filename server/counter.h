#include</usr/lib/qt-3.3/include/qobject.h>

  class Counter : public QObject
 {
     Q_OBJECT

 public:
     Counter() { m_value = 0; }
		~Counter(){};
     int value() const { return m_value; }

 public slots:
     void setValue(int value);

 signals:
     void valueChanged(int newValue);

 private:
     int m_value;
 };
