TEMPLATE	= app
LANGUAGE	= C++

CONFIG	+= qt warn_on release thread



HEADERS	+= utils.h
HEADERS += irrXML.h
HEADERS += guiClient.h
HEADERS += serial.h
HEADERS	+= client.h

SOURCES	+= utils.cc
SOURCES += irrXML.cc
SOURCES	+= guiClient.cc
SOURCES	+= client.cc


TARGET  = client

unix {
  UI_DIR = .ui
  MOC_DIR = .moc
  OBJECTS_DIR = .obj
}
QMAKE_CFLAGS_RELEASE -= -fno-exceptions
QMAKE_CXXFLAGS_RELEASE -= -fno-exceptions
#The following line was inserted by qt3to4
QT +=  qt3support 
