TEMPLATE	= app
LANGUAGE	= C++

CONFIG	+= qt warn_on release thread

QMAKE_CFLAGS   += -m32
QMAKE_CXXFLAGS += -m32
QMAKE_LFLAGS   += -m32

HEADERS	+= stage.h
HEADERS	+= utils.h
HEADERS	+= capser.h
HEADERS	+= serial.h
HEADERS += irrXML.h
HEADERS	+= laser.h
HEADERS	+= motion.h
HEADERS += xmlwriter.h
HEADERS += gui.h
#HEADERS += qfloatspinbox.h


SOURCES	+= utils.cc
SOURCES	+= capser.cc
SOURCES	+= serial.cc
SOURCES += irrXML.cc
SOURCES	+= laser.cc
SOURCES	+= stage.cc
SOURCES	+= motion.cc
SOURCES	+= gui.cc
#SOURCES += qfloatspinbox.cc

TARGET  = stage

unix {
  UI_DIR = .ui
  MOC_DIR = .moc
  OBJECTS_DIR = .obj
}
QMAKE_CFLAGS_RELEASE -= -fno-exceptions
QMAKE_CXXFLAGS_RELEASE -= -fno-exceptions
